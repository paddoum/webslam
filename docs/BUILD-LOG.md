# webslam build log

A running journal of how this engine was built, milestone by milestone —
decisions, dead ends, bugs, and how each step was verified. Newest milestone
appended at the bottom.

---

## Phase 0 — where this came from

This project started from a reverse-analysis of Niantic 8th Wall's
`xr-slam.js` (the world-tracking module of their XR Engine). Key findings that
shaped the design:

- The 5.5 MB `xr-slam.js` is mostly **one 5.46 MB base64 line** = a 3.8 MB
  WebAssembly binary. The SLAM engine is C++ (internal namespace `c8`),
  compiled with Emscripten; the JS is glue (Emscripten runtime, WebGL bindings,
  sensor plumbing).
- Decoding the WASM and reading its strings exposed the architecture: a `Gr8`
  image pyramid + `GORB` keypoints, `FrameFrameTracker`, `MapTracker`/`MapBuilder`,
  `PosePnP`, `EpipolarMatcher`, **Ceres Solver + Eigen** for bundle adjustment,
  Cap'n Proto map serialization, a `MetricScaleEstimator` for monocular scale
  from IMU, and a `VpsRequestManager` cloud client.
- The narrow JS↔WASM bridge (`c8EmAsm_*`): `engineInit`, `configureXr`,
  `prepareYuvBuffer`, `stageFrame`, `processStagedFrame`, `recenter`, `query`.

**Goal of webslam:** independently reimplement that *architecture* from open
algorithms and the same toolchain (C++ → WASM via Emscripten, Eigen, Ceres).
Not derived from Niantic source — a clean-room build of the published pipeline.

Decision: build in **independently-verifiable vertical slices** (M1…M5), each
runnable in the browser before moving on.

---

## Toolchain setup

Machine: Apple Silicon mac. Already present: cmake, ninja, clang++, node 20,
git, brew. Missing: Emscripten, Eigen.

```sh
# Emscripten (the C++ -> WASM compiler — same family Niantic used)
git clone --depth 1 https://github.com/emscripten-core/emsdk.git ~/emsdk
cd ~/emsdk && ./emsdk install latest && ./emsdk activate latest
# -> emcc 6.0.0

# Eigen (header-only linear algebra; used from M3/M4 onward)
brew install eigen        # -> /opt/homebrew/include/eigen3
```

`build.sh` sources `~/emsdk/emsdk_env.sh`, then `emcmake cmake … -GNinja` and
`cmake --build`. Output lands in `web/wasm/`.

**Gotcha:** `set(CMAKE_RUNTIME_OUTPUT_DIRECTORY …)` must come **before**
`add_executable`, or the artifacts land in the build dir instead of `web/wasm/`.

---

## M1 — camera → WASM → FAST corners ✅

The vision front-end: receive RGBA camera frames, convert to grayscale, detect
corners, draw them. Analogue of the engine's `stageFrame` + Gr8 keypoint stage.

### What was built
- `src/slam_engine.h/.cpp` — `SlamEngine` class:
  - RGBA→grayscale (Rec.601 integer luma).
  - **FAST-9** corner detector: 16-pixel Bresenham circle, contiguous-arc test
    (≥9), corner score = Σ|circle − center|, then 3×3 non-maximum suppression,
    keep strongest N.
- `src/bindings.cpp` — Embind bridge: `inputView()` (Uint8Array view of the
  RGBA staging buffer), `processFrame(w,h,threshold)`, `keypoints()`
  (Float32Array view of `[x,y,score,…]`).
- `web/index.html` + `web/main.js` — `getUserMedia` → draw to a 320×240 canvas
  → `getImageData` → `inputView().set(...)` → `processFrame` → draw corners on
  an overlay. Synthetic moving test pattern as fallback when no camera.

### Bug caught during verification
First native test detected **0 keypoints**. Cause: I used the classic FAST
"high-speed" pre-rejection (require ≥3 of the 4 compass pixels on one side).
That test is only valid for **FAST-12** — for FAST-9 it wrongly rejects exact
90° corners (only 2 compass pixels fall in the dark arc). Fix: loosen the
pre-check to ≥2, rely on the full segment test. Documented inline in
`slam_engine.cpp`.

### Memory-view bug (would have hit the browser too)
First attempt wrote frames via `Module.HEAPU8.set(...)`. `HEAPU8` isn't exported
by default, and is unsafe under `ALLOW_MEMORY_GROWTH` (view goes stale). Fix:
expose `inputView()` from Embind that rebuilds a typed memory view each frame.

### Verified three ways
| Check | How | Result |
|---|---|---|
| Algorithm | `test/test_native.cpp` — synthetic bright rectangle on dark bg | 4/4 rectangle corners, 0 px error |
| Compiled artifact | `test/test_wasm.mjs` — load real `.wasm` in Node, same scene | 4/4 corners, 0 px error |
| Live browser | preview server + screenshot | 88–97 corners on moving pattern, ~120 fps |

To run the Node test, `web/wasm/package.json` declares `{"type":"module"}` so
Node treats Emscripten's ES6 `slam.js` as a module; CMake link flags include
`-sENVIRONMENT=web,worker,node` so the same artifact runs in both.

### Build/run
```sh
./build.sh
./test/run.sh                            # all native tests + the WASM smoke test
cd web && python3 -m http.server 8080    # open on phone over https/localhost
```

---

## M2 — ORB descriptors + frame-to-frame tracking ✅

Goal: describe each frame's corners, match consecutive frames, and recover the
**relative camera pose** (rotation + unit translation). Analogue of the
engine's `FrameFrameTracker` + the essential-matrix path.

### Order of work (riskiest first)
Built and tested the **two-view geometry core before anything else**, because a
subtle bug there is invisible until the whole pipeline is wired and then
impossible to localize.

1. **`src/two_view.{h,cpp}` — geometry core** (Eigen):
   - Normalized **8-point essential matrix** with Hartley isotropic
     normalization for conditioning.
   - **RANSAC** (deterministic xorshift RNG, no global state — matters for WASM
     reproducibility) scoring by **Sampson distance**; refit `E` on all inliers.
   - **Pose decomposition** `E = U diag(1,1,0) Vᵀ` → 4 candidate (R,t); pick the
     one with the most points triangulated **in front of both cameras**
     (cheirality), via linear DLT `triangulate()`.
   - Verified by `test/test_two_view.cpp`: project a known 3D cloud into two
     cameras with a known pose. **Clean: 120/120 inliers, 0.000° rotation &
     translation error.** **Noisy (0.5 px + 25 % gross outliers): RANSAC kept
     the 95 clean matches, recovered rotation to 0.14°, translation to 0.87°.**

2. **`src/orb.{h,cpp}` — descriptors + matching**:
   - **Oriented FAST** (intensity-centroid angle) + **steered BRIEF** (256-bit,
     fixed deterministic sampling pattern rotated by the keypoint orientation,
     3×3-box-smoothed samples). Hamming distance via `__builtin_popcount`.
   - **Brute-force matcher** with Lowe ratio test (0.8) + mutual cross-check.
   - Verified by `test/test_orb.cpp`: a textured image shifted by a known
     (dx,dy); matches must reproduce the displacement. **186/186 matches
     displacement-consistent (100%).**

3. **Wired into `SlamEngine`**: `setIntrinsics()` enables tracking; each
   `processFrame()` now also runs `computeOrb` → `matchHamming(prev,cur)` →
   `estimateRelativePose`, accumulates rotation, and exposes inlier match
   segments + pose via new Embind views (`matchLines`, `translationDir`,
   `relativeRotationDeg`, `accumulatedRotation`, `trackingInliers/Ok`).

### Refactors & gotchas hit along the way
- **FAST extracted to a free function** `detectFAST()` so tracking and tests can
  reuse it (was private to `SlamEngine`).
- **Header name collision:** named the shared header `features.h`. Emscripten's
  system `ctype.h` does `#include <features.h>` (a glibc header) and picked up
  *ours* via `-I src`, giving "no template named 'vector'". **Fix: renamed to
  `fast_features.h`.** (Lesson: never name a local header after a system one.)
- **Circular include:** `orb.h` and `slam_engine.h` each needed `Keypoint`.
  Resolved by putting `Keypoint` + `detectFAST` in `fast_features.h`, included
  by both; only `slam_engine.cpp` pulls in Eigen (via `two_view.h`), keeping the
  M1 headers Eigen-free.
- **Test sign-flip (mine, not the code's):** the first `test_orb` rendered the
  scene at `+offset` but asserted `+offset` displacement → 0% consistent. The
  matcher was right; the rig had the shift backwards. Also learned the BRIEF
  sampling radius (13 px) must be smaller than feature spacing, and the test
  background must be **non-repeating** or descriptors alias.

### Browser demo + a real finding
`web/main.js` now drives a **synthetic 3D point cloud with a moving virtual
camera** (so there's genuine parallax — a planar pattern is degenerate for the
essential matrix) and displays **estimated vs ground-truth relative rotation**
as a live accuracy check. Match segments drawn in green show parallax (near
points trace long lines, far points barely move).

- **Preview quirk found:** the headless preview **pauses `requestAnimationFrame`**
  when the tab isn't being painted (`rafCount` stayed 0), so the loop never ran.
  Added a `window.__step(k)` debug hook to drive frames deterministically from
  the inspector — this exercises the exact browser code path (render →
  `inputView` → `processFrame` → read pose).
- **Live result (real WASM):** tracking OK every frame, 30–100 inlier matches,
  estimated rotation **follows the true motion's trend**.
- **Honest limitation:** per-frame monocular rotation at *small* angles with a
  dominant sideways translation is **biased** (~0.3° mean error) — the classic
  rotation/translation ambiguity. The geometry is exact when motion/parallax is
  adequate (the 10° unit test → 0.14°); the live small-angle bias is precisely
  what the map + bundle adjustment (M3/M4) and IMU scale (M5) exist to fix.

### Verified
`./test/run.sh` → **ALL TESTS PASSED** (M1 FAST, two-view clean+noisy, ORB
matching, WASM smoke) + live browser tracking via the real compiled module.

---

## M3 — keyframe map + triangulation + PnP tracking ✅

Goal: stop tracking frame-to-frame (noisy) and instead build a **persistent 3D
map** and track each frame against it by PnP. Analogue of MapBuilder +
MapTracker + PosePnP. This is where the per-frame pose noise from M2 gets damped.

### Order of work (geometry core first, again)
1. **`src/pnp.{h,cpp}` — PnP solver** (the new geometry core):
   - **DLT** (6-point) hypothesis + **RANSAC** (reprojection inlier test) +
     **Gauss-Newton** refinement minimizing pixel reprojection error on SE(3)
     (left-multiplicative `exp(ξ)` update, Huber robustifier). Accepts an
     optional pose prior — the tracking loop always has the previous pose, which
     makes PnP fast and robust.
   - Verified by `test/test_pnp.cpp`: **clean → 0.000° / 0.0000 t error;
     30 % gross outliers → all rejected, 0.000° / 0.0000.**

2. **`src/map.{h,cpp}` — the map + orchestrator** (`SlamMap`): a small state
   machine `kInit → kTracking → kLost`.
   - **Init:** hold a reference frame; once a later frame has enough matches
     *and* enough median parallax, run the M2 two-view estimator, set KF0 =
     identity (world) and KF1 = recovered pose (scale fixed by `|t|=1`),
     triangulate inlier matches into world points (cheirality-checked).
   - **Track:** match the frame's descriptors to map-point descriptors, solve
     PnP seeded with the previous pose.
   - **Keyframe insertion:** on motion / low inliers / every N frames, add a
     keyframe and triangulate *new* points by matching against the previous
     keyframe (linear triangulation with the two known poses).
   - Verified by `test/test_map.cpp` (integration): drive a synthetic camera
     trajectory through the whole system; align the recovered trajectory to
     ground truth with a similarity transform (Umeyama, since monocular is only
     defined up to scale/rotation/translation). **40/41 frames tracked, ~180
     map points, 7 keyframes, trajectory RMS ≈ 4 % of path length.**

3. **Wired into `SlamEngine`** via a forward-declared `unique_ptr<SlamMap>`
   (keeps Eigen out of the public header; out-of-line destructor). New
   `enableMapping()`; `processFrame` routes to the map path when enabled, and
   exposes pose / map points / trajectory / state through Embind views.

### Two real findings the unit tests could NOT have surfaced
The integration test uses **unique fixed descriptors per point**, so matching is
trivially perfect. The browser (real ORB on rendered imagery) exposed two
genuine robustness issues:

1. **Stale map descriptors.** Each map point kept the descriptor from its first
   triangulation forever. As the rendered appearance drifts, current descriptors
   diverge and matches collapse → tracking dies after a few frames. **Fix:**
   refresh each inlier map point's descriptor to its latest observation every
   frame (a standard ORB-SLAM technique). This alone extended continuous
   tracking from ~5 frames to ~36.
2. **Strict matching starves PnP.** A tight ratio test kills matches when many
   map descriptors look alike. **Fix:** match loosely (higher `maxDist`, ratio
   0.95) and let PnP's RANSAC reject the wrong candidates.
   Also tuned keyframe insertion (`kfMinTrackInliers` 60→25, `kfEveryNFrames`
   8→12) which cut the keyframe count from 29 to 7 on the integration test —
   less map bloat.

### Browser demo
`web/main.js` builds a map from the moving synthetic 3D scene and draws: the
**persistent map reprojected with the current pose** (green — locks onto scene
features when tracking), this frame's corners (blue), and a top-down trajectory
plot. Live (real WASM): initializes a map, grows to ~160–240 points over 5–9
keyframes, and **tracks continuously for ~3 dozen frames** (40+ PnP inliers).

### Honest limitation (motivates M4/M5)
On the low-texture synthetic scene, tracking eventually **loses and does not
relocalize** (there's no relocalization yet, and uniform-gray blobs give
ambiguous descriptors). Real-camera texture sustains tracking far better. The
quantitative correctness proof is the integration test (~4 % trajectory error);
**bundle adjustment (M4)** will reduce drift, and a relocalizer would restore
tracking after loss.

### Verified
`./test/run.sh` → **ALL TESTS PASSED** (now 6 suites: M1 FAST, two-view, ORB,
PnP, map/PnP tracking, WASM smoke) + live in-browser map building & tracking.

---

## M4 — bundle adjustment ✅

Goal: jointly refine all keyframe poses and 3D points to minimize total
reprojection error, cutting the drift left by per-frame PnP. The engine's
`BundleAdjustment` stage.

### Decision: hand-rolled BA, not the Ceres library
The milestone is "bundle adjustment with Ceres," but cross-compiling the actual
Ceres library (+ glog/gflags/SuiteSparse) through Emscripten is a heavy
build-system effort that adds no *algorithmic* value to a from-scratch project.
BA is exactly the Levenberg-Marquardt + Schur-complement algorithm Ceres runs
for this problem, so it's implemented directly in Eigen — compiles to WASM
trivially and is fully testable. (Linking the literal Ceres library is a
separate, larger task if ever wanted.)

### What was built
`src/ba.{h,cpp}` — `bundleAdjust(BAProblem&)`:
- Variables: 6-DoF per (non-fixed) camera, 3 per point. At least one camera is
  fixed to anchor the gauge.
- Analytic Jacobians: reprojection w.r.t. point (`Jproj·R`) and w.r.t. camera
  pose (`Jproj·[I | -skew(Xc)]`, left-multiplicative se(3)).
- **Levenberg-Marquardt** outer loop with accept/reject + λ adaptation, Huber
  robustifier on residuals.
- **Schur complement**: marginalize the (cheap 3×3) point blocks to form a dense
  reduced camera system `S δc = b`, solve with LDLT, back-substitute the point
  updates. This is the sparse structure that makes BA tractable — exactly what
  Ceres exploits.
- Verified by `test/test_ba.cpp`: perturb a 5-camera / 120-point synthetic
  problem; BA drives mean reprojection **11.78 px → 0.12 px in 4 iterations**,
  recovering camera rotation to **0.03°** and translation to **0.007**.

### Integrated as local BA
`SlamMap::localBundleAdjust()` runs after each keyframe insertion over a sliding
window (last 7 keyframes + the points they observe), fixing the oldest keyframe
in the window, then writes the refined poses/points back and adopts the newest
keyframe's refined pose as the current pose.

### Demonstrated benefit (A/B on identical input)
`test/test_map.cpp` now runs the same synthetic trajectory twice, BA off vs on:
- **BA off:** 178 points, 7 kf, trajectory RMS **4.03 %** of path length.
- **BA on:**  158 points, 5 kf, trajectory RMS **2.95 %** — **27 % less drift.**
Confirmed running inside the WASM build in-browser (BA executes on keyframe
insertion; tracking unaffected).

### A bug worth recording
The A/B rewrite of `test_map` regressed init to 0 tracked frames. Cause: a
helper returned `std::make_pair(R, -R * C)` — `-R * C` is an **Eigen expression
template** that captures a reference to the local `C`, which dangles after
return, giving garbage poses. Isolated repro worked only because it wrapped the
result in `Vector3d(...)`. **Lesson: never return/store an Eigen expression with
`auto`/`make_pair`; force it into a concrete `Vector3d`.**

### Verified
`./test/run.sh` → **ALL TESTS PASSED** (7 suites: + BA, + map A/B with BA).

---

## M5 — IMU metric scale ✅

Goal: monocular SLAM only knows camera motion up to an unknown scale. The
accelerometer measures motion in m/s². Recover the scalar that converts visual
units → meters. The engine's `MetricScaleEstimator`.

### What was built
`src/scale.{h,cpp}` — `MetricScaleEstimator`: a sliding window of synchronized
`(visual position, world-frame linear acceleration)` samples, relating them by
`imu ≈ scale · visual`.

### Two algorithmic fixes the test forced (this was the hard milestone)
1. **Don't double-differentiate position.** First attempt aligned *accelerations*
   — second-differencing the visual positions. Clean data recovered scale
   exactly, but with realistic position noise it collapsed to **0.022 vs 0.050**:
   double-differencing amplifies high-frequency noise catastrophically, and
   ordinary least-squares slope is biased toward zero when the regressor is
   noisy (regression dilution). **Fix: match at the VELOCITY level** —
   differentiate visual position *once* (central difference) and integrate IMU
   acceleration *once* (trapezoid, mean-subtracted to drop the integration
   constant). Both are single-order ops, so noise isn't blown up.
2. **Use total least squares, not ordinary LS.** Both velocity signals are
   noisy (errors-in-variables), so the slope is estimated with TLS (the
   closed-form symmetric estimator), which is unbiased under symmetric noise.
   A Gaussian-smoothing variant was tried and rejected — it biased the *clean*
   case (smoothing interacts badly with differencing).
- Verified by `test/test_scale.cpp`: **clean 0.0500 (exact), noisy (position +
  accel noise) 0.0497 — 0.6 % error**, and the near-static case is correctly
  rejected (no motion → no scale claimed).

### Wired into the engine + browser
`SlamEngine::addScaleSample()/metricScale()/scaleConfidence()` expose it through
Embind. The browser:
- **Synthetic mode** runs a self-test: feeds the known camera path + a clean
  IMU derived from it (× a ground-truth 0.05). The dashboard shows the recovered
  scale converging to **0.0500 (true 0.05), confidence 1.00** — a live in-WASM
  proof the estimator works.
- **Camera mode** requests iOS DeviceMotion permission (button, user gesture,
  HTTPS), reads gravity-removed `event.acceleration`, rotates it device→camera
  (`x, -y, -z`)→world via the SLAM pose, and feeds it with the tracked camera
  centre. Displays scale, confidence, and a metric path length ("moved X m").

### Honest limitation
Real-device scale is **best-effort**: browser IMU is uncalibrated and not
time-synced to the camera, the camera-IMU extrinsic is approximated by an axis
flip (real VIO calibrates it), and `event.acceleration` quality varies by
device. Low-confidence estimates are gated out rather than shown wrong. The
synthetic self-test is the rigorous proof; the phone demo is the live
approximation.

### Verified
`./test/run.sh` → **ALL TESTS PASSED** (8 suites). The full M1–M5 monocular
visual-inertial SLAM pipeline is complete.

---

## M6 — tracking robustness ✅

Device testing exposed the real bottleneck: the tracker bloated (1300 points /
40+ keyframes) and, once lost, stayed lost (needed a manual reset) — which also
starved the M5 scale estimator of a clean trajectory. Three fixes, all in
`SlamMap`:

1. **Relocalization.** `process()` is now an explicit `kInit → kTracking →
   kLost` machine. When lost, it RELOCALIZES: match the frame against the whole
   map and solve PnP **globally (no pose prior, more RANSAC iterations)**,
   accepting only on a strict inlier count (`relocMinInliers`) to avoid false
   recovery. On success it resumes tracking with the map intact — no reset.
   - Verified by `test/test_reloc.cpp`: force loss with blank frames, then
     resume → **recovers to tracking, keyframes/points retained** (not reset).

2. **Motion-gated keyframe insertion + culling.** A keyframe is inserted only
   when the camera has actually moved (`> kfMinTranslation` or `>
   kfMinRotationDeg` since the last keyframe) or tracking is weak — never just
   because N frames elapsed while near-stationary (the old bloat cause).
   Keyframes are capped (`maxKeyframes`), culling the oldest non-anchor.
   - Effect in-browser over 250 frames: **max 15 keyframes / 313 points**
     (was 41 / 1300+), and the map self-recovers across loss episodes.

3. **Init-quality guard.** After two-view initialization, the median
   reprojection error of the triangulated points is checked; a poorly
   conditioned init (`> initMaxReprojErrPx`) is rejected so the map never starts
   from bad geometry.

### Track-by-projection (fixes loss when orbiting/strafing)
Device testing showed tracking dying as soon as you move *to the side* around a
mapped object — the points stay in view geometrically, but their stored ORB
descriptors were captured head-on and ORB tolerates only limited viewpoint
change, so global appearance-matching collapses. **Fix: track by projection
with a constant-velocity motion model.** Each frame:
1. predict the pose by extrapolating the last relative motion;
2. project every map point with the predicted pose;
3. match each to the nearest detected corner within a pixel search window
   (`trackByProjection`, spatially hashed) — geometry constrains the match, so
   a loose descriptor gate is fine.
Global descriptor matching is kept only for **relocalization** (no prior).
Effect on the synthetic sweep: tracked frames jumped from **66/250 → 244/250**;
the map stays bounded (≤34 keyframes). This is the standard "track the local map
by projection" that real SLAM relies on.

### Bonus: accuracy improved too
Cleaner, better-placed keyframes made bundle adjustment converge better — the
map A/B test's BA-on trajectory error dropped from **2.95 % → 0.97 %** (69 %
lower) versus before M6.

### Guided scan coverage (ARKit-style)
The initial two-view map covers only a small region, so — like ARKit/ARCore's
"move your device to scan" phase — the UI guides the user to build coverage with
a **scan-coverage bar** (`keep scanning…` → `map well covered — tracking solid`).

First attempt scored coverage from keyframe count + the bounding-box spread of
keyframe centres — but it hit 100 % almost immediately: the two-view init
baseline is *normalized to 1*, so the spread term saturates with a tiny motion.
Second attempt used a single scale-free angular span — better, but a one-axis
pan could still complete it.

**Final (ARKit-style): require BOTH pan and tilt.** `coverageYawDeg()` and
`coveragePitchDeg()` report the yaw and pitch span of keyframe viewing
directions (yaw = `atan2(dx,dz)`, pitch = `atan2(dy, hypot(dx,dz))` of the
camera forward `d` = third row of `R`). The bar is `0.5·(yaw/30°) +
0.5·(pitch/18°)` and only goes "ready" when *both* targets are met; the hint
names the deficient axis (`keep panning left/right` / `keep tilting up/down`).
Verified: the yaw-only synthetic sweep fills the pan axis but correctly stalls
asking to tilt — a single-axis scan can no longer read as complete.

### Verified
`./test/run.sh` → **ALL TESTS PASSED** (9 suites: + relocalization). Bounded map,
post-loss recovery, and the guided scan-coverage meter confirmed live in-browser.

---

## M7 — sliding map for exploration ✅

Device testing showed tracking dropping when you walk to a *new* part of the
room: the old hard point cap (1500) froze the map once full, so a freshly-seen
area had no points to track against → lost. **Fix: a sliding-window map.**

Each map point records the frame it was last matched (`lastSeen`). `process()`
keeps a monotonic frame counter; every inlier match stamps its point. After each
keyframe insertion, `cullMapPoints()` keeps the `maxMapPoints` most-recently-seen
points and drops the rest — then **compacts the points vector and remaps every
keyframe's feature→point index** so the index references stay valid (the tricky
part; verified by the tests still passing). New points are always triangulated
now (no cap-freeze), so the camera can explore indefinitely while the total
stays bounded — stale regions fall out as you move on.

- Verified by `test/test_explore.cpp`: a camera slides down a long corridor of
  points with a **small 200-point budget**; the init region leaves view well
  before the end. Result: **89/90 tracked, all 12 final frames tracked** (far
  from start), **map bounded at 200**. Staying tracked at the far end is only
  possible because new area was continuously mapped as old points were culled.
- No regression on the oscillating synthetic scene (194/200 tracked in-browser).

### Verified
`./test/run.sh` → **ALL TESTS PASSED** (10 suites: + exploration).

---

## M8 — metric-scale camera-IMU calibration ✅

On-device, scale confidence stayed near zero (~0.14) because the accelerometer
lives in the *device* frame and I rotated it to world with a **guessed fixed
axis flip** (`x,-y,-z`). If that guess is wrong, the IMU acceleration points the
wrong way versus the visual motion → no correlation → no scale.

**Fix: auto-calibrate the device→camera rotation.** A phone's camera-IMU
extrinsic is essentially one of the 24 signed axis alignments (det +1 signed
permutations). The estimator now stores the *raw* device accel plus the
world→camera rotation per sample, and at each update **tries all 24 alignments
and keeps the one whose accelerometer signal best correlates** (highest cosine)
with the visual trajectory — then reports that alignment's TLS scale. No manual
calibration gesture needed; it self-selects from the motion you're already
making. The chosen alignment index is exposed (`scaleAxis()`).

Plumbing: `addScaleSample` now takes the camera centre + the pose as a
**quaternion** (11 args; the engine rebuilds `Rwc` via `Eigen::Quaterniond`) +
raw device accel. The browser converts `poseR`→quaternion (`matToQuat`) and
stops pre-rotating the accel.

- Verified by `test/test_scale.cpp`: the clean/noisy IMU is fed in a
  deliberately **rotated device frame** (90° about X). The estimator
  **discovers the correct alignment (axis 5)** and recovers scale to
  **0.0497 / 0.0496 (true 0.05)** — it could not without finding the right
  rotation. Synthetic browser self-test still recovers 0.051 @ conf 0.98.

### Verified
`./test/run.sh` → **ALL TESTS PASSED** (10 suites). The from-scratch
visual-inertial SLAM now spans M1–M8: detection → tracking → mapping → BA →
metric scale → robustness (reloc, culling, sliding map) → camera-IMU calibration.

---

## M9.1 — learned monocular depth: inference + viz ✅ (foundation)

Scoped in `docs/DEPTH-SCOPE.md`. Decision: **Transformers.js** running
**Depth-Anything-V2-small (q8)** in a **Web Worker** — it bundles model
download, preprocessing and ONNX inference (WebGPU→WASM) far more robustly than
hand-wiring raw ONNX Runtime.

- `web/depth-worker.js`: loads the pipeline from the HF CDN, runs depth on a
  posted RGBA frame, returns the depth tensor (transferable).
- `web/main.js`: lazy worker spawn (graceful failure if the model can't load),
  sends a frame ~2 Hz (throttled, off the tracking loop), renders the result as
  a top-right **heatmap thumbnail**. A `depth: off/on` button toggles it.
- Verified in-browser: model loads ("depth model ready"), inference returns a
  depth map, heatmap renders, **SLAM tracking is unaffected** (depth is async,
  off-thread). On the synthetic dots-scene the depth content is meaningless (no
  real depth cues) — it proves the *pipeline*; real depth needs a camera.

M9.1 validated on device: loads, ~0.5 Hz, depth looks correct.

## M9.2 — align depth to the map ✅
Monocular depth is relative (affine-ambiguous). Each depth frame we fit
`invDepth ≈ a·(net/255) + b` by least squares, using the **visible sparse map
points as anchors** (project each, pair its network depth with its triangulated
inverse depth). Reports a correlation `r` and the anchor count `n` in a
**depth align** readout. `metricDepthAt(u,v)` then returns map-consistent depth
for any pixel.

**Temporal-mismatch fix:** first on-phone test gave `r≈0.04` (uncorrelated).
Cause: depth is async (~2 s), but the fit projected map points with the
*current* pose — if the camera moved in those 2 s, points landed on the wrong
pixels of the stale depth map. Fix: snapshot the pose when the frame is *sent*
to the worker and align with that pose when the depth returns (matching the
depth map's true viewpoint). Verified running; real `r` to be re-checked on
device. Note: a flat scene (low depth variance) also yields low `r` even when
correct — needs near+far structure to validate.

## M9.3 — depth-based anchor placement ✅
Tapping now back-projects the tapped pixel at its aligned depth → a 3D world
anchor, so the sphere can be placed **anywhere**, including textureless spots
with no nearby map point. Gated on alignment quality (`r > 0.5`); otherwise it
falls back to nearest-map-point / median-depth-ray placement.

**On-device load fix:** after an IP change the model "loaded forever" — a new
IP is a new browser origin, so iOS Safari re-downloaded the ~25 MB model with no
progress shown. Fixed with: `progress_callback` (show download %), `device:
'wasm'` (avoid iOS WebGPU stalls), and a 60 s watchdog.

**Alignment robustness:** raw Pearson `r` was crushed by triangulation outliers +
the nonlinearity of relative depth. Added a robust fit (trim worst 40 % by
residual, refit → `rRobust`) and a scale/nonlinearity-free **Spearman** readout
(`sp`) to distinguish "noisy-but-correct" from a genuine spatial-mapping bug.
3×3-median sampling avoids edge artifacts. Trust gate now uses `|rRobust| > 0.4`.

## M9.3 — depth densification ✅
On-phone Spearman confirmed `sp > 0.5` on a 3D scene → depth genuinely tracks
the scene, so densification is safe to build.

`SlamMap::densifyFromDepth()` (+ `SlamEngine` depth staging buffer / `depthView`
binding): for each current-frame corner that did NOT match an existing map
point, back-project it with the aligned depth + current pose and add a
**descriptor-bearing** map point. This fills low-texture gaps where
cross-keyframe triangulation can't keep up (the wood-floor case). Depth-seeded
points start with `observations=1` and are trackable by projection next frames;
ones that never re-match age out via the sliding-window `lastSeen` cull.

Safeguards (the part that took iteration):
- **Gated on Spearman `|sp| > 0.5`**, NOT `rRobust` — outlier-trimming inflates
  `rRobust` (synthetic showed `rob=0.49` while `sp=0.17`), so it would inject
  garbage; Spearman is the honest, un-inflatable signal. Verified: synthetic
  densifies `+0 pts` (correctly skipped), real scenes will fire.
- **Gated on viewpoint match**: only densifies when the camera is still within
  ~10° of the pose the (async, ~2 s old) depth frame was captured at, so the
  depth matches the current corners.
- Capped at 250 pts/frame; the map cull bounds the total.

Verified: all 10 native+WASM suites pass; densify path runs in-browser with no
errors and correctly self-gates. M9 (M9.1 inference → M9.2 alignment → M9.3
anchors + densification) complete; M9.4 single-frame init remains out of scope.

---

## M10 — gyro-aided fast-motion tracking ✅

Symptom: slow motion tracks fine, fast motion loses it. Two causes — motion
blur (physical, unfixable) and features jumping outside the projection search
window faster than the constant-velocity model predicts (it lags on sudden
acceleration). Fix the second:

- **Adaptive search radius**: the projection-tracking window now grows with the
  expected feature motion = `max(constant-velocity predicted pixel displacement,
  gyro hint)`, clamped to 150 px. Sustained motion → the const-vel term widens
  it; *sudden* motion → the gyro term widens it **instantly** (no one-frame lag).
- **Gyro hint (calibration-free)**: JS reads `devicemotion.rotationRate`,
  computes expected shift `≈ |ω|·dt·fx`, and passes it via `setMotionHint()`.
  Only the *magnitude* is needed (window size), so no camera-IMU rotation
  calibration is required. The gyro is now enabled automatically when the camera
  turns on (it aids tracking, not just M5/M8 scale).

Verified: synthetic unaffected (no gyro → hint 0 → const-vel adaptation only,
194/200 tracked); all suites green. Blur remains a hard limit; this targets the
displacement/onset half of fast-motion loss.

---

## M11 — IMU coasting (gyro as source of truth when lost) ✅

Goal: never lose the pose. When SLAM drops, dead-reckon on the gyro so the AR
sphere (and map overlay) stay anchored, then snap back on relocalization.
Implemented as a JS fusion layer over the engine (gyro + rendering live there).

- **Fused pose**: = SLAM while tracking; while lost, the fused rotation is
  propagated by integrating the gyro into the camera frame
  (`R ← exp(-[ω_cam·dt]) R`), translation held (accel double-integration drifts
  too fast to trust over a dropout).
- **Gyro→camera calibration (`Rci`)**: estimated *while tracking* by correlating
  SLAM camera angular velocity (`logSO3` of the frame-to-frame rotation) against
  the gyro vector over the 24 signed-axis candidates — same trick as M8, no SVD.
  Defaults to a phone remap until enough rotation is seen.
- **Sphere always drawn** with the fused pose (amber/dimmed while coasting); the
  map overlay also coasts so the view never blinks out. State readout shows
  `coasting (gyro)`.
- Verified in-browser: tracking → `tracking`; forced loss → `coasting (gyro)`
  with the sphere still rendering; no errors. (Synthetic has no gyro, so it
  coasts by *holding* the pose — the rotation propagation is exercised on the
  phone where the gyro streams.)

Honest limits: translation isn't propagated (pure-translation during a dropout
drifts the sphere; rotation is handled — the common "look away and back" case),
and gyro orientation drifts over a long loss. Relocalization (M6) does the
re-lock; coasting just bridges the gap.

**Fix (on-device coasting rotated the wrong way):** two bugs. (1) Sign error —
for `R = world→cam`, `R_t = exp(-[ω]dt)·R_{t-1}`, so the camera body angular
velocity is `ω = −logSO3(R_t R_{t-1}ᵀ)/dt`; I'd dropped the minus, so `Rci`
mapped the gyro to *negative* ω and coasting rotated opposite. (2) Before `Rci`
calibrates (needs real rotation while tracking), coasting used a guessed remap
that could be wrong. Now: correct sign; coasting only rotates once `Rci` is
locked (avg-cosine > 0.6 over the 24 candidates) and otherwise *holds*; a
`gyro✓ / calibrating… / uncal` indicator surfaces the state.

---

## M12 — tight visual-inertial fusion (in progress)

Scoped in `docs/VIO-SCOPE.md`. The goal: replace loose coupling with one
estimator fusing camera + IMU every frame (the Niantic `VioTracker` architecture).

### M12.0 — IMU de-risk ✅
Built `web/imu.html` and ran it on the iPhone. **Results were excellent:**
60 Hz, 0.5 ms timestamp jitter, gyro bias 0.0005 rad/s, gyro noise 0.0039 rad/s,
accel magnitude 9.81 (calibrated gravity), accel noise 0.045. Camera↔IMU sync
correlation was weak (0.23, offset ≈ −65 ms) — not blocking, since tight VIO
estimates the time offset online; a rough prior suffices. **Verdict: usable —
proceed.** (The IMU quality was the biggest risk and it cleared.)

### M12.1 — IMU preintegration ✅
`src/imu_preint.{h,cpp}`: Forster-style on-manifold preintegration of gyro+accel
into ΔR/Δv/Δp between keyframes (bias-corrected), plus the 9-vector residual
relating two VI states (R,v,p) + gravity, and SO(3) exp/log. Verified by
`test/test_imu_preint.cpp`: synthesize a known rotating+translating trajectory,
generate the IMU it would produce (with biases), preintegrate → **residual ≈ 0
at the true endpoint states** (rv 0.0007, rp 0.0009), grows with wrong bias
(velocity residual 0.082) and with a perturbed endpoint (0.20 ≈ injected 0.2 m).

### M12.2 — VI initialization ✅
`src/vi_init.{h,cpp}`: VINS-Mono-style linear visual-inertial alignment. From
up-to-scale visual keyframe poses + the preintegration between them, build and
solve one linear system for per-frame velocity, gravity (in the c0 frame), and
the metric scale. (Assumes camera≈IMU extrinsic and negligible gyro bias —
justified by M12.0's 0.0005 rad/s measurement; accel bias is left to the full
optimizer.) Verified by `test/test_vi_init.cpp`: synthesize a metric trajectory,
down-scale positions to fake SfM, generate the IMU between keyframes →
**recovers scale to 0.7 %, gravity to 0.11° / 0.014 m/s², velocity to 0.01 m/s.**

### M12.3 — sliding-window VI optimizer ✅
`src/vi_optimizer.{h,cpp}`: a windowed estimator that jointly refines keyframe
**poses + velocities** by minimizing visual pose residuals + IMU preintegration
residuals (gravity fixed from M12.2), Levenberg–Marquardt with numerical
Jacobians on the manifold (first keyframe pose fixed for gauge). Verified by
`test/test_vi_opt.cpp` with the headline scenario: two middle keyframes have
**no visual measurement (a dropout)** and are initialized 0.30 m wrong — the
optimizer **bridges them with the IMU to 0.007 m**, max trajectory error 6 mm,
velocity error 0.05 m/s. This is the core tight-VIO benefit demonstrated.

**Scoped-out (honest):** bias is NOT estimated here — `imuResidual` uses
preintegration baked at zero bias, so it has no bias dependence; estimating bias
needs bias-corrected re-preintegration (Forster's first-order Jacobians). Given
M12.0 measured a 0.0005 rad/s gyro bias and calibrated gravity, that refinement
isn't worth it for this hardware. Velocity + pose fusion is what matters and it
works.

### M12 reassessment (the "through M12.3" checkpoint)
The VI building blocks are proven offline: preintegration (M12.1), VI init
(M12.2), windowed optimizer that bridges dropouts (M12.3). **M12.4 (live wiring)
remains** — replacing the loose stack with this estimator on the device, fed by
the real high-rate IMU. The de-risk (M12.0) says the IMU can support it; the
remaining work is integration + camera-IMU time-offset handling + perf, which is
a substantial, invasive change to the live pipeline. Decision point for whether
to take that on.

### M12.4 (lite) — live VI initialization on the device ✅
The chosen middle step out of the reassessment: wire **just M12.2 (VI init)**
into the live pipeline to replace/augment the bolt-on M5/M8 correlation scale —
real value (a clean one-shot metric scale **and a gravity direction**) at
contained risk, *without* ripping out the tracker or taking on the full tight
estimator's time-sync/perf unknowns.

**`src/vi_window.{h,cpp}` — `ViWindow`.** A self-contained driver that runs the
VINS-Mono bootstrap continuously:
- `addImu()` accumulates high-rate IMU between visual keyframes.
- `onTrackedFrame()` snapshots the current up-to-scale pose when the camera has
  moved enough (translation > 0.08 visual units or rotation > 6°), preintegrating
  the buffered IMU into the gap; keeps a sliding window of ≤12 snapshots.
- Once the window is ≥5 keyframes it runs `viInitialize()` and **gates** the
  result on a plausible gravity magnitude (7–12 m/s²), so a degenerate
  low-motion solve never gets reported.
- `onLost()` breaks the preintegration chain on a tracking drop but **retains the
  last good scale/gravity** (so an anchor stays metric across a brief loss).

**Engine + bridge.** `SlamEngine::addImuSample()` feeds the window; `mapFrame()`
drives `onTrackedFrame()` / `onLost()` from the map state each frame (body =
camera, so Rwb = Rᵀ, the camera centre is the up-to-scale position). New Embind
getters: `viOk / viScale / viGravity{X,Y,Z} / viGravityMag / viConfidence /
viKeyframes / resetVi`.

**Web app.** `onMotion()` now also feeds the engine the **camera-frame** IMU —
gyro and accelerometer (specific force *including* gravity, since gravity is
solved for) rotated by the already-calibrated device→camera `Rci`, with dt from
`event.interval`. The metric readout **prefers the VI scale** (tagged `VI`, shown
with `Nkf`) when a plausible solution exists and falls back to M5/M8 otherwise;
the status line shows `vi✓ |g|9.8 12kf` once converged.

**Honest scope.** This reuses main.js's existing gyro-derived `Rci` calibration
rather than estimating the camera-IMU extrinsic in the solve, and it does **not**
estimate the camera-IMU time offset — the two hard parts deferred from full
M12.4. The snapshot/IMU alignment is therefore good to ~one frame, fine for a
bootstrap scale + gravity but not yet a tight per-frame fusion.

**Verified.** `test/test_vi_window.cpp` streams a 200 Hz IMU + ~28 Hz visual
poses through `ViWindow` exactly as the device does → **recovers scale 0.4982
(true 0.5000, 0.4 % err), |g| 9.817, gravity direction err 0.003**, and confirms
`onLost()` clears the window while retaining the last good estimate. All 14
native suites + the WASM smoke test pass. In-browser: the new build loads clean
(no console errors), the per-frame VI readout code runs over 150+ synthetic
frames without throwing, and the M5 fallback still recovers 0.0527 (true 0.05)
when VI has no device IMU to work with. **Pending: live phone test** (camera +
motion) to confirm the VI scale converges and reads ~metric in a real room.

## M13 — gyro rotation prior for tracking robustness + recovery

Phone testing surfaced the classic monocular failure modes: tracking drops on
fast turns and when orbiting an object, and recovery after a loss is slow. Root
cause in the code: the tracker predicted the next pose with a **constant-velocity
model** ([map.cpp](../src/map.cpp) `predictPose`) and the gyro only widened the
search *radius* — it never **reoriented** the prediction. A quick turn makes the
CV guess lag, map points project far from where they land, too few match → loss;
and while lost, only the *blind* global relocalizer ran (no pose prior).

**Change.** Feed the calibrated **camera-frame gyro rotation increment** into the
prediction. `SlamMap::setGyroDelta(camAngVel, dt)` stores `exp(-[ω]·dt)` (the
codebase's world→camera coasting convention); `predictPose` now takes its
**rotation from the gyro** when a prior is set (instant, reliable through fast
turns) and translation from constant-velocity. While **lost**, the rotation is
**dead-reckoned on the gyro** each frame and projection tracking is retried from
that coasted pose *before* falling back to global relocalization — so when the
view overlaps the map again it re-acquires immediately. Wired
`engine.setGyroDelta` → `main.js` (camera-frame ω = `Rci · gyroVec`, gated on
`RciCalibrated`). Added a **`gyro-aid` toggle** button so the effect can be A/B'd
live on-device.

**Honest verification caveat.** `test/test_track_gyro.cpp` confirms **no
regression** (gyro prior tracks ≥ CV frames and ≥ CV inliers: 40/40 vs 40/40,
4494 vs 4354 inliers). But it canNOT prove the on-device win: on *clean*
synthetic features the existing adaptive search radius (≤150 px) already
compensates for a lagging prediction, so the simulated margin is small — and at
extreme rates the perfect-gyro guess just reshuffles RANSAC inliers. The real
benefit is on real data (motion blur, sparse/aliased features, recovery), which
simulation doesn't model. The flip side — IMU latency or an imperfect `Rci`
making the prediction *worse* than CV — also only shows on hardware; hence the
live toggle. All 16 native suites + WASM smoke pass; in-browser loads clean, the
toggle flips, 60 synthetic frames run without error. **Live phone test (2026-06-15): gyro-aid: on beats gyro-aid: off.** Confirmed on
device — tracking is noticeably more stable with the gyro prior enabled. M13
validated end-to-end.

---

## Perf Phase 0 + 1 — measurement harness + build flags (2026-07-02)

Kicked off the optimization track (full plan in `docs/OPTIMIZATION.md`).

**Phase 0 — measurement.** `SlamEngine` now times each frame stage with portable
`std::chrono::steady_clock` (~5 reads/frame, negligible) and exposes
`stageTimesFlat()` → `[grayscale, detect, orb, mapProcess, total]` ms via the
Embind `engine.stageTimes()` view (+ `window.__stages()` debug hook). The `.wsrec`
replay gained a **`?bench=1` mode**: it runs unpaced (flat out), collects every
frame's stage times, and downloads two CSVs — `bench-summary` (p50/p95/max per
stage) and `bench-frames` (full trace). This turns any recording into a
deterministic, phone-free benchmark.

**Phase 1 — build flags.** `-msimd128` + `-flto` applied globally; `-ffast-math`
scoped to the vision kernels only (`orb.cpp`, `slam_engine.cpp`) — deliberately
NOT on the VI-init/IMU/BA/PnP solvers, which rely on well-behaved float semantics
(`allFinite()` guards, associativity). `INITIAL_MEMORY` 32→64 MB to cut
growth-copy jank.

**Verification.** All 16 native suites + WASM smoke pass; VI/BA/scale outputs
unchanged (numerical parity held, so the scoped fast-math is safe). The built
`slam.wasm` contains **2354 v128 SIMD instructions** (grayscale dropped to
~0.01 ms — effectively free). In-browser: boots clean on the new build, synthetic
scene tracks (2200 pts / 80 KFs / 101 inliers), scale self-test recovers 0.0528
(true 0.05), no console errors. Post-Phase-1 synthetic baseline: grayscale 0.01 ·
detect 0.65 · orb 1.76 · mapProc 3.99 · total 6.4 ms. **Real baseline pending a
camera `.wsrec`** — synthetic is not representative of a textured phone workload.
