# webslam — Performance Optimization Plan

Status: analysis + plan (M14 baseline). Ordered cheapest-impact-first. Every phase
is gated on the measurement harness in Phase 0 so we tune against numbers, not vibes.

## Progress

- ✅ **Phase 0 (measurement)** — done. `SlamEngine::stageTimesFlat()` exposes
  `[grayscale, detect, orb, mapProcess, total]` ms/frame (Embind: `engine.stageTimes()`,
  live debug hook: `window.__stages()`). Replaying a `.wsrec` with `?bench=1` runs
  unpaced and downloads two CSVs: `bench-summary` (p50/p95/max per stage) and
  `bench-frames` (full per-frame trace). All native + WASM tests pass.
- ✅ **Phase 1 (build flags)** — done. `-msimd128` + `-flto` global, `-ffast-math`
  scoped to `orb.cpp`/`slam_engine.cpp` only, `INITIAL_MEMORY` 32→64 MB. Verified:
  2354 v128 SIMD instructions in the binary (grayscale now ~0.01 ms — near free);
  VI/BA/scale tests unchanged (numerical parity held). Awaiting a real orbit `.wsrec`
  to quantify the phone-workload win vs baseline.
- ✅ **Phase 3 (ORB)** — partial. **Integral-image BRIEF smoothing shipped**:
  bit-identical descriptors, **−20% total p95 / −33% orb p95** on the orbit clip,
  zero tracking change (see `docs/bench/baseline-orbit.md`). **Grid feature selection
  reverted** — didn't help orbit robustness and risks starving inliers; shelved as a
  separate map-aware experiment. Also fixed a replay-fidelity bug (wall-clock dt →
  recorded dt) so the bench is now deterministic. Remaining ORB headroom: orientation
  patch cost, SIMD BRIEF compare/pack.
- ✅ **Phase 5 (RANSAC/matching)** — shipped. **Adaptive RANSAC termination** in
  `solvePnP` (early-exit at 99% confidence) + **64-bit popcount** in `hamming`. On the
  orbit clip: **worst frame 120→47 ms (−61%), mapProc spike frames 18→9, total p95
  −11%** — and tracking *improved* (losses 12→5) because early-exit keeps the
  motion-consistent guess pose. Deterministic; `test_pnp`/`test_reloc` pass. Note:
  adaptive RANSAC changes the accepted hypothesis (not a pure speedup) — stabilizing by
  design, but watch lost/inlier on future clips. See `docs/bench/baseline-orbit.md`.
- ⏳ Phases 2, 4 — pending. Remaining tail is **KF-insert BA on the main thread** →
  best addressed by **Phase 4** (move SLAM/BA off the main thread) rather than cutting
  BA quality. Cumulative so far: total p95 29.5→23.4 ms (−21%), max 61.8→47 ms (−24%).

> Baseline (synthetic scene, full 2200-pt/80-KF map, this dev machine, post-Phase-1):
> grayscale 0.01 · detect 0.65 · orb 1.76 · mapProc 3.99 · **total 6.4 ms**.
> Note: synthetic ≠ phone workload — the real baseline comes from a camera `.wsrec`.

The single most valuable enabler already exists: the `.wsrec` record/replay path.
It gives us a **deterministic, phone-free benchmark** — replay the same clip, measure
per-stage ms, change one thing, replay again. Use it everywhere below.

---

## Phase 0 — Measurement harness (do this first)

Without per-stage timing we're guessing. Cheap and unblocks everything.

- **C++ side:** add a lightweight scoped timer around each stage in `processFrame`
  (`toGrayscale`, `detectFAST`, `computeOrb`, `trackByProjection` / `relocalizeGlobal`,
  `insertKeyframe`/`localBundleAdjust`). Accumulate into a struct, expose via one
  Embind getter returning a Float32 view. Guard behind a `#ifdef WSLAM_PROFILE` so it
  compiles out of the shipping build.
- **JS side:** in replay mode, log `performance.now()` deltas per frame and dump a CSV
  (frame, total ms, stage ms, mapPts, kfs, inliers, state). Replay already exists in
  `loadAndReplay`; add an opt-in `?bench=1` that runs replay as fast as possible (no
  pacing `setTimeout`) and prints percentiles (p50/p95/max ms).
- **Deliverable:** a table of ms/stage on a representative orbit clip + a floor-scan
  clip. This is the baseline every later phase is compared against.

Acceptance: reproducible p50/p95 frame time per stage from a `.wsrec` file.

---

## Phase 1 — Build flags (near-free, largest ratio)

All in `CMakeLists.txt` LINK_FLAGS / add matching COMPILE flags.

- **`-msimd128`** — enable WASM SIMD. Autovectorizes luma, FAST scoring, and lets
  Eigen use packed math. Biggest single lever for the scalar kernels.
- **`-flto`** — link-time optimization; inlines across TUs (e.g. `hammingDistance`,
  `fastScore`). 
- **`-msse4.2` / `-mrelaxed-simd`** (via Emscripten) if targeting modern mobile Safari;
  gate on browser support and keep a non-SIMD fallback build.
- **`-fno-exceptions -fno-rtti`** — we don't use either in the hot path; shrinks and
  speeds code. Verify Embind still links (it can, with care) or scope the flag to the
  vision TUs only.
- Consider **`-ffast-math`** *only* on the vision kernels (NOT on the VI/BA solvers,
  where NaN/inf handling and associativity matter). Apply per-file, not globally.
- Bump **`-sINITIAL_MEMORY`** so `ALLOW_MEMORY_GROWTH` rarely triggers mid-session
  (growth causes a copy + jank). Size it to the steady-state working set.

Acceptance: rebuild, replay bench, expect a meaningful drop on `detectFAST`/`computeOrb`
with **zero code changes**. Confirm numerical parity on the VI/BA tests.

---

## Phase 2 — Kill per-frame allocations + hot-loop data structures

No algorithm change, just stop thrashing the allocator and the cache.

1. **`detectFAST` score buffer** (slam_engine.cpp:75): promote the `std::vector<float>`
   to a reusable member buffer on `SlamEngine`, cleared with `memset`, not reallocated.
2. **`trackByProjection` grid** (map.cpp:204): replace `unordered_map<long long,vector<int>>`
   with a **flat grid** — a `std::vector<int>` bucket array sized to the frame, reused
   across frames. Removes hashing + per-cell heap allocs on every tracked frame.
3. **Hamming popcount** (orb.cpp:101): operate on `uint64_t` (4 popcounts instead of 8),
   or `__builtin_popcountll`; with `-msimd128` consider a vectorized popcount. Hamming is
   called millions of times in matching/tracking.
4. **Reuse scratch vectors** in `trackByProjection` / `solvePnP` (the `mm`, `wp`, `uv`,
   `used` buffers) as members instead of per-frame locals.

Acceptance: allocations/frame near-zero in a heap profile; measurable p95 improvement.

---

## Phase 3 — ORB descriptor & FAST algorithmic cost (the biggest CPU sink)

`computeOrb` is ~7 M reads/frame at 1200 features. Attack both the per-feature cost and
the feature count.

1. **Lower & redistribute features.** `MAX_FEATURES=1200` (main.js:6) is high for QVGA.
   Drop toward 700–900 **with grid-based selection**: divide the frame into cells, keep
   the top-K FAST corners per cell. Better spatial spread (helps tracking/orbit) at lower
   total count — a quality *and* speed win.
2. **Cheaper BRIEF smoothing.** `sampleSmooth` does a 9-tap box filter per BRIEF endpoint
   (512 endpoints × 9 reads/kp). Replace with an **integral image** computed once per
   frame → O(1) box sample, or blur the gray image once and sample single pixels. Removes
   the inner 9-tap loop entirely.
3. **Orientation cost.** The radius-15 intensity-centroid patch (~700 px/kp) can use a
   **precomputed circular offset list** (skip the `dx*dx+dy*dy>r2` test and bounds checks
   for interior keypoints by guaranteeing a border). Or reduce radius to 11–13.
4. **FAST early-out.** The Pass-1 compass reject is good; add a coarser **decimated first
   pass** (score every 2nd pixel to seed NMS candidates) or restrict scoring to a border-
   inset region matching the ORB border (18 px) so we don't score pixels ORB will discard.
5. **SIMD BRIEF (with Phase 1).** The 256-bit descriptor build is embarrassingly parallel;
   vectorize the compare-and-pack once smoothing is single-sample.

Acceptance: `computeOrb` ms cut by ≥2× at equal or better tracking inlier counts on the
orbit clip.

---

## Phase 4 — Architecture: get SLAM off the main thread

Today `getImageData` + `processFrame` + all overlay drawing share the RAF frame, so SLAM
latency directly caps FPS and janks the UI.

- **Move the engine into a dedicated Web Worker.** Main thread only grabs frames and
  draws overlays; the worker runs WASM SLAM. Decouples SLAM rate from render rate — the
  AR overlay can interpolate/coast on the last pose + gyro at 60fps while SLAM runs at
  whatever rate it sustains.
- **Frame transfer without a copy:** use **WebCodecs `VideoFrame`** or `grabFrame` /
  `OffscreenCanvas` in the worker instead of a main-thread `getImageData` (307 KB
  readback/frame). Transfer buffers (zero-copy) rather than structured-clone.
- **Single readback:** today depth mode calls `getImageData` a *second* time (main.js:538)
  for the same frame. Reuse the one buffer.
- If pthreads are acceptable, `-pthread` + `SharedArrayBuffer` needs COOP/COEP headers
  (check `serve-phone.sh` and the GitHub Pages deploy). The single-worker offload above is
  simpler and needs no cross-origin isolation.

Acceptance: render loop holds ~60fps regardless of SLAM cost; SLAM runs concurrently.

---

## Phase 5 — PnP / matching / BA numerics

Lower priority (only hot during reloc or KF insert) but worth it.

1. **RANSAC iteration counts.** Tracking PnP uses 60 iters *with* a pose guess (map.cpp:239)
   — the guess usually wins immediately; make iters adaptive (stop early when inlier ratio
   is high). Reloc uses 300 (map.cpp:268) — adaptive termination cuts this a lot.
2. **DLT SVD cost.** `dltPnP` builds `MatrixXd A(2n,12)` and runs `JacobiSVD` every RANSAC
   iter. For the minimal n=6 case use a fixed-size solver / `BDCSVD` or a P3P minimal
   solver; reuse the `A` allocation.
3. **`matchHamming` is brute-force O(nₐ·n_b)** (orb.cpp:110), doubled by cross-check.
   For relocalization (2200 map descs × 1200) this is millions of ops. Options: restrict
   candidates by the last-known pose region, or a coarse **LSH / BoW-style** prefilter on
   the descriptor. Cross-check can be replaced by the ratio test alone in the hot paths.
4. **Local BA** (`localBundleAdjust`, 8 iters) runs on the main thread at KF insert — move
   with Phase 4, or budget it (skip if the previous BA hasn't converged).

Acceptance: reloc latency and KF-insert spikes drop out of the p95 frame time.

---

## Non-goals / watch-outs

- **Don't** apply `-ffast-math` globally — the VI init, IMU preintegration, and BA rely on
  well-behaved float semantics. Scope it to `orb.cpp`/`slam_engine.cpp` only.
- **Don't** regress orbit robustness (M14) for speed. The grid-based feature selection in
  Phase 3 should *help* coverage; verify inlier counts on the orbit `.wsrec` before/after.
- Keep a **non-SIMD fallback build** until SIMD support on the target devices is confirmed.

## Suggested order of execution

Phase 0 → 1 (measure the free win) → 2 → 3 (the real CPU sink) → 4 (smoothness) → 5.
Re-run the same two `.wsrec` clips after each phase; keep the CSVs to show the trend.
