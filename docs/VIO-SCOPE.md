# M12 (scope) — tight visual-inertial fusion

Goal: replace the **loosely-coupled** stack (vision computes the pose; IMU only
assists/falls back) with a **tightly-coupled** estimator that fuses camera +
gyro + accel in one solve every frame — the architecture behind Niantic's
`VioTracker` / `Alignment.kalman*` / `MotionPrediction`. This is the single
highest-leverage change toward "the Niantic feel."

## What it fixes (all at once)
- **Fast motion / blur**: the IMU is always in the loop, so the pose is
  propagated through frames where vision is weak — not just used as a fallback.
- **Dropouts**: coasting becomes *automatic and metric* (with velocity + biases),
  not a separate gyro-only hack (M11).
- **Scale**: metric scale becomes part of the estimated state — continuous and
  well-conditioned — instead of the bolt-on correlation of M5/M8.
- **Gravity-aligned world**: gravity is estimated, so the world is level (needed
  for believable AR — objects sit on real horizontal planes).
- **Drift**: tighter constraints + bias estimation reduce accumulated error.

It **subsumes** several current pieces: M5/M8 scale (now a state variable), M11
coasting (emergent from the fused state), M10 gyro search-hint (the motion model
*is* the IMU now).

## State being estimated (per keyframe in the window)
- Pose `R, p` (orientation, position)
- Velocity `v`
- **Gyro bias `b_g`**, **accel bias `b_a`** (these drift; assuming zero — as we
  do now — is a real error source)
- Plus shared: gravity direction `g` and metric scale (resolved at init, then
  refined)

## Core components
1. **IMU model + on-manifold preintegration** (Forster et al.): integrate the
   high-rate gyro+accel between consecutive keyframes into a single relative
   motion constraint (Δrotation, Δvelocity, Δposition) with a covariance and
   bias-update Jacobians — so we don't re-integrate raw samples inside the
   optimizer. This is the technical heart and the most intricate math.
2. **Estimator** — two options:
   - **(A) Sliding-window optimizer (VINS-Mono-lite) — recommended.** Optimize a
     window of recent keyframe states + visible map points by minimizing
     **visual reprojection residuals + IMU preintegration residuals** jointly
     (Levenberg–Marquardt, Schur-complement). **We already have this machinery**
     in `ba.cpp` — extend it with velocity/bias states and IMU residual blocks.
     More accurate; reuses our investment; matches their `BundleAdjustment`+Ceres
     lineage.
   - **(B) MSCKF (EKF).** Lighter, real-time-friendly filter that marginalizes
     features. Different code path (a filter, from scratch). Lower CPU, lower
     accuracy, more bookkeeping.
3. **VI initialization** (the VINS-Mono init): from a short window of vision
   (up-to-scale poses) + IMU, solve a linear system for **gravity, metric scale,
   per-frame velocities, and gyro bias**. This is also where startup scale comes
   from (replacing M5/M8).
4. **Camera–IMU extrinsic + time offset**: tight VIO needs the device→camera
   rotation (we approximate it in M8/M11 via the 24-candidate search — use that
   as the init) and ideally a **time offset** (browser camera/IMU timestamps
   aren't synced — model it as a state or estimate a constant offset).

## Integration / plumbing changes
- **High-rate IMU intake**: today we pass one accel sample per *frame* (for
  scale). Tight VIO needs the **raw IMU stream** (~60–100 Hz) with timestamps.
  Add `addImuSample(t, gx,gy,gz, ax,ay,az)` writing into a C++ ring buffer;
  preintegration consumes the samples between frame timestamps.
- **The pose now comes from the fused state** — always available, so tracking +
  coasting + scale all read from one source. The M3 PnP becomes the *visual
  residual provider* feeding the optimizer, not the pose authority.
- Keep the **loose stack behind a flag** as a fallback during bring-up.

## Milestones (incremental, synthetic-tested like the rest)
- **M12.1 — IMU preintegration + bias.** Implement on-manifold preintegration.
  *Test:* integrate synthetic IMU over an interval; recover the ground-truth
  Δpose/Δvelocity; verify bias-update Jacobians by finite differences.
- **M12.2 — VI initialization.** Recover gravity + scale + velocities + gyro bias
  from a synthetic vision+IMU window. *Test:* known gravity/scale recovered.
- **M12.3 — Sliding-window VI optimizer.** Add velocity/bias states + IMU
  residuals to `ba.cpp`; joint solve. *Test:* synthetic trajectory with noisy
  IMU — recovered poses/scale/gravity beat vision-only BA, biases converge.
- **M12.4 — Live wiring.** High-rate IMU binding, fused pose drives rendering,
  gravity-aligned world. *Test on phone:* fast motion holds, dropouts coast
  metrically, scale locks and stays, world is level.

M12.1–M12.3 are offline/synthetic-verifiable (high confidence). M12.4 is where
real-device IMU quality decides the outcome.

## Risks / honest unknowns
- **Browser IMU quality is the wildcard.** Tight VIO *amplifies* IMU
  requirements (rate, noise, calibration, timestamp accuracy). We've already
  seen the iOS Safari IMU is marginal and not time-synced to the camera. The
  math will be correct on synthetic; on-device gains depend on whether the
  sensor stream is good enough. **This is the biggest risk — the algorithm is
  the easy part; the data may not cooperate.**
- **Preintegration + Jacobians are intricate** (rotation manifold, bias
  correction, covariance propagation) — well-documented but error-prone; the
  synthetic tests are essential.
- **Time synchronization**: camera and IMU timestamps from the browser are loose;
  a wrong offset wrecks tight fusion. Likely need to estimate it.
- **Performance**: a sliding-window optimizer at frame rate in single-threaded
  WASM may force a small window (e.g. 8–10 keyframes) and/or SIMD/threads.
- **Invasiveness**: this restructures the core; keep the loose path as fallback.

## M12.0 — IMU de-risk (gate, build first) ✅ tool ready
Before committing to the VIO build, measure whether the browser IMU can support
it. `web/imu.html` + `imu.js` (standalone page, no SLAM) reports:
- **sample rate** (Hz) and **inter-sample timestamp jitter** (σ ms),
- **at-rest gyro bias** (|mean ω|) and **gyro noise** (σ),
- **accel mean magnitude** (should be ≈9.81 — sanity that gravity is calibrated)
  and **linear-accel noise**,
- **camera↔IMU time offset** via cross-correlating gyro |ω| against camera
  frame-motion energy during a shake (ms offset + correlation quality),
- **handler latency** (perf.now − event.timeStamp).

Verdict thresholds (rough VIO viability): rate ≥50 Hz good / ≥30 marginal;
jitter <5 ms good / <15 marginal; gyro noise <0.02 rad/s good / <0.05 marginal.
Logic verified on synthetic events (bias/noise/gravity correct); real numbers
come from the phone. **If the verdict is "poor", tight VIO won't help on this
hardware and we stop here; if "usable/marginal", proceed to M12.1.**

## Decisions needed before building
1. **Estimator**: sliding-window optimizer (reuse our BA — recommended) vs MSCKF.
2. **How far now**: build + synthetic-verify through **M12.3** (the VI optimizer,
   provable offline) and *then* decide on M12.4 live wiring? Or commit to M12.4.
3. **Accept the IMU risk**: proceed knowing the math will be proven on synthetic
   but on-device benefit hinges on browser IMU quality (which is marginal)?
