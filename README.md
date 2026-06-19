# webslam — a browser monocular-inertial SLAM engine, built from scratch

A from-scratch reimplementation of the *architecture* of Niantic's 8th Wall
`xr-slam.js` world tracker, built on open algorithms and the same toolchain
(C++ → WebAssembly via Emscripten, Eigen, Ceres). This is **not** derived from
Niantic's proprietary source — it is an independent implementation of the
published SLAM pipeline that engine is built on.

## Pipeline (target architecture)

```
camera frame ─► grayscale ─► FAST corners ─► ORB descriptors        [vision front-end]
                                                  │
                                                  ▼
                          frame-to-frame match ─► relative pose       [tracking]
                                                  │
              ┌───────────────────────────────────┤
              ▼                                     ▼
        keyframe map ◄─► triangulation        PnP vs map             [mapping]
              │
              ▼
        Ceres bundle adjustment                                       [optimization]
              │
   IMU (gyro/accel) ─► metric scale + Kalman fusion ─► 6-DoF pose     [inertial fusion]
```

## Milestones

| # | Goal | Mirrors in xr-slam.js | Status |
|---|------|------------------------|--------|
| M1 | Camera→WASM bridge + FAST corners drawn on video | `stageFrame` / Gr8 keypoints | ✅ done & verified |
| M2 | ORB descriptors + frame-to-frame tracking | `FrameFrameTracker` | ✅ done & verified |
| M3 | Keyframe map + triangulation + PnP | `MapBuilder` / `MapTracker` / `PosePnP` | ✅ done & verified |
| M4 | Bundle adjustment (LM + Schur, from scratch) | `BundleAdjustment` | ✅ done & verified |
| M5 | IMU metric scale (velocity-level alignment + TLS) | `MetricScaleEstimator` | ✅ done & verified |
| M6 | Robustness: relocalization, KF culling, init guard | `MapTracker` reloc / KF mgmt | ✅ done & verified |

Each milestone is independently runnable and visually verifiable in the browser
before the next is started.

## Build

```sh
./build.sh        # configures + builds to web/wasm/
./test/run.sh     # all native tests + WASM smoke test
cd web && python3 -m http.server 8080   # open http://localhost:8080 on this machine
```

Requires: Emscripten (emcc), CMake, Eigen headers. See `build.sh` for paths.

## Run on a phone (camera)

The camera needs a secure context, so off-localhost it must be HTTPS:

```sh
./serve-phone.sh        # self-signed HTTPS on the LAN; prints the URL
```

Then on a phone **on the same Wi-Fi**:
1. Open `https://<your-mac-ip>:8443` (printed by the script).
2. Accept the self-signed cert warning (Advanced → Proceed).
3. Tap **switch to camera**, allow camera access.
4. Move/translate the phone over a textured scene (pure in-place rotation gives
   no parallax). Green segments = tracked matches; the pose readout updates.

If the phone can't connect: allow `node` through the macOS firewall
(System Settings → Network → Firewall), and confirm both devices share one Wi-Fi.
Intrinsics are approximate (≈65° FOV); tracking is qualitative until M3–M5.

## Why each piece is hard in a browser (notes from the analysis)

- No ARKit/ARCore — only `getUserMedia` video + `devicemotion` events.
- Browser IMU is uncalibrated, low-rate, and not time-synced to the camera;
  recovering metric scale (M5) is the hardest part to match.
- WASM gives near-native speed for feature extraction and Ceres optimization.
