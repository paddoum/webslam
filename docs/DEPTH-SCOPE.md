# M9 (scope) — learned monocular depth

Goal: add a learned monocular **depth** network to webslam to give a *dense
per-pixel depth prior*, addressing the limits of pure sparse-geometry SLAM
(low-texture tracking, slide-to-init, sparse anchors). This is the piece the
original Niantic engine had via its `.tflite` depth/semantics models +
`DepthRenderer` that we deliberately skipped.

## What depth buys us (in priority order)
1. **Single-frame initialization** — back-project corners with depth → an
   instant map from ONE frame, removing the "slide sideways to init" step and
   the pure-rotation failure.
2. **Low-texture robustness** — a dense depth prior lets us seed/track where
   there are too few corners (your wood-floor case).
3. **Better anchor placement** — real depth under the tapped pixel instead of
   snapping to the nearest existing map point.
4. **Densification** — fill the map with depth-derived points between the
   sparse triangulated ones.

## The core technical catch: affine ambiguity
Monocular depth nets (MiDaS, Depth Anything) output **relative / affine-invariant
inverse depth** — correct *up to an unknown scale and shift, per frame*. So raw
network depth is not metric and not even consistent frame-to-frame. The fix is
to **align it to the SLAM map every frame**:

- take the sparse triangulated map points visible in the frame, project them to
  pixels, and read the network's depth there;
- solve a 2-parameter least squares `map_depth ≈ a · net_depth + b` (or in
  inverse-depth space) using those anchor points;
- apply `(a, b)` to the whole depth map → now it's in SLAM-map units.

This reuses the sparse geometry we already trust to calibrate the dense net.
(Metric still comes from M5's IMU scale; depth just gets the map *shape* dense.)

## Model + runtime options

| Model | Params | Notes |
|---|---|---|
| MiDaS v2.1 **small** | ~21M (EfficientNet) | fastest, MIT, relative depth — recommended |
| Depth Anything V2 **small** | ~25M (ViT-S) | better quality, Apache-2.0, heavier in-browser |
| FastDepth / tiny CNN | ~1–4M | fastest, lower quality, may need self-training |

| Runtime | Pros | iOS Safari |
|---|---|---|
| **ONNX Runtime Web** | WASM + WebGL + WebGPU backends | WebGPU on iOS 18+, WebGL fallback |
| TensorFlow.js | easy, WebGL/WebGPU | same |

**Recommendation:** MiDaS-small via ONNX Runtime Web (WebGL backend for iOS
reliability, WebGPU when available), input ~256×192, **run in a Web Worker**
(like Niantic's `media-worker.js`) so inference never blocks the SLAM loop.

## Performance budget (the hard constraint)
A ~21M model at 256×192 is roughly **80–200 ms/frame on a phone browser** — far
too slow for the 30–60 fps tracking loop. So depth runs **asynchronously, ~1–3
Hz, on keyframes only**, not per tracking frame. That's fine for init,
densification, and anchors; it is NOT a per-frame tracking signal. Model
download is ~20–25 MB (cache after first load).

## Architecture (how it plugs into what exists)
```
camera frame ─┬─► (main loop) FAST/ORB → track/map        [unchanged, C++/WASM]
              └─► (Web Worker) depth net → relative depth  [new, async ~2 Hz]
                                   │
                 affine-align to map points (a,b)  [JS, uses engine.mapPoints]
                                   │
                 engine.setDepth(depthBuf, w, h, a, b)     [new Embind binding]
                                   │
        C++ map uses depth to: back-project un-triangulated corners → points,
        seed single-frame init, give tap-anchor depth
```
The depth inference + alignment live in **JS/worker**; the map consumption lives
in **C++** behind one new binding (`setDepth`) so descriptors/geometry stay in
the engine.

## Milestones
- **M9.1 — Inference**: load MiDaS-small in a worker (ONNX Runtime Web), run on
  frames, return depth. Visualize as a heatmap overlay. *Verify: runs ≥2 Hz on
  the phone, plausible depth.*
- **M9.2 — Alignment**: fit `(a,b)` against visible map points each depth frame;
  show aligned depth error at the anchor points. *Verify: aligned depth at map
  points matches triangulated depth within a few %.*
- **M9.3 — Use it**: (a) depth-based anchor placement (tap → real depth);
  (b) map densification on keyframes. *Verify: anchor lands at correct depth;
  map point count rises in low-texture areas.*
- **M9.4 — Single-frame init + low-texture seeding**: bootstrap the map from one
  frame via depth; seed points where corners are too sparse. *Verify: init with
  no sideways slide; wood-floor tracking holds.*

Each milestone is independently shippable; M9.1–M9.2 are the foundation, M9.3 is
the first user-visible win, M9.4 is the big robustness payoff.

## Risks / unknowns
- **Perf on older iPhones** — may force a smaller model or lower rate.
- **Temporal flicker** — monocular depth jitters frame-to-frame; mitigate by
  using it only on keyframes + the affine alignment + light temporal smoothing.
- **Model is ML, not from-scratch** — unlike the rest of webslam, this is a
  pretrained network (the honest line: we *built* the SLAM; depth is an
  integrated off-the-shelf model, as Niantic also uses a trained model here).
- **iOS WebGPU variance** — keep the WebGL backend as the reliable path.

## Decisions needed before building
1. **Model/runtime**: MiDaS-small + ONNX Runtime Web (recommended) — OK?
2. **How far**: stop at M9.3 (anchors + densification) or go to M9.4
   (single-frame init + low-texture seeding)?
3. **Accept** a ~20–25 MB model download and depth running at ~2 Hz (not
   per-frame)?
