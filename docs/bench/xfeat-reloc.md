# XFeat relocalization channel — built, measured, SHELVED (off by default)

Motivated by the "hard time relocating" clip and the LingBot-Map/SuperMap
discussion: learned features (XFeat, CVPR 2024) are more distinctive than ORB
on repetitive texture, so an XFeat appearance-reloc channel *should* recover
where the binary-Hamming matcher drowns. Built end-to-end behind `?xfeat=1`.

## Phase 0 (spike) — PASSED
XFeat backbone -> ONNX static 320x256, 1.5M params, 2.7 MB. Parity vs PyTorch
= 1.00000. Latency 9.3 ms native / ~64 ms in-browser WASM single-thread.
Match quality vs ORB on reloc-clip pairs: 2-3x more geometric inliers at
small/moderate baseline, but mixed (one pair ORB won; widest baselines hard for
both). (fp16 export unusable — onnxruntime-web WASM rejects the mixed-precision
Conv; fp32 shipped, still tiny.)

## Phase 1 — built
- `web/feat-worker.js`: XFeat ONNX worker (ORT from CDN, model local), backbone
  + JS post-processing (softmax-fold, 5x5 NMS, top-K=320, bilinear descriptor
  sample, L2-norm). Returns kp+desc.
- Engine: `MapPoint.xdesc/hasX` (64-D float), `setFrameXFeat` staging (mirrors
  the depth buffer path), `tagKeyframeXFeat` (nearest-XFeat-kp within 4 px of a
  KF's mapped ORB features), `relocalizeByXFeat` (L2 mutual-NN + ratio -> PnP),
  wired into `process()` while lost, after seeded-KF reloc and before the global
  fallback. Inert unless `?xfeat=1` stages features.
- `web/main.js`: `?xfeat=1`, worker lifecycle, KF-cadence + lost-frame send;
  deterministic bench-sync path (`benchXFeatSync`) so the async worker
  participates on EVERY lost frame in unpaced replay (generous upper bound;
  on-device it runs at ~half the frame rate).

## Phase 2 — A/B (deterministic, XFeat attempted every lost frame) → SHELVE
| clip   | baseline (seeded reloc) | + XFeat | Δ |
|--------|------------------------:|--------:|---|
| reloc  | 841 lost | 835 | -6 (noise) |
| sphere | 25 lost  | 25  | 0 |
| pan    | 13 lost  | 13  | 0 |
| orbit  | 0 lost   | 0   | 0 (inert; regression check) |

No measurable benefit on any clip, even at the every-frame upper bound.

## Why (honest)
1. **Seeded-KF reloc already recovers the recoverable cases.** It runs before
   XFeat and re-acquires geometrically (gyro-ranked KF poses) wherever the view
   overlaps the map — leaving XFeat almost no window.
2. **The remaining losses are a COVERAGE problem, not an appearance one.** The
   lost frames look at scenery never mapped (reloc clip wanders the room; sphere
   orbits past the mapped floor patch). No relocalizer — XFeat included — can
   localize against a map that lacks the current view. XFeat's distinctiveness
   edge only matters in a narrow moderate-baseline-with-overlap band that seeded
   reloc already owns.

## Disposition
Kept behind `?xfeat=1`, **off by default, inert, zero regression** (orbit 0 lost,
native suites pass). Revisit only if the reloc bottleneck ever shifts to
appearance — e.g. after a coverage/mapping improvement makes more return-views
mappable, or for wide-baseline loop closure. The real lever for these clips is
map COVERAGE (keep more of the explored scene mappable), not a better descriptor.
