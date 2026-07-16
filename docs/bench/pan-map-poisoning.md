# Pan clip — map self-poisoning + the scaffold/priority-cull fix

Source: `fail to track during simple pan movement -webslam-1784206768084.wsrec`
(local `web/pan.wsrec`) — 924 frames, 22.9 s, 40 fps, gyro peak ~21°/s. A slow
pan across a well-textured room (sofa, shelves, desk), past a backlit window.

## Failure mechanism (baseline: 162/924 frames lost)

1. At 20°/s the 4° rotation KF trigger fires every ~8 frames → 50 KFs by f450,
   each triangulating hundreds of points against the PREVIOUS keyframe — with
   near-zero baseline (a pan is near-pure rotation). Depth is unobservable;
   garbage-depth points flood the map.
2. The map hits the 2200 cap at exactly f300 — where losses begin. The cull
   kept the most-recently-SEEN points, i.e. the fresh garbage, and EVICTED the
   good old geometry. When the pan returned to the sofa (mapped at f150), its
   points were gone → 67 straight lost frames.
3. Environmental: the f440–590 sweep is genuinely motion-blurred (Laplacian
   sharpness 162–220 vs ~500 elsewhere) + backlit window exposure swings.

## What did NOT work: hard triangulation gates

Requiring 1° parallax before accepting a point made the pan UNTRACKABLE
(162 → 669 lost): low-parallax points are a legitimate **2D tracking
scaffold** — their projections are valid near the creation viewpoint, which is
exactly where a pan needs them. The map just must not let them displace real
geometry.

## Shipped fix (three parts, map.h/map.cpp)

1. **Reprojection gate** (`triMaxReprojErrPx = 3`, both views + cheirality in
   both): hard-rejects descriptor mismatches. These were pure poison.
2. **Provisional scaffold points** (`triMinParallaxDeg = 1`): low-parallax
   triangulations enter the map but are marked `!solid`.
3. **Priority cull**: keep-order = (recently-seen within 40 frames — the
   working set, never culled) > (recency + 300-frame bonus for solid points).
   Scaffold floods can no longer evict well-triangulated geometry, yet the
   frontier survives exploration with tiny budgets (test_explore).

## Results (deterministic replay, fresh page per run)

| clip | metric | before | after |
|---|---|---:|---:|
| pan | lost frames | 162 | **82** (−49%) |
| pan | first loss | f325 | f525 |
| pan | median inliers | 143 | **185** |
| orbit (regression) | lost frames | 5 | **0** |
| orbit (regression) | orbit-region inlier p50 | 29 | **138** |
| sphere (regression) | lost frames | 27 | 27 (onset 74→78) |

The orbit clip improving 5→0 lost / 29→138 inliers shows the same poisoning was
active there (marginal triangulations of repetitive floor texture entering the
map); the reproj gate + priority cull fixed both clips with one mechanism.

Remaining pan losses (f525–626) are the motion-blurred backlit-window sweep —
an image-quality problem (exposure/blur), not a map-logic problem. The sphere
clip's permanent tail still needs KF-pose-seeded relocalization (see
docs/bench/sphere-orbit-failure.md).
