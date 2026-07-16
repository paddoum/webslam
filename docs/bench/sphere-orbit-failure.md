# Sphere-orbit clip — tracking failure diagnosis + reloc budget

Source: `Fail when orbit around sphere - webslam-1784196283304.wsrec` (checked in
as `web/sphere.wsrec`) — 107 frames, 5.9 s, ~18 fps capture, 60 Hz IMU. User
places the AR sphere on a wood floor and orbits it, camera pointed down.

## What the clip actually is

Not a sphere-tracking problem — the tracker sees only the **floor**: planar
(weakly-conditioned PnP), highly repetitive (plank grain → ambiguous ORB
descriptors), and the orbit rotates the plank orientation through ~90° in view.
Close to a worst-case monocular scene.

## Failure timeline (deterministic replay)

| frames | state | inliers | note |
|---|---|---|---|
| 2–50   | tracking | 60–190 | healthy |
| 52–73  | degrading | 74→17 | orbit rotates texture, matches decay |
| 74     | first LOST | 0 | 113 ms reloc spike |
| 75–81  | flicker | 8–26 | re-acquires at tiny inlier counts |
| 82–106 | permanent LOST | 0 | reloc fails 25×, never recovers |

## Root causes

1. **Reloc can't work here**: appearance-only brute-force match (ratio 0.95, one
   descriptor/point) floods RANSAC with plank-to-plank false matches on coplanar
   points → never reaches 25 inliers. It also ignores the keyframe pose bank.
2. **Compute death-spiral (fixed below)**: every lost frame ran full reloc
   (~100 ms). Capture dt spiked 43→98–128 ms at frames 75–84 — the recovery
   attempts starved the camera pipeline, widening inter-frame motion, preventing
   re-acquisition. The recovery was what prevented recovery.
3. (minor) Translation frozen while lost — mid-orbit the camera keeps moving, so
   coasted projections land off-window.

## Fix shipped: budgeted relocalization (`relocCooldownFrames = 4`)

Global reloc runs on the FIRST lost frame (brief losses still recover instantly),
then only every 4th lost frame; skipped frames run the cheap gyro-coasted
projection re-acquire that already executes every frame.

A/B on this clip (same binary determinism, tracking outcome unchanged):

| metric | before | after |
|---|---:|---:|
| lost frames | 27 | 27 (identical — reloc was failing anyway) |
| lost-frame cost | 63–117 ms EVERY frame | ~100 ms every 5th, **14–35 ms** otherwise |
| mean lost-frame cost | ~95 ms | **~35 ms** |

Orbit-clip regression check (fresh page per replay — `loadAndReplay` does NOT
reset the map, so replays on a used page are contaminated): **lost = 5, orbit
inlier p50 = 29 — bit-identical to Phase 5.** No regression.

The on-device benefit is the part replay can't show: lost frames no longer
starve the camera pipeline, so real inter-frame motion stays small and
projection re-acquire gets a fair chance.

## Still open (next steps for this scene)

- **KF-pose-seeded reloc** — projection-match from each diverse keyframe pose;
  geometry disambiguates repetitive texture where appearance can't.
- Tighten reloc ratio test 0.95→0.8; stop triangulating new points while
  tracking is weak (frames 52–73 polluted the map right before the loss).
- Longer term: homography-aware planar handling; scale pyramid for ORB.
