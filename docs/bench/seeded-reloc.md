# Relocalization fix — KF quality floor + keyframe-pose-seeded reloc

Trigger clip: `better tracking but hard time relocating webslam-1784214635279.wsrec`
(local `web/reloc.wsrec`) — 43 s, 1224 frames. Scene: striped rug + table.
Fast rotation (30–60°/s) degrades tracking around f157–289; then LOST for the
remaining **33 seconds** while the user actively tries to relocalize (including
looking straight at the mapped rug at ~f500). Appearance-only reloc: 0 recoveries.

## Root causes (two, compounding)

1. **The map was corrupted BEFORE the loss.** During the degradation window
   (inliers 8–20), the "weak tracking → insert keyframe" rule fired repeatedly:
   11 KFs (31→42) stamped with unreliable poses, triangulating garbage.
   Once lost, no relocalizer can match a corrupt recent map. Verified by
   ablation: seeded reloc alone (without the floor) recovered NOTHING.
2. **Appearance-only reloc can't do repetitive texture.** The striped rug is
   the wood-plank problem again: brute-force descriptor matching drowns in
   wrong-stripe candidates and never reaches 25 RANSAC-consistent inliers.

## Shipped

- **KF-insert quality floor** (`kfInsertMinInliers = 20`): never insert a
  keyframe below 20 inliers — the pose is too unreliable to extend the map.
  The weak-tracking densify trigger still fires in [20, 25).
- **Keyframe-pose-seeded relocalization** (`relocalizeByKeyframes`): while
  lost, hypothesize "I'm near keyframe k" and run projection tracking from
  k's stored pose with a wide (70 px) window — geometry constrains matching,
  which disambiguates repetitive texture. 3 seeds/frame: best-aligned with
  the gyro-coasted heading first, round-robin through the ranked bank so all
  KFs get tried across consecutive lost frames. Accepts at relocMinInliers
  (25) to avoid false relocalization. Refactor: `trackFromPose` now backs
  both normal tracking and seeded reloc.
- Appearance reloc stays as the throttled last resort (every 4th lost frame).

## Results (deterministic replays, fresh page per run)

| clip | before | after |
|---|---|---|
| reloc (43 s) | 1007 lost, **permanent** from f289 | 841 lost, **recovers repeatedly, ends tracking (301 inliers)** |
| pan | 82 lost | **13** (blurred-sweep frames re-acquire next frame) |
| orbit | 0 lost | 0 (unchanged, inl p50 138) |
| sphere | 27 lost | 25 |

All 16 native suites pass.

## Notes / remaining headroom

- Lost-frame cost rose (3 seeded attempts/frame; each failed seed with ≥25
  candidate matches runs a 60-iter guess-PnP). If on-device lost frames feel
  heavy, cut seed PnP iters (~15) or seeds to 2.
- The sphere clip still ends lost: map saturated with repetitive floor from
  a decayed-quality window; the floor limits future damage but that recording
  predates it.
