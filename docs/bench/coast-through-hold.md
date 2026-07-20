# Coast-through hold — motion robustness for brief tracking dropouts (M15)

The real coverage lever (see docs/bench/coverage-retention.md): every recorded
failure is "tracking dropped during aggressive motion (fast rotation / pan /
blur), so mapping stopped, so the area now in view was never covered." Keeping
tracking ALIVE through the transient keeps the mapper running — which extends
real coverage without poisoning the match set the way retention did.

## Change (map.cpp process())

On a projection-tracking dropout, instead of immediately declaring LOST and
relocalizing, COAST: dead-reckon the pose on the motion model (constant-velocity
translation + gyro rotation) for up to `holdMaxFrames` (=20, ~0.6 s at 33 fps)
and keep re-attempting projection each frame. State stays kTracking (soft), the
pose/AR-anchor stay coherent, and projection re-acquires the instant sharp
frames return. Only if the hold expires do we drop to lost + the (budgeted)
reloc chain. The key vs the old hard-lost path: it does NOT freeze translation
while the camera keeps moving — that stranded the projection prior and delayed
recovery. No keyframes are inserted while coasting (last_inliers_=0 gates it),
so a dead-reckoned pose never poisons the map. Self-correcting: if the coasted
pose is wrong, projection won't re-acquire and the hold expires into reloc.

Runtime-tunable for A/B and on-device: `?hold=N` (0 = pre-M15 drop-to-lost).

## Results (deterministic replays, hold=20 vs hold=0, same build)

| clip | hold=0 (was) | hold=20 | Δ |
|------|-------------:|--------:|---|
| new clip (65°/s whip-pan + blur) | 73 lost | **19** | -74% |
| reloc clip (fast rotation + explore) | 841 lost | **421** | -50% |
| sphere (orbit over floor)         | 25 lost | **8**  | -68% |
| pan                                | 13 lost | **10** | -23% |
| orbit (regression guardrail)       | 0 lost  | **0**  | 0 |

Recovery is genuine, not relabeling: the reloc clip ends at 162 inliers
(healthy tracking), sphere only stays lost in its final frames. The reloc-clip
halving is the coverage thesis in action — coasting through the fast-rotation
stretches kept the mapper alive, so more of the room got mapped and later views
re-acquired.

All 16 native suites pass (test_reloc updated: its forced-loss gap now exceeds
the hold window so it still exercises a genuine loss + reloc recovery).

## Note
While coasting, the app reports (soft) tracking with a dead-reckoned pose for up
to ~0.6 s. For AR this is strictly better than a hard freeze/jump — the sphere
rides the gyro through the whip-pan and re-locks smoothly. On-device is the real
test of the coast pose quality; `?hold=0` reverts instantly if needed.
