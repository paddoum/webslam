# Map point budget sweep — should maxMapPoints exceed 2200?

Question (user): "should we accept more than 2200 keypoints?" — i.e. raise
`maxMapPoints` in `src/map.h`. (Not per-frame keypoints — that's the feature
slider / `MAX_FEATURES`.) Both the pan and orbit clips saturate the 2200 cap,
so more budget plausibly = more retained coverage = better re-acquisition.

Method: `engine.setMaxMapPoints()` (new Embind setter, survives map resets) +
`?mapPts=N` URL param; deterministic `?bench=1` replays, one fresh page load
per run. Build `b41dcfb+` (scaffold/priority-cull already in).

## Results

| clip | metric | 2200 (default) | 3500 | 5000 |
|---|---|---:|---:|---:|
| pan    | lost frames   | 82  | **38**  | 167 |
| pan    | inlier p50    | 185 | 193     | 151 |
| pan    | total p95 ms  | ~30 | 33.4    | 185 |
| orbit  | lost frames   | **0** | **151 — permanently lost from f647** | not run¹ |
| orbit  | orbit-region inlier p50 | 138 | 0 | — |
| sphere | lost frames   | 27  | 27 (map peaks at 652 pts — caps never reached) | not run¹ |

¹ decision already forced by orbit@3500; sphere never exceeds 652 points so
all budgets are identical there.

## Reading — the budget is a regularizer, not just capacity

- **Distinctive textured scene (pan clip room): more budget genuinely helps.**
  82→38 lost at 3500 — richer retained geometry means projection re-acquires
  instantly between motion-blurred frames. 5000 collapses it again (167):
  reloc goes O(N·features) (~230 ms/attempt) and matching ambiguity rises
  (inliers 185→151).
- **Repetitive scene (floor orbit): more budget is poison.** 0→151 lost at
  3500. The extra ~350 retained points are exactly the stale repetitive-floor
  points the 2200 cap used to flush; they feed wrong associations to
  projection matching in the fragile orbit region and tracking never returns
  (reloc can't recover on that floor — known). The forced cull was silently
  acting as a map-quality regularizer.
- Timing scales as predicted: mapProc p50 ~10 ms at 3500 (dev machine), reloc
  attempts 160→230 ms as N grows. Phone is 2–4× slower — pan@3500's 33.4 ms
  p95 already sits on the frame budget.

## Verdict

**Keep `maxMapPoints = 2200`.** A bigger global budget trades the repetitive-
scene failure mode for the textured-scene one; 2200 is the better default of
the two. The `?mapPts=` override stays for experiments.

The real headroom is smarter, not bigger: the pan@3500 win shows retained
coverage pays off when association stays clean. Candidate follow-ups —
match gating per keyframe region (geometry-scoped candidates instead of
whole-map), descriptor-distinctiveness weighting in the cull, KF-pose-seeded
reloc (docs/bench/sphere-orbit-failure.md).
