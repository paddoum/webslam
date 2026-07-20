# Coverage-aware map retention — tried, REVERTED

Directive: "keep more of the explored scene mappable so there's something to
relocalize against." Idea: instead of culling map points by pure recency (which
drops whole old regions as you explore), retain a spatially-diverse skeleton —
bin points into an adaptive voxel grid over the map bbox and protect each
occupied cell's best few points — so a returned-to region always has points to
relocalize against, all within the fixed 2200 budget.

Implemented in cullMapPoints (coverageGrid=6, coverageMinPerCell=6), with the
working set + AR anchor still outranking the skeleton.

## Result: no benefit, and evidence of harm → reverted

| test | pure recency (baseline) | + coverage skeleton |
|---|---|---|
| reloc clip (explores a room) | 841 lost | 840 (noise) |
| orbit clip | 0 lost, inl 138 | 0 lost, inl 139 |
| synthetic long corridor (explore-and-return) | **1 lost / 240** | **111 lost / 240** |

- **No benefit on the real clips.** Their losses are not "returned to a culled
  region" — they're looking at scenery that was NEVER mapped (the camera can't
  map while lost, so fast motion through new areas leaves them uncovered) and
  the already-mapped regions ARE still retained by recency (reloc recovers when
  it looks back). Retention wasn't the limiter.
- **Harm in exploration.** In a synthetic corridor that actually explores far
  and returns, the coverage skeleton destabilized tracking (111 vs 1 lost):
  retained old-viewpoint points become false-match candidates for projection
  tracking. This is the same effect the map-budget sweep found (more retained
  stale points hurt). Recency isn't just a memory bound — it doubles as a
  RELEVANCE filter: recent points are the ones plausibly in view now.

## The real lever (redirect)

Coverage in these clips is gated by TRACKING SURVIVING aggressive motion, not by
retention. Every failure is "tracking dropped during fast rotation / pan / blur,
so mapping stopped, so the area you're now looking at was never covered." The
productive direction is motion robustness (e.g. a fast-rotation gyro-coast
"hold" so tracking survives whip-pans and keeps mapping) — that extends real
coverage by keeping the mapper alive, without poisoning the match set.
