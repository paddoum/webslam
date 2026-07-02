# Baseline — orbit clip (post-Phase-1)

Source: `webslam-1782990557225.wsrec` — 798 frames, 1384 IMU samples, 23.1 s,
34.6 fps capture, 60 Hz IMU, Rci pre-calibrated (VI + gyro-aid active on replay).
Method: loaded live site with `?bench=1`, replayed unpaced through the WASM engine
(SIMD/LTO build), `engine.stageTimes()` collected per frame. Dev machine, Chrome.

## Per-stage frame time (ms)

| stage    | p50  | p95  | max  | mean |
|----------|-----:|-----:|-----:|-----:|
| grayscale| 0.0  | 0.1  | 0.4  | 0.03 |
| detect   | 0.7  | 3.3  | 4.4  | 1.24 |
| orb      | 3.5  | 13.1 | 32.3 | 5.57 |
| mapProc  | 5.2  | 14.8 | 46.1 | 6.86 |
| **total**| **9.3** | **29.5** | **61.8** | **13.75** |

## Reading

- **grayscale is free** — SIMD (Phase 1) took it to ~0.0 ms. Done; ignore henceforth.
- **The median frame (9.3 ms) is fine (~108 fps headroom). The problem is variance:**
  p95 29.5 ms (~34 fps) and max 61.8 ms — occasional frames blow the 33 ms budget and
  drop. Optimizing the mean matters less than crushing the tail.
- **Two things drive the tail:**
  1. **orb** scales with feature count. Frames 0–~309 (before the adaptive FAST
     threshold settled + textured scene) run orb 8–32 ms, total 20–60 ms. After ~310
     it drops to orb 2–4 ms, total 6–10 ms. → **Phase 3 (cap/grid-distribute features)
     attacks both the mean and this early-heavy period directly.**
  2. **mapProc spikes to 40–46 ms** exactly on (a) KF-insert-with-BA and (b) global
     relocalization (300 RANSAC iters). Frames 638, 653, 670, 682 = 43.6 / 39.9 / 43.6 /
     46.1 ms. → **Phase 5 (adaptive RANSAC for reloc, budget/async BA).**

## Tracking quality (the orbit-loss region)

- 792/798 frames tracked; only **3 hard-lost frames (0.4%)**: 638, 653, 670 — each
  recovered within 1 frame via relocalization. Median inliers 138 (healthy).
- **But the orbit stresses it hard:** through frames ~640–730 inliers hover at 10–40
  (barely above the `relocMinInliers=25` / PnP floors), with the map saturated at the
  caps (2200 pts, 80 KFs). Inliers recover to 60–124 by the end. So M14 is holding on
  this clip but on the edge — this is the regime a robustness pass (denser/better
  descriptors, motion-only BA) should target, and any speed change here must NOT
  regress the inlier floor.

## Baseline to beat (the number that matters)

**total p95 = 29.5 ms**, **max = 61.8 ms**. Regenerate anytime:
`serve web/`, open `/?bench=1`, replay `rec.wsrec` → downloads the summary + frames CSV.
Re-run this after every optimization phase and diff the p95/max.

---

## Harness fidelity fix (deterministic replay)

While A/B-testing Phase 3, the tracking metric (lost-frame count) swung wildly
between identical runs (0, 12, 39) on the *same* binary. Cause: `step()` computed
its per-frame `dt` from **wall-clock** time, so under an unpaced bench the gyro
prior's magnitude depended on machine load → non-deterministic RANSAC/loss. Fixed:
replay now feeds the **recorded** inter-frame dt (main.js `replayFrameDt`). Tracking
is now bit-reproducible run-to-run (lost=12 every time), and the replay faithfully
reproduces the ~29 ms/frame the phone actually saw. Stage *timings* were always
stable; only the tracking outcome was affected.

Deterministic baseline (post-fix): **lost = 12 / 798 (1.5%)**, orbit-region
(640–730) inlier p50 = 19.

---

## Phase 3 result — integral-image BRIEF (shipped); grid selection (reverted)

Two changes were tried. A/B on `rec.wsrec` (baseline `web/wasm` vs candidate,
deterministic harness):

**Integral-image BRIEF smoothing — SHIPPED.** Replaces the 9-tap box average per
BRIEF endpoint (512/keypoint) with an O(1) integral-image lookup. Bit-identical
descriptors (proven: `test_orb` passes, and lost/inlier counts match baseline
*exactly*).

| metric        | baseline | integral-only | Δ |
|---------------|---------:|--------------:|---:|
| orb p95 (ms)  | 12.9     | 8.6           | **−33%** |
| total p95 (ms)| 30.5     | 24.2          | **−20%** |
| lost frames   | 12       | 12            | 0 |
| orbit inlier p50 | 19    | 19            | 0 |

**Grid-based feature selection — REVERTED.** Intended to spread features spatially
for orbit robustness, but it did *not* help here and risks the opposite: during an
orbit it can spend the feature budget on cells that no longer overlap the map,
starving inliers. Given the "don't regress orbit robustness" guardrail, it's shelved
as a separate robustness experiment (must spread only among map-overlapping cells and
be validated on its own). Kept the global top-K selection.

**Takeaway:** the p95 tail (jank) dropped 20% with zero tracking change. The `mapProc`
spikes (reloc 300-iter RANSAC + KF-insert BA, up to ~46 ms) remain the next target —
Phase 5.

---

## Phase 5 result — adaptive RANSAC + 64-bit popcount (shipped)

Target: the `mapProc` spikes (global-relocalization 300-iter RANSAC on LOST frames;
KF-insert BA). Two changes: **adaptive RANSAC termination** in `solvePnP` (stop once
the best inlier ratio makes more sampling pointless at 99% confidence — the seeded
pose guess usually clears this immediately, and the no-guess reloc path exits far
earlier when it finds a strong solution) and **64-bit popcount** in `hamming` (4
popcountll vs 8 popcount; identical result; helps the reloc brute-force match most).

A/B on `rec.wsrec` (Phase 3 = A, Phase 5 = B), deterministic harness, both reproduced:

| metric              | Phase 3 (A) | Phase 5 (B) | Δ |
|---------------------|------------:|------------:|---:|
| **total max (ms)**  | 120         | **47**      | **−61%** |
| **mapProc max (ms)**| 108.6       | **43.7**    | **−60%** |
| mapProc frames >20ms| 18          | **9**       | **−50%** |
| mapProc p95 (ms)    | 13.5        | 11.8        | −13% |
| total p95 (ms)      | 26.2        | 23.4        | −11% |
| lost frames         | 12          | **5**       | fewer |
| orbit inlier p50    | 19          | **29**      | higher |

The headline is the **worst-frame spike: 120→47 ms** — the dropped-frame stutters are
gone. Tracking also *improved* (losses 12→5, inliers up): adaptive RANSAC only
early-exits when the inlier ratio is confidently high, so it keeps the motion-consistent
guess-refined pose instead of switching to a marginally-different random hypothesis —
which stabilizes tracking frame-to-frame. On a poor guess it still runs full iterations,
so it self-adjusts. Deterministic (lost=5 both runs); all native suites incl. `test_pnp`
/`test_reloc` pass.

**Note:** adaptive RANSAC is not a pure speedup — it changes which hypothesis is
accepted (searches less). On this clip that helped; the mechanism is stabilizing by
design, but future clips should keep an eye on the lost/inlier numbers, not just timing.

### Cumulative (baseline → Phase 5)

| metric        | M14 baseline | Phase 5 | Δ |
|---------------|-------------:|--------:|---:|
| total p95 (ms)| 29.5         | 23.4    | −21% |
| total max (ms)| 61.8         | 47      | −24% |

(Remaining tail is KF-insert BA on the main thread — best addressed by **Phase 4**,
moving SLAM/BA off the main thread, rather than cutting BA quality.)
