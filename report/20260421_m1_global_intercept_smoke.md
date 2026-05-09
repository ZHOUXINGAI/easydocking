# 2026-04-21 M1 Global Intercept Smoke

## Scope
- Milestone: `M1`
- Change: controller-side `APPROACH` now uses mini orbit plan to select a same-direction far-field intercept target
- Validation mode: `PX4 SIH`, `prehold`

## Config
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=prehold`
- `global_intercept_route_min_forward_cos=0.45`
- `global_intercept_alignment_weight=10.0`

## Latest smoke runs

| run | classification | start_t_s | time_to_tracking_s | post_start_path_m | docking_path_m | hold_lat_0p2_zband_s | note |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| `20260421_135441_px4_sih` | `geometry-fail` | 79.68 | 8.26 | 70.04 | 47.18 | 0.66 | shortest path of this batch |
| `20260421_135804_px4_sih` | `geometry-fail` | 75.30 | 7.90 | 111.70 | 64.53 | 0.80 | still shorter than old heuristic baseline |
| `20260421_140034_px4_sih` | `start-window-fail` | n/a | n/a | n/a | n/a | 0.00 | did not leave `IDLE` |
| `20260421_142316_px4_sih` | `start-window-fail` | n/a | n/a | n/a | n/a | 0.00 | global-only gate missed release when no valid orbit intercept existed after arm |
| `20260421_142637_px4_sih` | `geometry-fail` | 61.04 | 7.80 | 248.46 | 186.89 | 0.00 | release recovered after adding local predictor fallback behind the global gate |

## Comparison vs earlier baseline
- Reference run: `20260421_125940_px4_sih`
- Baseline `post_start_path_length_m = 127.60`
- Baseline `docking_path_length_m = 71.39`
- New best (`20260421_135441_px4_sih`) reduced these to:
  - `post_start_path_length_m = 70.04`
  - `docking_path_length_m = 47.18`

## What improved
- controller command after `START` stays same-direction in the tested success runs
- far-field carrier path is visibly shorter and closer to the intended tangent intercept shape
- `TRACKING`/`DOCKING` entry happens earlier after `START`
- `wait_for_docking_window.py` now evaluates the same orbit intercept candidate used by the controller before falling back to the local predictor
- prehold smoke no longer gets stuck in `IDLE` when the global gate has no valid candidate after orbit arm/completion

## What is still open
- actual vehicle velocity still shows a tiny brief opposite-sign transient near handoff in some runs, even when controller command is same-direction
- the recovered release in `20260421_142637_px4_sih` produces an overlong path, so M1 path-shaping is still not stable enough
- terminal completion is still the main blocker: these runs reached good geometry/hold, but ended in `geometry-fail` instead of `FINAL_PASS`

## Next step
- keep the global-first release gate, but tighten the post-start intercept path so the fallback case does not explode to `248 m` post-start path
- then move back to terminal completion / M2 front-consistency
