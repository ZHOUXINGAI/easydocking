# 2026-04-26 terminal_smoke release reject `0.74 / 2.00` batch-5

## Goal

Retest the release side with a narrower but better targeted smoke reject pair after instrumentation showed:

- the early risky family still leaked through accepted windows around
  `pred_score≈0.78` and `diag_margin≈1.99`
- the `TRACKING_ENTRY_REARM_GUARD` was already firing online, so that fail family was not caused by a dead controller path

The intent of this probe was simple:

- block the risky early release family
- keep healthy later windows available
- avoid another speculative controller change

## Config

Common config:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.74`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=2.00`
- `START_RVIZ=false`

Runs:

- `20260426_003737_px4_sih`
- `20260426_003950_px4_sih`
- `20260426_004230_px4_sih`
- `20260426_004522_px4_sih`
- `20260426_004751_px4_sih`

## Result

- `final-pass = 5/5`
- no `start-window-fail`
- no `geometry-fail`
- no run fell back into the old `start_t ≈ 21–27s` early family

Per-run:

| run | classification | start_t_sec | phase_err_deg | pred_lat_m | pred_score | diag_margin_mps | post_start_path_length_m | docking_path_length_m |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `20260426_003737_px4_sih` | `final-pass` | `32.46` | `93.6` | `-2.35` | `0.76` | `3.15` | `289.693` | `223.124` |
| `20260426_003950_px4_sih` | `final-pass` | `67.64` | `91.4` | `-3.15` | `0.82` | `2.95` | `146.750` | `81.752` |
| `20260426_004230_px4_sih` | `final-pass` | `63.24` | `90.3` | `-2.01` | `0.67` | `3.24` | `327.404` | `261.182` |
| `20260426_004522_px4_sih` | `final-pass` | `66.30` | `90.7` | `-1.71` | `0.65` | `2.95` | `159.987` | `136.613` |
| `20260426_004751_px4_sih` | `final-pass` | `34.54` | `91.1` | `-2.58` | `0.74` | `2.68` | `234.564` | `163.528` |

Batch means:

- `start_t_sec ≈ 52.84`
- `accepted_phase_err_deg ≈ 91.42`
- `accepted_pred_lat_m ≈ -2.36`
- `accepted_pred_score ≈ 0.73`
- `accepted_diag_margin_mps ≈ 2.99`
- `post_start_path_length_m ≈ 231.68`
- `docking_path_length_m ≈ 173.24`

## Comparison against the risky family

The known risky early fail (`20260426_002707_px4_sih`) looked like:

- `start_t = 25.70s`
- `phase_err = 87.9°`
- `pred_lat = -4.13m`
- `pred_score = 0.78`
- `diag_margin = 1.99m/s`

This new batch stayed away from that region:

- `start_t = 32.46–67.64s`
- `phase_err = 90.3–93.6°`
- `pred_lat = -3.15 .. -1.71m`
- `diag_margin = 2.68–3.24m/s`

So the probe did the intended thing:

- it suppressed the early risky family
- it did not block later healthy windows, including a clean later run with `pred_score=0.82` because its `diag_margin` stayed high (`2.95`)

## Controller-side note

The new instrumentation shows all `5/5` passes ended with:

- `controller_tracking_rearm_guard_enabled = 1`
- `controller_tracking_rearm_guard_used = 1`
- `controller_tracking_rearm_last_trigger_code = 2`

That matters because it means:

- the controller recovery path is active online
- this batch quality is not coming from a hidden dead branch
- the release-side reject and the existing controller recovery now work together cleanly enough on this sample

## Decision

Promote these values into the `terminal_smoke` defaults in:

- `scripts/run_px4_sih_docking_experiment.sh`

New defaults:

- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.74`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=2.00`

## Next step

Do not churn the release thresholds again immediately.

The next bottleneck is the remaining terminal quality tail inside successful release families:

- repeated `DOCKING` entries
- long `docking_path_length_m` runs such as `20260426_004230_px4_sih`
- late `DOCKING` geometry quality, not early window selection
