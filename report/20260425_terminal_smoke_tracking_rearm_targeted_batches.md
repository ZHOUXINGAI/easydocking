# 2026-04-25 targeted early-`TRACKING` re-arm validation

## Goal

After the fresh baseline `5/5` non-regression batch, the next question was narrower:

- can we run the new controller-side early-`TRACKING` re-arm guard in **historically bad release families**
- and actually observe `TRACKING -> APPROACH` re-arm

Two targeted probe families were chosen because they previously produced the residual bad family:

- `combo-reject` family:
  - historical fails: `20260424_220319`, `20260424_221224`
- `pred-margin` family:
  - historical fail: `20260424_231539`

## Common config

- `START_RVIZ=false`
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`

Guard thresholds remained unchanged:

- Branch A (`5s`): `min_term < 3.0 && min_lat_abs > 1.6 && max_along > 1.0`
- Branch B (`8s`): `min_term > 2.5 && min_lat_abs > 2.2 && max_along < 0.2`

## Batch A — combo-reject family + guard

Additional release-side settings:

- `AUTO_START_REAR_ENTRY_SMOKE_EARLY_REJECT_TCA_MAX_SEC=4.45`
- `AUTO_START_REAR_ENTRY_SMOKE_EARLY_REJECT_SPEED_MIN_MPS=13.5`
- `AUTO_START_REAR_ENTRY_SMOKE_EARLY_REJECT_PRED_LAT_ABS_MIN_M=4.8`

### Runs

| run | classification | start_t_sec | post_start_path_length_m | guard_triggered | note |
|---|---:|---:|---:|---:|---|
| `20260425_002342_px4_sih` | `final-pass` | `36.80` | `372.975` | `0` | pass, but long family |
| `20260425_002617_px4_sih` | `start-window-fail` | `` | `` | `0` | never left `IDLE` |
| `20260425_003015_px4_sih` | `geometry-fail` | `36.34` | `1156.623` | `0` | failed after `DOCKING -> TRACKING` retry |
| `20260425_003413_px4_sih` | `geometry-fail` | `33.24` | `1616.103` | `0` | stayed in `TRACKING` long-tail family |
| `20260425_003810_px4_sih` | `final-pass` | `37.10` | `197.451` | `0` | acceptable pass |

### Result

- `final-pass = 2/5`
- `start-window-fail = 1/5`
- `geometry-fail = 2/5`
- observed `TRACKING -> APPROACH` re-arm count: `0/5`

### Interpretation

- this targeted family is **not a good validation carrier**
- the release-side combo gate itself regresses too much (`2/5`, plus one `start-window-fail`)
- despite that, the new controller guard still did **not** fire in any of these runs

## Batch B — pred-margin family + guard

Additional release-side settings:

- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.77`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.99`

### Runs

| run | classification | start_t_sec | post_start_path_length_m | guard_triggered | note |
|---|---:|---:|---:|---:|---|
| `20260425_004028_px4_sih` | `final-pass` | `33.50` | `343.185` | `0` | long pass family |
| `20260425_004254_px4_sih` | `final-pass` | `61.24` | `140.686` | `0` | late but clean |
| `20260425_004527_px4_sih` | `final-pass` | `67.34` | `155.994` | `0` | late but clean |
| `20260425_004757_px4_sih` | `final-pass` | `36.54` | `134.704` | `0` | short family |
| `20260425_005005_px4_sih` | `final-pass` | `36.90` | `282.962` | `0` | long pass family |

### Result

- `final-pass = 5/5`
- observed `TRACKING -> APPROACH` re-arm count: `0/5`

### Interpretation

- this family is a good **non-regression** stress case
- but it still does **not** demonstrate that the guard is doing useful work
- all five runs passed without ever hitting `TRACKING -> APPROACH`

## Main conclusion

This targeted validation answers one important question:

- the current early-`TRACKING` re-arm guard is still **safe enough** in the tested targeted families
- but we still do **not** have evidence that it is the cause of any recovery

More precisely:

- `combo-reject + guard` is not promotable because the release-side probe itself is too unstable
- `pred-margin + guard` is healthy (`5/5`), but does not exercise the guard path
- across all `10` targeted runs, observed `TRACKING -> APPROACH` re-arm count is:
  - `0 / 10`

## Practical next step

Do **not** promote the guard to default.

The next efficient move should be one of:

1. widen / retarget the online `TRACKING` discriminator so it can actually fire on currently observed long-tail runs, or
2. build a more replay-like targeted harness that reliably reproduces the old `220319 / 221224 / 231539` family before tuning the guard further
