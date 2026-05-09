# `terminal_smoke` combo-reject probe (`2026-04-24`)

## Goal

Try a **minimal smoke-only reject gate** on top of the current `orbit_progress=0.40` baseline:

- reject when all three are true:
  - `accepted_window_tca_sec <= 4.45`
  - `accepted_window_speed_xy_mps >= 13.5`
  - `abs(accepted_window_pred_lat_m) >= 4.8`

This was intended to block the remaining **early/aggressive release family** without touching terminal control.

## Probe setup

- `START_RVIZ=false`
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- baseline kept:
  - `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
  - carrier prehold gate enabled
- probe-only gate:
  - `AUTO_START_REAR_ENTRY_SMOKE_EARLY_REJECT_TCA_MAX_SEC=4.45`
  - `AUTO_START_REAR_ENTRY_SMOKE_EARLY_REJECT_SPEED_MIN_MPS=13.5`
  - `AUTO_START_REAR_ENTRY_SMOKE_EARLY_REJECT_PRED_LAT_ABS_MIN_M=4.8`

## Runs

| run | classification | start_t | post_start_path_length_m | note |
| --- | --- | ---: | ---: | --- |
| `20260424_220319_px4_sih` | `geometry-fail` | `32.94` | `1615.221` | accepted later, but still long-tail fail |
| `20260424_220717_px4_sih` | `final-pass` | `63.20` | `141.970` | pass, but start much later than desired |
| `20260424_220951_px4_sih` | `final-pass` | `36.50` | `413.296` | pass with longish family |
| `20260424_221224_px4_sih` | `geometry-fail` | `25.60` | `1733.853` | early family still leaked through |
| `20260424_221627_px4_sih` | `final-pass` | `25.18` | `158.227` | acceptable early pass |

## Result

- `final-pass = 3/5`
- `geometry-fail = 2/5`
- `prehold-valid = 5/5`

Compared to the current promoted baseline:

- baseline `orbit_progress=0.40` batch: `4/5 final-pass`
- this combo-reject probe: `3/5 final-pass`

## Interpretation

- The reject hook **does** fire on obviously bad early checks (`smoke_early_reject=1` appears in `start_command.log`).
- But the accepted-family outcome did **not** improve enough:
  - one bad early family still slipped through (`20260424_221224_px4_sih`)
  - another run failed later despite a later/cleaner accepted window (`20260424_220319_px4_sih`)
- So this probe is **not good enough to promote** as a new default.

## Decision

- Keep the combo-reject hook as **neutral infrastructure only**.
- Revert `terminal_smoke` defaults back to the proven mainline baseline:
  - `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
  - no default combo reject thresholds
- Next discriminator pass must preserve the `4/5` baseline first, then target the single remaining early-family outlier.
