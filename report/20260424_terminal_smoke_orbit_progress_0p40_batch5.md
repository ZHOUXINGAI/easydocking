# 2026-04-24 Terminal Smoke `min_orbit_progress_ratio=0.40` Batch (5 runs)

## Goal
- Stay on the M0/M1 mainline and test one focused release-family change only:
  - delay `terminal_smoke` a bit more within the first usable orbit pass
  - do not touch terminal controller behavior
- Candidate change:
  - `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
  - keep the new carrier prehold gate enabled

## Config
- `START_RVIZ=false`
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_CARRIER_PREHOLD_REQUIRED=true`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`

## Runs

| run | classification | start_t_sec | accepted_rel_z_m | accepted_phase_err_deg | accepted_pred_lat_m | accepted_pred_score | post_start_path_length_m | docking_path_length_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `20260424_210449_px4_sih` | `final-pass` | `33.56` | `3.180` | `92.4` | `-2.180` | `0.730` | `255.437` | `187.729` |
| `20260424_210655_px4_sih` | `final-pass` | `35.14` | `3.830` | `92.5` | `-1.870` | `0.690` | `180.333` | `115.379` |
| `20260424_210911_px4_sih` | `final-pass` | `33.80` | `3.690` | `92.3` | `-1.440` | `0.650` | `146.554` | `72.180` |
| `20260424_211112_px4_sih` | `geometry-fail` | `22.86` | `2.750` | `84.7` | `-4.970` | `0.800` | `1786.613` | `` |
| `20260424_211511_px4_sih` | `final-pass` | `25.68` | `2.730` | `85.4` | `-4.450` | `0.760` | `132.936` | `80.494` |

## Result
- `final-pass = 4/5`
- `geometry-fail = 1/5`
- all `5/5` runs are still prehold-valid:
  - `start_prehold_ready=1`
  - accepted-window `prehold_ok=1`

## Comparison to current baseline
- current clean-prehold baseline (`min_orbit_progress_ratio=0.25`) was:
  - `final-pass = 1/5`
  - `geometry-fail = 4/5`
- this probe (`0.40`) improves to:
  - `final-pass = 4/5`
  - `geometry-fail = 1/5`

## Main interpretation
- This is the first change after prehold cleanup that clearly improves the mainline smoke gate.
- The better family is still visible in the accepted-window geometry:
  - good runs usually start around `33.5–35.1s`
  - good runs show phase near `92°`
  - good runs show tighter accepted `pred_lat ≈ -1.4 to -2.2`
- The remaining fail is still the old early family:
  - `start_t=22.86s`
  - `phase_err_deg=84.7`
  - `pred_lat=-4.97`
  - `post_start_path_length_m=1786.613`

## Practical conclusion
- Promoting `terminal_smoke` default `min_orbit_progress_ratio` from `0.25 -> 0.40` is justified.
- This does not solve release-family selection completely.
- The next mainline step is to eliminate the remaining early-family outlier without regressing the new `4/5` baseline.
