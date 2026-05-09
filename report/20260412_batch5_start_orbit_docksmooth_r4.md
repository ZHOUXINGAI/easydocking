# Batch Check (Start Timing / Orbit Proximity / Docking Smoothness) — r4

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- Changes in this round: keep energy early-release + outside margin 6.0, but retune ahead constraint from aggressive `9.0 / 1.12 / 5.8` to `8.5 / 1.06 / 5.3` (min-margin / recover-gain / max-boost).
- final-pass: `5/5`
- start timing (`first_non_idle_t_sec`): mean `74.97s` (range `74.20s ~ 75.50s`)
- phase duration mean: `APPROACH 18.17s`, `TRACKING 3.04s`, `DOCKING 6.29s`
- carrier-to-orbit gap in APPROACH (`carrier_radius - mini_orbit_radius`): mean `5.01m`, min sample mean `2.87m`
- docking lateral smoothness: mean sign flips `1.6`, mean max |lat| `1.85m`
- ahead metric: mean `ahead_ratio_positive_pre_completed = 0.918`, mean `ahead_min_m = -0.786` (worst `-1.171`)

| run_id | final_pass | first_non_idle_t_sec | approach_sec | tracking_sec | docking_sec | ahead_ratio_positive_pre_completed | ahead_min_m | approach_carrier_orbit_gap_mean_m | docking_lateral_sign_flip_count | docking_lateral_abs_max_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_184446_px4_sih | 1 | 75.50 | 17.80 | 3.00 | 10.20 | 0.876 | -0.754 | 5.23 | 3 | 1.640 |
| 20260412_184739_px4_sih | 1 | 75.20 | 18.64 | 2.90 | 4.00 | 0.971 | -0.932 | 5.57 | 1 | 1.491 |
| 20260412_185025_px4_sih | 1 | 75.46 | 17.90 | 1.56 | 4.84 | 0.873 | -1.171 | 4.31 | 0 | 1.484 |
| 20260412_185310_px4_sih | 1 | 74.20 | 18.14 | 3.90 | 6.80 | 0.939 | -0.401 | 5.26 | 2 | 2.307 |
| 20260412_185559_px4_sih | 1 | 74.50 | 18.36 | 3.84 | 5.60 | 0.930 | -0.670 | 4.67 | 2 | 2.323 |
