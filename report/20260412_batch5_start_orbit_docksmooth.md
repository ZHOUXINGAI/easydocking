# Batch Check (Start Timing / Orbit Proximity / Docking Smoothness)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- start timing (`first_non_idle_t_sec`): mean `82.90s` (range `76.12s ~ 107.70s`)
- phase duration mean: `APPROACH 18.00s`, `TRACKING 3.17s`, `DOCKING 5.46s`
- carrier-to-orbit gap in APPROACH (`carrier_radius - mini_orbit_radius`): mean `5.22m`, min sample mean `4.65m`
- docking lateral smoothness: mean sign flips `2.0`, mean max |lat| `2.02m`
- ahead metric: mean `ahead_ratio_positive_pre_completed = 0.910`, mean `ahead_min_m = -1.051` (worst `-1.779`)

| run_id | final_pass | first_non_idle_t_sec | approach_sec | tracking_sec | docking_sec | ahead_ratio_positive_pre_completed | ahead_min_m | approach_carrier_orbit_gap_mean_m | docking_lateral_sign_flip_count | docking_lateral_abs_max_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_141651_px4_sih | 1 | 107.70 | 17.50 | 3.20 | 5.00 | 0.914 | -0.734 | 5.36 | 1 | 1.968 |
| 20260412_142010_px4_sih | 1 | 76.16 | 17.86 | 3.20 | 4.00 | 0.924 | -0.872 | 4.65 | 1 | 1.578 |
| 20260412_142256_px4_sih | 1 | 76.84 | 18.20 | 2.98 | 6.12 | 0.936 | -0.694 | 5.54 | 3 | 2.054 |
| 20260412_142546_px4_sih | 1 | 76.12 | 18.34 | 3.36 | 6.20 | 0.896 | -1.779 | 5.00 | 3 | 2.316 |
| 20260412_142833_px4_sih | 1 | 77.70 | 18.10 | 3.10 | 6.00 | 0.881 | -1.175 | 5.53 | 2 | 2.172 |
