# Batch Check (Start Timing / Orbit Proximity / Docking Smoothness) — r2

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- start timing (`first_non_idle_t_sec`): mean `83.16s` (range `76.34s ~ 103.14s`)
- phase duration mean: `APPROACH 18.20s`, `TRACKING 2.96s`, `DOCKING 9.09s`
- carrier-to-orbit gap in APPROACH (`carrier_radius - mini_orbit_radius`): mean `5.34m`, min sample mean `2.76m`
- docking lateral smoothness: mean sign flips `1.4`, mean max |lat| `1.68m`
- ahead metric: mean `ahead_ratio_positive_pre_completed = 0.907`, mean `ahead_min_m = -0.423` (worst `-0.937`)

| run_id | final_pass | first_non_idle_t_sec | approach_sec | tracking_sec | docking_sec | ahead_ratio_positive_pre_completed | ahead_min_m | approach_carrier_orbit_gap_mean_m | docking_lateral_sign_flip_count | docking_lateral_abs_max_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_150129_px4_sih | 1 | 103.14 | 18.30 | 3.20 | 19.44 | 0.858 | -0.937 | 5.47 | 3 | 2.535 |
| 20260412_150454_px4_sih | 1 | 77.86 | 17.80 | 2.76 | 8.90 | 0.878 | -0.754 | 5.77 | 2 | 1.466 |
| 20260412_150748_px4_sih | 1 | 79.44 | 18.06 | 3.00 | 9.44 | 0.908 | -0.658 | 5.08 | 1 | 1.446 |
| 20260412_151046_px4_sih | 1 | 79.04 | 18.26 | 3.86 | 7.24 | 0.889 | -0.662 | 5.53 | 1 | 2.144 |
| 20260412_151341_px4_sih | 1 | 76.34 | 18.60 | 2.00 | 0.44 | 1.000 | 0.895 | 4.86 | 0 | 0.792 |
