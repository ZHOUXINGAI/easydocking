# Batch Check (Start Timing / Orbit Proximity / Docking Smoothness) — r3

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- Changes in this round: energy early-release override enabled + carrier outside margin to 6.0 + stronger terminal lateral damping/rate-limit.
- final-pass: `5/5`
- start timing (`first_non_idle_t_sec`): mean `72.32s` (range `63.80s ~ 79.14s`)
- phase duration mean: `APPROACH 18.29s`, `TRACKING 3.44s`, `DOCKING 6.32s`
- carrier-to-orbit gap in APPROACH (`carrier_radius - mini_orbit_radius`): mean `5.11m`, min sample mean `3.33m`
- docking lateral smoothness: mean sign flips `1.6`, mean max |lat| `1.75m`
- ahead metric: mean `ahead_ratio_positive_pre_completed = 0.811`, mean `ahead_min_m = -1.405` (worst `-2.385`)

| run_id | final_pass | first_non_idle_t_sec | approach_sec | tracking_sec | docking_sec | ahead_ratio_positive_pre_completed | ahead_min_m | approach_carrier_orbit_gap_mean_m | docking_lateral_sign_flip_count | docking_lateral_abs_max_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_180509_px4_sih | 1 | 64.30 | 18.30 | 3.76 | 6.70 | 0.661 | -2.385 | 5.13 | 2 | 1.640 |
| 20260412_180744_px4_sih | 1 | 75.40 | 18.50 | 3.40 | 5.90 | 0.865 | -1.087 | 5.24 | 2 | 1.845 |
| 20260412_181033_px4_sih | 1 | 78.94 | 18.16 | 2.64 | 3.76 | 0.957 | -0.884 | 5.16 | 1 | 1.337 |
| 20260412_181323_px4_sih | 1 | 79.14 | 18.00 | 3.44 | 9.66 | 0.789 | -0.836 | 5.17 | 1 | 2.136 |
| 20260412_181621_px4_sih | 1 | 63.80 | 18.50 | 3.94 | 5.56 | 0.782 | -1.834 | 4.84 | 2 | 1.803 |
