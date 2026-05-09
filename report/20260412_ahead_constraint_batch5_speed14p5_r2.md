# Ahead Hard-Constraint Batch (5 runs, CARRIER_TRACKING_SPEED_LIMIT=14.5, round 2)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- mean `ahead_ratio_positive_pre_completed`: `0.806`
- mean `ahead_min_m`: `-1.941` (worst `-4.619`)
- mean `ahead_sign_flip_count`: `7.80`

| run_id | classification | final_pass | first_non_idle_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | ahead_sign_flip_count |
|---|---:|---:|---:|---:|---:|---:|
| 20260412_014229_px4_sih | final-pass | 1 | 78.40 | 0.946 | -0.854 | 9 |
| 20260412_014520_px4_sih | final-pass | 1 | 77.28 | 0.668 | -4.619 | 5 |
| 20260412_014810_px4_sih | final-pass | 1 | 79.10 | 0.907 | -0.961 | 5 |
| 20260412_015106_px4_sih | final-pass | 1 | 75.94 | 0.768 | -0.905 | 15 |
| 20260412_015356_px4_sih | final-pass | 1 | 103.44 | 0.743 | -2.367 | 5 |
