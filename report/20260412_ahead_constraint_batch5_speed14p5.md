# Ahead Hard-Constraint Batch (5 runs, CARRIER_TRACKING_SPEED_LIMIT=14.5)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `4/5`
- mean `ahead_ratio_positive_pre_completed`: `0.824`
- mean `ahead_min_m`: `-1.950` (worst `-3.865`)
- mean `ahead_sign_flip_count`: `2.60`

| run_id | classification | final_pass | first_non_idle_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | ahead_sign_flip_count |
|---|---:|---:|---:|---:|---:|---:|
| 20260412_005327_px4_sih | final-pass | 1 | 78.94 | 0.856 | -1.414 | 2 |
| 20260412_005623_px4_sih | final-pass | 1 | 77.54 | 0.808 | -1.754 | 3 |
| 20260412_005918_px4_sih | final-pass | 1 | 76.32 | 0.886 | -1.133 | 3 |
| 20260412_010208_px4_sih | final-pass | 1 | 103.50 | 0.896 | -1.584 | 2 |
| 20260412_010522_px4_sih | geometry-fail | 0 | 77.84 | 0.673 | -3.865 | 3 |
