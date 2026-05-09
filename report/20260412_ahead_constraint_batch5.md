# Ahead Hard-Constraint Batch (5 runs)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `3/5`
- mean `ahead_ratio_positive_pre_completed`: `0.820`
- mean `ahead_min_m`: `-1.898` (worst `-2.437`)
- mean `ahead_sign_flip_count`: `5.80`

| run_id | classification | final_pass | first_non_idle_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | ahead_sign_flip_count |
|---|---:|---:|---:|---:|---:|---:|
| 20260411_235752_px4_sih | final-pass | 1 | 78.84 | 0.735 | -1.667 | 3 |
| 20260412_000046_px4_sih | geometry-fail | 0 | 103.24 | 0.945 | -2.187 | 6 |
| 20260412_000441_px4_sih | geometry-fail | 0 | 78.80 | 0.822 | -2.437 | 2 |
| 20260412_000734_px4_sih | final-pass | 1 | 77.48 | 0.750 | -2.264 | 15 |
| 20260412_001028_px4_sih | final-pass | 1 | 78.90 | 0.848 | -0.936 | 3 |
