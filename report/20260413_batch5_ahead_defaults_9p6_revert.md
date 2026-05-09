# Batch Check (Ahead defaults 9.6/1.22/6.5 candidate, reverted)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.0 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- candidate final-pass: `4/5` vs baseline `5/5`
- candidate docking ahead ratio mean `0.469` vs baseline `0.659`
- candidate docking ahead min worst `-1.928m` vs baseline `-1.593m`
- candidate path mean `133.06m` vs baseline `147.38m`
- decision: **revert** stronger ahead defaults (`9.6/1.22/6.5 -> 9.4/1.18/6.2`) because stability and docking-ahead consistency regressed.

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260413_163737_px4_sih | 1 | 82.08 | 90.12 | 93.52 | 0.946 | -1.083 | 0.691 | -1.083 | 0.000 | 1.137 | 102.38 |
| 20260413_164012_px4_sih | 1 | 87.00 | 95.70 | 105.40 | 0.822 | -1.297 | 0.526 | -1.297 | 0.000 | 0.715 | 157.93 |
| 20260413_164300_px4_sih | 1 | 70.44 | 80.90 | 86.84 | 0.852 | -1.569 | 0.403 | -1.569 | 0.000 | 1.218 | 148.07 |
| 20260413_164535_px4_sih | 0 | 82.32 | 90.22 | nan | 0.812 | -1.928 | 0.341 | -1.928 | 0.000 | 0.672 | 116.27 |
| 20260413_164812_px4_sih | 1 | 71.18 | 81.14 | 86.58 | 0.853 | -1.764 | 0.385 | -1.764 | 0.000 | 0.886 | 140.66 |
