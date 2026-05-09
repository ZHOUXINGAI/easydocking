# Ahead Hard-Constraint Batch (5 runs, CARRIER_TRACKING_SPEED_LIMIT=14.5, round 3)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- mean `ahead_ratio_positive_pre_completed`: `0.920`
- mean `ahead_min_m`: `-0.600` (worst `-0.900`)
- mean `ahead_sign_flip_count`: `4.20`
- mean `tracking_ratio_positive`: `0.800` (tracking worst min `-0.900`)
- mean `docking_ratio_positive`: `0.791` (docking worst min `-0.896`)

| run_id | classification | final_pass | first_non_idle_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | ahead_sign_flip_count | tracking_ratio_positive | tracking_min_m | docking_ratio_positive | docking_min_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_030333_px4_sih | final-pass | 1 | 79.30 | 0.909 | -0.786 | 3 | 0.774 | -0.786 | 0.806 | -0.562 |
| 20260412_030629_px4_sih | final-pass | 1 | 120.80 | 0.917 | -0.563 | 4 | 0.746 | -0.295 | 0.733 | -0.563 |
| 20260412_030958_px4_sih | final-pass | 1 | 78.04 | 1.000 | 0.145 | 0 | 1.000 | 0.576 | 1.000 | 0.145 |
| 20260412_031238_px4_sih | final-pass | 1 | 79.04 | 0.894 | -0.900 | 5 | 0.791 | -0.900 | 0.739 | -0.879 |
| 20260412_031525_px4_sih | final-pass | 1 | 77.74 | 0.878 | -0.896 | 9 | 0.689 | -0.407 | 0.677 | -0.896 |
