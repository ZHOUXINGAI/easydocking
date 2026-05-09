# Ahead Hard-Constraint Batch (5 runs, CARRIER_TRACKING_SPEED_LIMIT=14.5, round 4)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- mean `ahead_ratio_positive_pre_completed`: `0.940`
- mean `ahead_min_m`: `-0.402` (worst `-0.772`)
- mean `ahead_sign_flip_count`: `4.20`
- mean `tracking_ratio_positive`: `0.810` (tracking worst min `-0.760`)
- mean `docking_ratio_positive`: `0.866` (docking worst min `-0.772`)

| run_id | classification | final_pass | first_non_idle_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | ahead_sign_flip_count | tracking_ratio_positive | tracking_min_m | docking_ratio_positive | docking_min_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_132434_px4_sih | final-pass | 1 | 77.24 | 0.956 | -0.321 | 3 | 0.636 | -0.321 | 0.966 | -0.143 |
| 20260412_132725_px4_sih | final-pass | 1 | 77.58 | 1.000 | 0.141 | 0 | 1.000 | 0.465 | 1.000 | 0.242 |
| 20260412_133007_px4_sih | final-pass | 1 | 78.96 | 0.864 | -0.772 | 10 | 1.000 | 0.902 | 0.585 | -0.772 |
| 20260412_133302_px4_sih | final-pass | 1 | 77.26 | 0.952 | -0.296 | 5 | 0.772 | -0.165 | 0.900 | -0.296 |
| 20260412_133551_px4_sih | final-pass | 1 | 107.34 | 0.926 | -0.760 | 3 | 0.644 | -0.760 | 0.877 | -0.444 |
