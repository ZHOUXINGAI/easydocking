# Ahead Hard-Constraint Batch (5 runs, CARRIER_TRACKING_SPEED_LIMIT=14.5, round 5)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- mean `ahead_ratio_positive_pre_completed`: `0.920`
- mean `ahead_min_m`: `-0.710` (worst `-1.328`)
- mean `ahead_sign_flip_count`: `4.20`
- mean `tracking_ratio_positive`: `0.788` (tracking worst min `-1.226`)
- mean `docking_ratio_positive`: `0.768` (docking worst min `-1.328`)

| run_id | classification | final_pass | first_non_idle_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | ahead_sign_flip_count | tracking_ratio_positive | tracking_min_m | docking_ratio_positive | docking_min_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_135552_px4_sih | final-pass | 1 | 78.90 | 0.956 | -0.398 | 3 | 1.000 | 0.332 | 0.708 | -0.398 |
| 20260412_135837_px4_sih | final-pass | 1 | 78.54 | 0.964 | -0.779 | 1 | 1.000 | 0.676 | 0.800 | -0.779 |
| 20260412_140124_px4_sih | final-pass | 1 | 75.44 | 0.853 | -1.226 | 6 | 0.308 | -1.226 | 0.749 | -1.079 |
| 20260412_140414_px4_sih | final-pass | 1 | 76.30 | 1.000 | 0.179 | 0 | 1.000 | 0.436 | 1.000 | 0.179 |
| 20260412_140653_px4_sih | final-pass | 1 | 78.84 | 0.826 | -1.328 | 11 | 0.632 | -0.643 | 0.582 | -1.328 |
