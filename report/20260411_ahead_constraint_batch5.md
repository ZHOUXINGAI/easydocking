# Ahead Hard-Constraint Batch (5 runs)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- mean `ahead_ratio_positive_pre_completed`: `0.757`
- mean `ahead_min_m`: `-2.486` (worst `-3.684`)
- mean `ahead_sign_flip_count`: `5.20`

| run_id | classification | final_pass | first_non_idle_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | ahead_sign_flip_count |
|---|---:|---:|---:|---:|---:|---:|
| 20260411_231604_px4_sih | final-pass | 1 | 103.10 | 0.842 | -2.176 | 7 |
| 20260411_231916_px4_sih | final-pass | 1 | 77.64 | 0.822 | -1.481 | 4 |
| 20260411_232203_px4_sih | final-pass | 1 | 77.30 | 0.655 | -3.684 | 5 |
| 20260411_232454_px4_sih | final-pass | 1 | 76.04 | 0.660 | -2.786 | 5 |
| 20260411_232743_px4_sih | final-pass | 1 | 107.70 | 0.805 | -2.305 | 5 |
