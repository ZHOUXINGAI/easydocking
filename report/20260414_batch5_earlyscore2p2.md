# Batch Check (Early release score threshold 2.2)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true AUTO_START_REAR_ENTRY_ENERGY_EARLY_RELEASE_SCORE_THRESHOLD=2.2 CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5` vs baseline `5/5`
- first TRACKING mean `81.56s`, first DOCKING mean `90.34s`, first COMPLETED mean `96.99s`
- ahead ratio (pre-completed) mean `0.882`, ahead min worst `-1.966m`
- docking ahead ratio mean `0.593` vs baseline `0.659`
- docking ahead min worst `-1.966m` vs baseline `-1.593m`
- opp ratio first 6s mean `0.000`
- docking lateral max |error| mean `0.960m`, path mean `134.13m` (baseline `147.38m`)

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260414_223741_px4_sih | 1 | 71.52 | 79.82 | 84.36 | 0.916 | -1.249 | 0.615 | -1.249 | 0.000 | 0.568 | 119.82 |
| 20260414_224015_px4_sih | 1 | 83.20 | 91.64 | 94.90 | 0.953 | -0.998 | 0.723 | -0.998 | 0.000 | 1.521 | 100.34 |
| 20260414_224253_px4_sih | 1 | 82.70 | 93.00 | 100.76 | 0.841 | -1.966 | 0.471 | -1.966 | 0.000 | 1.005 | 153.23 |
| 20260414_224539_px4_sih | 1 | 87.46 | 95.50 | 102.00 | 0.883 | -1.791 | 0.608 | -1.791 | 0.000 | 0.836 | 125.90 |
| 20260414_224827_px4_sih | 1 | 82.92 | 91.76 | 102.92 | 0.816 | -1.685 | 0.547 | -1.685 | 0.000 | 0.872 | 171.37 |
