# Batch Check (Fix-all pass) — r1

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- first TRACKING time mean `81.89s` (range `70.16~87.10`)
- first DOCKING time mean `90.61s` (range `78.76~95.40`)
- first COMPLETED time mean `95.94s` (range `84.90~102.70`)
- ahead ratio (pre-completed) mean `0.891`, ahead min mean `-1.574` (worst `-1.957`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `0.851m` (worst `1.166m`), sign flips mean `1.80`
- approach orbit gap mean `6.60m`; carrier path until completed mean `122.00m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | docking_lateral_sign_flip_count | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_222045_px4_sih | 1 | 81.88 | 92.02 | 98.18 | 0.888 | -1.448 | 0.000 | 0.607 | 3 | 134.45 |
| 20260412_222323_px4_sih | 1 | 87.10 | 95.40 | 102.70 | 0.829 | -1.596 | 0.000 | 1.166 | 3 | 136.33 |
| 20260412_222607_px4_sih | 1 | 83.44 | 91.64 | 94.34 | 0.965 | -1.228 | 0.000 | 0.627 | 1 | 91.86 |
| 20260412_222839_px4_sih | 1 | 86.88 | 95.24 | 99.60 | 0.908 | -1.957 | 0.000 | 0.909 | 0 | 111.80 |
| 20260412_223118_px4_sih | 1 | 70.16 | 78.76 | 84.90 | 0.866 | -1.642 | 0.000 | 0.946 | 2 | 135.55 |
