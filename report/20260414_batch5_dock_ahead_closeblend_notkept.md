# Batch Check (Dock ahead close-blend candidate, not kept)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- change under test: increase docking ahead margin only near terminal close-range (lateral/distance blended).
- final-pass: `4/5` vs baseline `5/5`
- docking ahead ratio mean `0.587` vs baseline `0.659`
- docking ahead min worst `-1.856m` vs baseline `-1.593m`
- path mean `157.41m` vs baseline `147.38m`
- decision: **not kept** (improves docking-ahead mean, but drops to 4/5 stability and increases path).

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260414_115408_px4_sih | 1 | 71.94 | 80.54 | 91.14 | 0.851 | -1.703 | 0.627 | -1.703 | 0.000 | 1.487 | 173.69 |
| 20260414_115638_px4_sih | 1 | 84.18 | 92.78 | 99.98 | 0.846 | -1.856 | 0.500 | -1.856 | 0.000 | 1.090 | 135.56 |
| 20260414_115921_px4_sih | 0 | 81.84 | 90.44 | nan | 0.755 | -1.600 | 0.410 | -1.600 | 0.000 | 1.092 | 164.38 |
| 20260414_120207_px4_sih | 1 | 81.36 | 89.46 | 95.30 | 0.869 | -1.507 | 0.521 | -1.507 | 0.000 | 0.682 | 117.02 |
| 20260414_120443_px4_sih | 1 | 82.64 | 90.74 | 105.64 | 0.940 | -1.240 | 0.876 | -1.240 | 0.000 | 1.240 | 196.38 |
