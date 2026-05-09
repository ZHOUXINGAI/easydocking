# Batch Check (R4E ahead release tuned)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh (5x)`

- final-pass: `5/5`
- first TRACKING time mean `79.94s` (range `72.06~87.44`)
- first DOCKING time mean `89.42s` (range `81.00~97.44`)
- first COMPLETED time mean `96.51s` (range `88.66~104.54`)
- ahead ratio (pre-completed) mean `0.899`, ahead min mean `-1.161` (worst `-1.593`)
- docking ahead ratio mean `0.659`, docking ahead min mean `-1.161` (worst `-1.593`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `1.180m` (worst `1.665m`), carrier path until completed mean `147.38m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260413_114715_px4_sih | 1 | 83.66 | 93.80 | 100.36 | 0.969 | -0.512 | 0.886 | -0.512 | 0.000 | 1.370 | 142.02 |
| 20260413_114956_px4_sih | 1 | 83.34 | 91.34 | 94.30 | 0.963 | -1.187 | 0.767 | -1.187 | 0.000 | 0.716 | 92.04 |
| 20260413_115230_px4_sih | 1 | 73.20 | 83.50 | 94.70 | 0.895 | -0.921 | 0.724 | -0.921 | 0.000 | 1.665 | 204.42 |
| 20260413_115502_px4_sih | 1 | 87.44 | 97.44 | 104.54 | 0.872 | -1.593 | 0.559 | -1.593 | 0.000 | 0.926 | 145.44 |
| 20260413_115751_px4_sih | 1 | 72.06 | 81.00 | 88.66 | 0.797 | -1.592 | 0.357 | -1.592 | 0.000 | 1.221 | 153.00 |
