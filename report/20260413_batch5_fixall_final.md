# Batch Check (Fix-all final)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- first TRACKING time mean `85.52s` (range `81.64~86.94`)
- first DOCKING time mean `94.05s` (range `89.88~95.64`)
- first COMPLETED time mean `99.41s` (range `94.54~101.24`)
- ahead ratio (pre-completed) mean `0.866`, ahead min mean `-1.639` (worst `-2.187`)
- docking ahead ratio mean `0.492`, docking ahead min mean `-1.639` (worst `-2.187`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `1.064m` (worst `1.518m`), sign flips mean `2.80`
- approach orbit gap mean `6.83m`; carrier path until completed mean `120.96m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260413_001536_px4_sih | 1 | 86.94 | 95.64 | 101.08 | 0.818 | -1.477 | 0.422 | -1.477 | 0.000 | 1.321 | 125.43 |
| 20260413_001820_px4_sih | 1 | 81.64 | 89.88 | 94.54 | 0.901 | -1.333 | 0.581 | -1.333 | 0.000 | 0.482 | 110.04 |
| 20260413_002055_px4_sih | 1 | 86.70 | 95.20 | 99.60 | 0.908 | -1.527 | 0.568 | -1.527 | 0.000 | 1.518 | 114.44 |
| 20260413_002334_px4_sih | 1 | 85.54 | 94.08 | 101.24 | 0.857 | -1.670 | 0.538 | -1.670 | 0.000 | 1.319 | 134.58 |
| 20260413_002617_px4_sih | 1 | 86.80 | 95.44 | 100.60 | 0.845 | -2.187 | 0.350 | -2.187 | 0.000 | 0.682 | 120.32 |
