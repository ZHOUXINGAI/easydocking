# Batch Check (Fix-all pass) — r4

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `4/5`
- first TRACKING time mean `83.19s` (range `81.90~85.60`)
- first DOCKING time mean `91.87s` (range `89.84~94.10`)
- first COMPLETED time mean `nans` (range `95.64~104.40`)
- ahead ratio (pre-completed) mean `0.850`, ahead min mean `-1.626` (worst `-1.991`)
- docking ahead ratio mean `0.583`, docking ahead min mean `-1.567` (worst `-1.991`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `1.340m` (worst `3.789m`), sign flips mean `1.00`
- approach orbit gap mean `6.62m`; carrier path until completed mean `305.82m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_232808_px4_sih | 1 | 82.64 | 91.34 | 95.64 | 0.913 | -1.487 | 0.581 | -1.487 | 0.000 | 0.270 | 113.11 |
| 20260412_233042_px4_sih | 1 | 83.50 | 92.94 | 104.40 | 0.720 | -1.991 | 0.301 | -1.991 | 0.000 | 0.977 | 177.51 |
| 20260412_233325_px4_sih | 1 | 81.90 | 89.84 | 96.00 | 0.882 | -1.842 | 0.577 | -1.842 | 0.000 | 0.794 | 119.45 |
| 20260412_233602_px4_sih | 0 | 82.30 | 91.14 | nan | 0.860 | -1.110 | 0.910 | -0.812 | 0.000 | 3.789 | 991.02 |
| 20260412_233958_px4_sih | 1 | 85.60 | 94.10 | 100.14 | 0.875 | -1.700 | 0.545 | -1.700 | 0.000 | 0.869 | 128.02 |
