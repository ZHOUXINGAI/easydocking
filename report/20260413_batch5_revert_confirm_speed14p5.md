# Batch Check (Revert confirm, speed 14.5)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5` (baseline `5/5`)
- first TRACKING time mean `84.34s` (range `81.62~87.04`)
- first DOCKING time mean `93.22s` (range `89.88~95.48`)
- first COMPLETED time mean `98.72s` (range `95.92~99.94`)
- ahead ratio (pre-completed) mean `0.873`, ahead min mean `-1.608` (worst `-1.789`)
- docking ahead ratio mean `0.511`, docking ahead min mean `-1.608` (worst `-1.789`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `0.932m` (worst `1.507m`), carrier path mean `123.08m`

Baseline comparison (`20260413_batch5_r4e_ahead_release_tuned`):
- docking ahead ratio mean: `0.659 -> 0.511`
- docking ahead min worst: `-1.593 -> -1.789`
- path mean: `147.38m -> 123.08m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260413_171813_px4_sih | 1 | 86.90 | 95.46 | 99.60 | 0.914 | -1.349 | 0.590 | -1.349 | 0.000 | 1.507 | 113.46 |
| 20260413_172055_px4_sih | 1 | 83.08 | 92.08 | 99.18 | 0.792 | -1.709 | 0.317 | -1.709 | 0.000 | 0.697 | 135.36 |
| 20260413_172337_px4_sih | 1 | 87.04 | 95.48 | 99.94 | 0.897 | -1.704 | 0.539 | -1.704 | 0.000 | 1.099 | 112.63 |
| 20260413_172619_px4_sih | 1 | 81.62 | 89.88 | 95.92 | 0.884 | -1.789 | 0.587 | -1.789 | 0.000 | 0.641 | 121.73 |
| 20260413_172857_px4_sih | 1 | 83.08 | 93.18 | 98.94 | 0.879 | -1.487 | 0.522 | -1.487 | 0.000 | 0.713 | 132.24 |
