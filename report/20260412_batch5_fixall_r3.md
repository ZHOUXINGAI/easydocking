# Batch Check (Fix-all pass) — r3

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `5/5`
- first TRACKING time mean `83.57s` (range `81.48~86.80`)
- first DOCKING time mean `92.20s` (range `89.88~95.10`)
- first COMPLETED time mean `98.84s` (range `95.64~103.10`)
- ahead ratio (pre-completed) mean `0.883`, ahead min mean `-1.299` (worst `-1.830`)
- docking ahead ratio mean `0.618`, docking ahead min mean `-1.195` (worst `-1.830`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `1.133m` (worst `1.628m`), sign flips mean `1.40`
- approach orbit gap mean `6.55m`; carrier path until completed mean `129.44m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_230543_px4_sih | 1 | 83.40 | 92.14 | 99.30 | 0.880 | -1.830 | 0.615 | -1.830 | 0.000 | 0.755 | 132.93 |
| 20260412_230822_px4_sih | 1 | 86.80 | 95.10 | 103.10 | 0.854 | -1.727 | 0.562 | -1.727 | 0.000 | 1.254 | 139.39 |
| 20260412_231106_px4_sih | 1 | 83.14 | 92.30 | 95.64 | 0.998 | 0.000 | 1.000 | 0.521 | 0.000 | 1.628 | 108.47 |
| 20260412_231339_px4_sih | 1 | 81.48 | 89.88 | 97.48 | 0.843 | -1.352 | 0.438 | -1.352 | 0.000 | 1.588 | 132.65 |
| 20260412_231617_px4_sih | 1 | 83.04 | 91.60 | 98.70 | 0.839 | -1.588 | 0.472 | -1.588 | 0.000 | 0.439 | 133.78 |
