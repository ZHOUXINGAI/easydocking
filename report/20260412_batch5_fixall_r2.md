# Batch Check (Fix-all pass) — r2

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `4/5`
- first TRACKING time mean `81.08s` (range `72.00~86.74`)
- first DOCKING time mean `89.44s` (range `80.64~95.08`)
- first COMPLETED time mean `nans` (range `86.80~102.14`)
- ahead ratio (pre-completed) mean `0.869`, ahead min mean `-1.584` (worst `-1.825`)
- docking ahead ratio mean `0.536`, docking ahead min mean `-1.584` (worst `-1.825`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `0.547m` (worst `0.782m`), sign flips mean `2.60`
- approach orbit gap mean `6.55m`; carrier path until completed mean `125.77m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_225119_px4_sih | 1 | 83.10 | 91.34 | 95.80 | 0.912 | -1.494 | 0.596 | -1.494 | 0.000 | 0.501 | 110.29 |
| 20260412_225353_px4_sih | 1 | 86.74 | 95.08 | 102.14 | 0.900 | -1.293 | 0.681 | -1.293 | 0.000 | 0.450 | 133.02 |
| 20260412_225636_px4_sih | 1 | 82.00 | 90.20 | 96.10 | 0.880 | -1.525 | 0.559 | -1.525 | 0.000 | 0.420 | 118.07 |
| 20260412_225912_px4_sih | 0 | 81.56 | 89.96 | nan | 0.773 | -1.825 | 0.283 | -1.825 | 0.000 | 0.782 | 130.51 |
| 20260412_230148_px4_sih | 1 | 72.00 | 80.64 | 86.80 | 0.878 | -1.784 | 0.561 | -1.784 | 0.000 | 0.581 | 136.95 |
