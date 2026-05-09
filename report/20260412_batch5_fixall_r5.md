# Batch Check (Fix-all pass) — r5

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `4/5`
- first TRACKING time mean `83.81s` (range `81.80~86.50`)
- first DOCKING time mean `92.82s` (range `89.90~97.14`)
- first COMPLETED time mean `nans` (range `95.70~100.20`)
- ahead ratio (pre-completed) mean `0.883`, ahead min mean `-1.593` (worst `-2.129`)
- docking ahead ratio mean `0.579`, docking ahead min mean `-1.593` (worst `-2.129`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `1.053m` (worst `1.772m`), sign flips mean `4.20`
- approach orbit gap mean `6.61m`; carrier path until completed mean `252.27m`

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_234444_px4_sih | 1 | 83.34 | 91.70 | 98.14 | 0.852 | -1.811 | 0.496 | -1.811 | 0.000 | 0.563 | 123.07 |
| 20260412_234721_px4_sih | 1 | 83.64 | 93.40 | 100.20 | 0.797 | -2.129 | 0.279 | -2.129 | 0.000 | 0.717 | 140.40 |
| 20260412_235000_px4_sih | 1 | 83.78 | 91.94 | 96.34 | 0.906 | -1.628 | 0.580 | -1.628 | 0.000 | 1.571 | 108.84 |
| 20260412_235235_px4_sih | 0 | 86.50 | 97.14 | nan | 0.982 | -0.608 | 0.978 | -0.608 | 0.000 | 1.772 | 768.61 |
| 20260412_235628_px4_sih | 1 | 81.80 | 89.90 | 95.70 | 0.880 | -1.787 | 0.560 | -1.787 | 0.000 | 0.643 | 120.44 |
