# 20260420 clean batch (M1-M4)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260420_132136_px4_sih`
- `20260420_132528_px4_sih`
- `20260420_132905_px4_sih`
- `20260420_133229_px4_sih`
- `20260420_133623_px4_sih`

## Headline

- final-pass: `1/5`
- M1 same-direction: `3/5`
- M2 docking-front strict: `1/5`
- M3 short-endgame: `0/5`
- all M1-M4: `0/5`

## Comparison

- reference round4: final-pass `4/5`, M2 `1/5`, all `1/5`
- clean batch: final-pass `1/5`, M2 `1/5`, all `0/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260420_132136_px4_sih | TRACKING | 0.000000 | 82.600000 | - | - | 1.000000 | 0.762251 | - | - | - | 0.000000 | 0.000000 | - | True | False | False | False | False |
| 20260420_132528_px4_sih | DOCKING | 0.000000 | 79.440000 | 87.800000 | - | 0.988072 | -0.924631 | 1.000000 | 1.723586 | 0.000000 | 0.041322 | 110.640000 | - | False | True | False | False | False |
| 20260420_132905_px4_sih | NO_DATA | 0.000000 | - | - | - | - | - | - | - | - | - | 0.000000 | - | False | False | False | False | False |
| 20260420_133229_px4_sih | DOCKING | 0.000000 | 67.240000 | 75.480000 | - | 0.916223 | -1.181272 | 0.986815 | -0.250620 | 0.779000 | 0.000000 | 58.876000 | - | True | False | False | False | False |
| 20260420_133623_px4_sih | COMPLETED | 1.000000 | 83.280000 | 131.040000 | 170.240000 | 0.828637 | -10.545767 | 0.753304 | -0.413708 | 2.800000 | 0.000000 | 14.150000 | 39.200000 | True | False | False | True | False |

Raw metrics CSV: `report/20260420_batch5_m1m4_clean_metrics.csv`
