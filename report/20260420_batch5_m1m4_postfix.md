# 20260420 clean batch after startup/odom robustness fix (M1-M4)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260420_140347_px4_sih`
- `20260420_140754_px4_sih`
- `20260420_141046_px4_sih`
- `20260420_141456_px4_sih`
- `20260420_141855_px4_sih`

## Headline

- final-pass: `1/5`
- M1 same-direction: `1/5`
- M2 docking-front strict: `1/5`
- M3 short-endgame: `0/5`
- all M1-M4: `0/5`

## Comparison

- previous clean batch (`report/20260420_batch5_m1m4_clean_metrics.csv`): final-pass `1/5`, M1 `3/5`, M2 `1/5`, M3 `0/5`, all `0/5`
- current postfix batch: final-pass `1/5`, M1 `1/5`, M2 `1/5`, M3 `0/5`, all `0/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260420_140347_px4_sih | TRACKING | 0.000000 | 86.200000 | 94.660000 | - | 0.432452 | -11.116454 | 1.000000 | 0.359783 | 0.000000 | 0.041322 | 0.000000 | - | False | True | False | False | False |
| 20260420_140754_px4_sih | COMPLETED | 1.000000 | 87.040000 | 96.700000 | 110.140000 | 1.000000 | 1.197918 | 0.832714 | -0.733804 | 2.260000 | 0.000000 | 14.700000 | 13.440000 | True | False | False | True | False |
| 20260420_141046_px4_sih | APPROACH | 0.000000 | - | - | - | - | - | - | - | 0.000000 | - | 0.000000 | - | False | False | False | False | False |
| 20260420_141456_px4_sih | APPROACH | 0.000000 | - | - | - | - | - | - | - | 0.000000 | - | 0.000000 | - | False | False | False | False | False |
| 20260420_141855_px4_sih | IDLE | 0.000000 | - | - | - | - | - | - | - | 0.000000 | - | 0.000000 | - | False | False | False | False | False |

Raw metrics CSV: `report/20260420_batch5_m1m4_postfix_metrics.csv`
