# 20260416 latest batch (M1-M4, round6 first-entry 6.3)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260416_001023_px4_sih`
- `20260416_001413_px4_sih`
- `20260416_001701_px4_sih`
- `20260416_001918_px4_sih`
- `20260416_002152_px4_sih`

## Headline

- final-pass: `3/5`
- M1 same-direction: `4/5`
- M2 docking-front strict: `0/5`
- M3 short-endgame: `3/5`
- M4 final-pass: `3/5`
- all M1-M4: `0/5`

## Comparison

- round4 (kept reference): final-pass `4/5`, M2 `1/5`, all `1/5`
- round5 (close-front guard): final-pass `3/5`, M2 `1/5`, all `0/5`
- round6 (first-entry 6.3): final-pass `3/5`, M2 `0/5`, all `0/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260416_001023_px4_sih | APPROACH | 0.000000 | - | - | - | - | - | - | - | - | - | 0.000000 | - | False | False | False | False | False |
| 20260416_001413_px4_sih | COMPLETED | 1.000000 | 86.740000 | 106.380000 | 111.040000 | 0.977366 | -0.299858 | 0.881720 | -0.299858 | 0.550000 | 0.000000 | 5.900000 | 4.660000 | True | False | True | True | False |
| 20260416_001701_px4_sih | COMPLETED | 1.000000 | 83.000000 | 91.140000 | 96.740000 | 0.800000 | -0.862329 | 0.508929 | -0.862329 | 2.750000 | 0.000000 | 6.800000 | 5.600000 | True | False | True | True | False |
| 20260416_001918_px4_sih | COMPLETED | 1.000000 | 83.300000 | 92.400000 | 97.700000 | 0.861111 | -1.230524 | 0.622642 | -1.230524 | 2.000000 | 0.000000 | 6.550000 | 5.300000 | True | False | True | True | False |
| 20260416_002152_px4_sih | DOCKING | 0.000000 | 83.540000 | 92.440000 | - | 0.982902 | -0.883905 | 0.981164 | -0.883905 | 1.320000 | 0.000000 | 70.040000 | - | True | False | False | False | False |

Raw metrics CSV: `report/20260416_batch5_m1m4_latest_r6_metrics.csv`
