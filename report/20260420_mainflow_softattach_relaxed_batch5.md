# 2026-04-20 Mainflow Batch (soft-attach relaxed + near-capture retry guard)

- Goal: reduce first DOCKING->COMPLETED outlier while keeping mainflow stability.
- Command: `EXPERIMENT_DURATION_SEC=180 START_RVIZ=false RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`
- Runs: 20260420_212529_px4_sih, 20260420_212930_px4_sih, 20260420_213229_px4_sih, 20260420_213621_px4_sih, 20260420_213918_px4_sih

## Per-run metrics

| run | phase_end | final_pass | first_dock_to_completed_path_m | max_docking_segment_path_m | last_docking_segment_path_m | docking_segment_count |
|---|---:|---:|---:|---:|---:|---:|
| 20260420_212529_px4_sih | TRACKING | 0 |  |  |  | 0 |
| 20260420_212930_px4_sih | COMPLETED | 1 | 39.50 | 39.09 | 39.09 | 1 |
| 20260420_213229_px4_sih | COMPLETED | 1 | 591.98 | 68.20 | 32.50 | 9 |
| 20260420_213621_px4_sih | COMPLETED | 1 | 218.92 | 68.32 | 65.91 | 3 |
| 20260420_213918_px4_sih | COMPLETED | 1 | 94.10 | 93.78 | 93.78 | 1 |

## Summary

- Completed: 4/5, final_pass: 4/5.
- DOCKING segment count (completed): min/mean/max = 1/3.50/9.
- Last DOCKING segment path: min/mean/max = 32.50/57.82/93.78 m.
- First DOCKING->COMPLETED path: min/mean/max = 39.50/236.13/591.98 m.
