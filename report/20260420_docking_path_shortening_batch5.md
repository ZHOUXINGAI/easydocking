# 2026-04-20 Docking Path Shortening Batch (5 runs)

- Goal: shorten late DOCKING path to around mini orbit radius scale (80m).
- Command: `EXPERIMENT_DURATION_SEC=180 START_RVIZ=false RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`
- Runs: 20260420_150109_px4_sih, 20260420_150346_px4_sih, 20260420_150626_px4_sih, 20260420_151005_px4_sih, 20260420_151238_px4_sih

## Per-run metrics

| run | phase_end | final_pass | first_dock_to_completed_path_m | max_docking_segment_path_m | last_docking_segment_path_m | docking_segment_count |
|---|---:|---:|---:|---:|---:|---:|
| 20260420_150109_px4_sih | COMPLETED | 1 | 34.30 | 33.89 | 33.89 | 1 |
| 20260420_150346_px4_sih | COMPLETED | 1 | 37.15 | 36.80 | 36.80 | 1 |
| 20260420_150626_px4_sih | COMPLETED | 1 | 173.88 | 23.88 | 23.88 | 5 |
| 20260420_151005_px4_sih | COMPLETED | 1 | 33.07 | 32.67 | 32.67 | 1 |
| 20260420_151238_px4_sih | COMPLETED | 1 | 38.38 | 37.96 | 37.96 | 1 |

## Summary

- Completed: 5/5, final_pass: 5/5.
- Last DOCKING segment path (completed runs): min/mean/max = 23.88/33.04/37.96 m.
- First DOCKING->COMPLETED integrated path: min/mean/max = 33.07/63.36/173.88 m.
- Note: one run (`20260420_150626_px4_sih`) had multiple DOCKING<->TRACKING retries, so first-docking integrated path was long despite short final docking segment.
