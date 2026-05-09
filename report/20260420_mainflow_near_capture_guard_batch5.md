# 2026-04-20 Mainflow Batch (near-capture retry guard)

- Goal: keep final DOCKING path short (<= mini orbit radius 80m) and reduce near-capture DOCKING->TRACKING bounce.
- Command: `EXPERIMENT_DURATION_SEC=180 START_RVIZ=false RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`
- Runs: 20260420_185000_px4_sih, 20260420_185240_px4_sih, 20260420_185510_px4_sih, 20260420_185744_px4_sih, 20260420_190018_px4_sih

## Per-run metrics

| run | phase_end | final_pass | first_dock_to_completed_path_m | max_docking_segment_path_m | last_docking_segment_path_m | docking_segment_count |
|---|---:|---:|---:|---:|---:|---:|
| 20260420_185000_px4_sih | COMPLETED | 1 | 25.54 | 25.08 | 25.08 | 1 |
| 20260420_185240_px4_sih | COMPLETED | 1 | 39.40 | 38.97 | 38.97 | 1 |
| 20260420_185510_px4_sih | COMPLETED | 1 | 120.82 | 67.14 | 43.58 | 2 |
| 20260420_185744_px4_sih | COMPLETED | 1 | 53.06 | 52.66 | 52.66 | 1 |
| 20260420_190018_px4_sih | COMPLETED | 1 | 32.69 | 32.30 | 32.30 | 1 |

## Summary

- Completed: 5/5, final_pass: 5/5.
- DOCKING segment count (completed runs): min/mean/max = 1/1.20/2.
- Last DOCKING segment path: min/mean/max = 25.08/38.52/52.66 m.
- First DOCKING->COMPLETED path: min/mean/max = 25.54/54.30/120.82 m.
