# Batch check: low-speed relaxed ahead-band (NOT KEPT)

Code experiment (reverted): allow first-entry ahead upper bound to relax from `6.0m` to `6.3m` only under low relative-speed conditions.

Validation command (5x):
`START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260415_144721_px4_sih`
- `20260415_145109_px4_sih`
- `20260415_145454_px4_sih`
- `20260415_145840_px4_sih`
- `20260415_150112_px4_sih`

## Result

- final-pass: `1/5` (baseline reference `3/5`)
- TRACKING+DOCKING ahead ratio mean: `0.960` (worst min `-1.165m`)
- DOCKING ahead ratio mean: `0.776` (worst min `-1.165m`, mean negative duration `1.50s`)
- first-6s opposite-direction ratio mean: `0.008`

Interpretation:
- The change does not improve reliability; it increases TRACKING-only failures.
- DOCKING front-consistency is still not stable enough.
- Decision: **not promoted**; code has been reverted.

## Entry-blocker cross-check

Using `scripts/analyze_tracking_entry_blockers.py`:
- output CSV: `report/20260415_entry_blocker_ahead6p3_lowspeed_batch.csv`
- TRACKING failures still show `entry_like_rows=0` with positive `ahead_upper_gap_m` (e.g. `0.54m`, `1.05m`, `0.77m`), meaning the run remains blocked by first-entry ahead upper gate.

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260415_144721_px4_sih | TRACKING | 0 | 82.74 | - | - | 1.000 | 0.596 | nan | nan | nan |
| 20260415_145109_px4_sih | TRACKING | 0 | 83.04 | - | - | 1.000 | 0.827 | nan | nan | nan |
| 20260415_145454_px4_sih | DOCKING | 0 | 83.54 | 93.34 | - | 1.000 | 0.048 | 1.000 | 1.453 | 0.000 |
| 20260415_145840_px4_sih | COMPLETED | 1 | 83.14 | 91.44 | 98.14 | 0.800 | -1.165 | 0.552 | -1.165 | 3.000 |
| 20260415_150112_px4_sih | TRACKING | 0 | 83.18 | - | - | 1.000 | 1.739 | nan | nan | nan |

Raw metrics CSV: `report/20260415_batch5_rviz_silent_ahead6p3_lowspeed_metrics.csv`
