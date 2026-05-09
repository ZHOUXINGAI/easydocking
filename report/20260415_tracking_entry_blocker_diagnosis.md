# TRACKING->DOCKING entry blocker diagnosis

Tool: `scripts/analyze_tracking_entry_blockers.py`

## Inputs

- Baseline 5-run (12:10 batch): `report/20260415_entry_blocker_baseline1210.csv`
- Relaxed-ahead experiment 5-run: `report/20260415_entry_blocker_ahead6p3_lowspeed_batch.csv`

## Main finding

In TRACKING-fail runs, most entry conditions are repeatedly satisfied, but the first-entry ahead upper bound remains blocked:
- gate form: `terminal_along_error > -6.0`
- observed fail runs have `max_along_core_m < -6.0`, with `ahead_upper_gap_m` typically `0.19m` to `1.05m`

Examples:
- Baseline fail `20260415_121033_px4_sih`: `ahead_upper_gap_m=0.220`
- Baseline fail `20260415_122202_px4_sih`: `ahead_upper_gap_m=0.191`
- Experiment fail `20260415_145109_px4_sih`: `ahead_upper_gap_m=1.050`

## Why last experiment was rejected

The relaxed-ahead attempt did not stabilize conversion:
- batch final-pass dropped to `1/5`
- TRACKING-only failures remained frequent
- one completed run still had large DOCKING back-cross window (`docking_negative_duration_sec=3.0`)

## Next practical direction

- Keep controller baseline (already reverted)
- Focus on runtime/start-window side:
  - reduce cases where carrier reaches the near-entry core envelope with `terminal_along_error` permanently below `-6.0`
  - then re-check with this blocker tool before controller-side tuning
