# Afternoon long endgame path diagnosis

Question: why some afternoon runs show overly long terminal straight-line segment, and is it solved?

## Observation from logs

Source metrics: `report/20260415_afternoon_long_endgame_metrics.csv`

Representative long cases:
- `20260415_143939_px4_sih` (COMPLETED): `dock_to_completed_sec=12.04`, `straight_sec=10.60`
- `20260415_145840_px4_sih` (COMPLETED): `dock_to_completed_sec=6.70`, `straight_sec=7.95`
- `20260415_155603_px4_sih` (DOCKING fail): `straight_sec=14.063`
- `20260415_145454_px4_sih` (DOCKING fail): `straight_sec=69.320`

## Root-cause summary

Not a single cause, mainly two patterns:

1. **DOCKING entered but completion hold not reached**
   - mini stays in terminal straight mode for a long time, producing visually long straight segment.
   - seen in fail runs with large `straight_sec` (e.g. 14s / 69s).

2. **Release/start timing spread is too wide**
   - some runs release around `~13-15m`, some much later in DOCKING phase.
   - this variability makes endgame geometry inconsistent, so some runs drag long before completion.

## Tried this round (and result)

- Tightened `close_tracking_release_max_distance` to 14m: **not kept** (introduced late-release/failed runs).
- Added terminal history fallback attempt: **not kept** (5-run validation degraded to `1/5`).
  - evidence: `report/20260415_batch5_term_history_fallback_metrics.csv`

## Current status

- **Not solved yet in promoted baseline**.
- Current promoted baseline remains unchanged (those two experiments were reverted).
