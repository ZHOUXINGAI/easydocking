# 2026-04-24 Terminal Smoke Prehold-Gate Batch (5 runs)

## Goal
- Keep work on the M0/M1 mainline: clean up `terminal_smoke` sample validity before changing release-family scoring.
- Eliminate the intermittent polluted baseline where `carrier_activate_on_launch=true` but `START` still happens while the carrier is effectively on the ground.

## Code change in this probe
- Added a new carrier prehold-ready gate to `scripts/wait_for_docking_window.py`.
- `terminal_smoke` / `prehold` now require:
  - `carrier_prehold_required=true`
  - `carrier_z >= 24.4m` (derived from `carrier_idle_hover_altitude=29.4` with `5m` margin)
  - `abs(carrier_vz) <= 2.5 m/s`
  - `8` consecutive good samples
- Plumbed the new parameters through:
  - `scripts/start_docking_command.sh`
  - `scripts/run_px4_sih_docking_experiment.sh`
- Added report / batch fields so each run now records:
  - `start_carrier_z_m`
  - `start_prehold_ready`
  - accepted-window `carrier_z / carrier_vz / prehold_ok`
- Added batch classification support for `prehold-start-fail`.

## Config
- `START_RVIZ=false`
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_CARRIER_PREHOLD_REQUIRED=true`
- `AUTO_START_CARRIER_PREHOLD_MIN_ALTITUDE_M=24.4`
- `AUTO_START_CARRIER_PREHOLD_MAX_ABS_VZ_MPS=2.5`
- `AUTO_START_CARRIER_PREHOLD_MIN_SAMPLES=8`

## Runs

| run | classification | start_t_sec | start_carrier_z_m | start_prehold_ready | accepted_rel_z_m | accepted_phase_err_deg | accepted_pred_score | diag_route_ok | diag_route_rej | post_start_path_length_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `20260424_201705_px4_sih` | `geometry-fail` | `24.40` | `29.480` | `1` | `3.200` | `88.5` | `0.800` | `0` | `16` | `1750.176` |
| `20260424_202103_px4_sih` | `geometry-fail` | `33.00` | `29.634` | `1` | `3.160` | `90.4` | `0.570` | `0` | `16` | `1615.066` |
| `20260424_202504_px4_sih` | `geometry-fail` | `27.04` | `29.330` | `1` | `2.730` | `86.5` | `0.750` | `0` | `16` | `1726.254` |
| `20260424_202900_px4_sih` | `geometry-fail` | `22.38` | `29.408` | `1` | `2.880` | `85.8` | `0.800` | `0` | `16` | `1783.305` |
| `20260424_203256_px4_sih` | `final-pass` | `36.64` | `29.529` | `1` | `2.770` | `92.5` | `0.570` | `0` | `16` | `368.747` |

## What this fixed
- The dirty baseline issue is blocked in this batch:
  - all `5/5` runs have `start_prehold_ready=1`
  - all accepted windows have `accepted_window_prehold_ok=1`
  - no run released with `carrier_z≈0`
- So the new gate is doing the intended cleanup: release-family analysis is no longer contaminated by the known prehold anomaly.

## Main result
- Validity improved, but release-family selection is still bad:
  - `final-pass = 1/5`
  - `geometry-fail = 4/5`
  - the fail family is now extremely long-tail again: `post_start_path_length_m ≈ 1615–1783m`

## Mainline interpretation
- This confirms the current blocker is still M0/M1 release-family selection, not terminal tuning.
- After removing the dirty prehold sample, the accepted windows are still too permissive and can trigger a very early family:
  - fail runs start at `22.38s`, `24.40s`, `27.04s`, `33.00s`
  - the only pass starts later at `36.64s`
- The new `diag_*` fields are not yet separating the family in their current meaning:
  - all `5/5` runs show `diag_route_ok=0`
  - all `5/5` runs show `diag_route_rej=16`
  - that includes the single `final-pass`
- So the next discriminator should not be “just use `diag_route_ok` as a gate”.

## Practical conclusion
- The prehold gate should be kept.
- The next mainline step is:
  1. keep only prehold-valid samples
  2. revisit why `terminal_smoke` is still accepting the early orbit-progress family
  3. build the next discriminator around release timing / future route family, not terminal-side tuning
