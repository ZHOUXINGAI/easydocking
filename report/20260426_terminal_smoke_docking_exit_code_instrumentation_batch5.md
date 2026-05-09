# 2026-04-26 terminal_smoke docking-exit instrumentation batch-5

## Goal

After the failed late precision re-entry probes, the next question was:

- which `DOCKING -> TRACKING` path is actually firing online in the surviving long-tail family?

To answer that without another speculative controller change, a new controller debug field was added:

- `controller_passive_docking_last_exit_trigger_code`

This records the last `DOCKING` exit cause seen by the controller.

## Exit-code mapping

Current internal mapping:

- `1` = completed through normal completion / capture envelope
- `2` = passive timeout soft-attach completion
- `3` = passive timeout retrack
- `4` = retry from corridor stall
- `5` = retry after early release
- `6` = retry from first-entry stall
- `7` = retry from vertical regression
- `8` = retry on front loss
- `9` = generic corridor low-score retrack
- `10` = large-distance release retrack

## Batch

Common config:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.74`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=2.00`
- `START_RVIZ=false`

Runs:

- `20260426_031737_px4_sih`
- `20260426_032010_px4_sih`
- `20260426_032248_px4_sih`
- `20260426_032520_px4_sih`
- `20260426_032757_px4_sih`

Headline result:

- `final-pass = 5/5`

## Exit-code sequences

Per run:

- `20260426_031737_px4_sih`
  - sequence: `entry1 -> code4 -> entry2 -> code9 -> entry3 -> completed`
  - `docking_path_length_m = 114.405`
- `20260426_032010_px4_sih`
  - sequence: `entry1 -> code4 -> entry2 -> code3 -> entry3 -> completed`
  - `docking_path_length_m = 142.754`
- `20260426_032248_px4_sih`
  - sequence: `entry1 -> code7 -> entry2 -> completed`
  - `docking_path_length_m = 108.398`
- `20260426_032520_px4_sih`
  - sequence: `entry1 -> code4 -> entry2 -> completed`
  - `docking_path_length_m = 84.282`
- `20260426_032757_px4_sih`
  - sequence: `entry1 -> code4 -> entry2 -> completed`
  - `docking_path_length_m = 71.777`

## Main finding

This batch changes the interpretation of the long-tail family:

- `code4` on the first `DOCKING` entry is **common even in good runs**
- so first-entry `corridor stall` retrack is not the main blocker by itself

The longer families appear later, after the second entry:

- `code9` family:
  - second entry drifts into large lateral error while already far ahead
  - representative run: `20260426_031737_px4_sih`
- `code3` family:
  - second entry keeps lateral small enough, but stays too far ahead long enough to hit passive timeout retrack
  - representative run: `20260426_032010_px4_sih`

By contrast, the short families:

- `20260426_032520_px4_sih`
- `20260426_032757_px4_sih`

recover after the first `code4` retrack and complete directly from the second entry.

## Practical conclusion

The residual bottleneck is now better localized:

- not “first entry retrack is wrong”
- not “late precision re-entry gate is too loose”

It is specifically a **second-entry endgame split**:

1. second-entry lateral divergence family (`code9`)
2. second-entry ahead-timeout family (`code3`)

## Next step

Compare these pairs directly:

- long pass `20260426_031737_px4_sih` (`code4 -> code9`)
- short pass `20260426_032757_px4_sih` (`code4 -> completed`)

and:

- longer pass `20260426_032010_px4_sih` (`code4 -> code3`)
- shorter pass `20260426_032520_px4_sih` (`code4 -> completed`)

Focus should stay on **second-entry DOCKING behavior**, not on release thresholds or first-entry re-entry gating.
