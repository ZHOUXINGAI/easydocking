# 2026-04-25 early-`TRACKING` re-arm hold probe

## Goal

After widening the online `TRACKING` discriminator, the main unresolved question became:

- is the guard really not firing,
- or is it firing so briefly that the run artifacts never show `TRACKING -> APPROACH`

To answer that, the controller was changed so that a guard-triggered re-arm **holds `APPROACH` for about `1.0s`** before re-entering normal phase logic.

## Code-side change

- when the early-`TRACKING` re-arm guard trips:
  - set `current_phase = APPROACH`
  - reset corridor / release state as before
  - latch `APPROACH` for `~1.0s`

This is not a release-policy promotion. It is an observability / behavior-stabilization probe so that a re-arm is long enough to matter and long enough to appear in logs.

## Probe config

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `START_RVIZ=false`

## Focused runs

### `20260425_013124_px4_sih`

- classification: `geometry-fail`
- phase sequence:
  - `IDLE -> APPROACH -> TRACKING -> APPROACH -> TRACKING`
- result:
  - **guard path is now definitely observable online**
  - but this run still failed later

Key summary:

- `start_t_sec = 33.10`
- `post_start_path_length_m = 1610.561`
- `final_pass = 0`

### `20260425_013558_px4_sih`

- classification: `final-pass`
- phase sequence:
  - `IDLE -> APPROACH -> TRACKING -> APPROACH -> TRACKING -> DOCKING -> ... -> COMPLETED`
- result:
  - **guard path is again visible**
  - and this time the run still converted to `final-pass`

Key summary:

- `start_t_sec = 33.24`
- `post_start_path_length_m = 320.610`
- `docking_path_length_m = 236.492`
- `final_pass = 1`

## Main conclusion

This resolves one important uncertainty:

- the early-`TRACKING` re-arm path was not just “theory”
- the missing visibility problem was real
- adding an `APPROACH` hold makes the re-arm observable and behaviorally meaningful

But this does **not** mean the policy is ready:

- one triggered run still failed
- one triggered run passed
- so the next question is now:
  - **when** should the re-arm trigger,
  - not whether the system can represent that transition

## Practical next step

Run a small triggered batch and compare:

- trigger count
- `final-pass`
- post-start path length
- whether triggered runs improve relative to the old long-tail family

At this point, the main blocker has shifted from “guard visibility” to **guard usefulness / trigger quality**.
