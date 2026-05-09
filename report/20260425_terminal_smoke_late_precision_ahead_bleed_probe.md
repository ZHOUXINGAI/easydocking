# 2026-04-25 terminal_smoke late-precision ahead-bleed probe

## Goal

After the first-timeout second-entry patch, the next remaining controller-side issue was:

- first timeout recovery became reliable,
- but some passing runs still fell into later repeated `DOCKING -> TRACKING` loops,
- with long terminal path even though lateral geometry was already small.

Representative comparison:

- long pass: `20260425_180755_px4_sih`
- cleaner pass: `20260425_181220_px4_sih`

## Observed later-loop pattern

In the long pass, later `DOCKING` entries (`entry3+`) were not failing because of large lateral miss anymore.

Instead they showed:

- very small lateral error
- relatively high `rel_z` near the top of the docking band
- carrier still too far ahead (`along ≈ -3m` class)
- then terminal geometry drifted outward again before timeout

Example from `20260425_180755_px4_sih`, `entry3`:

- around `61.10s`: `dist≈2.28`, `lat≈-0.22`, `along≈-2.09`, `z≈0.88`
- but later in the same entry it drifts back out:
  - around `63.90s`: `dist≈3.39`, `along≈-3.25`, `z≈0.95`

So the issue looked less like “re-enter too early” and more like:

- after late re-entry, the carrier can still creep too far ahead again in the endgame.

## Patch

File:

- `src/easydocking_control/src/docking_controller.cpp`

Change:

- add a narrow `terminal_late_precision_ahead_bleed_blend` in `dockingPhaseControl()`
- activate only when:
  - `passive_docking_entry_count_ >= 2`
  - terminal distance is already close
  - lateral error is already small
  - `rel_z` is high in-band / near the upper side
  - carrier is still significantly ahead

Then feed that blend into the existing ahead-bleed slowdown / floor logic:

- stronger cap on along velocity
- slightly lower along floor in that specific late-entry precision family

This is intentionally **not** a new retry trigger.

It only shapes along closure inside late passive `DOCKING`.

## Build

- `colcon build --packages-select easydocking_control`
- result: success

## Smoke validation (`3-run`)

Common baseline:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`

Runs:

- `20260425_183530_px4_sih`
- `20260425_183728_px4_sih`
- `20260425_183929_px4_sih`

## Result

- `final-pass = 3/3`
- no `start-window-fail`
- no `geometry-fail`

Per-run:

| run | classification | post_start_path_length_m | docking_path_length_m | docking_entry_count | note |
|---|---|---:|---:|---:|---|
| `20260425_183530_px4_sih` | `final-pass` | `145.094` | `72.365` | `1` | direct `entry1 -> COMPLETED` |
| `20260425_183728_px4_sih` | `final-pass` | `79.519` | `24.293` | `1` | very short clean run |
| `20260425_183929_px4_sih` | `final-pass` | `146.800` | `83.398` | `3` | still some relapses, but much shorter than old late-loop family |

Pass averages:

- `post_start_path_length_m ≈ 123.8`
- `docking_path_length_m ≈ 60.0`
- `docking_entry_count ≈ 1.67`

## Why this looks promising

Compared with the previous first-timeout patch `5-run` batch:

- old pass-only average `docking_path_length_m ≈ 196.9`
- this probe gives `≈ 60.0` on the `3-run` smoke sample

And the worst run in this probe:

- `20260425_183929_px4_sih`
- `docking_entry_count = 3`
- `docking_path_length_m = 83.398`

is still much cleaner than the earlier long-tail pass family such as:

- `20260425_180755_px4_sih`
- `docking_entry_count = 5`
- `docking_path_length_m = 294.903`

## Current interpretation

- this is the first controller-side change after the first-timeout patch that appears to improve **later-entry terminal monotonicity**
- it is nicely scoped:
  - no new retry family
  - no broad gate churn
  - only late passive endgame along-shaping

But it is still only `3-run` scale.

## Next step

- run a `5-run` confirmation batch on this exact patch
- verify whether:
  - short-family behavior persists
  - `docking_entry_count` stays low
  - no silent starter anomaly reappears
