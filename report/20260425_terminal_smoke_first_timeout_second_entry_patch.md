# 2026-04-25 terminal_smoke first-timeout second-entry patch

## Goal

After reverting the speculative `soft_vertical_stall` branch, the next question was:

- why did residual fail `20260425_141103_px4_sih` still miss recovery,
- even though nearby passes were already using existing retry infrastructure?

## Root-cause comparison

Compared runs:

- fail: `20260425_141103_px4_sih`
- pass: `20260425_140622_px4_sih`
- pass: `20260425_141710_px4_sih`

Observed difference:

- `141103` enters `DOCKING` once, stays there for the full `~8.5s` passive timeout budget, then falls back to `TRACKING`
- but that fallback is the plain timeout path, **not** the existing “second-entry retry” path

So after the timeout, `141103` loses the more permissive retry re-entry logic and goes back to ordinary `TRACKING` re-entry rules.

By contrast, the nearby passes get into a real retry cycle:

- `140622`: first `DOCKING -> TRACKING` at `76.48s`, then back to `DOCKING` at `77.44s`
- `141710`: first `DOCKING -> TRACKING` at `48.20s`, then back to `DOCKING` at `48.76s`

## Why existing retry clauses did not catch `141103` earlier

Late in the first `DOCKING` entry of `141103` (`~56.7–57.1s`), the geometry is:

- `distance ≈ 2.8–3.1m`
- `lat ≈ 0.35–0.47m`
- `along ≈ -2.7 to -2.9m`
- `rel_z ≈ 0.92–0.95m`
- `relative_speed ≈ 0.51–0.57m/s`
- corridor inactive

This misses the current early retry families:

- `corridor_stall`: expects corridor-active + larger lateral error
- `first_entry_stall`: expects `|lat| > 1.0` and larger along miss
- `vertical_regression`: expects larger lateral error
- `front_loss`: expects positive along error

So `141103` is not “one more obvious retry case”. It is a **first-entry timeout** case that should still preserve retry intent.

## Patch

File:

- `src/easydocking_control/src/docking_controller.cpp`

Change:

- when passive `DOCKING` hits the normal timeout retrack path,
- and it is still the **first** passive docking entry,
- arm the already-existing retry state:
  - `passive_retry_used_ = true`
  - `passive_retry_pending_second_entry_ = true`
  - reset `passive_retry_tracking_best_lateral_abs_`

This does **not** add a new retry geometry family.

It reuses the existing second-entry retry machinery that the controller already knows how to handle.

## Build

- `colcon build --packages-select easydocking_control`
- result: success

## Smoke validation

Explicit baseline used:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`

Runs:

- `20260425_163614_px4_sih` → `final-pass`
- `20260425_163901_px4_sih` → `final-pass`
- `20260425_164133_px4_sih` → `final-pass`

Most useful evidence:

- `20260425_164133_px4_sih`
  - `TRACKING -> DOCKING(entry1)` at `48.34s`
  - `DOCKING(entry1) -> TRACKING` at `56.90s`
  - `TRACKING -> DOCKING(entry2)` at `57.34s`
  - final `COMPLETED` at `73.04s`

That `0.44s` timeout-to-second-entry handoff is the exact path this patch was meant to recover.

## Current interpretation

- this is a better-shaped mainline step than the reverted `soft_vertical_stall` branch
- it fixes a state-machine gap instead of introducing a new broad geometry trigger
- online evidence is positive, but still only smoke-scale

## Next step

- run a larger `5-run` validation on this patch
- specifically watch whether:
  - first-entry timeout cases now re-enter `DOCKING` quickly,
  - `TRACKING` long-tail families shrink,
  - and path length does not regress into repeated late timeout loops
