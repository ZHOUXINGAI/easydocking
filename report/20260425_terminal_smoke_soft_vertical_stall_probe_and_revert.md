# 2026-04-25 terminal_smoke soft-vertical-stall retry probe and revert

## Goal

After the narrow pred-margin release fix, one residual fail remained:

- `20260425_141103_px4_sih`
- accepted release looked healthy
- but the run still fell into a `TRACKING -> DOCKING -> TRACKING` long-tail geometry fail

The question was whether a **very narrow passive `DOCKING -> TRACKING` retry trigger** could recover that family earlier, without broadening the controller too much.

## Probe

Added a narrow retry condition in `src/easydocking_control/src/docking_controller.cpp`:

- name: `passive_retry_from_soft_vertical_stall`
- intended geometry slice:
  - `distance > 2.2`
  - `distance > min_docking_distance_ + 1.10`
  - `relative_position.z() > 0.78`
  - `abs(terminal_lateral_error) < 0.65`
  - `terminal_along_error < -2.0`
  - `relative_speed < 0.60`
  - `abs(relative_velocity.z()) < 0.35`

Intent:

- catch the case where terminal geometry looks nearly aligned laterally,
- but closure stalls high / behind,
- and force one earlier rebuild instead of waiting for a later timeout-like relapse.

## Validation batch

Common baseline stayed on:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`

Runs:

- `20260425_160512_px4_sih`
- `20260425_160719_px4_sih`
- `20260425_160915_px4_sih`
- `20260425_161152_px4_sih`
- `20260425_161425_px4_sih`

Headline result:

- `final-pass = 4/5`

## Why this was rejected

The raw pass rate was not enough. The trigger **over-fired** in later retries and stretched path shape:

- representative bad run: `20260425_161425_px4_sih`
- observed phase family:
  - `IDLE -> APPROACH -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> TRACKING -> DOCKING`
- repeated retry times clustered around:
  - `59.7s`
  - `69.16s`
  - `78.56s`

These later retries match the new trigger’s geometry too closely:

- `distance ≈ 2.4–3.3m`
- small lateral error
- high `rel_z`
- low relative speed

So the new clause is not just rescuing the targeted family once. Its early first retrack appears to push the run into a repeated later timeout-loop family, and the terminal leg gets longer instead of cleaner.

That is why this was treated as a bad mainline direction even though the raw `4/5` headline did not collapse.

## Decision

- revert `passive_retry_from_soft_vertical_stall`
- keep the already-proven release-side default improvement
- keep the previously validated first-entry re-arm infrastructure only
- do **not** promote a new broad `DOCKING` retry clause into mainline

## Practical conclusion

This probe answered the question:

- yes, the controller can be made to retry earlier in that geometry slice
- but this particular trigger is too permissive and harms terminal monotonicity

So the next step should be narrower:

- compare residual fail `20260425_141103_px4_sih` against successful retry/pass runs such as:
  - `20260425_140622_px4_sih`
  - `20260425_141710_px4_sih`
- identify which **existing** retry clause should trigger slightly earlier,
- instead of adding another generic `DOCKING` retry family.
