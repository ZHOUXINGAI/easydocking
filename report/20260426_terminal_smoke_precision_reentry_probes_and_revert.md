# 2026-04-26 terminal_smoke precision-reentry controller probes and revert

## Goal

After promoting the release-side `0.74 / 2.00` reject defaults, the next suspected bottleneck was:

- repeated late `DOCKING` entries
- long `docking_path_length_m` tails such as `20260426_004230_px4_sih`

The working hypothesis was that some of those tails were caused by **bad late precision re-entry timing** rather than the release window itself.

Two narrow controller-side probes were tried against that hypothesis.

## Baseline

Common config for all runs in this note:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.74`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=2.00`
- `START_RVIZ=false`

## Probe A — tighter `passive_precision_reentry_window`

Change idea:

- tighten the later precision re-entry window so it cannot jump back into `DOCKING`
  from very ahead / fast / barely-settled geometry

Main constraints added in that probe:

- extra `TRACKING` settle time
- tighter distance / lateral / `rel_z`
- tighter `relative_speed` / along-speed gate

Validation batch:

- `20260426_024626_px4_sih` → `final-pass`
- `20260426_024827_px4_sih` → `start-window-fail`
- `20260426_025226_px4_sih` → `geometry-fail`
- `20260426_025625_px4_sih` → `final-pass`
- `20260426_025917_px4_sih` → `final-pass`

Result:

- `final-pass = 3/5`
- `start-window-fail = 1/5`
- `geometry-fail = 1/5`

Why it was rejected:

- this was a clear regression from the current `5/5` release-side baseline
- the fail `20260426_025226_px4_sih` showed that the tighter gate blocked too much useful controller recovery and left a long `TRACKING` tail (`docking_path_length_m = 1114.975`)

Decision:

- **revert**

## Probe B — early `DOCKING` retrack on late ahead-runaway

Change idea:

- do not block re-entry itself
- instead, if a later `DOCKING` entry quickly runs away into a clearly bad
  “too far ahead + high `z` + small lateral” shape, retrack early instead of
  spending the full passive timeout budget

Validation mini-batch:

- `20260426_030304_px4_sih` → `geometry-fail`
- `20260426_030531_px4_sih` → `final-pass`
- `20260426_030740_px4_sih` → `final-pass`

Result:

- `final-pass = 2/3`
- `geometry-fail = 1/3`

Why it was rejected:

- the fail `20260426_030304_px4_sih` still accumulated `5` docking entries and ended as a late `DOCKING` geometry fail
- so the new early-retrack clause did not isolate the real root cause
- keeping it would add controller churn without a validated win

Decision:

- **revert**

## Combined interpretation

The evidence from both failed probes is useful:

- the remaining long-tail family is **not** explained well enough by “late precision re-entry happens too early”
- tightening or early-aborting that path directly causes regressions before it yields a stable improvement
- some residual bad runs already look unhealthy from:
  - first `DOCKING` entry geometry, or
  - first timeout / second-entry geometry,
  not only from `entry3+` precision re-entry

## Current code state

Both controller probes in this note were reverted.

The kept mainline baseline remains:

- release-side defaults:
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.74`
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=2.00`
- controller-side:
  - keep the first passive-timeout → second-entry retry patch
  - keep the rearm instrumentation

## Next step

The next useful step is not more blind re-entry threshold churn.

It should compare:

- fail `20260426_025226_px4_sih`
- fail `20260426_030304_px4_sih`
- short pass `20260426_024626_px4_sih`
- short pass `20260426_025917_px4_sih`

with focus on:

- first-entry `DOCKING` overshoot shape
- timeout-to-second-entry geometry
- whether the real blocker is along-control behavior inside `DOCKING`, not the re-entry gate itself
