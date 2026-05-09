# `terminal_smoke` tracking-family analysis (`2026-04-24`)

## Goal

Explain why some `terminal_smoke` runs still become long-tail `geometry-fail` even when the accepted release window no longer looks obviously early/dirty.

## Runs compared

- fail:
  - `20260424_220319_px4_sih`
  - `20260424_221224_px4_sih`
- pass:
  - `20260424_220951_px4_sih`
  - `20260424_220717_px4_sih`
  - `20260424_221627_px4_sih`

## Main finding

The remaining bad family is **not** “never reaches TRACKING”.

It is:

- release accepted
- `APPROACH -> TRACKING` happens normally
- but inside `TRACKING`, the carrier never converts the **behind-entry band** into a low-lateral terminal geometry
- the closest terminal point then happens with `along > 0`, so the controller never satisfies the first passive `TRACKING -> DOCKING` entry conditions

In short:

- old problem: bad release family
- current residual problem: **TRACKING corridor collapse quality**

## Evidence

### Fail family

#### `20260424_220319_px4_sih`

- `tracking_nearest_terminal_distance_m = 1.190`
- `tracking_nearest_along_error_m = +0.376`
- `tracking_nearest_lateral_abs_m = 1.094`
- `tracking_behind_best_terminal_distance_m = 2.909`
- `tracking_behind_best_lateral_abs_m = 2.348`
- `docking_entry_count = 0`

#### `20260424_221224_px4_sih`

- `tracking_nearest_terminal_distance_m = 1.653`
- `tracking_nearest_along_error_m = +0.306`
- `tracking_nearest_lateral_abs_m = 1.550`
- `tracking_behind_best_terminal_distance_m = 2.635`
- `tracking_behind_best_lateral_abs_m = 2.525`
- `docking_entry_count = 0`

### Pass family

#### `20260424_220951_px4_sih`

- `tracking_nearest_terminal_distance_m = 0.452`
- `tracking_nearest_along_error_m = +0.172`
- `tracking_nearest_lateral_abs_m = 0.001`
- `tracking_behind_best_terminal_distance_m = 3.040`
- `tracking_behind_best_lateral_abs_m = 0.311`
- `docking_entry_count = 5`

#### `20260424_220717_px4_sih`

- `tracking_nearest_terminal_distance_m = 1.347`
- `tracking_nearest_along_error_m = -0.398`
- `tracking_nearest_lateral_abs_m = 1.073`
- `tracking_behind_best_terminal_distance_m = 2.954`
- `tracking_behind_best_lateral_abs_m = 1.173`
- `docking_entry_count = 3`

#### `20260424_221627_px4_sih`

- `tracking_nearest_terminal_distance_m = 0.612`
- `tracking_nearest_along_error_m = +0.076`
- `tracking_nearest_lateral_abs_m = 0.530`
- `docking_entry_count = 1`

This run shows a special short-family case: it goes into `DOCKING` quickly enough that the “best behind-band row” is less informative than the nearest-tracking geometry itself.

## Interpretation

- accepted-window metrics still overlap too much to cleanly separate the remaining fail family
- the sharper separator is now **inside TRACKING**:
  - fail runs do get many “behind-band” rows
  - but while still behind, their lateral error only collapses to about `2.35–2.52m`
  - pass runs can collapse that quantity to about `0.31–1.17m`, or else reach a very small nearest-tracking lateral error quickly

## Practical next step

Do **not** promote another accepted-window threshold yet.

Instead, the next probe should target a metric like:

- “best lateral abs while still in first-entry behind band”
- or “nearest tracking terminal geometry before first docking entry”

This keeps the mainline aligned:

- still no terminal-controller retune yet
- first isolate a stable discriminator for the remaining long-tail family

## Repo update

To make this reusable in future batches, batch summary now exports:

- `tracking_nearest_*`
- `tracking_behind_rows`
- `tracking_behind_best_*`

These are computed from `TRACKING` rows only and are intended for smoke-family diagnosis, not as final acceptance metrics.
