# 2026-04-25 relapse-`TRACKING` re-arm probe and pivot

## Goal

After the earlier `guard+hold` effectiveness batch, one remaining hypothesis was:

- maybe the residual long-tail family is not only the **first** `TRACKING` entry
- maybe some runs need a second rebuild opportunity after `DOCKING -> TRACKING`

This probe tested that idea, but only as a narrow experiment. The change was **not** promoted.

## Probe 1 — first-entry lateral-convergence gate

An initial attempt tightened first-entry `TRACKING -> DOCKING` admission by requiring better lateral convergence on the first passive entry.

### Result

- validation batch:
  - `20260425_130704 / 130923 / 131119 / 131522 / 131922`
- outcome:
  - `final-pass = 1/5`

### Interpretation

This was a clear regression. It blocked too many runs that should still be allowed to progress.

Decision:

- **reverted immediately**

## Probe 2 — narrow relapse re-arm branch

After reverting the entry-gate attempt, a narrower controller-side branch was tested:

- only after `DOCKING -> TRACKING`
- only on the first passive retry path
- only for a very specific geometry slice

## Batch A — baseline `terminal_smoke`

Runs:

- `20260425_132701`
- `20260425_132932`
- `20260425_133136`
- `20260425_133536`
- `20260425_133813`

### Result

- `final-pass = 3/5`

But the key point is **why**:

- the two fails (`133136`, `133813`) were both early release families:
  - `start_t_sec ≈ 26s`
  - phase sequence stayed at `IDLE -> APPROACH -> TRACKING`
- these runs never reached the relapse family that the new branch was meant to handle

Observed `TRACKING -> APPROACH` transitions in this batch were still only the old **first-entry** re-arm path, not the new relapse path.

## Batch B — `pred-margin` mini-batch to suppress early release noise

Runs:

- `20260425_134432`
- `20260425_134832`
- `20260425_135231`

Config additions:

- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.77`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.99`

### Result

- `start-window-fail = 2/3`
- `final-pass = 1/3`

And again, the new relapse branch still did **not** produce online evidence of:

- `DOCKING -> TRACKING -> APPROACH`

The only observed `TRACKING -> APPROACH` remained the pre-existing first-entry guard path.

## Main conclusion

This probe does **not** justify keeping the new relapse branch:

- it was never exercised online in the intended way
- recent poor batches were dominated by **release/start-window variability**, not by the targeted relapse family

## Decision

- revert the speculative relapse branch
- keep the previously validated **first-entry re-arm + hold** infrastructure only
- pivot the mainline focus back to:
  - `terminal_smoke` early release / start-window family control
  - not additional controller-side relapse logic yet

## Practical next step

The current blocker is again:

- some runs still start too early (`start_t ≈ 21–27s`)
- those runs never even reach the terminal family we were trying to refine

So the next efficient step is:

- go back to release-side accepted-window discrimination for the early-start family,
- then return to controller tuning after the release family is quiet enough.
