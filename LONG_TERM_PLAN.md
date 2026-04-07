# Long-Term Plan: easydocking (M3 → Final Docking)

Last updated: 2026-04-07

This file is the **single source of truth** for long-term progress.  
At the start of every session: read this file, then update the status checkboxes + “Notes/Next”.

## Current milestone snapshot
- [x] **M3 5/5 PASS** (hold-based semantic pass)
- [x] **FINAL_PASS implemented (v1)** (COMPLETED + tight terminal constraints + sustained hold)
- [x] **FINAL_PASS dual-track output** (`v1` strict + `loose` trend gate)
- [ ] **Terminal convergence not dependent on run duration**
- [ ] **Terminal error tightened** (<=0.1m class)
- [ ] **Regression harness** (N-run + top failure reasons)

## Workstream 1 — FINAL_PASS (start here)
Goal: avoid “M3 pass but not truly docked”. Make FINAL_PASS explicit and measurable.

**Definition (initial draft; adjust as needed):**
- Must reach `final_phase=COMPLETED`
- Must satisfy terminal constraints inside a short continuous window:
  - `abs(rel_x) <= 0.10m`
  - `abs(rel_y) <= 0.10m`
  - `rel_z ∈ [0.15, 0.45]`
  - `relative_distance <= 0.30m`
  - `relative_speed <= 0.40m/s`
  - Sustained for `>= 0.30s`

**How to configure thresholds**
- Environment variables (override defaults / profile):
  - `FINAL_PASS_PROFILE` (`v1` or `loose`)
  - `FINAL_PASS_XY_ABS_MAX_M`
  - `FINAL_PASS_Z_MIN_M`, `FINAL_PASS_Z_MAX_M`
  - `FINAL_PASS_DISTANCE_MAX_M`
  - `FINAL_PASS_REL_SPEED_MAX_MPS`
  - `FINAL_PASS_HOLD_MIN_SEC`
- `scripts/run_px4_sih_docking_experiment.sh` writes these into each run’s `metadata.txt` as `final_pass_*` for reproducibility.

**Tasks**
- [x] Add `FINAL_PASS` metrics to per-run `summary.txt` / report
- [x] Add `final_pass` + supporting columns to `results/px4_sih_batch_summary.csv`
- [x] Update `scripts/classify_px4_sih_result.py` to emit `final-pass` vs `completed-but-not-final`
- [ ] Decide & document FINAL_PASS thresholds (tight vs loose)

**Notes / Next**
- Next: tune thresholds so “final-pass” aligns with what we want physically (current v1 is intentionally strict).
- Current practice: treat `final-pass` as **v1 strict**; also watch `final_pass_loose` to track trend and speed up iteration.

## Workstream 2 — Make terminal convergence time-invariant
Goal: remove reliance on extending `EXPERIMENT_DURATION_SEC` to “finish shrinking XY”.

**Tasks**
- [ ] Make corridor→non-corridor transitions monotonic (no flip-flop near terminal window)
- [ ] Add “non-corridor minimum dwell time” once in-band (z + xy)
- [ ] Add a dedicated “terminal settle” sub-state before COMPELTED if needed

## Workstream 3 — Tighten terminal error (<=0.1m)
Goal: improve physical docking quality.

**Tasks**
- [ ] Add a tight terminal-frame PD window with strict rate/accel limits (avoid overshoot)
- [ ] Prevent premature `COMPLETED` if `|rel_x|/|rel_y|` still large
- [ ] Confirm no regressions in M3 hold metrics

## Workstream 4 — Regression harness (fast feedback)
Goal: every change is validated with a consistent automatic summary.

**Tasks**
- [ ] One command: run N experiments + summarize
- [ ] Output: pass rate + worst-case run IDs + failure reason histogram
- [ ] Record the exact env/config used for the batch
