# Long-Term Plan: easydocking (M0–M4 + Sim-to-Real)

Last updated: 2026-05-05

This file is the **single source of truth** for long-term progress.  
At the start of every session: read this file first, then update the checkboxes and the current focus.

## Current snapshot

**What is already true**
- [x] `M3_PASS` / `FINAL_PASS` reporting exists in the batch workflow
- [x] `prehold` start sequencing is back to the intended order: mini stable loiter first, then carrier release
- [x] pre-start carrier drift / small-circle behavior is removed
- [x] current default prehold runs can reach `final-pass`
- [x] controller-side `M1` far-field intercept now uses mini orbit plan instead of pure chase heuristic
- [x] release gate now checks the same global intercept selector first, with local predictor fallback to avoid `IDLE` regressions
- [x] current `prehold` release gate starts `5/5` in the latest smoke batch (no `start-window-fail`)
- [x] `terminal_smoke` now has a gate-based validation path that can repeatedly leave `IDLE` and reach `final-pass` without fixed `START_DELAY`

**What is still not solved enough**
- [ ] `M2` carrier-front consistency is not yet “all-run hard constraint”
- [ ] `M3` docking path is sometimes too long; carrier does not always抢位 early enough
- [ ] terminal completion quality is still the bottleneck after the shorter M1 path is achieved
- [ ] `terminal_smoke` is now usable, but it still falls into long-path families and is not yet a short-family-only probe
- [ ] sim-to-real robustness (wind / delay / noise / actuator realism) is not yet in the main validation gate

**Latest probe (2026-04-21 PM)**
- controller-side far-field shaping was tightened again, but the early left/up detour is still present in the bad family
- a hard min-after-orbit delay on the global primary release caused `start-window-fail` and was reverted
- the next M1 step remains: improve **release geometry selection**, not just post-release aggressiveness

**Latest probe (2026-04-22 AM)**
- global intercept candidate scoring now uses the carrier’s **reachable future point** in the mini tangent frame
- focused smokes entered the short-path family again (`post_start_path_length_m ≈ 106–112m`, `time_to_first_tracking_sec ≈ 5.1–5.4s`)
- the dominant bottleneck has shifted from release-family selection back to terminal / docking precision
- 5-run mini-batch stayed in the short-path family, so this release-shape fix is now the new baseline to preserve

**Latest probe (2026-04-22 PM)**
- terminal-side probe added near-field **ahead-gap bleed** and earlier **staged-target collapse** in `DOCKING`
- one short-family smoke (`20260422_012536_px4_sih`) reduced the terminal along miss into the `~0.7m` range, but still did not capture
- another run showed the staged target collapse can pull terminal `z` much closer to target, but along closure remained the limiting axis
- auto-start variability reappeared in the probe set, so terminal validation now needs a more deterministic smoke path
- follow-up probe added close-range `TRACKING` lateral-recover shaping and stronger near-terminal ahead-gap bleed
- one short-family run (`20260422_103540_px4_sih`) showed better `TRACKING -> DOCKING` handoff but still failed on terminal along drift
- one long-family run (`20260422_103942_px4_sih`) reached real `final-pass`, which means terminal control is now capable enough when the geometry eventually lines up
- fixed `START_DELAY` was confirmed to be a poor deterministic smoke path; the next validation helper should be gate-based, not time-based

**Latest probe (2026-04-23)**
- strict `terminal_smoke` with **global-primary-only** gating still produced `start-window-fail`, even after fixing the `180s` duration and orbit-progress override
- fixed two gate-path issues:
  - `rear_entry_elapsed_after_orbit_sec` is now computed correctly even when the energy timing gate is disabled
  - orbit-progress release now uses `abs(accumulated_angle)` and the profile can really override `min_orbit_progress_ratio`
- the practical smoke path is now:
  - `orbit_progress >= 0.25`
  - no secondary gate
  - no energy timing gate
  - tight local primary gate (`prediction_score_threshold=0.9`, `prediction_lateral_max_m=8.0`)
- validation runs:
  - `20260423_175711_px4_sih`: `final-pass`, `start_t_sec=26.48`, `first_completed_t_sec=49.18`
  - `20260423_180005_px4_sih`: `final-pass`, `start_t_sec=36.34`, `first_completed_t_sec=78.44`
- 5-run smoke batch:
  - `20260423_192026/192252/192443/192705/192918`
  - `final-pass=5/5`
  - start timing is now repeatable (`start_t_sec ≈ 33.2s`)
  - but path length is still bimodal:
    - short family `post_start_path_length_m ≈ 114–118m`
    - long family `post_start_path_length_m ≈ 318–397m`
- conclusion:
  - the deterministic terminal smoke blocker is resolved enough for controller work
  - the next improvement is to reduce smoke-path long-tail variability, not to go back to fixed-time start

**Latest probe (2026-04-24)**
- added a new **soft accepted-`rel_z` hook** in `scripts/wait_for_docking_window.py` so future smoke tuning can bias against low-`rel_z` release windows without another brittle hard threshold
- hardened reporting / classification against NUL-corrupted `docking_log.csv` rows:
  - `scripts/generate_report.py`
  - `scripts/classify_px4_sih_result.py`
  - `scripts/summarize_px4_sih_batch.py`
- recovered missing run `20260423_212151_px4_sih`:
  - `classification=geometry-fail`
  - `post_start_path_length_m=276.771`
  - `accepted_window_rel_z=2.110`
- first soft-tuning attempt for `terminal_smoke` (`min_rel_z_target=3.0`, `weight=1.0`) was rejected:
  - under the **correct carrier prehold baseline** (`carrier_activate_on_launch=true`, `carrier_idle_hover_altitude=29.4`), the observed candidate windows stayed around `rel_z ≈ 1.4–2.1`
  - the new term therefore behaved like an effective veto and did not produce a usable smoke release
- conclusion:
  - keep the new hook as infrastructure
  - keep `terminal_smoke` defaults neutral for now (`target=0.0`, `weight=0.0`)
  - the next discriminator pass must use more correct-baseline samples before re-enabling any `rel_z` bias
- follow-up correct-baseline batch (`20260424_181703 / 181928 / 182316 / 182519 / 182910`):
  - `final-pass = 2/5`
  - short family `post_start_path_length_m ≈ 175–201`
  - bad family `≈ 391–1676`
  - accepted-window local metrics still overlap heavily across good / bad runs:
    - `accepted_rel_z`
    - `accepted_phase_err_deg`
    - `accepted_ahead`
    - `accepted_pred_score`
  - one fail (`20260424_182910_px4_sih`) is visibly worse in accepted `pred_lat` / `pred_score`, but the other bad runs are not cleanly separable this way
- added early post-start route-shape instrumentation:
  - `post_start_10s_path_length_m`
  - `post_start_10s_net_displacement_m`
  - `post_start_10s_path_efficiency_ratio`
  - same for `20s`
- result:
  - these early-efficiency metrics help flag the worst long-tail family,
  - but they still do not cleanly separate all bad runs from the good family by themselves
- updated interpretation:
  - the main blocker is still **release-family selection**
  - current smoke accepts are still local-gate-like (`accepted_window_int_* = nan`)
  - the next M0/M1 step should bias on **future route family**, not another tiny local-state threshold
- follow-up probe added a **carrier prehold-ready gate** to the window starter:
  - `carrier_z` must be above a derived minimum altitude before `terminal_smoke` can accept
  - batch/reporting now records `start_prehold_ready` and accepted-window `prehold_ok`
  - new batch classification can separate `prehold-start-fail` from geometry failures
- valid-baseline batch after the prehold gate (`20260424_201705 / 202103 / 202504 / 202900 / 203256`):
  - `final-pass = 1/5`
  - all `5/5` runs are now prehold-valid (`start_prehold_ready=1`)
  - fail family is still very long-tail (`post_start_path_length_m ≈ 1615–1783`)
  - the single pass starts later (`36.64s`) than the four fails (`22.38–33.00s`)
  - `accepted_window_diag_route_ok=0` and `diag_route_rej=16` on all five runs, including the pass
- latest interpretation:
  - dirty prehold samples are no longer the blocker
  - the next M0/M1 step is to stop `terminal_smoke` from accepting the **too-early orbit-progress family**
  - current relaxed `diag_route_ok` is not yet a usable discriminator by itself
- focused timing probe with `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40` (`20260424_210449 / 210655 / 210911 / 211112 / 211511`):
  - `final-pass = 4/5`
  - `geometry-fail = 1/5`
  - all `5/5` remain prehold-valid
  - good family usually starts around `33.5–35.1s` with accepted `phase_err ≈ 92°` and tighter `pred_lat`
  - the remaining fail is still the early family (`start_t=22.86s`, `phase_err=84.7°`, `pred_lat=-4.97`)
- action taken:
  - promote `terminal_smoke` default `rear_entry_min_orbit_progress_ratio` from `0.25 -> 0.40`
- follow-up probes on top of the `0.40` baseline were both rejected:
  - pure `rear_entry_tca_min_sec=4.5` validation (`20260424_213249 / 213448 / 213642 / 214044 / 214302`) regressed to `final-pass = 3/5`
  - a smoke-only combo reject hook (`low tca + high speed_xy + large |pred_lat|`) was added as **neutral infrastructure**, but the first `5-run` probe (`20260424_220319 / 220717 / 220951 / 221224 / 221627`) also regressed to `final-pass = 3/5`
  - the combo gate did block some clearly bad early checks, but it did **not** improve the accepted-family outcome enough and introduced a later false-negative/long-tail batch
- focused family analysis after the rejected combo probe:
  - the remaining bad family is no longer just “bad accepted window”
  - representative fails (`20260424_220319`, `20260424_221224`) enter `TRACKING`, but never collapse the **behind-entry-band lateral error** enough to convert into `DOCKING`
  - fail samples still get many behind-band rows, but `tracking_behind_best_lateral_abs_m ≈ 2.35–2.52`
  - pass samples can reduce that quantity into about `0.31–1.17`, or else reach a very small nearest-tracking lateral error quickly
- instrumentation added:
  - batch summary / classification now export `tracking_nearest_*` and `tracking_behind_best_*`
  - these are for diagnosing remaining smoke families before touching controller tuning again
- additional accepted-window proxy probe (`prediction_score >= 0.77` and `diag_margin <= 1.99`) was also rejected:
  - `5-run` validation (`20260424_231342 / 231539 / 231940 / 232202 / 232443`) stayed at `final-pass = 4/5`
  - it did suppress early bad candidates, but often delayed release too much (`start_t ≈ 62–67s` in 3/5 runs)
  - one run still became a late long-tail `TRACKING` fail (`20260424_231539`)
- current mainline decision:
  - keep the new combo-reject hook and pred-margin reject hook available for future env-only probes
  - keep `terminal_smoke` defaults on the proven `orbit_progress=0.40` baseline only
- current interpretation:
  - this is a real mainline improvement, but not full closure
  - next step is to remove the last long-tail smoke family without losing the new `4/5` baseline
  - the next discriminator pass should likely use the new `TRACKING` family metrics, or an early-`TRACKING` re-arm policy, not another accepted-window-only threshold

**Latest probe (2026-04-25)**
- implemented a **controller-side early-`TRACKING` re-arm guard** as default-off infrastructure:
  - passive mode only
  - first pre-`DOCKING` `TRACKING` entry only
  - one re-arm max
  - action is `TRACKING -> APPROACH`, then rebuild the normal intercept
- current guard thresholds are the offline-tested two-branch rule:
  - Branch A (`5s`): `min_term < 3.0 && min_lat_abs > 1.6 && max_along > 1.0`
  - Branch B (`8s`): `min_term > 2.5 && min_lat_abs > 2.2 && max_along < 0.2`
- fresh correct-baseline validation batch:
  - `20260425_000334 / 000529 / 000748 / 000951 / 001157`
  - config stayed on:
    - `CARRIER_ACTIVATE_ON_LAUNCH=true`
    - `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
    - `AUTO_START_WINDOW_PROFILE=terminal_smoke`
    - `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
    - plus `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
  - result: `final-pass = 5/5`
  - all `5/5` remain prehold-valid
- important nuance:
  - this fresh batch is a **non-regression success**, but none of the `5/5` runs actually exercised the new re-arm path
  - so this is **not yet proof** that the guard fixed the residual fail family; it only proves the guard did not obviously hurt the correct baseline
- offline replay on the known historical slice still matches the intended target family:
  - hits:
    - `20260424_220319`
    - `20260424_221224`
    - `20260424_231539`
  - no false positives on representative passes:
    - `20260424_220951`
    - `20260424_220717`
    - `20260424_221627`
- current mainline decision:
  - keep the guard in repo as **default-off infrastructure**
  - do **not** promote it to default yet
  - next useful validation is either:
    - a larger fresh batch until the residual `TRACKING` fail family reappears, or
    - a targeted probe that makes the old fail family more likely so the guard path is actually exercised
- follow-up targeted validation on the same day:
  - `combo-reject + guard` batch (`20260425_002342 / 002617 / 003015 / 003413 / 003810`) regressed to:
    - `final-pass = 2/5`
    - `start-window-fail = 1/5`
    - `geometry-fail = 2/5`
  - `pred-margin + guard` batch (`20260425_004028 / 004254 / 004527 / 004757 / 005005`) reached:
    - `final-pass = 5/5`
  - but the key result across **all 10 targeted runs** is:
    - observed early-`TRACKING` `TRACKING -> APPROACH` re-arm count = `0/10`
- updated interpretation:
  - the current guard still looks safe enough as default-off infrastructure
  - but it is **not yet validated as effective**, because the actual guard path still has not been exercised online
  - the next step should be to retarget / widen the online `TRACKING` discriminator, or construct a more replay-like reproduction path for the old `220319 / 221224 / 231539` family
- follow-up after widening the online discriminator:
  - discovered a second issue: even when the guard likely fired, it could be too brief to appear in run artifacts
  - added a short `APPROACH` hold (`~1.0s`) after guard-triggered `TRACKING -> APPROACH`
- focused validation after the hold change:
  - `20260425_013124`: visible `TRACKING -> APPROACH -> TRACKING`, but still `geometry-fail`
  - `20260425_013558`: visible `TRACKING -> APPROACH -> TRACKING -> ... -> COMPLETED`, and `final-pass`
- updated interpretation:
  - the “guard invisibility” problem is resolved enough
  - the next blocker is no longer transition visibility, but **trigger quality / usefulness**
  - next validation should measure:
    - how often the guard triggers
    - whether triggered runs improve `final-pass` / path length versus the old long-tail family
- follow-up `5-run` effectiveness batch on the same baseline:
  - `20260425_020031 / 020254 / 020535 / 020925 / 021133`
  - result:
    - `final-pass = 4/5`
    - observed guard-trigger count = `2/5`
  - triggered runs:
    - both `2/2` reached `final-pass`
    - average `post_start_path_length_m ≈ 129.6`
    - average `docking_path_length_m ≈ 65.4`
  - non-triggered runs:
    - `final-pass = 2/3`
    - one run (`20260425_020535`) still fell into a very long geometry-fail family
    - average `post_start_path_length_m ≈ 573.6`
    - average `docking_path_length_m ≈ 504.0`
- updated interpretation:
  - the guard is now more than observability infrastructure; it shows **promising online usefulness**
  - but it still does **not** cover the whole residual fail space
  - the remaining bad family is now at least partly:
    - `TRACKING -> DOCKING -> TRACKING` relapse
    - not just late first-entry `TRACKING` long-tail
- current mainline decision:
  - keep the guard in repo as **default-off** infrastructure
  - keep the proven `terminal_smoke` baseline unchanged
  - next controller-side step should target the remaining uncovered family by either:
    - tightening `TRACKING -> DOCKING` admission, or
    - extending re-arm logic to the first post-`DOCKING` relapse into `TRACKING`
- follow-up controller probes on the same day were used to test that idea:
  - a first-entry lateral-convergence admission gate regressed badly (`20260425_130704 / 130923 / 131119 / 131522 / 131922`):
    - `final-pass = 1/5`
    - decision: **revert**
  - a narrower relapse-`TRACKING` re-arm branch was then tested, but the intended path was not exercised online:
    - baseline batch (`20260425_132701 / 132932 / 133136 / 133536 / 133813`) ended at `final-pass = 3/5`
    - the two fails were both early start families (`start_t ≈ 26s`) that never reached the targeted relapse path
    - a `pred-margin` mini-batch (`20260425_134432 / 134832 / 135231`) then regressed to:
      - `start-window-fail = 2/3`
      - `final-pass = 1/3`
    - across these follow-ups, there was still **no online evidence** of the intended new path:
      - `DOCKING -> TRACKING -> APPROACH`
- updated interpretation:
  - the relapse-side controller probe is **not yet justified** for mainline retention
  - recent bad batches are dominated again by **release / start-window variability**
  - the current blocker is not “one more controller branch”, but re-suppressing the early `start_t ≈ 21–27s` family
- current mainline decision:
  - keep only the previously validated **first-entry re-arm + hold** infrastructure
  - do **not** keep the speculative relapse re-arm branch
  - pivot the next step back to release-side accepted-window discrimination for the early-start family
- follow-up release-side probe used the existing smoke-only reject hook with a narrower discriminator:
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`
  - validation batch:
    - `20260425_140622 / 140902 / 141103 / 141457 / 141710`
    - `final-pass = 4/5`
  - important effect:
    - the old early-start family (`start_t ≈ 21–27s`) disappeared in this batch
    - accepted windows moved back to the healthier `start_t ≈ 31–37s` band, plus one late clean run
  - the remaining fail (`20260425_141103`) no longer looks like a release-family problem:
    - `start_t = 36.88`
    - `accepted_phase_err_deg = 92.9`
    - `accepted_pred_score = 0.70`
    - `accepted_diag_margin = 3.38`
    - but it still becomes a `TRACKING / DOCKING` long-tail geometry fail
- mainline action taken:
  - promote the narrow pred-margin reject values into the `terminal_smoke` default profile in
    `scripts/run_px4_sih_docking_experiment.sh`
- post-promotion sanity:
  - `20260425_142230`: anomalous `start-window-fail` where `start_command.log` stopped after orbit completion and never resumed normal `window_check` logging
  - `20260425_143114`: `final-pass`
- updated interpretation:
  - keep the new narrow pred-margin defaults
  - monitor the occasional silent starter timeout separately
  - with early-start release mostly suppressed again, the main bottleneck shifts back to the residual `TRACKING / DOCKING` long-tail family
- follow-up terminal-side probe on the same day tested a narrow passive `DOCKING` retry hook for the residual fail family:
  - new clause: `passive_retry_from_soft_vertical_stall`
  - target motivation: residual fail `20260425_141103` had a healthy accepted release, but still fell into `TRACKING -> DOCKING -> TRACKING`
  - validation batch:
    - `20260425_160512 / 160719 / 160915 / 161152 / 161425`
    - headline `final-pass = 4/5`
  - but deeper inspection rejected it:
    - `20260425_161425` showed repeated `DOCKING -> TRACKING` loops
    - retries clustered around `59.7s / 69.16s / 78.56s`
    - geometry matched the new trigger too broadly, so terminal path got stretched instead of cleaned up
- current mainline decision:
  - **revert** the speculative `passive_retry_from_soft_vertical_stall` clause
  - keep only controller branches that show clear online benefit without widening retry loops
  - next controller-side step should compare residual fail `20260425_141103` against successful retry/pass runs and tighten an **existing** retry cue earlier, not add another broad `DOCKING` retry family
- follow-up comparison did isolate a cleaner controller-side gap:
  - residual fail `20260425_141103` is not mainly “missing one more retry geometry clause”
  - it reaches the normal first `DOCKING` timeout retrack, but that path did **not** arm the already-existing second-entry retry state
  - so after timeout it fell back to stricter ordinary `TRACKING` re-entry logic, unlike nearby passes that re-entered `DOCKING` quickly
- mainline action taken:
  - on the **first passive `DOCKING` timeout retrack only**, arm the existing second-entry retry state:
    - `passive_retry_used_ = true`
    - `passive_retry_pending_second_entry_ = true`
    - reset `passive_retry_tracking_best_lateral_abs_`
  - this reuses the existing retry machinery instead of inventing another new `DOCKING` trigger family
- smoke validation after that patch:
  - `20260425_163614 / 163901 / 164133`
  - `final-pass = 3/3`
  - strongest evidence:
    - `20260425_164133` showed
      `DOCKING(entry1) -> TRACKING(timeout) -> DOCKING(entry2)` in about `0.44s`
  - interpretation:
    - positive online signal
    - but still only smoke-scale; next step is a larger `5-run` validation
- follow-up `5-run` validation on the same baseline:
  - `20260425_180156 / 180553 / 180755 / 181018 / 181220`
  - result:
    - `final-pass = 4/5`
    - `start-window-fail = 1/5`
    - no geometry-fail runs
  - key controller-side signal:
    - all `4/4` passing runs exercised the intended first-timeout recovery
    - first `DOCKING -> TRACKING` redock delays were consistently about `0.44–0.50s`
  - important remaining gap:
    - this patch fixes the first-timeout state-machine hole,
    - but it does **not** eliminate later long-tail looping by itself
    - representative long pass:
      - `20260425_180755`
      - `docking_entry_count = 5`
      - `post_start_path_length_m = 373.597`
- practical conclusion:
  - keep the patch
  - separate the next work into:
    - starter anomaly (`never_left_idle` / silent starter family)
    - later repeated `DOCKING -> TRACKING` long-tail cleanup
- follow-up controller-side comparison on the same day used:
  - long pass `20260425_180755`
  - cleaner pass `20260425_181220`
- new interpretation:
  - later `entry3+` long tails are not mainly a missing retry-state problem anymore
  - they are a **late passive endgame along-shaping** problem:
    - lateral is already small
    - but the carrier can still drift too far ahead again near the upper `z` band
- mainline action taken:
  - add a narrow late-entry ahead-bleed term inside `dockingPhaseControl()`
  - only active when:
    - `passive_docking_entry_count_ >= 2`
    - distance close
    - lateral small
    - `rel_z` high in-band
    - carrier still too far ahead
  - this only strengthens the existing along slowdown / floor shaping; it does **not** add a new retry branch
- smoke validation (`3-run`):
  - `20260425_183530 / 183728 / 183929`
  - `final-pass = 3/3`
  - pass averages:
    - `post_start_path_length_m ≈ 123.8`
    - `docking_path_length_m ≈ 60.0`
    - `docking_entry_count ≈ 1.67`
  - strongest signal:
    - two runs completed on `entry1`
    - the third run still needed relapses, but stayed far shorter than the old late-loop family
- current interpretation:
  - promising controller-side progress on the real M3 bottleneck
  - next required step is a `5-run` confirmation before treating this as a new baseline
- follow-up `5-run` confirmation on the same probe:
  - `20260425_184435 / 184633 / 184910 / 185135 / 185344`
  - result:
    - `final-pass = 4/5`
    - `geometry-fail = 1/5`
  - mixed outcome:
    - two runs were short / clean
    - but one long late-loop pass remained:
      - `20260425_184910`
      - `docking_entry_count = 6`
      - `docking_path_length_m = 367.237`
    - and one run never reached `DOCKING`:
      - `20260425_185344`
      - phase sequence stayed at `IDLE -> APPROACH -> TRACKING`
- mainline decision:
  - **revert** the speculative late-entry ahead-bleed patch
  - keep only the previously proven first-timeout second-entry patch
  - the next work should split the remaining fail space instead of stacking more broad terminal shaping:
    - `TRACKING`-only family (`20260425_185344`)
    - late-loop pass family (`20260425_184910`)

**Latest probe (2026-04-26 early AM)**
- added controller instrumentation to record whether the `TRACKING_ENTRY_REARM_GUARD` actually fires online:
  - `controller_tracking_rearm_guard_enabled`
  - `controller_tracking_rearm_guard_used`
  - `controller_tracking_rearm_last_trigger_code`
  - `controller_passive_docking_entry_count`
- instrumentation immediately separated the residual fail space into two real families:
  - early risky release that still ends in `TRACKING` only (`20260426_002707`)
  - later `DOCKING` geometry-fail family (`20260426_001227`)
- key observation from the early risky fail:
  - accepted window at `start_t=25.70s`
  - `phase_err=87.9°`
  - `pred_lat=-4.13m`
  - `pred_score=0.78`
  - `diag_margin=1.99m/s`
  - and the re-arm guard **did** fire online (`used=1`, `code=2`)
- implication:
  - the remaining early fail was not “guard never triggered”
  - it was still a release-family leak, so the next step stayed on the release side first
- follow-up env-only validation widened the smoke reject hook just enough to target that leak:
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.74`
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=2.00`
  - batch:
    - `20260426_003737 / 003950 / 004230 / 004522 / 004751`
    - `final-pass = 5/5`
- important result:
  - no run re-entered the old `start_t ≈ 21–27s` risky family
  - accepted windows stayed in healthier bands:
    - `start_t = 32.46–34.54s` for the normal family, or
    - `63.24–67.64s` for the later clean family
  - accepted geometry stayed materially better than the risky fail:
    - `phase_err = 90.3–93.6°`
    - `pred_lat = -3.15 .. -1.71m`
    - `diag_margin = 2.68–3.24m/s`
- additional note:
  - the re-arm guard fired online in all `5/5` passes (`used=1`, `code=2`)
  - so this batch is consistent with a cleaner split:
    - release-side reject removes the bad early family
    - existing controller recovery then handles the remaining terminal path cleanly enough
- mainline action taken:
  - promote the new reject defaults into `terminal_smoke` in
    `scripts/run_px4_sih_docking_experiment.sh`
- current interpretation:
  - this is the best terminal-smoke baseline so far after the first-timeout retry patch
  - next work should stay focused on the residual late terminal quality issues:
    - repeated `DOCKING` entries / long docking-path tails
    - not another release-threshold churn pass first
- follow-up controller-side attempts to directly clamp late precision re-entry were both rejected and reverted:
  - tighter `passive_precision_reentry_window` probe:
    - `20260426_024626 / 024827 / 025226 / 025625 / 025917`
    - result: `final-pass = 3/5`
    - regression reason:
      - one `start-window-fail`
      - one long `TRACKING` geometry fail (`20260426_025226`)
  - early `DOCKING` retrack on late ahead-runaway probe:
    - `20260426_030304 / 030531 / 030740`
    - result: `final-pass = 2/3`
    - regression reason:
      - `20260426_030304` still accumulated `5` docking entries and ended in `DOCKING` geometry-fail
- updated interpretation:
  - the remaining long-tail family is not explained well enough by “late precision re-entry happens too early”
  - some bad runs are already unhealthy from first-entry or timeout-to-second-entry geometry
  - next controller step should compare those earlier entry families directly instead of tightening re-entry thresholds again
- follow-up instrumentation added `controller_passive_docking_last_exit_trigger_code` and a fresh baseline batch was used to map real `DOCKING -> TRACKING` causes online:
  - `20260426_031737 / 032010 / 032248 / 032520 / 032757`
  - result: `final-pass = 5/5`
- key interpretation from that batch:
  - first-entry `code4` (`retry_from_corridor_stall`) is common even in short successful runs
  - so the residual long-tail family is **not** mainly a bad first-entry retrack policy
  - the real split is later, after second entry:
    - `code9` family: second-entry lateral divergence while already far ahead
    - `code3` family: second-entry ahead-timeout retrack
- updated next step:
  - compare long pass `20260426_031737` against short pass `20260426_032757`
  - compare longer pass `20260426_032010` against shorter pass `20260426_032520`
  - target second-entry `DOCKING` behavior directly, not release thresholds or first-entry gates
- follow-up validation on 2026-04-27 rejected the speculative narrow `code9` corridor-release hook and pivoted to a narrower second-entry `code3` fix:
  - first reference mini-batch (`20260427_015630 / 015847 / 020102`) showed:
    - `final-pass = 2/3`
    - one remaining `geometry-fail`
    - the active long family was again `entry1 -> code4 -> entry2 -> code3`
  - mainline action:
    - remove the unvalidated `code9`-only corridor-release hook
    - add a narrow **second-entry ahead-bleed** inside non-corridor `DOCKING`
      - passive mode only
      - `passive_docking_entry_count >= 2`
      - only when the carrier is already ahead and lateral/z are reasonably aligned
  - fresh validation batch (`20260427_020835 / 021044 / 021248`) reached:
    - `final-pass = 3/3`
    - all starts returned to the normal `~33.1–33.3s` band
  - important nuance:
    - `code3` still fired in all `3/3` runs
    - but about `4s` after second-entry corridor release the ahead gap was much smaller than before:
      - pre-patch bad family: along still around `-3.24m`, distance around `3.38–3.63m`
      - new batch: along around `-1.17 / -1.77 / -1.43m`, distance around `1.53 / 1.93 / 1.62m`
  - updated interpretation:
    - this is a real mainline improvement because it restores `3/3 final-pass` on the standard smoke baseline
    - but it does **not** close the second-entry `code3` family yet
    - the next step remains to reduce the `entry2 -> code3` retrack itself, or make the first post-`code3` re-entry finish directly without extra `entry4/entry5` loops

## North-star target

We want the system to behave like this:

1. `mini` enters stable loiter and completes one round  
2. `carrier` uses **global planned-trajectory prediction** to choose an active intercept point  
3. `carrier` flies a short, energy-efficient, same-direction抢位 path to the mini tangent/corridor  
4. close range switches to **local predictive terminal control**  
5. docking finishes with short path, low relative speed, stable hold, and reproducible `FINAL_PASS`

In plain terms:
- far field: **global plan-based intercept**
- mid field: **corridor / tangent tracking**
- near field: **local prediction / MPC-like terminal regulation**

## Three engineering routes

### Route 1 — Global prediction, no MPC

Use the shared mission reference directly.

**Core idea**
- `carrier` should not chase `mini` current position
- `carrier` should read `mini` planned trajectory / progress and choose a future intercept point

**Inputs**
- mini reference trajectory: `p_ref(t), v_ref(t), a_ref(t), yaw_ref(t)`
- current reference progress / phase / orbit state
- carrier current state and dynamic limits

**Output**
- a time-tagged intercept target or short rendezvous reference segment

**Why this is not MPC**
- the future mini path is already known
- the problem is mainly geometric / feasibility search, not full online optimal control

**Primary purpose**
- solve early active抢位
- shorten post-start path
- reduce “slow conservative merge” behavior

### Route 2 — Mid-course corridor tracking

Track the chosen intercept corridor instead of loosely chasing the target.

**Core idea**
- once Route 1 picks a rendezvous point / tangent segment, the controller should follow that corridor explicitly
- the carrier should stay in front / on the correct side until final merge

**Primary purpose**
- stabilize `M1` same-direction behavior
- stabilize `M2` front-consistency
- reduce long tracking tails and ugly S-turns

### Route 3 — Local prediction / terminal MPC

Only use predictive terminal control in close range.

**Core idea**
- inside the near field, use mini measured state plus short-horizon prediction
- control in terminal-relative coordinates instead of mission-global geometry

**Suggested state**
- along / lateral / z
- along_dot / lateral_dot / z_dot
- optionally heading mismatch

**Suggested horizon**
- about `1–2 s` short horizon

**Primary purpose**
- solve short-endgame oscillation
- reduce terminal wobble
- enforce low relative-speed capture quality

## Mapping to M0–M4

## M0 — Metrics / instrumentation / regression baseline

Goal: make the new architecture measurable before heavy controller changes.

- [x] terminal alignment / final-pass summary columns exist
- [ ] add **global intercept diagnostics**:
  - selected intercept time
  - selected intercept point
  - predicted mini tangent direction
  - carrier feasibility margin
- [ ] add **path-efficiency metrics**:
  - `post_start_path_length_m`
  - `time_to_corridor_sec`
  - `time_to_first_tracking_sec`
  - `time_to_completed_sec`
- [ ] add **front-shape metrics**:
  - signed along distance in mini tangent frame
  - duration carrier is behind / ahead
  - first front-consistent time
- [ ] save the chosen global intercept reference into each run’s artifacts

**M0 exit**
- every run can explain:
  - when carrier was released
  - what intercept it chose
  - whether it reached the corridor as planned
  - where path length was wasted

## M1 — Same-direction active intercept

Goal: the carrier should actively fly the correct shape immediately after release.

**Main issue this milestone solves**
- current system often starts correctly but still flies too conservatively after `START`
- the path is same-direction, but not aggressive enough in抢位

**Work**
- [x] Route 1 v1: build a plan-based intercept selector from mini planned trajectory
- [x] replace “window only” release geometry with “window + selected intercept candidate”
- [x] Route 2 v1: make `APPROACH` follow intercept corridor, not just lead/chase heuristics
- [ ] keep anti-reverse guard only as a safety layer, not as the main trajectory shaper

**M1 hard acceptance**
- [ ] no reverse-direction horizontal motion after `START`
- [ ] carrier initial path shape matches active intercept intent
- [ ] carrier reaches the mini tangent corridor without obvious conservative detour

## M2 — Front-consistency through docking

Goal: before docking completes, carrier should remain in the intended front / rendezvous geometry.

**Main issue this milestone solves**
- some runs still let mini overtake or create an ugly front/back swap window

**Work**
- [ ] make corridor progress monotonic once intercept is committed
- [ ] keep carrier-front constraints in the corridor frame, not only local velocity frame
- [ ] define explicit fallback / re-plan policy when the chosen intercept becomes infeasible
- [ ] add batch report for front-consistency pass/fail

**M2 hard acceptance**
- [ ] carrier front/back signed metric matches intended geometry through the docking approach
- [ ] no long negative-duration back-cross windows
- [ ] batch-level front-consistency is reproducible, not just single-run pretty cases

## M3 — Short endgame and short docking path

Goal: shorten the total docking leg and remove wasteful terminal path.

**Main issue this milestone solves**
- docking succeeds, but too late
- carrier sometimes enters terminal region from a poor geometric state, causing long tail

**Work**
- [ ] Route 1 v2: intercept scoring includes energy / path-length penalty
- [ ] Route 2 v2: earlier corridor capture when feasible
- [ ] Route 3 v1: local terminal predictive control for final `~20 m`
- [ ] monotonic transition: global intercept → corridor tracking → terminal local controller

**M3 hard acceptance**
- [ ] docking path length is consistently near the intended orbit-radius-scale target, not long-tail outliers
- [ ] terminal wobble is visibly reduced
- [ ] “good cases” are no longer late by construction

## M4 — Final-pass robustness and sim-to-real gate

Goal: turn “good-looking simulation” into a release-quality engineering gate.

**Main issue this milestone solves**
- current behavior is still tuned mainly for nominal simulation conditions

**Work**
- [ ] Route 3 v2: tighten terminal relative-speed / relative-acceleration behavior
- [ ] validate fixed-wing / multirotor heterogeneous envelope against realistic limits
- [ ] integrate wind / gust robustness from `PlanReview/Question/Q1.md`
- [ ] integrate timing delay / noise / actuator realism into batch validation
- [ ] define release gate: nominal batch + disturbed batch + failure reason histogram

**M4 hard acceptance**
- [ ] `FINAL_PASS` is reproducible under nominal conditions
- [ ] success remains acceptable under graded wind / gust disturbance
- [ ] logs support hardware-facing parameter limits and safety reasoning

## Sim-to-Real extension (after M1–M4 baseline is stable)

This is the next layer after the docking shape is correct.

### Q1. Wind / gust
- [ ] add graded steady wind + gust scenarios into the standard batch
- [ ] report 95/99 percentile terminal errors, not only mean/best
- [ ] verify whether global intercept selection shifts under wind bias

### Q2. Delay / sync / noise
- [ ] inject state delay and timestamp skew
- [ ] evaluate local predictor / terminal MPC sensitivity
- [ ] add delayed-state compensation if needed

### Q3. Dynamics realism
- [ ] check whether fixed-wing and carrier speed / accel / turn-rate envelopes are hardware-feasible
- [ ] move from “sim works” to “trajectory is physically executable”

## Execution order

Do not attack everything at once. The intended order is:

1. **M0 instrumentation**
2. **M1 global planned-trajectory intercept**
3. **M2 front-consistency**
4. **M3 short-endgame / local predictive terminal control**
5. **M4 disturbed-condition robustness**
6. **wind / delay / HIL / real-flight prep**

## Immediate next focus

**Latest probe (2026-05-05)**
- analyzed regressed batch `final-pass=2/5` (`20260504_205719/205759/205911`, `20260505_103558/105233`)
- bad run `20260505_105233`: TRACKING lateral oscillated +11m..-24m for 66s, entered DOCKING with 14m lateral error → 700m total path (spiral)
- good run `20260505_103558`: clean TRACKING 7.8s, DOCKING entry at 6m/1.5m lateral → 133m total path
- root cause 1: `passive_ready_to_enter_docking` bypassed lateral check when `tracking_horizontal_distance > 10.0`, allowing DOCKING entry with 14m lateral
  - fix: replaced bypass with proportional lateral cap `|lat| < min(7.0, max(1.6, 0.55*h_dist))`
- root cause 2: `tracking_lat_mid` boosted lateral P gain by 1.40x at close range (dist<12m, |lat|>0.25m), causing overshoot against mini's rotating velocity frame
  - fix: at close range (dist<8m), damp P gain from 1.40x→1.00x and boost D gain 1.25x→2.00x
- root cause 3: `TRACKING_ENTRY_REARM_GUARD_ENABLED` defaulted to `true` (should be default-off per plan)
  - fix: restored default to `false` in both run script and launch file
- added second-entry near-capture completion (exit code 11): when on entry 2+ and path budget expires, complete if geometry is "close enough" (dist<3.0m, |lat|<0.65, |z_err|<0.45, rel_speed<1.20)
- user defined 5 docking path expectations saved as permanent memory

Current recommended focus for the next sessions:

- [x] M3: fix DOCKING entry lateral gate (proportional cap replaces h_dist>10 bypass)
- [x] M3: damp TRACKING mid-range lateral P gain at close range to reduce oscillation
- [x] M3: add second-entry near-capture completion (code 11)
- [ ] M3: run 5-run validation on the combined changes
- [ ] M3: strengthen near-field along closure once `lat/z` are acceptable
- [ ] M3: clean up `TRACKING -> DOCKING` handoff so `DOCKING` does not start with large residual lateral error
- [x] M0: add a gate-based deterministic terminal smoke path so controller probes are not dominated by auto-start variability
- [ ] M0: tighten `terminal_smoke` so it prefers the short intercept family instead of long-tail completion paths
- [ ] M0: log or score accepted-time **future route family** so smoke release can distinguish short-family vs long-family windows
- [x] M3: run a `5-run` validation on the new first-timeout second-entry patch and check whether timeout-retrack cases now redock quickly without creating new late-loop families
- [ ] M0: investigate the occasional `never_left_idle` / silent starter anomaly under `terminal_smoke`
- [x] M3: compare long pass `20260425_180755` against cleaner pass `20260425_181220` and target the later repeated `DOCKING -> TRACKING` loop family
- [x] M3: run a `5-run` confirmation on the new late-entry ahead-bleed patch
- [ ] M3: compare `TRACKING`-only fail `20260425_185344` against short pass `20260425_184435` to isolate why some runs never collapse into `DOCKING`
- [ ] M3: compare long pass `20260425_184910` against short pass `20260425_184435` and identify whether the remaining long-tail is admission-side or DOCKING-side

## Notes

- Global prediction should be **plan-based**, not a large MPC.
- Local prediction should be **short-horizon**, and may use MPC if needed.
- The current heuristic prediction stack in:
  - `scripts/wait_for_docking_window.py`
  - `src/easydocking_control/scripts/px4_offboard_bridge.py`
  - `src/easydocking_control/src/docking_controller.cpp`
  is a useful baseline, but it is **not** the target final architecture.
