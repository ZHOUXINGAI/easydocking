# Resume From 2026-03-29

## Why this file exists

If the user restarts Codex to switch models, the next session should continue from here without re-discovering the whole project.

## Model-switch note

- The user's `~/.codex/config.toml` is set to `gpt-5.2` / `gpt-5.2-codex`.
- But the current VS Code-originated session was created as `gpt-5.4`.
- Restarting the session is required if the user wants the next Codex run to actually use `5.2`.
- This does not lose project state, because the current engineering state is captured in files and result directories.

## Current project goal

Keep pushing the PX4 SIH heterogeneous docking stack toward more reproducible `COMPLETED`, with long-term target of stable `0.2 m`.

Current working line:

1. Do not touch global speeds unless there is direct evidence.
2. Do not use crude squeeze hacks.
3. Keep using automated classification + batch summary.
4. Focus on `good START but downstream fail`.
5. Focus specifically on passive `corridor release / terminal handoff`.

## Important kept files

- [scripts/classify_px4_sih_result.py](/home/hw/easydocking/scripts/classify_px4_sih_result.py)
- [scripts/summarize_px4_sih_batch.py](/home/hw/easydocking/scripts/summarize_px4_sih_batch.py)
- [scripts/run_px4_sih_docking_experiment.sh](/home/hw/easydocking/scripts/run_px4_sih_docking_experiment.sh)
- [scripts/start_docking_command.sh](/home/hw/easydocking/scripts/start_docking_command.sh)
- [scripts/start_px4_sih_simulation.sh](/home/hw/easydocking/scripts/start_px4_sih_simulation.sh)
- [src/easydocking_control/src/docking_controller.cpp](/home/hw/easydocking/src/easydocking_control/src/docking_controller.cpp)
- [src/easydocking_control/include/easydocking_control/docking_controller.hpp](/home/hw/easydocking/src/easydocking_control/include/easydocking_control/docking_controller.hpp)
- [src/easydocking_control/scripts/experiment_logger.py](/home/hw/easydocking/src/easydocking_control/scripts/experiment_logger.py)
- [src/easydocking_control/launch/docking.launch.py](/home/hw/easydocking/src/easydocking_control/launch/docking.launch.py)
- [src/easydocking_control/launch/simulation.launch.py](/home/hw/easydocking/src/easydocking_control/launch/simulation.launch.py)

## What is already fixed

### 1. Auto-analysis / no more manual log scraping

- Batch summary now contains:
  - `start_cluster_min_score`
  - `first_docking_*`
  - `best_active_*`
  - `post_start_energy_bad_rows`
- Classification now better separates:
  - `completed`
  - `geometry-fail`
  - `start-window-fail`

Main table:

- [results/px4_sih_batch_summary.csv](/home/hw/easydocking/results/px4_sih_batch_summary.csv)

### 2. Controller-side corridor debug is now logged

New CSV columns exist:

- `controller_corridor_release_score`
- `controller_corridor_release_armed`
- `controller_corridor_release_accept_counter`
- `controller_completion_hold_counter`

These are essential for diagnosing:

- corridor never released
- corridor released too late / not at all
- terminal failed after release

### 3. Auto-start baseline was restored

The current defaults were restored toward the previously good line:

- `carrier_outside_angle_deg = -135`
- window starter defaults near:
  - `alignment >= 0.82`
  - `tca in [5.8, 7.2]`
  - `distance in [75, 120]`
  - `relative_speed >= 9`

Also:

- coarse geometry cluster gate is enabled in a loose form
- intended to block obviously bad STARTs, not over-constrain the good cluster

## Current controller change kept in tree

In [docking_controller.cpp](/home/hw/easydocking/src/easydocking_control/src/docking_controller.cpp):

- A narrow passive-DOCKING early retry path is currently kept.
- Semantics:
  - if passive docking has entered with corridor still active
  - and it begins to diverge early
  - and corridor release score remains very low
  - then return to `TRACKING` to reorganize the intercept, instead of waiting until much later

This is the only new controller-side behavioral change currently kept from the latest round.

## Latest representative runs

### Historical successful references

- [results/20260329_155734_px4_sih](/home/hw/easydocking/results/20260329_155734_px4_sih)
- [results/20260329_213812_px4_sih](/home/hw/easydocking/results/20260329_213812_px4_sih)
- [results/20260329_214823_px4_sih](/home/hw/easydocking/results/20260329_214823_px4_sih)

### Regressed bad-START evidence found this round

These showed the starter was releasing around `66 m`, which was wrong for this line:

- [results/20260329_222944_px4_sih](/home/hw/easydocking/results/20260329_222944_px4_sih)
- [results/20260329_223153_px4_sih](/home/hw/easydocking/results/20260329_223153_px4_sih)
- [results/20260329_223400_px4_sih](/home/hw/easydocking/results/20260329_223400_px4_sih)

### After restoring the good START geometry line

- [results/20260329_224811_px4_sih](/home/hw/easydocking/results/20260329_224811_px4_sih)
  - `start_cluster_min_score ≈ 0.848`
  - `first_docking_distance ≈ 2.799`
  - `best_active_distance ≈ 2.768`

### After adding the early retry path

- [results/20260329_231438_px4_sih](/home/hw/easydocking/results/20260329_231438_px4_sih)
  - `first_docking_distance ≈ 2.575`
  - `best_active_distance ≈ 2.563`
- [results/20260329_231649_px4_sih](/home/hw/easydocking/results/20260329_231649_px4_sih)
  - `first_docking_distance ≈ 2.745`
  - `best_active_distance ≈ 2.727`

These are still `geometry-fail`, but they are better than the recent `3.6 m` class failures and confirm the line is now back on a useful downstream regime.

## What did NOT work and was reverted

- A trial corridor lateral-velocity widening was tested and then removed.
- It did not show a clear advantage in the first fresh good-START samples.
- The tree should already be back off that line.

## Current diagnosis

### What is separated cleanly now

- `start-window-fail`
  - no START or bad window
- `good START but downstream fail`
  - START near the cluster
  - enters `DOCKING`
  - but lateral error remains too large
  - corridor release score often remains low

### Main bottleneck now

Still not global speed.
Still not a generic completion threshold issue.

Main bottleneck remains:

- after a good START,
- first docking often still has `terminal_lateral_error` around `2.4 ~ 2.7 m`,
- corridor release score stays weak,
- terminal handoff cannot collapse into capture.

## Practical filters for the next Codex

Use [results/px4_sih_batch_summary.csv](/home/hw/easydocking/results/px4_sih_batch_summary.csv).

Quick filter for "good START but downstream fail":

- `classification == geometry-fail`
- `start_cluster_min_score <= 3.0`
- `post_start_energy_bad_rows == 0`
- `first_docking_distance_m <= 3.2` or `best_active_distance_m <= 3.0`

Good recent samples in that sense include:

- `20260329_224811_px4_sih`
- `20260329_231438_px4_sih`
- `20260329_231649_px4_sih`

## Recommended next step

Do not switch topics.

Continue exactly here:

1. Keep the restored good START baseline.
2. Keep the classification + batch summary workflow.
3. Keep analyzing only good-START downstream failures.
4. Focus only on passive `corridor retained -> retry -> second intercept -> handoff`.
5. Avoid broad relaxations of capture/completion.
6. Avoid global speed retuning unless there is very direct evidence.

## Suggested prompt for the next Codex

Use this exact resume prompt if needed:

```text
From docs/resume_20260329_next_codex.md continue the PX4 SIH docking work.
Do not re-open the old problem broadly.
Keep the restored good START baseline.
Use results/px4_sih_batch_summary.csv to isolate good START but downstream fail samples.
Focus only on passive corridor release / terminal handoff.
Do not change global speeds.
Do not add crude squeeze hacks.
Start by comparing:
- results/20260329_224811_px4_sih
- results/20260329_231438_px4_sih
- results/20260329_231649_px4_sih
Then continue iterating from there.
```

## Long-term execution checklist (do not lose)

For the "lateral <= 0.2 m + vertical z-band (0.2~1.0 m above)" long-term plan with checkboxes:

- [docs/px4_sih_terminal_alignment_plan.md](/home/hw/easydocking/docs/px4_sih_terminal_alignment_plan.md)
