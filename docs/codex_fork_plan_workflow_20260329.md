# Codex Fork + Plan Workflow

This document fixes the project workflow for the PX4 SIH heterogeneous docking stack.

## Main Thread

Responsibility:
- Keep the current best non-regressive version.
- Maintain the long-term roadmap.
- Accept only changes that improve reproducibility or observability.

Current best reference runs:
- `results/20260329_155734_px4_sih`: `min_distance_m=0.200`, `final_phase=COMPLETED`
- `results/20260329_155943_px4_sih`: `min_distance_m=1.270`, near-success baseline

Main-thread rules:
- Do not blindly relax capture conditions.
- Do not change global speeds unless there is direct evidence.
- Revert any line that clearly regresses in the first 1-2 samples.
- Keep failure analysis split into `energy fail`, `geometry fail`, and `start-window fail`.

## Fork A: Start Geometry

Goal:
- Turn historical successful START geometries into a stable trigger cluster.

Scope:
- `scripts/wait_for_docking_window.py`
- `src/easydocking_control/scripts/auto_start_docking.py`
- `scripts/start_docking_command.sh`
- `scripts/run_px4_sih_docking_experiment.sh`

Success criteria:
- START falls near the successful geometry cluster.
- First docking geometry becomes more consistent across runs.

## Fork B: Geometry / Terminal Closeout

Goal:
- Improve `corridor release`, `first docking`, and the terminal closeout from `1.27 m` toward `0.5 m` and `0.2 m`.

Scope:
- `src/easydocking_control/src/docking_controller.cpp`
- `src/easydocking_control/include/easydocking_control/docking_controller.hpp`

Primary reference:
- `results/20260329_155943_px4_sih`

Success criteria:
- Stable `< 2 m` first.
- Then push repeatable `COMPLETED`.

## Fork C: Energy Fail Isolation

Goal:
- Prevent fixed-wing post-start bad energy states from contaminating docking evaluation.

Scope:
- `src/easydocking_control/scripts/px4_fixed_wing_bridge.py`
- `src/easydocking_control/scripts/experiment_logger.py`

Current instrumentation:
- `mini_energy_guard_active`
- `mini_energy_guard_bad_airspeed`
- `mini_energy_guard_bad_underspeed`
- `mini_energy_guard_true_airspeed_mps`
- `mini_energy_guard_underspeed_ratio`
- `mini_energy_guard_recover_time_remaining_sec`

Success criteria:
- Failed runs can be cleanly labeled as `energy fail` or not.
- Bad energy states trigger recovery instead of silent degradation.

## Fork D: Auto Analysis and Report

Goal:
- Stop relying on manual log inspection.

Scope:
- Result classification scripts
- Report generation scripts
- Plot generation scripts

Target outputs:
- Trajectory plots
- Distance convergence plots
- Phase timeline
- Energy status plots
- Automatic failure labels

## Long-Term Roadmap

### Layer 1: Stable PX4 SIH Reproduction
- Convert occasional `0.2 m` runs into high-reproducibility `0.2 m / COMPLETED`.
- Stabilize the controller and state machine in SIH first.

### Layer 2: Failure Classification and Auto Debug
- Automatically tag failures as:
  - `energy fail`
  - `geometry fail`
  - `start-window fail`

### Layer 3: Report-Ready Outputs
- Automatically generate:
  - trajectory figures
  - convergence figures
  - phase figures
  - energy figures

### Layer 4: Migration Toward Real Task Semantics
- Keep moving toward the real heterogeneous "airborne carrier" task.
- Transition from "simulation can dock once" to "control logic is explainable, stable, and transferable".

## Immediate Next Actions

1. Use historical successful and near-success runs to define a START geometry cluster.
2. Keep `energy_guard` active and continue separating `energy fail` from `geometry fail`.
3. Use `results/20260329_155943_px4_sih` as the main terminal-closeout reference.
4. Keep only non-regressive changes in the main thread.
