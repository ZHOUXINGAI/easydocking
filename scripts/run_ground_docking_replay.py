#!/usr/bin/env python3
"""Run deterministic, hardware-free two-rover leader replays."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import FrozenInstanceError
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ground_corridor_geometry import (  # noqa: E402
    add,
    compute_ground_corridor_plan,
    external_tangent_candidates,
    scale,
    tangent_direction,
)
from ground_docking_leader import (  # noqa: E402
    CommandWatchdog,
    GroundDockingLeader,
    LeaderConfig,
    LeaderPhase,
    VehicleState,
)


class ReplayHarness:
    def __init__(self, config: LeaderConfig | None = None) -> None:
        self.config = config or LeaderConfig()
        self.leader = GroundDockingLeader(self.config)
        self.now_ms = 0
        self.sender_ms = 1_000_000
        self.carrier_sequence = 0
        self.mini_sequence = 0
        self.phases: list[str] = []

    def _state(
        self,
        *,
        vehicle_id: int,
        sequence: int,
        position: tuple[float, float],
        velocity: tuple[float, float],
        sender_ms: int | None = None,
    ) -> VehicleState:
        return VehicleState(
            vehicle_id=vehicle_id,
            sequence=sequence,
            sender_monotonic_ms=(
                self.sender_ms if sender_ms is None else sender_ms
            ) & 0xFFFFFFFF,
            received_local_ms=self.now_ms,
            frame_id=self.config.frame_id,
            origin_id=self.config.origin_id,
            position=position,
            velocity=velocity,
            yaw_rad=math.atan2(velocity[1], velocity[0])
            if math.hypot(*velocity) > 1.0e-9
            else 0.0,
        )

    def feed(
        self,
        *,
        carrier_position: tuple[float, float],
        carrier_velocity: tuple[float, float],
        mini_position: tuple[float, float],
        mini_velocity: tuple[float, float],
        dt_ms: int = 100,
    ):
        self.carrier_sequence = (self.carrier_sequence + 1) & 0xFFFFFFFF
        self.mini_sequence = (self.mini_sequence + 1) & 0xFFFFFFFF
        self.leader.accept_carrier_state(
            self._state(
                vehicle_id=1,
                sequence=self.carrier_sequence,
                position=carrier_position,
                velocity=carrier_velocity,
            )
        )
        self.leader.accept_mini_state(
            self._state(
                vehicle_id=2,
                sequence=self.mini_sequence,
                position=mini_position,
                velocity=mini_velocity,
            )
        )
        snapshot = self.leader.step(
            now_local_ms=self.now_ms,
            sender_monotonic_ms=self.sender_ms,
        )
        if not self.phases or self.phases[-1] != snapshot.phase.value:
            self.phases.append(snapshot.phase.value)
        self.now_ms += dt_ms
        self.sender_ms = (self.sender_ms + dt_ms) & 0xFFFFFFFF
        return snapshot


def orbit_state(
    config: LeaderConfig,
    phase_rad: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    position = (
        config.orbit_center[0] + config.orbit_radius_m * math.cos(phase_rad),
        config.orbit_center[1] + config.orbit_radius_m * math.sin(phase_rad),
    )
    direction = tangent_direction(phase_rad, config.turn_direction)
    velocity = scale(direction, config.mini_speed_mps)
    return position, velocity


def advance_to_terminal(
    harness: ReplayHarness,
    *,
    carrier_behind: bool = False,
) -> Any:
    config = harness.config
    carrier_start = (-7.0, -6.0)
    harness.leader.start_attempt(attempt_id=101, now_local_ms=harness.now_ms)

    phase = math.radians(315.0)
    angular_sign = 1.0 if config.turn_direction == "ccw" else -1.0
    angular_step = angular_sign * config.mini_speed_mps / config.orbit_radius_m * 0.1
    snapshot = None
    for _ in range(400):
        mini_position, mini_velocity = orbit_state(config, phase)
        snapshot = harness.feed(
            carrier_position=carrier_start,
            carrier_velocity=(0.0, 0.0),
            mini_position=mini_position,
            mini_velocity=mini_velocity,
        )
        phase += angular_step
        if snapshot.plan is not None:
            break
    if snapshot is None or snapshot.plan is None:
        raise RuntimeError("replay failed to qualify the Mini orbit")

    # Advance once so PLAN_VALIDATED becomes the executable approach phase.
    mini_position, mini_velocity = orbit_state(config, phase)
    snapshot = harness.feed(
        carrier_position=carrier_start,
        carrier_velocity=(0.0, 0.0),
        mini_position=mini_position,
        mini_velocity=mini_velocity,
    )
    phase += angular_step
    plan = snapshot.plan
    assert plan is not None

    elapsed_ms = 0
    while snapshot.phase == LeaderPhase.CARRIER_ARC_MINI_ORBIT:
        elapsed_ms += 100
        fraction = min(1.0, elapsed_ms / plan.carrier_approach.duration_ms)
        carrier_position = (
            carrier_start[0]
            + fraction * (plan.tangent_point[0] - carrier_start[0]),
            carrier_start[1]
            + fraction * (plan.tangent_point[1] - carrier_start[1]),
        )
        if fraction >= 1.0:
            if carrier_behind:
                carrier_position = add(
                    plan.tangent_point,
                    scale(plan.tangent_direction, -0.20),
                )
            else:
                lead_time_s = max(
                    0.0,
                    (elapsed_ms - plan.carrier_approach.duration_ms) / 1000.0,
                )
                carrier_position = add(
                    plan.tangent_point,
                    scale(plan.tangent_direction, 0.40 * lead_time_s),
                )
        mini_position, mini_velocity = orbit_state(config, phase)
        snapshot = harness.feed(
            carrier_position=carrier_position,
            carrier_velocity=scale(
                plan.tangent_direction,
                plan.carrier_approach.planned_speed_mps,
            ),
            mini_position=mini_position,
            mini_velocity=mini_velocity,
        )
        phase += angular_step
        if elapsed_ms > plan.mini_arrival_delay_ms + 2_000:
            raise RuntimeError("replay did not reach the tangent trigger")

    if snapshot.phase != LeaderPhase.MINI_TANGENT_EXIT:
        return snapshot

    carrier_progress = 0.40
    carrier_position = add(
        plan.tangent_point,
        scale(plan.tangent_direction, carrier_progress),
    )
    snapshot = harness.feed(
        carrier_position=carrier_position,
        carrier_velocity=scale(plan.tangent_direction, 0.7),
        mini_position=plan.tangent_point,
        mini_velocity=scale(plan.tangent_direction, 0.7),
    )
    return snapshot


def run_nominal_replay() -> dict[str, Any]:
    harness = ReplayHarness()
    snapshot = advance_to_terminal(harness)
    if snapshot.phase != LeaderPhase.SHARED_TERMINAL:
        raise RuntimeError(
            f"expected SHARED_TERMINAL, got {snapshot.phase.value}: "
            f"{snapshot.abort_reason}"
        )
    plan = snapshot.plan
    assert plan is not None

    # Preserve a 0.40 m Carrier-ahead gap for longer than the configured hold.
    for index in range(8):
        mini_progress = 0.10 + 0.02 * index
        carrier_progress = mini_progress + 0.40
        snapshot = harness.feed(
            carrier_position=add(
                plan.tangent_point,
                scale(plan.tangent_direction, carrier_progress),
            ),
            carrier_velocity=scale(plan.tangent_direction, 0.2),
            mini_position=add(
                plan.tangent_point,
                scale(plan.tangent_direction, mini_progress),
            ),
            mini_velocity=scale(plan.tangent_direction, 0.2),
        )
        if snapshot.phase == LeaderPhase.COMPLETE_HOLD:
            break

    immutable = False
    try:
        plan.plan_id = 999  # type: ignore[misc]
    except FrozenInstanceError:
        immutable = True
    return {
        "scenario": "nominal",
        "pass": snapshot.phase == LeaderPhase.COMPLETE_HOLD and immutable,
        "final_phase": snapshot.phase.value,
        "abort_reason": snapshot.abort_reason,
        "phases": harness.phases,
        "completion_hold_ms": snapshot.completion_hold_ms,
        "plan_immutable": immutable,
        "plan": plan.to_dict(),
    }


def run_fault_replays() -> dict[str, Any]:
    stale = ReplayHarness()
    stale.leader.start_attempt(attempt_id=201, now_local_ms=0)
    mini_position, mini_velocity = orbit_state(stale.config, 0.0)
    stale.feed(
        carrier_position=(-7.0, -6.0),
        carrier_velocity=(0.0, 0.0),
        mini_position=mini_position,
        mini_velocity=mini_velocity,
    )
    stale.now_ms += stale.config.mini_state_stale_ms + 1
    stale_snapshot = stale.leader.step(
        now_local_ms=stale.now_ms,
        sender_monotonic_ms=stale.sender_ms,
    )

    regression = ReplayHarness()
    regression.leader.start_attempt(attempt_id=202, now_local_ms=0)
    regression.mini_sequence = 10
    regression.feed(
        carrier_position=(-7.0, -6.0),
        carrier_velocity=(0.0, 0.0),
        mini_position=mini_position,
        mini_velocity=mini_velocity,
    )
    regression.leader.accept_mini_state(
        regression._state(
            vehicle_id=2,
            sequence=9,
            position=mini_position,
            velocity=mini_velocity,
        )
    )

    behind = ReplayHarness()
    behind_snapshot = advance_to_terminal(behind, carrier_behind=True)

    before_tangent = ReplayHarness()
    before_tangent_snapshot = advance_to_terminal(before_tangent)
    before_tangent_plan = before_tangent_snapshot.plan
    assert before_tangent_plan is not None
    for _ in range(7):
        before_tangent_snapshot = before_tangent.feed(
            carrier_position=add(
                before_tangent_plan.tangent_point,
                scale(before_tangent_plan.tangent_direction, -0.05),
            ),
            carrier_velocity=scale(before_tangent_plan.tangent_direction, 0.1),
            mini_position=add(
                before_tangent_plan.tangent_point,
                scale(before_tangent_plan.tangent_direction, -0.20),
            ),
            mini_velocity=scale(before_tangent_plan.tangent_direction, 0.1),
        )

    tangent_degeneracy_rejected = False
    try:
        external_tangent_candidates((0.0, 0.0), 4.5, (4.5, 0.0))
    except ValueError:
        tangent_degeneracy_rejected = True

    plan_validity_rejected = False
    plan_validity_reason = None
    try:
        compute_ground_corridor_plan(
            plan_id=206,
            sequence=206,
            frame_id="field_enu",
            origin_id=1,
            sender_monotonic_ms=1_000_000,
            carrier_position=(-7.0, -6.0),
            mini_phase_rad=math.radians(315.0),
            orbit_center=(0.0, 0.0),
            orbit_radius_m=4.5,
            turn_direction="ccw",
            mini_speed_mps=0.9,
            mini_max_accel_mps2=0.5,
            carrier_max_speed_mps=0.7,
            carrier_max_accel_mps2=0.3,
            validity_ms=1,
        )
    except ValueError as exc:
        plan_validity_reason = str(exc)
        plan_validity_rejected = plan_validity_reason.startswith(
            "plan_validity_insufficient:"
        )

    nominal = run_nominal_replay()
    command = nominal["plan"]
    command_harness = ReplayHarness()
    command_harness.leader.start_attempt(attempt_id=203, now_local_ms=0)
    command_harness.feed(
        carrier_position=(-7.0, -6.0),
        carrier_velocity=(0.0, 0.0),
        mini_position=mini_position,
        mini_velocity=mini_velocity,
    )
    hold_command = command_harness.leader.step(
        now_local_ms=100,
        sender_monotonic_ms=1_000_100,
    ).mini_command
    watchdog = CommandWatchdog()
    watchdog.accept(hold_command, local_received_ms=7)

    latch = GroundDockingLeader()
    latch.start_attempt(attempt_id=204, now_local_ms=0)
    invalid_identity = VehicleState(
        vehicle_id=2,
        sequence=1,
        sender_monotonic_ms=10,
        received_local_ms=0,
        frame_id="wrong_frame",
        origin_id=1,
        position=mini_position,
        velocity=mini_velocity,
        yaw_rad=0.0,
    )
    latch.accept_mini_state(invalid_identity)
    first_abort_reason = latch.abort_reason
    latch.accept_mini_state(
        VehicleState(
            vehicle_id=99,
            sequence=2,
            sender_monotonic_ms=20,
            received_local_ms=10,
            frame_id="wrong_again",
            origin_id=1,
            position=mini_position,
            velocity=mini_velocity,
            yaw_rad=0.0,
        )
    )
    abort_latched = (
        latch.phase == LeaderPhase.ABORT_LATCHED
        and latch.abort_reason == first_abort_reason
    )
    latch.reset_to_hold()
    latch.start_attempt(attempt_id=205, now_local_ms=20)
    reset_new_attempt = (
        latch.phase == LeaderPhase.WAIT_MINI_STABLE_ORBIT
        and latch.attempt_id == 205
        and latch.abort_reason is None
    )

    checks = {
        "stale_state": stale_snapshot.abort_reason == "mini_state_stale",
        "sequence_regression": (
            regression.leader.abort_reason == "mini_sequence_regression"
        ),
        "carrier_behind": behind_snapshot.abort_reason
        == "carrier_behind_violation",
        "pre_tangent_completion_blocked": (
            before_tangent_snapshot.phase == LeaderPhase.SHARED_TERMINAL
            and before_tangent_snapshot.completion_hold_ms == 0
        ),
        "tangent_degeneracy_rejected": tangent_degeneracy_rejected,
        "plan_validity_rejected": plan_validity_rejected,
        "expired_command": watchdog.stop_reason(508) == "command_expired",
        "abort_latched": abort_latched,
        "reset_new_attempt": reset_new_attempt,
        "nominal_reference": bool(command),
    }
    return {
        "scenario": "faults",
        "pass": all(checks.values()),
        "checks": checks,
        "reasons": {
            "stale": stale_snapshot.abort_reason,
            "sequence": regression.leader.abort_reason,
            "carrier_behind": behind_snapshot.abort_reason,
            "command": watchdog.stop_reason(508),
            "plan_validity": plan_validity_reason,
        },
    }


def run_all() -> dict[str, Any]:
    nominal_first = run_nominal_replay()
    nominal_second = run_nominal_replay()
    deterministic = nominal_first == nominal_second
    faults = run_fault_replays()
    return {
        "schema_version": 2,
        "mode": "offline_replay",
        "hardware_access": False,
        "nominal": nominal_first,
        "faults": faults,
        "deterministic_repeatability": deterministic,
        "pass": nominal_first["pass"] and faults["pass"] and deterministic,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=("nominal", "faults", "all"),
        default="all",
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.scenario == "nominal":
        result = run_nominal_replay()
    elif args.scenario == "faults":
        result = run_fault_replays()
    else:
        result = run_all()
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
