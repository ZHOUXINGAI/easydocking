"""Protocol-neutral Carrier leader for the two-rover EasyDocking experiment."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any

from ground_corridor_geometry import (
    GroundCorridorPlan,
    TurnDirection,
    Vec2,
    compute_ground_corridor_plan,
    dot,
    norm,
    subtract,
    tangent_direction,
    uint32_add,
    unit,
    wrap_pi,
)

UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000


def uint32_delta(candidate: int, previous: int) -> int:
    return (int(candidate) - int(previous)) & UINT32_MASK


def is_newer_sequence(candidate: int, previous: int) -> bool:
    delta = uint32_delta(candidate, previous)
    return 0 < delta < UINT32_HALF_RANGE


class LeaderPhase(str, Enum):
    HOLD = "HOLD"
    WAIT_MINI_STABLE_ORBIT = "WAIT_MINI_STABLE_ORBIT"
    PLAN_VALIDATED = "PLAN_VALIDATED"
    CARRIER_ARC_MINI_ORBIT = "CARRIER_ARC/MINI_ORBIT"
    MINI_TANGENT_EXIT = "MINI_TANGENT_EXIT"
    SHARED_TERMINAL = "SHARED_TERMINAL"
    COMPLETE_HOLD = "COMPLETE_HOLD"
    ABORT_LATCHED = "ABORT_LATCHED"


class CommandPhase(str, Enum):
    HOLD = "HOLD"
    ORBIT = "ORBIT"
    ARC = "ARC"
    TERMINAL = "TERMINAL"
    STOP = "STOP"
    ABORT = "ABORT"


@dataclass(frozen=True)
class LeaderConfig:
    frame_id: str = "field_enu"
    origin_id: int = 1
    orbit_center: Vec2 = (0.0, 0.0)
    orbit_radius_m: float = 4.5
    turn_direction: TurnDirection = "ccw"
    required_stable_orbit_laps: float = 1.0
    orbit_radius_tolerance_m: float = 0.35
    orbit_tangent_alignment_min: float = 0.80
    orbit_min_speed_mps: float = 0.05
    mini_speed_mps: float = 0.9
    mini_max_accel_mps2: float = 0.50
    carrier_max_speed_mps: float = 0.7
    carrier_max_accel_mps2: float = 0.30
    terminal_lead_ms: int = 800
    terminal_length_m: float = 8.0
    target_front_gap_m: float = 0.35
    carrier_ahead_tolerance_m: float = 0.05
    terminal_cross_track_limit_m: float = 0.20
    completion_distance_m: float = 0.50
    completion_hold_ms: int = 500
    tangent_ready_tolerance_m: float = 0.25
    mini_state_stale_ms: int = 300
    plan_validity_ms: int = 30_000
    command_ttl_ms: int = 500
    local_command_watchdog_ms: int = 750

    def __post_init__(self) -> None:
        if not self.frame_id or self.origin_id <= 0:
            raise ValueError("frame_id and nonzero origin_id are required")
        if self.turn_direction not in {"ccw", "cw"}:
            raise ValueError("turn_direction must be ccw or cw")
        positive = (
            self.orbit_radius_m,
            self.required_stable_orbit_laps,
            self.orbit_radius_tolerance_m,
            self.orbit_tangent_alignment_min,
            self.orbit_min_speed_mps,
            self.mini_speed_mps,
            self.mini_max_accel_mps2,
            self.carrier_max_speed_mps,
            self.carrier_max_accel_mps2,
            self.terminal_length_m,
            self.completion_distance_m,
            self.completion_hold_ms,
            self.mini_state_stale_ms,
            self.plan_validity_ms,
            self.command_ttl_ms,
            self.local_command_watchdog_ms,
        )
        if any(
            not math.isfinite(float(value)) or value <= 0
            for value in positive
        ):
            raise ValueError("leader limits and timing values must be finite and positive")
        nonnegative = (
            self.terminal_lead_ms,
            self.target_front_gap_m,
            self.carrier_ahead_tolerance_m,
            self.terminal_cross_track_limit_m,
            self.tangent_ready_tolerance_m,
        )
        if any(
            not math.isfinite(float(value)) or value < 0
            for value in nonnegative
        ):
            raise ValueError("leader tolerances and terminal lead must be finite and nonnegative")
        if not 0.0 < self.orbit_tangent_alignment_min <= 1.0:
            raise ValueError("orbit tangent alignment must be in (0, 1]")


@dataclass(frozen=True)
class VehicleState:
    vehicle_id: int
    sequence: int
    sender_monotonic_ms: int
    received_local_ms: int
    frame_id: str
    origin_id: int
    position: Vec2
    velocity: Vec2
    yaw_rad: float
    health_ok: bool = True

    @property
    def speed_mps(self) -> float:
        return norm(self.velocity)


@dataclass(frozen=True)
class GroundPlanCommand:
    schema_version: int
    plan_id: int
    sequence: int
    target_role: str
    phase: CommandPhase
    sender_monotonic_ms: int
    valid_until_sender_monotonic_ms: int
    ttl_ms: int
    body_speed_mps: float
    yaw_rate_radps: float
    max_speed_mps: float
    max_accel_mps2: float
    frame_id: str
    origin_id: int

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["phase"] = self.phase.value
        return result


@dataclass(frozen=True)
class LeaderSnapshot:
    phase: LeaderPhase
    attempt_id: int | None
    plan: GroundCorridorPlan | None
    carrier_command: GroundPlanCommand
    mini_command: GroundPlanCommand
    abort_reason: str | None
    stable_orbit_laps: float
    signed_tangent_gap_m: float | None
    completion_hold_ms: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "phase": self.phase.value,
            "attempt_id": self.attempt_id,
            "plan": self.plan.to_dict() if self.plan else None,
            "carrier_command": self.carrier_command.to_dict(),
            "mini_command": self.mini_command.to_dict(),
            "abort_reason": self.abort_reason,
            "stable_orbit_laps": self.stable_orbit_laps,
            "signed_tangent_gap_m": self.signed_tangent_gap_m,
            "completion_hold_ms": self.completion_hold_ms,
        }


class StableOrbitQualifier:
    def __init__(self, config: LeaderConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._last_phase: float | None = None
        self._accumulated_phase = 0.0

    @property
    def laps(self) -> float:
        return self._accumulated_phase / (2.0 * math.pi)

    @property
    def qualified(self) -> bool:
        return self.laps >= self.config.required_stable_orbit_laps

    def accept(self, state: VehicleState) -> bool:
        radial = subtract(state.position, self.config.orbit_center)
        radius = norm(radial)
        speed = state.speed_mps
        if (
            not state.health_ok
            or abs(radius - self.config.orbit_radius_m)
            > self.config.orbit_radius_tolerance_m
            or speed < self.config.orbit_min_speed_mps
        ):
            self.reset()
            return False

        phase = math.atan2(radial[1], radial[0])
        expected_tangent = tangent_direction(phase, self.config.turn_direction)
        alignment = dot(unit(state.velocity), expected_tangent)
        if alignment < self.config.orbit_tangent_alignment_min:
            self.reset()
            return False

        if self._last_phase is None:
            self._last_phase = phase
            return False
        signed_delta = wrap_pi(phase - self._last_phase)
        if self.config.turn_direction == "cw":
            signed_delta = -signed_delta
        self._last_phase = phase
        if signed_delta < -1.0e-4 or signed_delta > math.pi / 2.0:
            self.reset()
            self._last_phase = phase
            return False
        self._accumulated_phase += max(0.0, signed_delta)
        return self.qualified


class CommandWatchdog:
    """Receiver-side command expiry using only local receipt time."""

    def __init__(self, watchdog_ms: int = 750, maximum_ttl_ms: int = 5_000) -> None:
        if watchdog_ms <= 0 or maximum_ttl_ms <= 0:
            raise ValueError("watchdog and maximum TTL must be positive")
        self.watchdog_ms = watchdog_ms
        self.maximum_ttl_ms = maximum_ttl_ms
        self.last_sequence: int | None = None
        self.local_received_ms: int | None = None
        self.local_expiry_ms: int | None = None

    def accept(self, command: GroundPlanCommand, local_received_ms: int) -> None:
        ttl_ms = uint32_delta(
            command.valid_until_sender_monotonic_ms,
            command.sender_monotonic_ms,
        )
        if ttl_ms <= 0 or ttl_ms > self.maximum_ttl_ms or ttl_ms != command.ttl_ms:
            raise ValueError("invalid_command_ttl")
        if self.last_sequence is not None:
            delta = uint32_delta(command.sequence, self.last_sequence)
            if delta == 0:
                raise ValueError("duplicate_command")
            if delta >= UINT32_HALF_RANGE:
                raise ValueError("command_sequence_regression")
        self.last_sequence = command.sequence
        self.local_received_ms = local_received_ms
        self.local_expiry_ms = local_received_ms + ttl_ms

    def stop_reason(self, now_local_ms: int) -> str | None:
        if self.local_received_ms is None or self.local_expiry_ms is None:
            return "no_command"
        if now_local_ms > self.local_expiry_ms:
            return "command_expired"
        if now_local_ms - self.local_received_ms > self.watchdog_ms:
            return "command_watchdog_expired"
        return None


class GroundDockingLeader:
    """Carrier-owned deterministic leader with no transport or executor code."""

    def __init__(self, config: LeaderConfig | None = None) -> None:
        self.config = config or LeaderConfig()
        self.phase = LeaderPhase.HOLD
        self.attempt_id: int | None = None
        self.plan: GroundCorridorPlan | None = None
        self.abort_reason: str | None = None
        self._orbit = StableOrbitQualifier(self.config)
        self._carrier_state: VehicleState | None = None
        self._mini_state: VehicleState | None = None
        self._last_mini_sequence: int | None = None
        self._last_mini_sender_ms: int | None = None
        self._last_plan_phase: float | None = None
        self._mini_phase_after_plan = 0.0
        self._plan_local_created_ms: int | None = None
        self._completion_started_ms: int | None = None
        self._completion_hold_ms = 0
        self._command_sequence = 0

    @property
    def stable_orbit_laps(self) -> float:
        return self._orbit.laps

    def start_attempt(
        self,
        *,
        attempt_id: int,
        now_local_ms: int,
    ) -> None:
        if self.phase != LeaderPhase.HOLD:
            raise RuntimeError("a new attempt can only start from HOLD")
        if attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        if now_local_ms < 0:
            raise ValueError("local monotonic time cannot be negative")
        self.attempt_id = attempt_id
        self.phase = LeaderPhase.WAIT_MINI_STABLE_ORBIT
        self.plan = None
        self.abort_reason = None
        self._orbit.reset()
        self._last_plan_phase = None
        self._mini_phase_after_plan = 0.0
        self._plan_local_created_ms = None
        self._completion_started_ms = None
        self._completion_hold_ms = 0
        self._command_sequence = 0
        # States are retained only if they are fresh when step() is called.

    def reset_to_hold(self) -> None:
        """Local/operator reset; a wireless plan cannot call this method."""

        self.phase = LeaderPhase.HOLD
        self.attempt_id = None
        self.plan = None
        self.abort_reason = None
        self._orbit.reset()
        self._last_mini_sequence = None
        self._last_mini_sender_ms = None
        self._last_plan_phase = None
        self._mini_phase_after_plan = 0.0
        self._plan_local_created_ms = None
        self._completion_started_ms = None
        self._completion_hold_ms = 0
        self._command_sequence = 0

    def accept_carrier_state(self, state: VehicleState) -> None:
        if state.vehicle_id != 1:
            self._abort("invalid_carrier_vehicle_id")
            return
        if not self._state_identity_valid(state):
            self._abort("invalid_carrier_frame_or_health")
            return
        self._carrier_state = state

    def accept_mini_state(self, state: VehicleState) -> bool:
        if state.vehicle_id != 2:
            self._abort("invalid_mini_vehicle_id")
            return False
        if not self._state_identity_valid(state):
            self._abort("invalid_mini_frame_or_health")
            return False

        if self._last_mini_sequence is not None:
            sequence_delta = uint32_delta(state.sequence, self._last_mini_sequence)
            if sequence_delta == 0:
                return False
            if sequence_delta >= UINT32_HALF_RANGE:
                self._abort("mini_sequence_regression")
                return False
        if self._last_mini_sender_ms is not None:
            timestamp_delta = uint32_delta(
                state.sender_monotonic_ms,
                self._last_mini_sender_ms,
            )
            if timestamp_delta >= UINT32_HALF_RANGE:
                self._abort("mini_sender_timestamp_regression")
                return False

        self._last_mini_sequence = state.sequence
        self._last_mini_sender_ms = state.sender_monotonic_ms
        self._mini_state = state
        if self.phase == LeaderPhase.WAIT_MINI_STABLE_ORBIT:
            self._orbit.accept(state)
        if self.plan is not None:
            phase = math.atan2(
                state.position[1] - self.config.orbit_center[1],
                state.position[0] - self.config.orbit_center[0],
            )
            if self._last_plan_phase is None:
                self._last_plan_phase = phase
            elif self.phase in {
                LeaderPhase.PLAN_VALIDATED,
                LeaderPhase.CARRIER_ARC_MINI_ORBIT,
            }:
                delta = wrap_pi(phase - self._last_plan_phase)
                if self.config.turn_direction == "cw":
                    delta = -delta
                if 0.0 <= delta < math.pi / 2.0:
                    self._mini_phase_after_plan += delta
                self._last_plan_phase = phase
        return True

    def step(
        self,
        *,
        now_local_ms: int,
        sender_monotonic_ms: int,
    ) -> LeaderSnapshot:
        if self.phase not in {
            LeaderPhase.HOLD,
            LeaderPhase.COMPLETE_HOLD,
            LeaderPhase.ABORT_LATCHED,
        }:
            self._check_freshness(now_local_ms)

        if (
            self.plan is not None
            and self._plan_local_created_ms is not None
            and self.phase
            not in {
                LeaderPhase.COMPLETE_HOLD,
                LeaderPhase.ABORT_LATCHED,
            }
            and now_local_ms - self._plan_local_created_ms > self.plan.validity_ms
        ):
            self._abort("plan_expired")

        if self.phase == LeaderPhase.WAIT_MINI_STABLE_ORBIT and self._orbit.qualified:
            self._create_plan(now_local_ms, sender_monotonic_ms)
        elif self.phase == LeaderPhase.PLAN_VALIDATED:
            self.phase = LeaderPhase.CARRIER_ARC_MINI_ORBIT
        elif self.phase == LeaderPhase.CARRIER_ARC_MINI_ORBIT:
            self._maybe_release_mini()
        elif self.phase == LeaderPhase.MINI_TANGENT_EXIT:
            if self._terminal_safety_ok():
                self._maybe_enter_shared_terminal()
        elif self.phase == LeaderPhase.SHARED_TERMINAL:
            if self._terminal_safety_ok():
                self._update_completion_hold(now_local_ms)

        return self._snapshot(sender_monotonic_ms)

    def _state_identity_valid(self, state: VehicleState) -> bool:
        return (
            state.health_ok
            and state.frame_id == self.config.frame_id
            and state.origin_id == self.config.origin_id
            and 0 <= state.sequence <= UINT32_MASK
            and 0 <= state.sender_monotonic_ms <= UINT32_MASK
            and state.received_local_ms >= 0
            and all(math.isfinite(component) for component in state.position)
            and all(math.isfinite(component) for component in state.velocity)
            and math.isfinite(state.yaw_rad)
        )

    def _check_freshness(self, now_local_ms: int) -> None:
        if self._mini_state is None:
            self._abort("mini_state_missing")
            return
        if now_local_ms - self._mini_state.received_local_ms > self.config.mini_state_stale_ms:
            self._abort("mini_state_stale")
            return
        if self._carrier_state is None:
            self._abort("carrier_state_missing")
            return
        if now_local_ms - self._carrier_state.received_local_ms > self.config.mini_state_stale_ms:
            self._abort("carrier_state_stale")

    def _create_plan(self, now_local_ms: int, sender_monotonic_ms: int) -> None:
        if self.plan is not None:
            self._abort("plan_immutability_violation")
            return
        if self.attempt_id is None or self._carrier_state is None or self._mini_state is None:
            self._abort("plan_state_missing")
            return
        mini_phase = math.atan2(
            self._mini_state.position[1] - self.config.orbit_center[1],
            self._mini_state.position[0] - self.config.orbit_center[0],
        )
        try:
            self.plan = compute_ground_corridor_plan(
                plan_id=self.attempt_id,
                sequence=self.attempt_id,
                frame_id=self.config.frame_id,
                origin_id=self.config.origin_id,
                sender_monotonic_ms=sender_monotonic_ms,
                carrier_position=self._carrier_state.position,
                mini_phase_rad=mini_phase,
                orbit_center=self.config.orbit_center,
                orbit_radius_m=self.config.orbit_radius_m,
                turn_direction=self.config.turn_direction,
                mini_speed_mps=self.config.mini_speed_mps,
                mini_max_accel_mps2=self.config.mini_max_accel_mps2,
                carrier_max_speed_mps=self.config.carrier_max_speed_mps,
                carrier_max_accel_mps2=self.config.carrier_max_accel_mps2,
                required_stable_orbit_laps=self.config.required_stable_orbit_laps,
                terminal_lead_ms=self.config.terminal_lead_ms,
                terminal_length_m=self.config.terminal_length_m,
                target_front_gap_m=self.config.target_front_gap_m,
                validity_ms=self.config.plan_validity_ms,
                command_ttl_ms=self.config.command_ttl_ms,
                local_command_watchdog_ms=self.config.local_command_watchdog_ms,
                mini_state_stale_ms=self.config.mini_state_stale_ms,
            )
        except ValueError as exc:
            self._abort(f"planner_invalid:{exc}")
            return
        self._plan_local_created_ms = now_local_ms
        self._last_plan_phase = mini_phase
        self._mini_phase_after_plan = 0.0
        self.phase = LeaderPhase.PLAN_VALIDATED

    def _maybe_release_mini(self) -> None:
        if self.plan is None or self._carrier_state is None:
            self._abort("release_state_missing")
            return
        if self._mini_phase_after_plan + 1.0e-6 < self.plan.mini_exit_delta_rad:
            return
        carrier_progress, carrier_cross_track = self._corridor_coordinates(
            self._carrier_state.position
        )
        if (
            carrier_progress < -self.config.tangent_ready_tolerance_m
            or abs(carrier_cross_track) > self.config.terminal_cross_track_limit_m
        ):
            self._abort("carrier_not_ready_at_tangent_trigger")
            return
        # Enforce the terminal invariants on the trigger sample itself. Without
        # this check, one unsafe TERMINAL command could be emitted before the
        # next step noticed that the Carrier was behind the Mini.
        if not self._terminal_safety_ok():
            return
        self.phase = LeaderPhase.MINI_TANGENT_EXIT

    def _maybe_enter_shared_terminal(self) -> None:
        if self._mini_state is None:
            return
        mini_progress, mini_cross_track = self._corridor_coordinates(
            self._mini_state.position
        )
        if (
            mini_progress >= -self.config.tangent_ready_tolerance_m
            and abs(mini_cross_track) <= self.config.terminal_cross_track_limit_m
        ):
            self.phase = LeaderPhase.SHARED_TERMINAL

    def _terminal_safety_ok(self) -> bool:
        if self.plan is None or self._carrier_state is None or self._mini_state is None:
            self._abort("terminal_state_missing")
            return False
        carrier_progress, carrier_cross_track = self._corridor_coordinates(
            self._carrier_state.position
        )
        mini_progress, mini_cross_track = self._corridor_coordinates(
            self._mini_state.position
        )
        signed_gap = mini_progress - carrier_progress
        if signed_gap > self.config.carrier_ahead_tolerance_m:
            self._abort("carrier_behind_violation")
            return False
        if (
            abs(carrier_cross_track) > self.config.terminal_cross_track_limit_m
            or abs(mini_cross_track) > self.config.terminal_cross_track_limit_m
        ):
            self._abort("terminal_cross_track_violation")
            return False
        if max(carrier_progress, mini_progress) > self.config.terminal_length_m:
            self._abort("terminal_length_exceeded")
            return False
        return True

    def _update_completion_hold(self, now_local_ms: int) -> None:
        if self._carrier_state is None or self._mini_state is None:
            return
        carrier_progress, _ = self._corridor_coordinates(self._carrier_state.position)
        mini_progress, _ = self._corridor_coordinates(self._mini_state.position)
        separation = norm(subtract(self._mini_state.position, self._carrier_state.position))
        both_inside_terminal = min(carrier_progress, mini_progress) >= -1.0e-6
        if both_inside_terminal and separation <= self.config.completion_distance_m:
            if self._completion_started_ms is None:
                self._completion_started_ms = now_local_ms
            self._completion_hold_ms = now_local_ms - self._completion_started_ms
            if self._completion_hold_ms >= self.config.completion_hold_ms:
                self.phase = LeaderPhase.COMPLETE_HOLD
        else:
            self._completion_started_ms = None
            self._completion_hold_ms = 0

    def _corridor_coordinates(self, position: Vec2) -> tuple[float, float]:
        if self.plan is None:
            raise RuntimeError("corridor coordinates require a plan")
        relative = subtract(position, self.plan.tangent_point)
        tangent = self.plan.tangent_direction
        lateral = (-tangent[1], tangent[0])
        return (dot(relative, tangent), dot(relative, lateral))

    def _signed_gap(self) -> float | None:
        if self.plan is None or self._carrier_state is None or self._mini_state is None:
            return None
        relative = subtract(self._mini_state.position, self._carrier_state.position)
        return dot(relative, self.plan.tangent_direction)

    def _abort(self, reason: str) -> None:
        if self.phase != LeaderPhase.ABORT_LATCHED:
            self.abort_reason = reason
            self.phase = LeaderPhase.ABORT_LATCHED

    def _next_command_sequence(self) -> int:
        self._command_sequence = uint32_add(self._command_sequence, 1)
        if self._command_sequence == 0:
            self._command_sequence = 1
        return self._command_sequence

    def _build_command(
        self,
        *,
        target_role: str,
        phase: CommandPhase,
        sender_monotonic_ms: int,
        speed_mps: float,
        yaw_rate_radps: float,
        max_speed_mps: float,
        max_accel_mps2: float,
    ) -> GroundPlanCommand:
        plan_id = self.plan.plan_id if self.plan else 0
        return GroundPlanCommand(
            schema_version=1,
            plan_id=plan_id,
            sequence=self._next_command_sequence(),
            target_role=target_role,
            phase=phase,
            sender_monotonic_ms=sender_monotonic_ms & UINT32_MASK,
            valid_until_sender_monotonic_ms=uint32_add(
                sender_monotonic_ms,
                self.config.command_ttl_ms,
            ),
            ttl_ms=self.config.command_ttl_ms,
            body_speed_mps=speed_mps,
            yaw_rate_radps=yaw_rate_radps,
            max_speed_mps=max_speed_mps,
            max_accel_mps2=max_accel_mps2,
            frame_id=self.config.frame_id,
            origin_id=self.config.origin_id,
        )

    def _snapshot(self, sender_monotonic_ms: int) -> LeaderSnapshot:
        carrier_phase = CommandPhase.HOLD
        mini_phase = CommandPhase.HOLD
        carrier_speed = 0.0
        mini_speed = 0.0
        carrier_yaw_rate = 0.0
        mini_yaw_rate = 0.0

        if self.phase == LeaderPhase.WAIT_MINI_STABLE_ORBIT:
            mini_phase = CommandPhase.ORBIT
            mini_speed = self.config.mini_speed_mps
            mini_yaw_rate = (
                self.config.mini_speed_mps / self.config.orbit_radius_m
            ) * (1.0 if self.config.turn_direction == "ccw" else -1.0)
        elif self.phase in {
            LeaderPhase.PLAN_VALIDATED,
            LeaderPhase.CARRIER_ARC_MINI_ORBIT,
        }:
            mini_phase = CommandPhase.ORBIT
            mini_speed = self.config.mini_speed_mps
            mini_yaw_rate = (
                self.config.mini_speed_mps / self.config.orbit_radius_m
            ) * (1.0 if self.config.turn_direction == "ccw" else -1.0)
            if self.phase == LeaderPhase.CARRIER_ARC_MINI_ORBIT and self.plan:
                carrier_phase = CommandPhase.ARC
                carrier_speed = self.plan.carrier_approach.planned_speed_mps
        elif self.phase in {
            LeaderPhase.MINI_TANGENT_EXIT,
            LeaderPhase.SHARED_TERMINAL,
        }:
            carrier_phase = CommandPhase.TERMINAL
            mini_phase = CommandPhase.TERMINAL
            carrier_speed = self.config.carrier_max_speed_mps
            mini_speed = self.config.mini_speed_mps
        elif self.phase == LeaderPhase.ABORT_LATCHED:
            carrier_phase = CommandPhase.ABORT
            mini_phase = CommandPhase.ABORT

        carrier_command = self._build_command(
            target_role="carrier",
            phase=carrier_phase,
            sender_monotonic_ms=sender_monotonic_ms,
            speed_mps=carrier_speed,
            yaw_rate_radps=carrier_yaw_rate,
            max_speed_mps=self.config.carrier_max_speed_mps,
            max_accel_mps2=self.config.carrier_max_accel_mps2,
        )
        mini_command = self._build_command(
            target_role="mini",
            phase=mini_phase,
            sender_monotonic_ms=sender_monotonic_ms,
            speed_mps=mini_speed,
            yaw_rate_radps=mini_yaw_rate,
            max_speed_mps=self.config.mini_speed_mps,
            max_accel_mps2=self.config.mini_max_accel_mps2,
        )
        return LeaderSnapshot(
            phase=self.phase,
            attempt_id=self.attempt_id,
            plan=self.plan,
            carrier_command=carrier_command,
            mini_command=mini_command,
            abort_reason=self.abort_reason,
            stable_orbit_laps=self._orbit.laps,
            signed_tangent_gap_m=self._signed_gap(),
            completion_hold_ms=self._completion_hold_ms,
        )
