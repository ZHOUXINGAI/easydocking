"""Deterministic 2-D geometry for the ground EasyDocking experiment.

This module has no ROS or vehicle dependencies.  Positions are expressed in a
shared ENU field frame and angles are counter-clockwise from +x (East).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Literal

Vec2 = tuple[float, float]
TurnDirection = Literal["ccw", "cw"]

UINT32_MASK = 0xFFFFFFFF
GEOMETRY_EPSILON = 1.0e-9


def add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def subtract(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def scale(vector: Vec2, factor: float) -> Vec2:
    return (vector[0] * factor, vector[1] * factor)


def dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def norm(vector: Vec2) -> float:
    return math.hypot(vector[0], vector[1])


def unit(vector: Vec2) -> Vec2:
    length = norm(vector)
    if not math.isfinite(length) or length <= GEOMETRY_EPSILON:
        raise ValueError("cannot normalize a non-finite or zero-length vector")
    return scale(vector, 1.0 / length)


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def positive_phase_delta(
    start_phase_rad: float,
    end_phase_rad: float,
    direction: TurnDirection,
) -> float:
    if direction == "ccw":
        delta = (end_phase_rad - start_phase_rad) % (2.0 * math.pi)
    elif direction == "cw":
        delta = (start_phase_rad - end_phase_rad) % (2.0 * math.pi)
    else:
        raise ValueError(f"unsupported turn direction: {direction}")
    return delta if delta > GEOMETRY_EPSILON else 2.0 * math.pi


def tangent_direction(phase_rad: float, direction: TurnDirection) -> Vec2:
    if direction == "ccw":
        return (-math.sin(phase_rad), math.cos(phase_rad))
    if direction == "cw":
        return (math.sin(phase_rad), -math.cos(phase_rad))
    raise ValueError(f"unsupported turn direction: {direction}")


def uint32_add(value: int, delta: int) -> int:
    return (int(value) + int(delta)) & UINT32_MASK


@dataclass(frozen=True)
class TangentCandidate:
    point: Vec2
    direction: Vec2
    phase_rad: float
    approach_alignment: float
    radius_residual_m: float
    orthogonality_residual_m2: float


def external_tangent_candidates(
    orbit_center: Vec2,
    orbit_radius_m: float,
    carrier_position: Vec2,
    turn_direction: TurnDirection = "ccw",
) -> tuple[TangentCandidate, TangentCandidate]:
    """Return both exact external-point tangencies to a circle.

    For ``p = C - O`` and ``d = |p|``, a tangent radius vector is

        q = (R^2 / d^2) p +/- (R sqrt(d^2 - R^2) / d^2) perp(p).

    Consequently ``|q| = R`` and ``(C - (O + q)) dot q = 0``.  The equivalent
    angular offset is ``acos(R / d)``, not ``asin(R / d)``.
    """

    if not math.isfinite(orbit_radius_m) or orbit_radius_m <= 0.0:
        raise ValueError("orbit radius must be finite and positive")
    if not all(
        math.isfinite(component)
        for point in (orbit_center, carrier_position)
        for component in point
    ):
        raise ValueError("orbit center and carrier position must be finite")
    if turn_direction not in {"ccw", "cw"}:
        raise ValueError(f"unsupported turn direction: {turn_direction}")

    p = subtract(carrier_position, orbit_center)
    distance_sq = dot(p, p)
    radius_sq = orbit_radius_m * orbit_radius_m
    if distance_sq <= radius_sq + GEOMETRY_EPSILON:
        raise ValueError("carrier must be strictly outside the mini orbit circle")

    perpendicular = (-p[1], p[0])
    base = scale(p, radius_sq / distance_sq)
    offset_scale = (
        orbit_radius_m * math.sqrt(distance_sq - radius_sq) / distance_sq
    )

    candidates: list[TangentCandidate] = []
    for sign in (1.0, -1.0):
        radius_vector = add(base, scale(perpendicular, sign * offset_scale))
        point = add(orbit_center, radius_vector)
        phase = math.atan2(radius_vector[1], radius_vector[0])
        direction = tangent_direction(phase, turn_direction)
        approach = unit(subtract(point, carrier_position))
        candidates.append(
            TangentCandidate(
                point=point,
                direction=direction,
                phase_rad=phase % (2.0 * math.pi),
                approach_alignment=dot(approach, direction),
                radius_residual_m=norm(radius_vector) - orbit_radius_m,
                orthogonality_residual_m2=dot(
                    subtract(carrier_position, point),
                    radius_vector,
                ),
            )
        )
    return (candidates[0], candidates[1])


def select_direction_compatible_tangent(
    orbit_center: Vec2,
    orbit_radius_m: float,
    carrier_position: Vec2,
    turn_direction: TurnDirection = "ccw",
) -> TangentCandidate:
    candidates = external_tangent_candidates(
        orbit_center,
        orbit_radius_m,
        carrier_position,
        turn_direction,
    )
    selected = max(candidates, key=lambda candidate: candidate.approach_alignment)
    if selected.approach_alignment < 1.0 - 1.0e-7:
        raise ValueError("no tangent branch aligns with the requested orbit direction")
    return selected


@dataclass(frozen=True)
class CarrierApproach:
    """Carrier path from its planning position to the shared tangent.

    An external tangent computed from the Carrier position makes C->T itself
    tangent-compatible.  With no initial heading constraint, the unique
    minimum-curvature solution is therefore a straight, zero-curvature
    (degenerate circular-arc) segment.
    """

    kind: Literal["straight_tangent"]
    start: Vec2
    end: Vec2
    length_m: float
    duration_ms: int
    planned_speed_mps: float
    curvature_per_m: float = 0.0


@dataclass(frozen=True)
class GroundCorridorPlan:
    schema_version: int
    plan_id: int
    sequence: int
    frame_id: str
    origin_id: int
    sender_monotonic_ms: int
    valid_until_sender_monotonic_ms: int
    validity_ms: int
    requested_validity_ms: int
    required_validity_ms: int
    validity_margin_ms: int
    post_tangent_reserve_ms: int
    terminal_completion_budget_ms: int
    completion_hold_ms: int
    plan_timing_guard_ms: int
    validity_policy: Literal["reject"]
    validity_extended: bool
    orbit_center: Vec2
    orbit_radius_m: float
    turn_direction: TurnDirection
    tangent_point: Vec2
    tangent_direction: Vec2
    tangent_phase_rad: float
    mini_phase_at_plan_rad: float
    mini_exit_delta_rad: float
    mini_arrival_delay_ms: int
    required_stable_orbit_laps: float
    terminal_length_m: float
    target_front_gap_m: float
    mini_speed_mps: float
    mini_max_accel_mps2: float
    carrier_max_speed_mps: float
    carrier_max_accel_mps2: float
    command_ttl_ms: int
    local_command_watchdog_ms: int
    mini_state_stale_ms: int
    carrier_approach: CarrierApproach

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def minimum_rest_to_rest_time(
    distance_m: float,
    max_speed_mps: float,
    max_accel_mps2: float,
) -> float:
    """Conservative trapezoidal/triangular travel time in seconds."""

    if distance_m < 0.0:
        raise ValueError("distance cannot be negative")
    if max_speed_mps <= 0.0 or max_accel_mps2 <= 0.0:
        raise ValueError("speed and acceleration limits must be positive")
    if distance_m <= GEOMETRY_EPSILON:
        return 0.0
    acceleration_distance = max_speed_mps * max_speed_mps / (2.0 * max_accel_mps2)
    if distance_m <= 2.0 * acceleration_distance:
        return 2.0 * math.sqrt(distance_m / max_accel_mps2)
    cruise_distance = distance_m - 2.0 * acceleration_distance
    return 2.0 * max_speed_mps / max_accel_mps2 + cruise_distance / max_speed_mps


def ceil_milliseconds(value_ms: int, quantum_ms: int = 100) -> int:
    if value_ms < 0 or quantum_ms <= 0:
        raise ValueError("milliseconds and quantum must be nonnegative and positive")
    return ((value_ms + quantum_ms - 1) // quantum_ms) * quantum_ms


def compute_ground_corridor_plan(
    *,
    plan_id: int,
    sequence: int,
    frame_id: str,
    origin_id: int,
    sender_monotonic_ms: int,
    carrier_position: Vec2,
    mini_phase_rad: float,
    orbit_center: Vec2,
    orbit_radius_m: float,
    turn_direction: TurnDirection,
    mini_speed_mps: float,
    mini_max_accel_mps2: float,
    carrier_max_speed_mps: float,
    carrier_max_accel_mps2: float,
    required_stable_orbit_laps: float = 1.0,
    terminal_lead_ms: int = 800,
    terminal_length_m: float = 8.0,
    target_front_gap_m: float = 0.35,
    validity_ms: int = 32_000,
    command_ttl_ms: int = 500,
    local_command_watchdog_ms: int = 750,
    mini_state_stale_ms: int = 300,
    terminal_completion_budget_ms: int = 2_000,
    completion_hold_ms: int = 500,
    plan_timing_guard_ms: int = 100,
) -> GroundCorridorPlan:
    if plan_id <= 0 or sequence <= 0:
        raise ValueError("plan_id and sequence must be positive")
    if not frame_id or origin_id <= 0:
        raise ValueError("a non-empty frame and nonzero origin are required")
    positive_finite = (
        mini_speed_mps,
        mini_max_accel_mps2,
        carrier_max_speed_mps,
        carrier_max_accel_mps2,
        required_stable_orbit_laps,
        terminal_length_m,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in positive_finite):
        raise ValueError("speed, acceleration, lap, and length limits must be finite and positive")
    if (
        not math.isfinite(mini_phase_rad)
        or not math.isfinite(target_front_gap_m)
        or target_front_gap_m < 0.0
        or terminal_lead_ms < 0
    ):
        raise ValueError("phase, terminal lead, and target gap must be valid")
    if (
        validity_ms <= 0
        or command_ttl_ms <= 0
        or local_command_watchdog_ms <= 0
        or mini_state_stale_ms <= 0
        or terminal_completion_budget_ms <= 0
        or completion_hold_ms <= 0
        or plan_timing_guard_ms < 0
    ):
        raise ValueError("plan and command timing limits must be positive")

    tangent = select_direction_compatible_tangent(
        orbit_center,
        orbit_radius_m,
        carrier_position,
        turn_direction,
    )
    approach_length = norm(subtract(tangent.point, carrier_position))
    minimum_carrier_time_s = minimum_rest_to_rest_time(
        approach_length,
        carrier_max_speed_mps,
        carrier_max_accel_mps2,
    )

    exit_delta = positive_phase_delta(
        mini_phase_rad,
        tangent.phase_rad,
        turn_direction,
    )
    mini_angular_speed = mini_speed_mps / orbit_radius_m
    required_delay_s = minimum_carrier_time_s + terminal_lead_ms / 1000.0
    while exit_delta / mini_angular_speed < required_delay_s:
        exit_delta += 2.0 * math.pi
    mini_arrival_delay_s = exit_delta / mini_angular_speed
    mini_arrival_delay_ms = int(round(mini_arrival_delay_s * 1000.0))
    post_tangent_reserve_ms = (
        terminal_completion_budget_ms
        + completion_hold_ms
        + max(command_ttl_ms, local_command_watchdog_ms)
        + plan_timing_guard_ms
    )
    required_validity_ms = ceil_milliseconds(
        mini_arrival_delay_ms + post_tangent_reserve_ms
    )
    if validity_ms < required_validity_ms:
        raise ValueError(
            "plan_validity_insufficient:"
            f"requested={validity_ms}ms,"
            f"required={required_validity_ms}ms,"
            f"mini_arrival={mini_arrival_delay_ms}ms,"
            f"post_tangent_reserve={post_tangent_reserve_ms}ms"
        )
    approach_duration_s = max(
        minimum_carrier_time_s,
        mini_arrival_delay_s - terminal_lead_ms / 1000.0,
    )
    approach_duration_ms = max(1, int(round(approach_duration_s * 1000.0)))

    approach = CarrierApproach(
        kind="straight_tangent",
        start=carrier_position,
        end=tangent.point,
        length_m=approach_length,
        duration_ms=approach_duration_ms,
        planned_speed_mps=approach_length / (approach_duration_ms / 1000.0),
    )
    return GroundCorridorPlan(
        schema_version=2,
        plan_id=plan_id,
        sequence=sequence,
        frame_id=frame_id,
        origin_id=origin_id,
        sender_monotonic_ms=sender_monotonic_ms & UINT32_MASK,
        valid_until_sender_monotonic_ms=uint32_add(
            sender_monotonic_ms,
            validity_ms,
        ),
        validity_ms=validity_ms,
        requested_validity_ms=validity_ms,
        required_validity_ms=required_validity_ms,
        validity_margin_ms=validity_ms - required_validity_ms,
        post_tangent_reserve_ms=post_tangent_reserve_ms,
        terminal_completion_budget_ms=terminal_completion_budget_ms,
        completion_hold_ms=completion_hold_ms,
        plan_timing_guard_ms=plan_timing_guard_ms,
        validity_policy="reject",
        validity_extended=False,
        orbit_center=orbit_center,
        orbit_radius_m=orbit_radius_m,
        turn_direction=turn_direction,
        tangent_point=tangent.point,
        tangent_direction=tangent.direction,
        tangent_phase_rad=tangent.phase_rad,
        mini_phase_at_plan_rad=mini_phase_rad % (2.0 * math.pi),
        mini_exit_delta_rad=exit_delta,
        mini_arrival_delay_ms=mini_arrival_delay_ms,
        required_stable_orbit_laps=required_stable_orbit_laps,
        terminal_length_m=terminal_length_m,
        target_front_gap_m=target_front_gap_m,
        mini_speed_mps=mini_speed_mps,
        mini_max_accel_mps2=mini_max_accel_mps2,
        carrier_max_speed_mps=carrier_max_speed_mps,
        carrier_max_accel_mps2=carrier_max_accel_mps2,
        command_ttl_ms=command_ttl_ms,
        local_command_watchdog_ms=local_command_watchdog_ms,
        mini_state_stale_ms=mini_state_stale_ms,
        carrier_approach=approach,
    )
