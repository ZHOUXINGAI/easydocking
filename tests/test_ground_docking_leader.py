from __future__ import annotations

import dataclasses
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPTS_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPTS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from ground_corridor_geometry import uint32_add  # noqa: E402
from ground_docking_leader import (  # noqa: E402
    CommandPhase,
    CommandWatchdog,
    GroundDockingLeader,
    GroundPlanCommand,
    LeaderConfig,
    LeaderPhase,
    VehicleState,
)
from run_ground_docking_replay import (  # noqa: E402
    ReplayHarness,
    advance_to_terminal,
    orbit_state,
    run_all,
    run_fault_replays,
    run_nominal_replay,
)


def make_command(
    *,
    sequence: int = 1,
    sender_ms: int = 4_000_000_000,
    ttl_ms: int = 500,
) -> GroundPlanCommand:
    return GroundPlanCommand(
        schema_version=1,
        plan_id=10,
        sequence=sequence,
        target_role="mini",
        phase=CommandPhase.HOLD,
        sender_monotonic_ms=sender_ms,
        valid_until_sender_monotonic_ms=uint32_add(sender_ms, ttl_ms),
        ttl_ms=ttl_ms,
        body_speed_mps=0.0,
        yaw_rate_radps=0.0,
        max_speed_mps=0.9,
        max_accel_mps2=0.3,
        frame_id="field_enu",
        origin_id=1,
    )


class GroundDockingLeaderTest(unittest.TestCase):
    def test_nominal_replay_requires_sustained_completion(self) -> None:
        result = run_nominal_replay()
        self.assertTrue(result["pass"])
        self.assertEqual(result["final_phase"], LeaderPhase.COMPLETE_HOLD.value)
        self.assertGreaterEqual(result["completion_hold_ms"], 500)
        self.assertEqual(
            result["phases"],
            [
                LeaderPhase.WAIT_MINI_STABLE_ORBIT.value,
                LeaderPhase.PLAN_VALIDATED.value,
                LeaderPhase.CARRIER_ARC_MINI_ORBIT.value,
                LeaderPhase.MINI_TANGENT_EXIT.value,
                LeaderPhase.SHARED_TERMINAL.value,
                LeaderPhase.COMPLETE_HOLD.value,
            ],
        )

    def test_nominal_replay_is_deterministic(self) -> None:
        self.assertEqual(run_nominal_replay(), run_nominal_replay())
        result = run_all()
        self.assertTrue(result["deterministic_repeatability"])
        self.assertTrue(result["pass"])

    def test_fault_matrix_covers_required_runtime_faults(self) -> None:
        result = run_fault_replays()
        self.assertTrue(result["pass"])
        self.assertEqual(
            result["checks"],
            {
                "stale_state": True,
                "sequence_regression": True,
                "carrier_behind": True,
                "pre_tangent_completion_blocked": True,
                "tangent_degeneracy_rejected": True,
                "plan_validity_rejected": True,
                "expired_command": True,
                "abort_latched": True,
                "reset_new_attempt": True,
                "nominal_reference": True,
            },
        )

    def test_command_expiry_uses_local_receipt_not_absolute_sender_clock(self) -> None:
        watchdog = CommandWatchdog()
        command = make_command(sender_ms=0xFFFFFF00, ttl_ms=500)
        watchdog.accept(command, local_received_ms=17)
        self.assertIsNone(watchdog.stop_reason(517))
        self.assertEqual(watchdog.stop_reason(518), "command_expired")

    def test_watchdog_can_expire_before_a_longer_command_ttl(self) -> None:
        watchdog = CommandWatchdog(watchdog_ms=750, maximum_ttl_ms=2_000)
        watchdog.accept(make_command(ttl_ms=1_000), local_received_ms=100)
        self.assertEqual(
            watchdog.stop_reason(851),
            "command_watchdog_expired",
        )

    def test_command_sequence_regression_and_uint32_wrap(self) -> None:
        watchdog = CommandWatchdog()
        watchdog.accept(make_command(sequence=0xFFFFFFFE), local_received_ms=0)
        watchdog.accept(make_command(sequence=1), local_received_ms=10)
        with self.assertRaisesRegex(ValueError, "command_sequence_regression"):
            watchdog.accept(make_command(sequence=0xFFFFFFFD), local_received_ms=20)

    def test_abort_latches_until_local_reset_and_new_attempt(self) -> None:
        leader = GroundDockingLeader()
        leader.start_attempt(attempt_id=1, now_local_ms=0)
        invalid = VehicleState(
            vehicle_id=2,
            sequence=1,
            sender_monotonic_ms=10,
            received_local_ms=0,
            frame_id="wrong_frame",
            origin_id=1,
            position=(4.5, 0.0),
            velocity=(0.0, 0.9),
            yaw_rad=math.pi / 2.0,
        )
        leader.accept_mini_state(invalid)
        self.assertEqual(leader.phase, LeaderPhase.ABORT_LATCHED)
        first_reason = leader.abort_reason

        leader.accept_mini_state(dataclasses.replace(invalid, vehicle_id=99))
        self.assertEqual(leader.abort_reason, first_reason)
        with self.assertRaises(RuntimeError):
            leader.start_attempt(attempt_id=2, now_local_ms=10)

        leader.reset_to_hold()
        self.assertEqual(leader.phase, LeaderPhase.HOLD)
        self.assertIsNone(leader.abort_reason)
        leader.start_attempt(attempt_id=2, now_local_ms=20)
        self.assertEqual(leader.phase, LeaderPhase.WAIT_MINI_STABLE_ORBIT)

    def test_plan_is_frozen_and_created_only_after_one_stable_lap(self) -> None:
        harness = ReplayHarness()
        harness.leader.start_attempt(attempt_id=77, now_local_ms=0)
        phase = 0.0
        angular_step = (
            harness.config.mini_speed_mps / harness.config.orbit_radius_m * 0.1
        )
        carrier = (-7.0, -6.0)
        snapshot = None
        for index in range(400):
            mini_position, mini_velocity = orbit_state(harness.config, phase)
            snapshot = harness.feed(
                carrier_position=carrier,
                carrier_velocity=(0.0, 0.0),
                mini_position=mini_position,
                mini_velocity=mini_velocity,
            )
            phase += angular_step
            if index < 300:
                self.assertIsNone(snapshot.plan)
            if snapshot.plan is not None:
                break
        assert snapshot is not None and snapshot.plan is not None
        self.assertGreaterEqual(snapshot.stable_orbit_laps, 1.0)
        self.assertEqual(snapshot.plan.plan_id, 77)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            snapshot.plan.plan_id = 88  # type: ignore[misc]

    def test_mini_state_sequence_and_sender_time_accept_uint32_wrap(self) -> None:
        config = LeaderConfig()
        leader = GroundDockingLeader(config)
        leader.start_attempt(attempt_id=5, now_local_ms=0)
        position, velocity = orbit_state(config, 0.0)

        first = VehicleState(
            vehicle_id=2,
            sequence=0xFFFFFFFE,
            sender_monotonic_ms=0xFFFFFFF0,
            received_local_ms=0,
            frame_id=config.frame_id,
            origin_id=config.origin_id,
            position=position,
            velocity=velocity,
            yaw_rad=math.pi / 2.0,
        )
        second = dataclasses.replace(
            first,
            sequence=1,
            sender_monotonic_ms=20,
            received_local_ms=10,
        )
        self.assertTrue(leader.accept_mini_state(first))
        self.assertTrue(leader.accept_mini_state(second))
        self.assertNotEqual(leader.phase, LeaderPhase.ABORT_LATCHED)

    def test_nonfinite_mini_state_is_rejected_as_latched_abort(self) -> None:
        config = LeaderConfig()
        leader = GroundDockingLeader(config)
        leader.start_attempt(attempt_id=6, now_local_ms=0)
        invalid = VehicleState(
            vehicle_id=2,
            sequence=1,
            sender_monotonic_ms=10,
            received_local_ms=0,
            frame_id=config.frame_id,
            origin_id=config.origin_id,
            position=(math.nan, 0.0),
            velocity=(0.0, math.inf),
            yaw_rad=math.nan,
        )
        self.assertFalse(leader.accept_mini_state(invalid))
        self.assertEqual(leader.phase, LeaderPhase.ABORT_LATCHED)
        self.assertEqual(leader.abort_reason, "invalid_mini_frame_or_health")

    def test_mini_sender_timestamp_regression_aborts_without_clock_comparison(
        self,
    ) -> None:
        config = LeaderConfig()
        leader = GroundDockingLeader(config)
        leader.start_attempt(attempt_id=7, now_local_ms=0)
        position, velocity = orbit_state(config, 0.0)
        first = VehicleState(
            vehicle_id=2,
            sequence=1,
            sender_monotonic_ms=1_000,
            received_local_ms=5,
            frame_id=config.frame_id,
            origin_id=config.origin_id,
            position=position,
            velocity=velocity,
            yaw_rad=math.pi / 2.0,
        )
        second = dataclasses.replace(
            first,
            sequence=2,
            sender_monotonic_ms=999,
            received_local_ms=6,
        )
        self.assertTrue(leader.accept_mini_state(first))
        self.assertFalse(leader.accept_mini_state(second))
        self.assertEqual(leader.phase, LeaderPhase.ABORT_LATCHED)
        self.assertEqual(leader.abort_reason, "mini_sender_timestamp_regression")

    def test_terminal_entry_rejects_carrier_behind_before_any_terminal_command(
        self,
    ) -> None:
        harness = ReplayHarness()
        snapshot = advance_to_terminal(harness, carrier_behind=True)
        self.assertEqual(snapshot.phase, LeaderPhase.ABORT_LATCHED)
        self.assertEqual(snapshot.abort_reason, "carrier_behind_violation")
        self.assertEqual(snapshot.carrier_command.phase, CommandPhase.ABORT)
        self.assertEqual(snapshot.mini_command.phase, CommandPhase.ABORT)
        self.assertEqual(snapshot.carrier_command.body_speed_mps, 0.0)
        self.assertEqual(snapshot.mini_command.body_speed_mps, 0.0)

    def test_completion_hold_does_not_start_before_both_rovers_cross_tangent(
        self,
    ) -> None:
        harness = ReplayHarness()
        snapshot = advance_to_terminal(harness)
        self.assertEqual(snapshot.phase, LeaderPhase.SHARED_TERMINAL)
        plan = snapshot.plan
        assert plan is not None

        for _ in range(7):
            snapshot = harness.feed(
                carrier_position=(
                    plan.tangent_point[0] - 0.05 * plan.tangent_direction[0],
                    plan.tangent_point[1] - 0.05 * plan.tangent_direction[1],
                ),
                carrier_velocity=(
                    0.1 * plan.tangent_direction[0],
                    0.1 * plan.tangent_direction[1],
                ),
                mini_position=(
                    plan.tangent_point[0] - 0.20 * plan.tangent_direction[0],
                    plan.tangent_point[1] - 0.20 * plan.tangent_direction[1],
                ),
                mini_velocity=(
                    0.1 * plan.tangent_direction[0],
                    0.1 * plan.tangent_direction[1],
                ),
            )
        self.assertEqual(snapshot.phase, LeaderPhase.SHARED_TERMINAL)
        self.assertEqual(snapshot.completion_hold_ms, 0)

    def test_invalid_orbit_sample_resets_stable_lap_qualification(self) -> None:
        harness = ReplayHarness(LeaderConfig(plan_validity_ms=50_000))
        harness.leader.start_attempt(attempt_id=91, now_local_ms=0)
        phase = 0.0
        angular_step = (
            harness.config.mini_speed_mps / harness.config.orbit_radius_m * 0.1
        )
        carrier = (-7.0, -6.0)

        for _ in range(150):
            position, velocity = orbit_state(harness.config, phase)
            snapshot = harness.feed(
                carrier_position=carrier,
                carrier_velocity=(0.0, 0.0),
                mini_position=position,
                mini_velocity=velocity,
            )
            phase += angular_step
        self.assertIsNone(snapshot.plan)

        snapshot = harness.feed(
            carrier_position=carrier,
            carrier_velocity=(0.0, 0.0),
            mini_position=harness.config.orbit_center,
            mini_velocity=(0.0, 0.0),
        )
        self.assertEqual(snapshot.stable_orbit_laps, 0.0)

        for _ in range(300):
            position, velocity = orbit_state(harness.config, phase)
            snapshot = harness.feed(
                carrier_position=carrier,
                carrier_velocity=(0.0, 0.0),
                mini_position=position,
                mini_velocity=velocity,
            )
            phase += angular_step
        self.assertIsNone(snapshot.plan)

        for _ in range(30):
            position, velocity = orbit_state(harness.config, phase)
            snapshot = harness.feed(
                carrier_position=carrier,
                carrier_velocity=(0.0, 0.0),
                mini_position=position,
                mini_velocity=velocity,
            )
            phase += angular_step
            if snapshot.plan is not None:
                break
        self.assertIsNotNone(snapshot.plan)

    def test_plan_expiry_uses_leader_local_elapsed_time(self) -> None:
        harness = ReplayHarness()
        snapshot = advance_to_terminal(harness)
        plan = snapshot.plan
        assert plan is not None
        harness.now_ms += plan.validity_ms + 1
        harness.sender_ms = uint32_add(harness.sender_ms, plan.validity_ms + 1)
        snapshot = harness.feed(
            carrier_position=plan.tangent_point,
            carrier_velocity=(0.0, 0.0),
            mini_position=plan.tangent_point,
            mini_velocity=(0.0, 0.0),
        )
        self.assertEqual(snapshot.phase, LeaderPhase.ABORT_LATCHED)
        self.assertEqual(snapshot.abort_reason, "plan_expired")

    def test_config_rejects_nonfinite_and_negative_safety_limits(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            LeaderConfig(mini_speed_mps=math.nan)
        with self.assertRaisesRegex(ValueError, "finite and nonnegative"):
            LeaderConfig(carrier_ahead_tolerance_m=-0.01)


if __name__ == "__main__":
    unittest.main()
