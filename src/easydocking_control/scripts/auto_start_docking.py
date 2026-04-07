#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from easydocking_msgs.msg import DockingCommand, DockingStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import AirspeedValidated, TecsStatus
except ImportError:  # pragma: no cover
    AirspeedValidated = None
    TecsStatus = None


class AutoStartDocking(Node):
    def __init__(self) -> None:
        super().__init__("auto_start_docking")
        self.declare_parameter("min_distance", 24.0)
        self.declare_parameter("max_distance", 34.0)
        self.declare_parameter("stable_samples", 3)
        self.declare_parameter("timeout_sec", 25.0)
        self.declare_parameter("min_wait_sec", 0.0)
        self.declare_parameter("max_rel_z", 1.2)
        self.declare_parameter("min_rel_z", -0.3)
        self.declare_parameter("min_rel_x", -1000.0)
        self.declare_parameter("max_rel_x", 1000.0)
        self.declare_parameter("min_rel_y", -1000.0)
        self.declare_parameter("max_rel_y", 1000.0)
        self.declare_parameter("republish_period_sec", 1.0)
        self.declare_parameter("max_republish_count", 4)
        self.declare_parameter("use_local_min_trigger", True)
        self.declare_parameter("local_min_increase_margin", 0.15)
        self.declare_parameter("local_min_max_distance", 8.0)
        self.declare_parameter("local_min_min_samples", 6)
        self.declare_parameter("local_min_max_abs_y", 6.0)
        self.declare_parameter("local_min_window_start_sec", 0.0)
        self.declare_parameter("local_min_window_end_sec", -1.0)
        self.declare_parameter("use_state_machine_trigger", True)
        self.declare_parameter("sm_window_start_sec", 0.0)
        self.declare_parameter("sm_window_end_sec", -1.0)
        self.declare_parameter("sm_observe_distance", 18.0)
        self.declare_parameter("sm_observe_abs_y", 10.0)
        self.declare_parameter("sm_trigger_distance", 6.5)
        self.declare_parameter("sm_trigger_abs_y", 4.5)
        self.declare_parameter("sm_trigger_abs_x", 8.0)
        self.declare_parameter("sm_min_converging_samples", 6)
        self.declare_parameter("sm_enter_max_closing_rate", -0.3)
        self.declare_parameter("sm_stay_max_closing_rate", 0.1)
        self.declare_parameter("sm_rebound_margin", 0.08)
        self.declare_parameter("sm_max_relative_speed", 9.0)
        self.declare_parameter("sm_stagnation_samples", 4)
        self.declare_parameter("sm_trigger_min_closing_rate", -0.4)
        self.declare_parameter("sm_trigger_max_abs_vy", 5.0)
        self.declare_parameter("sm_trigger_projected_abs_y", 2.2)
        self.declare_parameter("sm_enable_search_trigger", False)
        self.declare_parameter("sm_search_trigger_min_distance", 18.0)
        self.declare_parameter("sm_search_trigger_max_distance", 40.0)
        self.declare_parameter("sm_search_trigger_abs_y", 18.0)
        self.declare_parameter("sm_search_trigger_max_closing_rate", 1.5)
        self.declare_parameter("sm_search_trigger_projected_abs_y", 10.0)
        self.declare_parameter("sm_search_min_samples", 6)
        self.declare_parameter("sm_enable_orbit_phase_trigger", False)
        self.declare_parameter("sm_orbit_trigger_min_distance", 110.0)
        self.declare_parameter("sm_orbit_trigger_max_distance", 140.0)
        self.declare_parameter("sm_orbit_trigger_min_rel_x", -10.0)
        self.declare_parameter("sm_orbit_trigger_max_rel_x", 25.0)
        self.declare_parameter("sm_orbit_trigger_min_rel_y", -135.0)
        self.declare_parameter("sm_orbit_trigger_max_rel_y", -100.0)
        self.declare_parameter("sm_orbit_trigger_min_rel_vx", 10.0)
        self.declare_parameter("sm_orbit_trigger_max_abs_vy", 6.0)
        self.declare_parameter("sm_orbit_trigger_max_abs_vz", 0.30)
        self.declare_parameter("sm_orbit_trigger_min_samples", 4)
        self.declare_parameter("sm_orbit_trigger_stagnation_samples", 3)
        self.declare_parameter("sm_orbit_score_target_distance", 134.0)
        self.declare_parameter("sm_orbit_score_target_rel_x", 15.0)
        self.declare_parameter("sm_orbit_score_target_rel_y", -129.0)
        self.declare_parameter("sm_enable_idle_orbit_trigger", False)
        self.declare_parameter("sm_idle_orbit_trigger_min_distance", 38.0)
        self.declare_parameter("sm_idle_orbit_trigger_max_distance", 50.0)
        self.declare_parameter("sm_idle_orbit_trigger_min_rel_x", 20.0)
        self.declare_parameter("sm_idle_orbit_trigger_max_rel_x", 35.0)
        self.declare_parameter("sm_idle_orbit_trigger_min_rel_y", -20.0)
        self.declare_parameter("sm_idle_orbit_trigger_max_rel_y", -8.0)
        self.declare_parameter("sm_idle_orbit_trigger_min_rel_vx", 4.0)
        self.declare_parameter("sm_idle_orbit_trigger_max_rel_vx", 8.0)
        self.declare_parameter("sm_idle_orbit_trigger_min_rel_vy", 8.0)
        self.declare_parameter("sm_idle_orbit_trigger_max_rel_vy", 13.0)
        self.declare_parameter("sm_idle_orbit_trigger_max_abs_vz", 1.0)
        self.declare_parameter("sm_idle_orbit_trigger_min_samples", 8)
        self.declare_parameter("sm_enable_approach_side_trigger", True)
        self.declare_parameter("sm_approach_side_trigger_min_distance", 80.0)
        self.declare_parameter("sm_approach_side_trigger_max_distance", 110.0)
        self.declare_parameter("sm_approach_side_trigger_min_closing_rate", -2.8)
        self.declare_parameter("sm_approach_side_trigger_min_time_to_closest", 1.8)
        self.declare_parameter("sm_approach_side_trigger_max_time_to_closest", 3.5)
        self.declare_parameter("sm_approach_side_trigger_max_abs_vy", 4.0)
        self.declare_parameter("sm_approach_side_trigger_max_abs_vz", 1.0)
        self.declare_parameter("sm_approach_side_trigger_min_samples", 3)
        self.declare_parameter("require_orbit_completion_before_start", True)
        self.declare_parameter("orbit_center_x", 10.0)
        self.declare_parameter("orbit_center_y", -6.0)
        self.declare_parameter("orbit_radius", 80.0)
        self.declare_parameter("orbit_gate_ready_altitude", 28.0)
        self.declare_parameter("orbit_gate_ready_radius_tolerance", 30.0)
        self.declare_parameter("orbit_gate_min_valid_samples", 8)
        self.declare_parameter("orbit_gate_required_laps", 1.0)
        self.declare_parameter("orbit_gate_min_accumulated_angle_deg", 330.0)
        self.declare_parameter("orbit_gate_return_tolerance_deg", 45.0)
        self.declare_parameter("require_mini_energy_healthy", False)
        self.declare_parameter("mini_px4_namespace", "/px4_2")
        self.declare_parameter("health_min_true_airspeed_mps", 7.0)
        self.declare_parameter("health_max_underspeed_ratio", 0.35)
        self.declare_parameter("health_min_samples", 5)

        self.min_distance = float(self.get_parameter("min_distance").value)
        self.max_distance = float(self.get_parameter("max_distance").value)
        self.stable_samples = int(self.get_parameter("stable_samples").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.min_wait_sec = float(self.get_parameter("min_wait_sec").value)
        self.max_rel_z = float(self.get_parameter("max_rel_z").value)
        self.min_rel_z = float(self.get_parameter("min_rel_z").value)
        self.min_rel_x = float(self.get_parameter("min_rel_x").value)
        self.max_rel_x = float(self.get_parameter("max_rel_x").value)
        self.min_rel_y = float(self.get_parameter("min_rel_y").value)
        self.max_rel_y = float(self.get_parameter("max_rel_y").value)
        self.republish_period_sec = float(self.get_parameter("republish_period_sec").value)
        self.max_republish_count = int(self.get_parameter("max_republish_count").value)
        self.use_local_min_trigger = bool(self.get_parameter("use_local_min_trigger").value)
        self.local_min_increase_margin = float(self.get_parameter("local_min_increase_margin").value)
        self.local_min_max_distance = float(self.get_parameter("local_min_max_distance").value)
        self.local_min_min_samples = int(self.get_parameter("local_min_min_samples").value)
        self.local_min_max_abs_y = float(self.get_parameter("local_min_max_abs_y").value)
        self.local_min_window_start_sec = float(self.get_parameter("local_min_window_start_sec").value)
        self.local_min_window_end_sec = float(self.get_parameter("local_min_window_end_sec").value)
        self.use_state_machine_trigger = bool(self.get_parameter("use_state_machine_trigger").value)
        self.sm_window_start_sec = float(self.get_parameter("sm_window_start_sec").value)
        self.sm_window_end_sec = float(self.get_parameter("sm_window_end_sec").value)
        self.sm_observe_distance = float(self.get_parameter("sm_observe_distance").value)
        self.sm_observe_abs_y = float(self.get_parameter("sm_observe_abs_y").value)
        self.sm_trigger_distance = float(self.get_parameter("sm_trigger_distance").value)
        self.sm_trigger_abs_y = float(self.get_parameter("sm_trigger_abs_y").value)
        self.sm_trigger_abs_x = float(self.get_parameter("sm_trigger_abs_x").value)
        self.sm_min_converging_samples = int(self.get_parameter("sm_min_converging_samples").value)
        self.sm_enter_max_closing_rate = float(self.get_parameter("sm_enter_max_closing_rate").value)
        self.sm_stay_max_closing_rate = float(self.get_parameter("sm_stay_max_closing_rate").value)
        self.sm_rebound_margin = float(self.get_parameter("sm_rebound_margin").value)
        self.sm_max_relative_speed = float(self.get_parameter("sm_max_relative_speed").value)
        self.sm_stagnation_samples = int(self.get_parameter("sm_stagnation_samples").value)
        self.sm_trigger_min_closing_rate = float(self.get_parameter("sm_trigger_min_closing_rate").value)
        self.sm_trigger_max_abs_vy = float(self.get_parameter("sm_trigger_max_abs_vy").value)
        self.sm_trigger_projected_abs_y = float(self.get_parameter("sm_trigger_projected_abs_y").value)
        self.sm_enable_search_trigger = bool(self.get_parameter("sm_enable_search_trigger").value)
        self.sm_search_trigger_min_distance = float(
            self.get_parameter("sm_search_trigger_min_distance").value
        )
        self.sm_search_trigger_max_distance = float(
            self.get_parameter("sm_search_trigger_max_distance").value
        )
        self.sm_search_trigger_abs_y = float(
            self.get_parameter("sm_search_trigger_abs_y").value
        )
        self.sm_search_trigger_max_closing_rate = float(
            self.get_parameter("sm_search_trigger_max_closing_rate").value
        )
        self.sm_search_trigger_projected_abs_y = float(
            self.get_parameter("sm_search_trigger_projected_abs_y").value
        )
        self.sm_search_min_samples = int(self.get_parameter("sm_search_min_samples").value)
        self.sm_enable_orbit_phase_trigger = bool(
            self.get_parameter("sm_enable_orbit_phase_trigger").value
        )
        self.sm_orbit_trigger_min_distance = float(
            self.get_parameter("sm_orbit_trigger_min_distance").value
        )
        self.sm_orbit_trigger_max_distance = float(
            self.get_parameter("sm_orbit_trigger_max_distance").value
        )
        self.sm_orbit_trigger_min_rel_x = float(
            self.get_parameter("sm_orbit_trigger_min_rel_x").value
        )
        self.sm_orbit_trigger_max_rel_x = float(
            self.get_parameter("sm_orbit_trigger_max_rel_x").value
        )
        self.sm_orbit_trigger_min_rel_y = float(
            self.get_parameter("sm_orbit_trigger_min_rel_y").value
        )
        self.sm_orbit_trigger_max_rel_y = float(
            self.get_parameter("sm_orbit_trigger_max_rel_y").value
        )
        self.sm_orbit_trigger_min_rel_vx = float(
            self.get_parameter("sm_orbit_trigger_min_rel_vx").value
        )
        self.sm_orbit_trigger_max_abs_vy = float(
            self.get_parameter("sm_orbit_trigger_max_abs_vy").value
        )
        self.sm_orbit_trigger_max_abs_vz = float(
            self.get_parameter("sm_orbit_trigger_max_abs_vz").value
        )
        self.sm_orbit_trigger_min_samples = int(
            self.get_parameter("sm_orbit_trigger_min_samples").value
        )
        self.sm_orbit_trigger_stagnation_samples = int(
            self.get_parameter("sm_orbit_trigger_stagnation_samples").value
        )
        self.sm_orbit_score_target_distance = float(
            self.get_parameter("sm_orbit_score_target_distance").value
        )
        self.sm_orbit_score_target_rel_x = float(
            self.get_parameter("sm_orbit_score_target_rel_x").value
        )
        self.sm_orbit_score_target_rel_y = float(
            self.get_parameter("sm_orbit_score_target_rel_y").value
        )
        self.sm_enable_idle_orbit_trigger = bool(
            self.get_parameter("sm_enable_idle_orbit_trigger").value
        )
        self.sm_idle_orbit_trigger_min_distance = float(
            self.get_parameter("sm_idle_orbit_trigger_min_distance").value
        )
        self.sm_idle_orbit_trigger_max_distance = float(
            self.get_parameter("sm_idle_orbit_trigger_max_distance").value
        )
        self.sm_idle_orbit_trigger_min_rel_x = float(
            self.get_parameter("sm_idle_orbit_trigger_min_rel_x").value
        )
        self.sm_idle_orbit_trigger_max_rel_x = float(
            self.get_parameter("sm_idle_orbit_trigger_max_rel_x").value
        )
        self.sm_idle_orbit_trigger_min_rel_y = float(
            self.get_parameter("sm_idle_orbit_trigger_min_rel_y").value
        )
        self.sm_idle_orbit_trigger_max_rel_y = float(
            self.get_parameter("sm_idle_orbit_trigger_max_rel_y").value
        )
        self.sm_idle_orbit_trigger_min_rel_vx = float(
            self.get_parameter("sm_idle_orbit_trigger_min_rel_vx").value
        )
        self.sm_idle_orbit_trigger_max_rel_vx = float(
            self.get_parameter("sm_idle_orbit_trigger_max_rel_vx").value
        )
        self.sm_idle_orbit_trigger_min_rel_vy = float(
            self.get_parameter("sm_idle_orbit_trigger_min_rel_vy").value
        )
        self.sm_idle_orbit_trigger_max_rel_vy = float(
            self.get_parameter("sm_idle_orbit_trigger_max_rel_vy").value
        )
        self.sm_idle_orbit_trigger_max_abs_vz = float(
            self.get_parameter("sm_idle_orbit_trigger_max_abs_vz").value
        )
        self.sm_idle_orbit_trigger_min_samples = int(
            self.get_parameter("sm_idle_orbit_trigger_min_samples").value
        )
        self.sm_enable_approach_side_trigger = bool(
            self.get_parameter("sm_enable_approach_side_trigger").value
        )
        self.sm_approach_side_trigger_min_distance = float(
            self.get_parameter("sm_approach_side_trigger_min_distance").value
        )
        self.sm_approach_side_trigger_max_distance = float(
            self.get_parameter("sm_approach_side_trigger_max_distance").value
        )
        self.sm_approach_side_trigger_min_closing_rate = float(
            self.get_parameter("sm_approach_side_trigger_min_closing_rate").value
        )
        self.sm_approach_side_trigger_min_time_to_closest = float(
            self.get_parameter("sm_approach_side_trigger_min_time_to_closest").value
        )
        self.sm_approach_side_trigger_max_time_to_closest = float(
            self.get_parameter("sm_approach_side_trigger_max_time_to_closest").value
        )
        self.sm_approach_side_trigger_max_abs_vy = float(
            self.get_parameter("sm_approach_side_trigger_max_abs_vy").value
        )
        self.sm_approach_side_trigger_max_abs_vz = float(
            self.get_parameter("sm_approach_side_trigger_max_abs_vz").value
        )
        self.sm_approach_side_trigger_min_samples = int(
            self.get_parameter("sm_approach_side_trigger_min_samples").value
        )
        self.require_orbit_completion_before_start = bool(
            self.get_parameter("require_orbit_completion_before_start").value
        )
        self.orbit_center_x = float(self.get_parameter("orbit_center_x").value)
        self.orbit_center_y = float(self.get_parameter("orbit_center_y").value)
        self.orbit_radius = float(self.get_parameter("orbit_radius").value)
        self.orbit_gate_ready_altitude = float(
            self.get_parameter("orbit_gate_ready_altitude").value
        )
        self.orbit_gate_ready_radius_tolerance = float(
            self.get_parameter("orbit_gate_ready_radius_tolerance").value
        )
        self.orbit_gate_min_valid_samples = int(
            self.get_parameter("orbit_gate_min_valid_samples").value
        )
        self.orbit_gate_required_laps = float(
            self.get_parameter("orbit_gate_required_laps").value
        )
        self.orbit_gate_min_accumulated_angle_rad = math.radians(float(
            self.get_parameter("orbit_gate_min_accumulated_angle_deg").value
        ))
        self.orbit_gate_return_tolerance_rad = math.radians(float(
            self.get_parameter("orbit_gate_return_tolerance_deg").value
        ))
        self.require_mini_energy_healthy = bool(
            self.get_parameter("require_mini_energy_healthy").value
        )
        self.mini_px4_namespace = str(self.get_parameter("mini_px4_namespace").value)
        self.health_min_true_airspeed_mps = float(
            self.get_parameter("health_min_true_airspeed_mps").value
        )
        self.health_max_underspeed_ratio = float(
            self.get_parameter("health_max_underspeed_ratio").value
        )
        self.health_min_samples = int(self.get_parameter("health_min_samples").value)
        self.start_wall = time.time()
        self.below_counter = 0
        self.sent = False
        self.republish_count = 0
        self.last_publish_wall = 0.0
        self.sample_count = 0
        self.best_candidate = None
        self.last_distance = None
        self.increase_counter = 0
        self.trigger_state = "SEARCHING"
        self.trigger_best_candidate = None
        self.trigger_last_distance = None
        self.trigger_history = deque(maxlen=max(3, self.sm_min_converging_samples))
        self.trigger_state_announced = None
        self.trigger_stagnation_samples = 0
        self.trigger_candidate_announced = False
        self.search_ready_samples = 0
        self.search_best_candidate = None
        self.orbit_ready_samples = 0
        self.orbit_best_candidate = None
        self.orbit_best_score = None
        self.orbit_stagnation_samples = 0
        self.orbit_candidate_announced = False
        self.idle_orbit_ready_samples = 0
        self.idle_orbit_best_candidate = None
        self.idle_orbit_candidate_announced = False
        self.approach_side_ready_samples = 0
        self.approach_side_best_candidate = None
        self.approach_side_candidate_announced = False
        self.orbit_gate_ready_samples = 0
        self.orbit_gate_tracking_active = not self.require_orbit_completion_before_start
        self.orbit_gate_completed = not self.require_orbit_completion_before_start
        self.orbit_gate_initial_angle = None
        self.orbit_gate_previous_angle = None
        self.orbit_gate_accumulated_angle = 0.0
        self.orbit_gate_last_reported_quarter = -1
        self.orbit_gate_completed_wall = self.start_wall if self.orbit_gate_completed else None
        self.mini_true_airspeed_mps = math.nan
        self.mini_underspeed_ratio = math.nan
        self.mini_health_good_samples = 0
        self.mini_health_announced = None

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_pub = self.create_publisher(DockingCommand, "/docking/command", 10)
        self.command_latched_pub = self.create_publisher(DockingCommand, "/docking/command_latched", latched_qos)
        self.create_subscription(DockingStatus, "/docking/status", self._status_cb, 10)
        self.create_subscription(Odometry, "/mini/odom", self._mini_odom_cb, 10)
        if self.require_mini_energy_healthy and AirspeedValidated is not None:
            self.create_subscription(
                AirspeedValidated,
                f"{self.mini_px4_namespace}/fmu/out/airspeed_validated",
                self._mini_airspeed_cb,
                px4_qos,
            )
            self.create_subscription(
                AirspeedValidated,
                f"{self.mini_px4_namespace}/fmu/out/airspeed_validated_v1",
                self._mini_airspeed_cb,
                px4_qos,
            )
        if self.require_mini_energy_healthy and TecsStatus is not None:
            self.create_subscription(
                TecsStatus,
                f"{self.mini_px4_namespace}/fmu/out/tecs_status",
                self._mini_tecs_cb,
                px4_qos,
            )
            self.create_subscription(
                TecsStatus,
                f"{self.mini_px4_namespace}/fmu/out/tecs_status_v1",
                self._mini_tecs_cb,
                px4_qos,
            )
        self.timer = self.create_timer(0.1, self._timeout_cb)

    def _mini_odom_cb(self, msg: Odometry) -> None:
        if self.orbit_gate_completed:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        dx = x - self.orbit_center_x
        dy = y - self.orbit_center_y
        radius = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        ready = (
            z >= self.orbit_gate_ready_altitude and
            abs(radius - self.orbit_radius) <= self.orbit_gate_ready_radius_tolerance
        )

        if not self.orbit_gate_tracking_active:
            if ready:
                self.orbit_gate_ready_samples += 1
                if self.orbit_gate_ready_samples >= self.orbit_gate_min_valid_samples:
                    self.orbit_gate_tracking_active = True
                    self.orbit_gate_initial_angle = angle
                    self.orbit_gate_previous_angle = angle
                    self.orbit_gate_accumulated_angle = 0.0
                    self.orbit_gate_last_reported_quarter = 0
                    self.get_logger().info(
                        "Orbit gate armed "
                        f"altitude={z:.2f} radius={radius:.2f} "
                        f"laps_required={self.orbit_gate_required_laps:.2f}"
                    )
            else:
                self.orbit_gate_ready_samples = 0
            return

        if self.orbit_gate_previous_angle is None or self.orbit_gate_initial_angle is None:
            self.orbit_gate_initial_angle = angle
            self.orbit_gate_previous_angle = angle
            return

        delta = self._normalize_angle(angle - self.orbit_gate_previous_angle)
        self.orbit_gate_previous_angle = angle
        self.orbit_gate_accumulated_angle += delta
        completed_laps = abs(self.orbit_gate_accumulated_angle) / (2.0 * math.pi)
        reported_quarter = int(completed_laps * 4.0)
        if reported_quarter > self.orbit_gate_last_reported_quarter:
            self.orbit_gate_last_reported_quarter = reported_quarter
            self.get_logger().info(
                "Orbit gate progress "
                f"laps={completed_laps:.2f} altitude={z:.2f} radius={radius:.2f}"
            )

        returned_to_start = (
            abs(self._normalize_angle(angle - self.orbit_gate_initial_angle)) <=
            self.orbit_gate_return_tolerance_rad
        )
        if (
            completed_laps >= self.orbit_gate_required_laps and
            abs(self.orbit_gate_accumulated_angle) >= self.orbit_gate_min_accumulated_angle_rad and
            returned_to_start
        ):
            self.orbit_gate_completed = True
            self.orbit_gate_completed_wall = time.time()
            self.get_logger().info(
                "Orbit gate completed "
                f"laps={completed_laps:.2f} altitude={z:.2f} radius={radius:.2f}"
            )

    def _mini_airspeed_cb(self, msg: AirspeedValidated) -> None:
        self.mini_true_airspeed_mps = float(msg.true_airspeed_m_s)

    def _mini_tecs_cb(self, msg: TecsStatus) -> None:
        self.mini_underspeed_ratio = float(msg.underspeed_ratio)

    def _status_cb(self, msg: DockingStatus) -> None:
        if msg.is_active:
            self.get_logger().info("Auto START confirmed active")
            raise SystemExit(0)

        if self.sent:
            return
        if self.require_orbit_completion_before_start and not self.orbit_gate_completed:
            return
        elapsed = time.time() - self.start_wall
        if elapsed < self.min_wait_sec:
            return
        if not self._mini_energy_healthy():
            self.below_counter = 0
            self.best_candidate = None
            self.last_distance = None
            self.increase_counter = 0
            self._reset_trigger_state()
            return
        self.sample_count += 1

        distance = float(msg.relative_distance)
        rel_x = float(msg.relative_position.x)
        rel_y = float(msg.relative_position.y)
        rel_z = float(msg.relative_position.z)
        rel_vx = float(msg.relative_velocity.x)
        rel_vy = float(msg.relative_velocity.y)
        rel_vz = float(msg.relative_velocity.z)

        if self.use_state_machine_trigger:
            if self._handle_state_machine(
                elapsed=elapsed,
                distance=distance,
                rel_x=rel_x,
                rel_y=rel_y,
                rel_z=rel_z,
                rel_vx=rel_vx,
                rel_vy=rel_vy,
                rel_vz=rel_vz,
            ):
                return
            return

        if self.use_local_min_trigger:
            in_local_min_window = elapsed >= self.local_min_window_start_sec and (
                self.local_min_window_end_sec < 0.0 or elapsed <= self.local_min_window_end_sec
            )
            if not in_local_min_window:
                self.best_candidate = None
                self.last_distance = None
                self.increase_counter = 0
                return

            if (
                self.best_candidate is None or
                distance < self.best_candidate["distance"]
            ):
                self.best_candidate = {
                    "distance": distance,
                    "rel_x": rel_x,
                    "rel_y": rel_y,
                    "rel_z": rel_z,
                }

            if self.last_distance is not None and distance > self.last_distance + self.local_min_increase_margin:
                self.increase_counter += 1
            else:
                self.increase_counter = 0
            self.last_distance = distance

            if (
                self.best_candidate is not None and
                self.sample_count >= self.local_min_min_samples and
                self.best_candidate["distance"] <= self.local_min_max_distance and
                abs(self.best_candidate["rel_y"]) <= self.local_min_max_abs_y and
                self.increase_counter >= 1
            ):
                self.sent = True
                self._publish_start(reason=(
                    f"local-min distance={self.best_candidate['distance']:.3f} "
                    f"rel=({self.best_candidate['rel_x']:.3f},"
                    f"{self.best_candidate['rel_y']:.3f},{self.best_candidate['rel_z']:.3f})"
                ))
                return

        if (
            not self.use_local_min_trigger and
            self.min_distance <= distance <= self.max_distance and
            self.min_rel_x <= rel_x <= self.max_rel_x and
            self.min_rel_y <= rel_y <= self.max_rel_y and
            self.min_rel_z <= rel_z <= self.max_rel_z
        ):
            self.below_counter += 1
        else:
            self.below_counter = 0

        if self.below_counter >= self.stable_samples:
            self.sent = True
            self._publish_start(reason=(
                f"window distance={distance:.3f} "
                f"rel=({rel_x:.3f},"
                f"{rel_y:.3f},{rel_z:.3f})"
            ))

    def _handle_state_machine(
        self,
        *,
        elapsed: float,
        distance: float,
        rel_x: float,
        rel_y: float,
        rel_z: float,
        rel_vx: float,
        rel_vy: float,
        rel_vz: float,
    ) -> bool:
        in_window = elapsed >= self.sm_window_start_sec and (
            self.sm_window_end_sec < 0.0 or elapsed <= self.sm_window_end_sec
        )
        if not in_window:
            self._reset_trigger_state()
            return False

        relative_speed = math.sqrt(rel_vx ** 2 + rel_vy ** 2 + rel_vz ** 2)
        closing_rate = 0.0
        if distance > 1e-6:
            closing_rate = (rel_x * rel_vx + rel_y * rel_vy + rel_z * rel_vz) / distance

        sample = {
            "distance": distance,
            "rel_x": rel_x,
            "rel_y": rel_y,
            "rel_z": rel_z,
            "rel_vx": rel_vx,
            "rel_vy": rel_vy,
            "rel_vz": rel_vz,
            "closing_rate": closing_rate,
            "relative_speed": relative_speed,
        }
        distance_xy = math.hypot(rel_x, rel_y)
        relative_speed_xy_sq = rel_vx ** 2 + rel_vy ** 2
        time_to_closest_xy = math.inf
        if relative_speed_xy_sq > 1e-6:
            time_to_closest_xy = - (rel_x * rel_vx + rel_y * rel_vy) / relative_speed_xy_sq
        sample["time_to_closest_xy"] = time_to_closest_xy

        approach_side_trigger_ok = (
            self.sm_enable_approach_side_trigger and
            self.sm_approach_side_trigger_min_distance <= distance <= self.sm_approach_side_trigger_max_distance and
            closing_rate <= self.sm_approach_side_trigger_min_closing_rate and
            self.sm_approach_side_trigger_min_time_to_closest <= time_to_closest_xy <= self.sm_approach_side_trigger_max_time_to_closest and
            abs(rel_vy) <= self.sm_approach_side_trigger_max_abs_vy and
            abs(rel_vz) <= self.sm_approach_side_trigger_max_abs_vz
        )
        if approach_side_trigger_ok:
            self.approach_side_ready_samples += 1
            if (
                self.approach_side_best_candidate is None or
                distance < float(self.approach_side_best_candidate["distance"])
            ):
                self.approach_side_best_candidate = dict(sample)
            if (
                self.approach_side_ready_samples >= self.sm_approach_side_trigger_min_samples and
                self.approach_side_best_candidate is not None and
                not self.approach_side_candidate_announced
            ):
                candidate = self.approach_side_best_candidate
                self.get_logger().info(
                    "Auto START approach-side candidate-ready "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f}) "
                    f"time_to_closest_xy={candidate['time_to_closest_xy']:.3f}"
                )
                self.approach_side_candidate_announced = True
            if (
                self.approach_side_ready_samples >= self.sm_approach_side_trigger_min_samples and
                self.approach_side_best_candidate is not None
            ):
                candidate = self.approach_side_best_candidate
                self.sent = True
                self._publish_start(reason=(
                    "state-machine-approach-side-ready "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f}) "
                    f"time_to_closest_xy={candidate['time_to_closest_xy']:.3f}"
                ))
                self.trigger_state = "TRIGGERED"
                return True
        else:
            self.approach_side_ready_samples = 0
            self.approach_side_best_candidate = None
            self.approach_side_candidate_announced = False

        idle_orbit_trigger_ok = (
            self.sm_enable_idle_orbit_trigger and
            self.sm_idle_orbit_trigger_min_distance <= distance <= self.sm_idle_orbit_trigger_max_distance and
            self.sm_idle_orbit_trigger_min_rel_x <= rel_x <= self.sm_idle_orbit_trigger_max_rel_x and
            self.sm_idle_orbit_trigger_min_rel_y <= rel_y <= self.sm_idle_orbit_trigger_max_rel_y and
            self.min_rel_z <= rel_z <= self.max_rel_z and
            self.sm_idle_orbit_trigger_min_rel_vx <= rel_vx <= self.sm_idle_orbit_trigger_max_rel_vx and
            self.sm_idle_orbit_trigger_min_rel_vy <= rel_vy <= self.sm_idle_orbit_trigger_max_rel_vy and
            abs(rel_vz) <= self.sm_idle_orbit_trigger_max_abs_vz and
            relative_speed <= self.sm_max_relative_speed
        )
        if idle_orbit_trigger_ok:
            self.idle_orbit_ready_samples += 1
            if (
                self.idle_orbit_best_candidate is None or
                distance < float(self.idle_orbit_best_candidate["distance"])
            ):
                self.idle_orbit_best_candidate = dict(sample)

            if (
                self.idle_orbit_ready_samples >= self.sm_idle_orbit_trigger_min_samples and
                self.idle_orbit_best_candidate is not None and
                not self.idle_orbit_candidate_announced
            ):
                candidate = self.idle_orbit_best_candidate
                self.get_logger().info(
                    "Auto START idle-orbit candidate-ready "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f})"
                )
                self.idle_orbit_candidate_announced = True
        else:
            if (
                self.idle_orbit_ready_samples >= self.sm_idle_orbit_trigger_min_samples and
                self.idle_orbit_best_candidate is not None
            ):
                candidate = self.idle_orbit_best_candidate
                self.sent = True
                self._publish_start(reason=(
                    "state-machine-idle-orbit-window "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f})"
                ))
                self.trigger_state = "TRIGGERED"
                return True
            self._reset_idle_orbit_trigger_state()

        orbit_trigger_ok = (
            self.sm_enable_orbit_phase_trigger and
            self.sm_orbit_trigger_min_distance <= distance <= self.sm_orbit_trigger_max_distance and
            self.sm_orbit_trigger_min_rel_x <= rel_x <= self.sm_orbit_trigger_max_rel_x and
            self.sm_orbit_trigger_min_rel_y <= rel_y <= self.sm_orbit_trigger_max_rel_y and
            self.min_rel_z <= rel_z <= self.max_rel_z and
            rel_vx >= self.sm_orbit_trigger_min_rel_vx and
            abs(rel_vy) <= self.sm_orbit_trigger_max_abs_vy and
            abs(rel_vz) <= self.sm_orbit_trigger_max_abs_vz and
            relative_speed <= self.sm_max_relative_speed
        )
        if orbit_trigger_ok:
            self.orbit_ready_samples += 1
            score = self._orbit_phase_candidate_score(sample)
            if (
                self.orbit_best_candidate is None or
                self.orbit_best_score is None or
                score < self.orbit_best_score
            ):
                self.orbit_best_candidate = dict(sample)
                self.orbit_best_score = score
                self.orbit_stagnation_samples = 0
            else:
                self.orbit_stagnation_samples += 1

            if (
                self.orbit_ready_samples >= self.sm_orbit_trigger_min_samples and
                self.orbit_best_candidate is not None and
                not self.orbit_candidate_announced
            ):
                candidate = self.orbit_best_candidate
                self.get_logger().info(
                    "Auto START orbit-phase candidate-ready "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f}) "
                    f"score={self.orbit_best_score:.3f}"
                )
                self.orbit_candidate_announced = True

            if (
                self.orbit_ready_samples >= self.sm_orbit_trigger_min_samples and
                self.orbit_best_candidate is not None and
                self.orbit_stagnation_samples >= self.sm_orbit_trigger_stagnation_samples
            ):
                candidate = self.orbit_best_candidate or sample
                self.sent = True
                self._publish_start(reason=(
                    "state-machine-orbit-phase-stagnation "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f})"
                ))
                self.trigger_state = "TRIGGERED"
                return True
        else:
            if self.orbit_ready_samples >= self.sm_orbit_trigger_min_samples and self.orbit_best_candidate is not None:
                candidate = self.orbit_best_candidate
                self.sent = True
                self._publish_start(reason=(
                    "state-machine-orbit-phase-window-end "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f})"
                ))
                self.trigger_state = "TRIGGERED"
                return True
            self._reset_orbit_phase_trigger_state()

        observe_ok = (
            distance <= self.sm_observe_distance and
            abs(rel_y) <= self.sm_observe_abs_y and
            self.min_rel_x <= rel_x <= self.max_rel_x and
            self.min_rel_y <= rel_y <= self.max_rel_y and
            self.min_rel_z <= rel_z <= self.max_rel_z and
            relative_speed <= self.sm_max_relative_speed
        )

        if rel_x < 0.0 and rel_vx > 0.2:
            search_time_to_intercept = min(max(-rel_x / rel_vx, 0.0), 4.0)
        else:
            search_time_to_intercept = min(
                max(distance / max(relative_speed, 1e-3), 0.0),
                4.0,
            )
        projected_search_abs_y = abs(rel_y + rel_vy * search_time_to_intercept)
        search_trigger_ok = (
            self.sm_enable_search_trigger and
            self.sm_search_trigger_min_distance <= distance <= self.sm_search_trigger_max_distance and
            abs(rel_y) <= self.sm_search_trigger_abs_y and
            self.min_rel_z <= rel_z <= self.max_rel_z and
            relative_speed <= self.sm_max_relative_speed and
            closing_rate <= self.sm_search_trigger_max_closing_rate and
            projected_search_abs_y <= self.sm_search_trigger_projected_abs_y
        )
        if search_trigger_ok:
            self.search_ready_samples += 1
            if (
                self.search_best_candidate is None or
                self._search_candidate_score(sample, projected_search_abs_y) <
                self._search_candidate_score(
                    self.search_best_candidate,
                    float(self.search_best_candidate["projected_abs_y"]),
                )
            ):
                sample["projected_abs_y"] = projected_search_abs_y
                self.search_best_candidate = sample
        else:
            if self.search_ready_samples >= self.sm_search_min_samples and self.search_best_candidate is not None:
                candidate = self.search_best_candidate
                self.sent = True
                self._publish_start(reason=(
                    "state-machine-search-window "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f}) "
                    f"proj_y={candidate['projected_abs_y']:.3f}"
                ))
                self.trigger_state = "TRIGGERED"
                return True
            self.search_ready_samples = 0
            self.search_best_candidate = None

        if self.trigger_state == "SEARCHING":
            if observe_ok and closing_rate <= self.sm_enter_max_closing_rate:
                self.trigger_state = "CONVERGING"
                self.trigger_history.clear()
                self.trigger_history.append(sample)
                self.trigger_best_candidate = sample
                self.trigger_last_distance = distance
                self.trigger_stagnation_samples = 0
                self._log_trigger_state("CONVERGING", sample)
            return False

        if self.trigger_state != "CONVERGING":
            self._reset_trigger_state()
            return False

        if not observe_ok:
            self._reset_trigger_state()
            return False

        self.trigger_history.append(sample)
        if (
            self.trigger_best_candidate is None or
            self._candidate_score(sample) < self._candidate_score(self.trigger_best_candidate)
        ):
            self.trigger_best_candidate = sample
            self.trigger_stagnation_samples = 0
        else:
            self.trigger_stagnation_samples += 1

        if (
            len(self.trigger_history) >= self.sm_min_converging_samples and
            self._candidate_is_triggerable(sample) and
            closing_rate <= self.sm_trigger_min_closing_rate
        ):
            self.sent = True
            self._publish_start(reason=(
                "state-machine-current "
                f"distance={sample['distance']:.3f} "
                f"rel=({sample['rel_x']:.3f},{sample['rel_y']:.3f},{sample['rel_z']:.3f}) "
                f"vel=({sample['rel_vx']:.3f},{sample['rel_vy']:.3f},{sample['rel_vz']:.3f})"
            ))
            self.trigger_state = "TRIGGERED"
            return True

        if (
            len(self.trigger_history) >= self.sm_min_converging_samples and
            self.trigger_best_candidate is not None and
            self._candidate_is_triggerable(self.trigger_best_candidate)
        ):
            candidate = self.trigger_best_candidate
            if not self.trigger_candidate_announced:
                self.get_logger().info(
                    "Auto START candidate-ready "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f}) "
                    f"closing_rate={closing_rate:.3f} stagnation={self.trigger_stagnation_samples}"
                )
                self.trigger_candidate_announced = True
            if (
                self.trigger_last_distance is not None and
                distance > self.trigger_last_distance + self.sm_rebound_margin and
                closing_rate >= self.sm_stay_max_closing_rate
            ):
                candidate = self.trigger_best_candidate
                self.sent = True
                self._publish_start(reason=(
                    "state-machine "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f})"
                ))
                self.trigger_state = "TRIGGERED"
                return True
            if (
                self.trigger_stagnation_samples >= self.sm_stagnation_samples and
                float(candidate["closing_rate"]) <= self.sm_trigger_min_closing_rate
            ):
                candidate = self.trigger_best_candidate
                self.sent = True
                self._publish_start(reason=(
                    "state-machine-stagnation "
                    f"distance={candidate['distance']:.3f} "
                    f"rel=({candidate['rel_x']:.3f},{candidate['rel_y']:.3f},{candidate['rel_z']:.3f}) "
                    f"vel=({candidate['rel_vx']:.3f},{candidate['rel_vy']:.3f},{candidate['rel_vz']:.3f})"
                ))
                self.trigger_state = "TRIGGERED"
                return True

        if closing_rate > max(self.sm_stay_max_closing_rate, self.sm_enter_max_closing_rate + 0.6):
            if self.trigger_candidate_announced:
                self.get_logger().info(
                    "Auto START reset-after-window "
                    f"distance={distance:.3f} closing_rate={closing_rate:.3f}"
                )
            self._reset_trigger_state()
            return False

        self.trigger_last_distance = distance
        return False

    def _candidate_score(self, candidate: dict) -> float:
        return (
            float(candidate["distance"]) +
            0.18 * abs(float(candidate["rel_vx"])) +
            0.12 * abs(float(candidate["rel_vy"])) +
            0.10 * abs(float(candidate["rel_y"]))
        )

    def _search_candidate_score(self, candidate: dict, projected_abs_y: float) -> float:
        return (
            0.55 * float(candidate["distance"]) +
            0.75 * float(projected_abs_y) +
            0.20 * abs(float(candidate["rel_y"])) +
            0.08 * abs(float(candidate["rel_vx"])) +
            0.06 * abs(float(candidate["rel_vy"]))
        )

    def _orbit_phase_candidate_score(self, candidate: dict) -> float:
        return (
            abs(float(candidate["distance"]) - self.sm_orbit_score_target_distance) +
            0.6 * abs(float(candidate["rel_x"]) - self.sm_orbit_score_target_rel_x) +
            0.8 * abs(float(candidate["rel_y"]) - self.sm_orbit_score_target_rel_y) +
            0.3 * abs(float(candidate["rel_vy"])) +
            0.8 * abs(float(candidate["rel_vz"])) +
            0.1 * max(0.0, self.sm_orbit_trigger_min_rel_vx - float(candidate["rel_vx"]))
        )

    def _candidate_is_triggerable(self, candidate: dict) -> bool:
        rel_x = float(candidate["rel_x"])
        rel_y = float(candidate["rel_y"])
        rel_vx = float(candidate["rel_vx"])
        rel_vy = float(candidate["rel_vy"])
        relative_speed = float(candidate["relative_speed"])
        if rel_x < 0.0 and rel_vx > 0.2:
            time_to_intercept = min(max(-rel_x / rel_vx, 0.0), 2.5)
        else:
            time_to_intercept = min(
                max(float(candidate["distance"]) / max(relative_speed, 1e-3), 0.0),
                2.5,
            )
        projected_abs_y = abs(rel_y + rel_vy * time_to_intercept)
        return (
            float(candidate["distance"]) <= self.sm_trigger_distance and
            abs(rel_y) <= self.sm_trigger_abs_y and
            abs(rel_x) <= self.sm_trigger_abs_x and
            self.min_rel_z <= float(candidate["rel_z"]) <= self.max_rel_z and
            relative_speed <= self.sm_max_relative_speed and
            (
                abs(rel_vy) <= self.sm_trigger_max_abs_vy or
                projected_abs_y <= self.sm_trigger_projected_abs_y
            )
        )

    def _reset_trigger_state(self) -> None:
        self.trigger_state = "SEARCHING"
        self.trigger_best_candidate = None
        self.trigger_last_distance = None
        self.trigger_history.clear()
        self.trigger_state_announced = None
        self.trigger_stagnation_samples = 0
        self.trigger_candidate_announced = False
        self.search_ready_samples = 0
        self.search_best_candidate = None
        self._reset_orbit_phase_trigger_state()
        self._reset_idle_orbit_trigger_state()
        self.approach_side_ready_samples = 0
        self.approach_side_best_candidate = None
        self.approach_side_candidate_announced = False

    def _reset_orbit_phase_trigger_state(self) -> None:
        self.orbit_ready_samples = 0
        self.orbit_best_candidate = None
        self.orbit_best_score = None
        self.orbit_stagnation_samples = 0
        self.orbit_candidate_announced = False

    def _reset_idle_orbit_trigger_state(self) -> None:
        self.idle_orbit_ready_samples = 0
        self.idle_orbit_best_candidate = None
        self.idle_orbit_candidate_announced = False

    def _mini_energy_healthy(self) -> bool:
        if not self.require_mini_energy_healthy:
            return True
        healthy = (
            math.isfinite(self.mini_true_airspeed_mps) and
            math.isfinite(self.mini_underspeed_ratio) and
            self.mini_true_airspeed_mps >= self.health_min_true_airspeed_mps and
            self.mini_underspeed_ratio <= self.health_max_underspeed_ratio
        )
        if healthy:
            self.mini_health_good_samples += 1
        else:
            self.mini_health_good_samples = 0

        ready = self.mini_health_good_samples >= self.health_min_samples
        if self.mini_health_announced != ready:
            self.mini_health_announced = ready
            if ready:
                self.get_logger().info(
                    "Auto START mini-energy healthy "
                    f"tas={self.mini_true_airspeed_mps:.2f} "
                    f"underspeed={self.mini_underspeed_ratio:.3f}"
                )
            else:
                self.get_logger().info(
                    "Auto START waiting for mini-energy health "
                    f"tas={self.mini_true_airspeed_mps:.2f} "
                    f"underspeed={self.mini_underspeed_ratio:.3f}"
                )
        return ready

    def _log_trigger_state(self, state: str, sample: dict) -> None:
        if self.trigger_state_announced == state:
            return
        self.trigger_state_announced = state
        self.get_logger().info(
            f"Auto START state={state} "
            f"distance={sample['distance']:.3f} "
            f"rel=({sample['rel_x']:.3f},{sample['rel_y']:.3f},{sample['rel_z']:.3f}) "
            f"closing_rate={sample['closing_rate']:.3f}"
        )

    def _publish_start(self, reason: str) -> None:
        command = DockingCommand()
        command.command = "START"
        self.command_pub.publish(command)
        self.command_latched_pub.publish(command)
        self.last_publish_wall = time.time()
        self.republish_count += 1
        self.get_logger().info(f"Auto START sent ({self.republish_count}) {reason}")

    def _timeout_cb(self) -> None:
        if self.sent:
            if self.republish_count >= self.max_republish_count:
                self.get_logger().warning("Auto START exhausted republish budget without activation")
                raise SystemExit(1)
            if time.time() - self.last_publish_wall >= self.republish_period_sec:
                self._publish_start("republish while waiting for activation")
            return

        timeout_start_wall = self.start_wall
        if self.require_orbit_completion_before_start and self.orbit_gate_completed_wall is not None:
            timeout_start_wall = self.orbit_gate_completed_wall

        if time.time() - timeout_start_wall >= self.timeout_sec:
            self.get_logger().info(
                "Auto START timed out without finding a valid geometry window"
            )
            raise SystemExit(1)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main() -> None:
    rclpy.init()
    node = AutoStartDocking()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
