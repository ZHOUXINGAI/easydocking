#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from easydocking_msgs.msg import DockingCommand, DockingStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

try:
    from px4_msgs.msg import (
        AirspeedValidated,
        OffboardControlMode,
        PositionSetpoint,
        PositionSetpointTriplet,
        TecsStatus,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleCommandAck,
        VehicleGlobalPosition,
        VehicleLocalPosition,
        VehicleStatus,
    )
except ImportError:  # pragma: no cover
    AirspeedValidated = None
    OffboardControlMode = None
    PositionSetpoint = None
    PositionSetpointTriplet = None
    TecsStatus = None
    TrajectorySetpoint = None
    VehicleCommand = None
    VehicleCommandAck = None
    VehicleGlobalPosition = None
    VehicleLocalPosition = None
    VehicleStatus = None


GLIDE_SCORE_FEATURE_ORDER = (
    "relative_distance",
    "rel_x",
    "rel_y",
    "rel_vy",
    "closing_rate",
    "projected_abs_y",
)

TERMINAL_SYNC_DEBUG_FIELDS = (
    "terminal_straight_active",
    "terminal_sync_active",
    "terminal_sync_score",
    "terminal_sync_best_score",
    "terminal_sync_release_ready",
    "terminal_sync_good_count",
    "terminal_sync_plateau_count",
    "terminal_sync_cluster_code",
    "terminal_sync_distance_xy",
    "terminal_sync_line_progress",
    "terminal_sync_line_lateral_error",
    "terminal_sync_height_error",
    "terminal_sync_distance_score",
    "terminal_sync_progress_score",
    "terminal_sync_lateral_score",
    "terminal_sync_height_score",
)

HANDOFF_DEBUG_FIELDS = (
    "tangent_active",
    "handoff_distance_xy",
    "handoff_line_progress",
    "handoff_line_lateral_error",
    "handoff_height_error",
    "handoff_geom_ready",
)

ENERGY_DEBUG_FIELDS = (
    "energy_guard_active",
    "energy_guard_bad_airspeed",
    "energy_guard_bad_underspeed",
    "energy_guard_true_airspeed_mps",
    "energy_guard_underspeed_ratio",
    "energy_guard_recover_time_remaining_sec",
)


class Px4FixedWingBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_fixed_wing_bridge")
        self.declare_parameter("uav_name", "mini")
        self.declare_parameter("px4_namespace", "/px4_2")
        self.declare_parameter("vehicle_id", 3)
        self.declare_parameter("world_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("takeoff_altitude", 22.0)
        self.declare_parameter("orbit_center", [20.0, 0.0])
        self.declare_parameter("orbit_radius", 28.0)
        self.declare_parameter("orbit_speed", 12.0)
        self.declare_parameter("glide_entry_distance", 32.0)
        self.declare_parameter("glide_entry_height", 8.0)
        self.declare_parameter("final_relative_position", [0.0, 0.0, 0.8])
        self.declare_parameter("glide_speed", 11.0)
        self.declare_parameter("glide_descent_rate", 0.9)
        self.declare_parameter("glide_trigger_distance", 12.0)
        self.declare_parameter("glide_trigger_phase", "DOCKING")
        self.declare_parameter("glide_release_enabled", True)
        self.declare_parameter("glide_release_mode", "score_state_machine")
        self.declare_parameter("glide_score_observe_distance_min", 32.0)
        self.declare_parameter("glide_score_observe_distance_max", 48.0)
        self.declare_parameter("glide_score_observe_min_rel_x", -44.0)
        self.declare_parameter("glide_score_observe_max_rel_x", -30.0)
        self.declare_parameter("glide_score_observe_min_rel_y", 6.0)
        self.declare_parameter("glide_score_observe_max_rel_y", 24.0)
        self.declare_parameter("glide_score_observe_max_projected_abs_y", 15.5)
        self.declare_parameter("glide_score_observe_max_abs_vy", 10.5)
        self.declare_parameter("glide_score_observe_min_closing_rate", -4.0)
        self.declare_parameter("glide_score_observe_max_closing_rate", -1.4)
        self.declare_parameter("glide_score_accept_threshold", 9.0)
        self.declare_parameter("glide_score_hold_threshold", 11.0)
        self.declare_parameter("glide_score_good_samples", 3)
        self.declare_parameter("glide_score_plateau_samples", 2)
        self.declare_parameter("glide_score_rebound_margin", 0.8)
        self.declare_parameter("glide_score_min_improvement", 0.08)
        self.declare_parameter("glide_score_cluster_a_center", [39.325, -34.322, 19.116, -8.593, -2.578, 2.367])
        self.declare_parameter("glide_score_cluster_a_spread", [1.0, 1.2, 1.4, 0.3, 0.2, 1.5])
        self.declare_parameter("glide_score_cluster_b_center", [39.335, -37.232, 12.563, -8.844, -2.624, 9.548])
        self.declare_parameter("glide_score_cluster_b_spread", [1.0, 1.1, 1.7, 0.25, 0.6, 1.7])
        self.declare_parameter("glide_score_weights", [2.0, 1.1, 0.35, 1.5, 1.5, 2.4])
        self.declare_parameter("glide_score_cluster_a_min_projected_abs_y", 3.5)
        self.declare_parameter("close_tracking_release_enabled", True)
        self.declare_parameter("close_tracking_release_min_distance", 8.0)
        self.declare_parameter("close_tracking_release_max_distance", 15.0)
        self.declare_parameter("close_tracking_release_min_rel_x", -11.5)
        self.declare_parameter("close_tracking_release_max_rel_x", 6.5)
        self.declare_parameter("close_tracking_release_min_rel_y", 3.0)
        self.declare_parameter("close_tracking_release_max_rel_y", 11.0)
        self.declare_parameter("close_tracking_release_min_rel_vx", -6.5)
        self.declare_parameter("close_tracking_release_max_rel_vx", -2.8)
        self.declare_parameter("close_tracking_release_min_rel_vy", -4.2)
        self.declare_parameter("close_tracking_release_max_rel_vy", 5.5)
        self.declare_parameter("glide_tangent_exit_enable", True)
        self.declare_parameter("glide_tangent_exit_min_hold_sec", 5.0)
        self.declare_parameter("glide_tangent_exit_hold_sec", 6.0)
        self.declare_parameter("glide_tangent_exit_lookahead", 8.5)
        self.declare_parameter("glide_tangent_exit_lateral_gain", 0.18)
        self.declare_parameter("glide_tangent_exit_lateral_limit", 0.55)
        self.declare_parameter("glide_tangent_exit_sync_gate_distance_cap", 0.0)
        self.declare_parameter("glide_tangent_exit_release_distance_max", 0.0)
        self.declare_parameter("glide_tangent_exit_release_height_error_max", 0.0)
        self.declare_parameter("glide_tangent_exit_release_progress_min", -1.0)
        self.declare_parameter("refresh_orbit_period_sec", 3.0)
        self.declare_parameter("auto_takeoff_on_launch", True)
        self.declare_parameter("orbit_start_phase_deg", 180.0)
        self.declare_parameter("loiter_speed_command", 12.0)
        self.declare_parameter("glide_speed_command", 11.4)
        self.declare_parameter("tracking_speed_command", 10.8)
        self.declare_parameter("docking_speed_command", 9.6)
        self.declare_parameter("capture_speed_command", 8.8)
        self.declare_parameter("capture_distance", 2.0)
        self.declare_parameter("terminal_slowdown_start_distance", 3.4)
        self.declare_parameter("terminal_slowdown_finish_distance", 0.9)
        self.declare_parameter("terminal_slowdown_max_abs_y", 1.0)
        self.declare_parameter("terminal_sync_start_distance", 2.8)
        self.declare_parameter("terminal_sync_max_abs_y", 0.45)
        self.declare_parameter("terminal_sync_max_height_offset", 0.45)
        self.declare_parameter("terminal_sync_min_height_offset", 0.06)
        self.declare_parameter("terminal_sync_descent_rate", 0.45)
        self.declare_parameter("terminal_sync_score_accept", 0.56)
        self.declare_parameter("terminal_sync_score_hold", 0.48)
        self.declare_parameter("terminal_sync_score_good_samples", 3)
        self.declare_parameter("terminal_sync_score_plateau_samples", 2)
        self.declare_parameter("terminal_sync_score_rebound_margin", 0.05)
        self.declare_parameter("terminal_sync_score_improvement_margin", 0.02)
        self.declare_parameter("energy_guard_enable", True)
        self.declare_parameter("energy_guard_min_true_airspeed_mps", 7.0)
        self.declare_parameter("energy_guard_max_underspeed_ratio", 0.35)
        self.declare_parameter("energy_guard_hold_sec", 1.5)
        self.declare_parameter("use_offboard_orbit_hold", True)
        self.declare_parameter("max_orbit_entry_phase_correction_deg", 45.0)
        self.declare_parameter("orbit_hold_ready_altitude", 1.5)
        self.declare_parameter("orbit_hold_enable_delay_sec", 3.0)
        self.declare_parameter("orbit_radial_gain", 0.35)
        self.declare_parameter("orbit_radial_speed_limit", 2.0)
        self.declare_parameter("orbit_phase_lead_deg", 18.0)
        self.declare_parameter("orbit_recenter_distance_ratio", 1.3)
        self.declare_parameter("allow_orbit_recentering", False)
        self.declare_parameter("native_wait_orbit_refresh_sec", 4.0)

        if any(obj is None for obj in [
            AirspeedValidated,
            OffboardControlMode,
            PositionSetpoint,
            PositionSetpointTriplet,
            TrajectorySetpoint,
            VehicleCommand,
            VehicleCommandAck,
            VehicleGlobalPosition,
            VehicleLocalPosition,
            VehicleStatus,
        ]):
            self.get_logger().error("px4_msgs missing, fixed-wing bridge cannot start")
            return

        self.uav_name = self.get_parameter("uav_name").value
        self.px4_namespace = self.get_parameter("px4_namespace").value.rstrip("/")
        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.world_offset = [float(v) for v in self.get_parameter("world_offset").value]
        self.takeoff_altitude = float(self.get_parameter("takeoff_altitude").value)
        self.orbit_center = [float(v) for v in self.get_parameter("orbit_center").value]
        self.orbit_radius = float(self.get_parameter("orbit_radius").value)
        self.orbit_speed = float(self.get_parameter("orbit_speed").value)
        self.glide_entry_distance = float(self.get_parameter("glide_entry_distance").value)
        self.glide_entry_height = float(self.get_parameter("glide_entry_height").value)
        self.final_relative_position = [float(v) for v in self.get_parameter("final_relative_position").value]
        self.glide_speed = float(self.get_parameter("glide_speed").value)
        self.glide_descent_rate = float(self.get_parameter("glide_descent_rate").value)
        self.glide_trigger_distance = float(self.get_parameter("glide_trigger_distance").value)
        self.glide_trigger_phase = str(self.get_parameter("glide_trigger_phase").value).upper()
        self.glide_release_enabled = bool(self.get_parameter("glide_release_enabled").value)
        self.glide_release_mode = str(self.get_parameter("glide_release_mode").value).strip().lower()
        self.glide_score_observe_distance_min = float(
            self.get_parameter("glide_score_observe_distance_min").value
        )
        self.glide_score_observe_distance_max = float(
            self.get_parameter("glide_score_observe_distance_max").value
        )
        self.glide_score_observe_min_rel_x = float(
            self.get_parameter("glide_score_observe_min_rel_x").value
        )
        self.glide_score_observe_max_rel_x = float(
            self.get_parameter("glide_score_observe_max_rel_x").value
        )
        self.glide_score_observe_min_rel_y = float(
            self.get_parameter("glide_score_observe_min_rel_y").value
        )
        self.glide_score_observe_max_rel_y = float(
            self.get_parameter("glide_score_observe_max_rel_y").value
        )
        self.glide_score_observe_max_projected_abs_y = float(
            self.get_parameter("glide_score_observe_max_projected_abs_y").value
        )
        self.glide_score_observe_max_abs_vy = float(
            self.get_parameter("glide_score_observe_max_abs_vy").value
        )
        self.glide_score_observe_min_closing_rate = float(
            self.get_parameter("glide_score_observe_min_closing_rate").value
        )
        self.glide_score_observe_max_closing_rate = float(
            self.get_parameter("glide_score_observe_max_closing_rate").value
        )
        self.glide_score_accept_threshold = float(
            self.get_parameter("glide_score_accept_threshold").value
        )
        self.glide_score_hold_threshold = float(
            self.get_parameter("glide_score_hold_threshold").value
        )
        self.glide_score_good_samples = int(self.get_parameter("glide_score_good_samples").value)
        self.glide_score_plateau_samples = int(
            self.get_parameter("glide_score_plateau_samples").value
        )
        self.glide_score_rebound_margin = float(
            self.get_parameter("glide_score_rebound_margin").value
        )
        self.glide_score_min_improvement = float(
            self.get_parameter("glide_score_min_improvement").value
        )
        self.glide_score_cluster_a_center = [
            float(value) for value in self.get_parameter("glide_score_cluster_a_center").value
        ]
        self.glide_score_cluster_a_spread = [
            max(1e-3, float(value))
            for value in self.get_parameter("glide_score_cluster_a_spread").value
        ]
        self.glide_score_cluster_b_center = [
            float(value) for value in self.get_parameter("glide_score_cluster_b_center").value
        ]
        self.glide_score_cluster_b_spread = [
            max(1e-3, float(value))
            for value in self.get_parameter("glide_score_cluster_b_spread").value
        ]
        self.glide_score_weights = [
            float(value) for value in self.get_parameter("glide_score_weights").value
        ]
        self.glide_score_cluster_a_min_projected_abs_y = float(
            self.get_parameter("glide_score_cluster_a_min_projected_abs_y").value
        )
        self.close_tracking_release_enabled = bool(
            self.get_parameter("close_tracking_release_enabled").value
        )
        self.close_tracking_release_min_distance = float(
            self.get_parameter("close_tracking_release_min_distance").value
        )
        self.close_tracking_release_max_distance = float(
            self.get_parameter("close_tracking_release_max_distance").value
        )
        self.close_tracking_release_min_rel_x = float(
            self.get_parameter("close_tracking_release_min_rel_x").value
        )
        self.close_tracking_release_max_rel_x = float(
            self.get_parameter("close_tracking_release_max_rel_x").value
        )
        self.close_tracking_release_min_rel_y = float(
            self.get_parameter("close_tracking_release_min_rel_y").value
        )
        self.close_tracking_release_max_rel_y = float(
            self.get_parameter("close_tracking_release_max_rel_y").value
        )
        self.close_tracking_release_min_rel_vx = float(
            self.get_parameter("close_tracking_release_min_rel_vx").value
        )
        self.close_tracking_release_max_rel_vx = float(
            self.get_parameter("close_tracking_release_max_rel_vx").value
        )
        self.close_tracking_release_min_rel_vy = float(
            self.get_parameter("close_tracking_release_min_rel_vy").value
        )
        self.close_tracking_release_max_rel_vy = float(
            self.get_parameter("close_tracking_release_max_rel_vy").value
        )
        self.glide_tangent_exit_enable = bool(
            self.get_parameter("glide_tangent_exit_enable").value
        )
        self.glide_tangent_exit_min_hold_sec = float(
            self.get_parameter("glide_tangent_exit_min_hold_sec").value
        )
        self.glide_tangent_exit_hold_sec = float(
            self.get_parameter("glide_tangent_exit_hold_sec").value
        )
        self.glide_tangent_exit_lookahead = float(
            self.get_parameter("glide_tangent_exit_lookahead").value
        )
        self.glide_tangent_exit_lateral_gain = float(
            self.get_parameter("glide_tangent_exit_lateral_gain").value
        )
        self.glide_tangent_exit_lateral_limit = float(
            self.get_parameter("glide_tangent_exit_lateral_limit").value
        )
        self.glide_tangent_exit_sync_gate_distance_cap = float(
            self.get_parameter("glide_tangent_exit_sync_gate_distance_cap").value
        )
        self.glide_tangent_exit_release_distance_max = float(
            self.get_parameter("glide_tangent_exit_release_distance_max").value
        )
        self.glide_tangent_exit_release_height_error_max = float(
            self.get_parameter("glide_tangent_exit_release_height_error_max").value
        )
        self.glide_tangent_exit_release_progress_min = float(
            self.get_parameter("glide_tangent_exit_release_progress_min").value
        )
        expected_feature_count = len(GLIDE_SCORE_FEATURE_ORDER)
        if any(len(values) != expected_feature_count for values in [
            self.glide_score_cluster_a_center,
            self.glide_score_cluster_a_spread,
            self.glide_score_cluster_b_center,
            self.glide_score_cluster_b_spread,
            self.glide_score_weights,
        ]):
            raise RuntimeError("glide score cluster parameter length mismatch")
        self.refresh_orbit_period_sec = float(self.get_parameter("refresh_orbit_period_sec").value)
        self.auto_takeoff_on_launch = bool(self.get_parameter("auto_takeoff_on_launch").value)
        self.orbit_start_phase = math.radians(float(self.get_parameter("orbit_start_phase_deg").value))
        self.loiter_speed_command = float(self.get_parameter("loiter_speed_command").value)
        self.glide_speed_command = float(self.get_parameter("glide_speed_command").value)
        self.tracking_speed_command = float(self.get_parameter("tracking_speed_command").value)
        self.docking_speed_command = float(self.get_parameter("docking_speed_command").value)
        self.capture_speed_command = float(self.get_parameter("capture_speed_command").value)
        self.capture_distance = float(self.get_parameter("capture_distance").value)
        self.terminal_slowdown_start_distance = float(
            self.get_parameter("terminal_slowdown_start_distance").value
        )
        self.terminal_slowdown_finish_distance = float(
            self.get_parameter("terminal_slowdown_finish_distance").value
        )
        self.terminal_slowdown_max_abs_y = float(
            self.get_parameter("terminal_slowdown_max_abs_y").value
        )
        self.terminal_sync_start_distance = float(
            self.get_parameter("terminal_sync_start_distance").value
        )
        self.terminal_sync_max_abs_y = float(
            self.get_parameter("terminal_sync_max_abs_y").value
        )
        self.terminal_sync_max_height_offset = float(
            self.get_parameter("terminal_sync_max_height_offset").value
        )
        self.terminal_sync_min_height_offset = float(
            self.get_parameter("terminal_sync_min_height_offset").value
        )
        self.terminal_sync_descent_rate = float(
            self.get_parameter("terminal_sync_descent_rate").value
        )
        self.terminal_sync_score_accept = float(
            self.get_parameter("terminal_sync_score_accept").value
        )
        self.terminal_sync_score_hold = float(
            self.get_parameter("terminal_sync_score_hold").value
        )
        self.terminal_sync_score_good_samples = int(
            self.get_parameter("terminal_sync_score_good_samples").value
        )
        self.terminal_sync_score_plateau_samples = int(
            self.get_parameter("terminal_sync_score_plateau_samples").value
        )
        self.terminal_sync_score_rebound_margin = float(
            self.get_parameter("terminal_sync_score_rebound_margin").value
        )
        self.terminal_sync_score_improvement_margin = float(
            self.get_parameter("terminal_sync_score_improvement_margin").value
        )
        self.energy_guard_enable = bool(self.get_parameter("energy_guard_enable").value)
        self.energy_guard_min_true_airspeed_mps = float(
            self.get_parameter("energy_guard_min_true_airspeed_mps").value
        )
        self.energy_guard_max_underspeed_ratio = float(
            self.get_parameter("energy_guard_max_underspeed_ratio").value
        )
        self.energy_guard_hold_sec = float(self.get_parameter("energy_guard_hold_sec").value)
        self.use_offboard_orbit_hold = bool(self.get_parameter("use_offboard_orbit_hold").value)
        self.max_orbit_entry_phase_correction = math.radians(
            float(self.get_parameter("max_orbit_entry_phase_correction_deg").value)
        )
        self.orbit_hold_ready_altitude = float(self.get_parameter("orbit_hold_ready_altitude").value)
        self.orbit_hold_enable_delay_sec = float(self.get_parameter("orbit_hold_enable_delay_sec").value)
        self.orbit_radial_gain = float(self.get_parameter("orbit_radial_gain").value)
        self.orbit_radial_speed_limit = float(self.get_parameter("orbit_radial_speed_limit").value)
        self.orbit_phase_lead = math.radians(float(self.get_parameter("orbit_phase_lead_deg").value))
        self.orbit_recenter_distance_ratio = float(self.get_parameter("orbit_recenter_distance_ratio").value)
        self.allow_orbit_recentering = bool(self.get_parameter("allow_orbit_recentering").value)
        self.native_wait_orbit_refresh_sec = float(
            self.get_parameter("native_wait_orbit_refresh_sec").value
        )

        self.start_requested = False
        self.commanded_start = False
        self.takeoff_sent = False
        self.offboard_active = False
        self.offboard_mode_sent = False
        self.offboard_counter = 0
        self.glide_active = False
        self.loiter_command_sent = False
        self.last_loiter_command_time = None
        self.current_speed_mode = "none"
        self.last_speed_command = None
        self.orbit_phase = self.orbit_start_phase
        self.last_orbit_update_time = None
        self.glide_activation_logged = False
        self.glide_score_observing = False
        self.glide_score_good_count = 0
        self.glide_score_plateau_count = 0
        self.glide_score_best = math.inf
        self.glide_score_best_cluster = ""
        self.glide_score_last_score = math.inf
        self.glide_score_last_log_bucket = None
        self.glide_tangent_exit_active = False
        self.glide_tangent_exit_start_time_sec: Optional[float] = None
        self.glide_tangent_exit_axis_world = [1.0, 0.0]
        self.glide_tangent_exit_anchor_world = [0.0, 0.0]
        self.glide_tangent_exit_altitude = self.takeoff_altitude
        self.glide_tangent_exit_history_valid = False
        self.orbit_hold_initialized = False
        self.orbit_hold_ready_active = False
        self.orbit_hold_committed = False
        self.wait_loiter_active = False
        self.approach_axis_world = [1.0, 0.0]
        self.terminal_straight_active = False
        self.terminal_straight_start_time_sec: Optional[float] = None
        self.terminal_sync_active = False
        self.terminal_exit_axis_world = [1.0, 0.0]
        self.terminal_exit_anchor_world = [0.0, 0.0]
        self.terminal_exit_altitude = self.takeoff_altitude
        self.terminal_entry_distance = math.inf
        self.terminal_sync_score = 0.0
        self.terminal_sync_best_score = 0.0
        self.terminal_sync_good_count = 0
        self.terminal_sync_plateau_count = 0
        self.terminal_sync_release_ready = False
        self.terminal_sync_debug = self._empty_terminal_sync_debug()
        self.handoff_debug = self._empty_handoff_debug()
        self.wait_orbit_altitude = self.takeoff_altitude
        self.takeoff_sent_time_sec = None
        self.takeoff_reference_alt_m = None
        self.takeoff_target_alt_m = None
        self.last_status_debug_time_sec = None
        self.last_logged_nav_state = None
        self.last_logged_arming_state = None
        self.last_ack_signature = None
        self.last_docking_phase = "IDLE"
        self.approach_axis_world = [1.0, 0.0]

        self.vehicle_status: Optional[VehicleStatus] = None
        self.vehicle_global_position: Optional[VehicleGlobalPosition] = None
        self.vehicle_local_position: Optional[VehicleLocalPosition] = None
        self.vehicle_setpoint_triplet: Optional[PositionSetpointTriplet] = None
        self.airspeed_validated: Optional[AirspeedValidated] = None
        self.tecs_status: Optional[TecsStatus] = None
        self.carrier_odom: Optional[Odometry] = None
        self.docking_status: Optional[DockingStatus] = None
        self.energy_guard_recover_until_sec = 0.0

        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(DockingCommand, "/docking/command", self._direct_command_cb, 10)
        self.create_subscription(DockingCommand, "/docking/command_latched", self._latched_command_cb, latched_qos)
        self.create_subscription(DockingStatus, "/docking/status", self._status_cb, 10)
        self.create_subscription(Odometry, "/carrier/odom", self._carrier_cb, 10)
        self.create_subscription(
            VehicleStatus, f"{self.px4_namespace}/fmu/out/vehicle_status_v2", self._vehicle_status_cb, px4_qos
        )
        self.create_subscription(
            VehicleCommandAck,
            f"{self.px4_namespace}/fmu/out/vehicle_command_ack_v1",
            self._vehicle_command_ack_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            f"{self.px4_namespace}/fmu/out/vehicle_global_position",
            self._global_pos_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f"{self.px4_namespace}/fmu/out/vehicle_local_position_v1",
            self._local_pos_cb,
            px4_qos,
        )
        self.create_subscription(
            PositionSetpointTriplet,
            f"{self.px4_namespace}/fmu/out/position_setpoint_triplet",
            self._position_setpoint_triplet_cb,
            px4_qos,
        )
        self.create_subscription(
            AirspeedValidated,
            f"{self.px4_namespace}/fmu/out/airspeed_validated_v1",
            self._airspeed_cb,
            px4_qos,
        )
        if TecsStatus is not None:
            self.create_subscription(
                TecsStatus,
                f"{self.px4_namespace}/fmu/out/tecs_status",
                self._tecs_status_cb,
                px4_qos,
            )
            self.create_subscription(
                TecsStatus,
                f"{self.px4_namespace}/fmu/out/tecs_status_v1",
                self._tecs_status_cb,
                px4_qos,
            )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f"{self.px4_namespace}/fmu/in/offboard_control_mode", px4_qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, f"{self.px4_namespace}/fmu/in/trajectory_setpoint", px4_qos
        )
        self.position_setpoint_triplet_pub = self.create_publisher(
            PositionSetpointTriplet, f"{self.px4_namespace}/fmu/in/position_setpoint_triplet", px4_qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f"{self.px4_namespace}/fmu/in/vehicle_command", px4_qos
        )
        self.glide_path_pub = self.create_publisher(Path, f"/{self.uav_name}/glide_path", 10)
        self.glide_target_pub = self.create_publisher(PoseStamped, f"/{self.uav_name}/glide_target", 10)
        self.terminal_sync_debug_pub = self.create_publisher(
            Float64MultiArray, f"/{self.uav_name}/terminal_sync_debug", 10
        )
        self.handoff_debug_pub = self.create_publisher(
            Float64MultiArray, f"/{self.uav_name}/handoff_debug", 10
        )
        self.energy_debug_pub = self.create_publisher(
            Float64MultiArray, f"/{self.uav_name}/energy_debug", 10
        )

        self.timer = self.create_timer(0.05, self._timer_cb)
        self.debug_timer = self.create_timer(0.05, self._publish_terminal_sync_debug)
        self.energy_guard_last_active = False

    def _direct_command_cb(self, msg: DockingCommand) -> None:
        self._handle_command(msg)

    def _latched_command_cb(self, msg: DockingCommand) -> None:
        self._handle_command(msg)

    def _handle_command(self, msg: DockingCommand) -> None:
        command = msg.command.strip().upper()
        if command == "START":
            if self.start_requested:
                self.get_logger().info("fixed-wing: ignoring duplicate START")
                return
            self.start_requested = True
            if self.docking_status is not None and self.docking_status.is_active:
                self.commanded_start = True
                self.get_logger().info("fixed-wing: received START with active controller")
            else:
                self.get_logger().info("fixed-wing: received START, waiting for controller activation")
        elif command in {"STOP", "RESET"}:
            self.start_requested = False
            self.commanded_start = False
            self.offboard_active = False
            self.offboard_mode_sent = False
            self.offboard_counter = 0
            self.glide_active = False
            self.loiter_command_sent = False
            self.last_loiter_command_time = None
            self.current_speed_mode = "none"
            self.last_speed_command = None
            self.last_orbit_update_time = None
            self.glide_activation_logged = False
            self.glide_score_observing = False
            self.glide_score_good_count = 0
            self.glide_score_plateau_count = 0
            self.glide_score_best = math.inf
            self.glide_score_best_cluster = ""
            self.glide_score_last_score = math.inf
            self.glide_score_last_log_bucket = None
            self.glide_tangent_exit_active = False
            self.glide_tangent_exit_start_time_sec = None
            self.glide_tangent_exit_axis_world = [1.0, 0.0]
            self.glide_tangent_exit_anchor_world = [0.0, 0.0]
            self.glide_tangent_exit_altitude = self.takeoff_altitude
            self.glide_tangent_exit_history_valid = False
            self.orbit_hold_initialized = False
            self.orbit_hold_ready_active = False
            self.orbit_hold_committed = False
            self.wait_loiter_active = False
            self.terminal_straight_active = False
            self.terminal_straight_start_time_sec = None
            self.terminal_sync_active = False
            self.terminal_exit_axis_world = [1.0, 0.0]
            self.terminal_exit_anchor_world = [0.0, 0.0]
            self.terminal_exit_altitude = self.takeoff_altitude
            self.terminal_entry_distance = math.inf
            self._reset_terminal_sync_state()
            self.handoff_debug = self._empty_handoff_debug()
            self.wait_orbit_altitude = self.takeoff_altitude
            self.takeoff_sent_time_sec = None
            self.takeoff_reference_alt_m = None
            self.takeoff_target_alt_m = None
            self.last_status_debug_time_sec = None
            self.last_docking_phase = "IDLE"
            if command == "RESET":
                self.takeoff_sent = False
            self.get_logger().info(f"fixed-wing: received {command}")

    def _log_docking_tangent_release_if_needed(self) -> None:
        if self.docking_status is None:
            return
        self.get_logger().info(
            "fixed-wing: glide docking release accepted "
            f"distance={self.docking_status.relative_distance:.3f} "
            f"rel=({self.docking_status.relative_position.x:.3f},"
            f"{self.docking_status.relative_position.y:.3f},"
            f"{self.docking_status.relative_position.z:.3f}) "
            f"rel_v=({self.docking_status.relative_velocity.x:.3f},"
            f"{self.docking_status.relative_velocity.y:.3f},"
            f"{self.docking_status.relative_velocity.z:.3f})"
        )

    def _status_cb(self, msg: DockingStatus) -> None:
        self.docking_status = msg
        if msg.is_active and self.start_requested and not self.commanded_start:
            self.commanded_start = True
            self.get_logger().info("fixed-wing: controller active, enabling glide logic")

    def _carrier_cb(self, msg: Odometry) -> None:
        self.carrier_odom = msg

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        nav_state = int(msg.nav_state)
        arming_state = int(msg.arming_state)
        if nav_state == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD) and not self.glide_active:
            self.orbit_hold_committed = True
        if arming_state != int(VehicleStatus.ARMING_STATE_ARMED):
            self.orbit_hold_committed = False
        if nav_state != self.last_logged_nav_state or arming_state != self.last_logged_arming_state:
            self.last_logged_nav_state = nav_state
            self.last_logged_arming_state = arming_state
            self.get_logger().info(
                "fixed-wing: vehicle status "
                f"nav={self._nav_state_name(nav_state)} "
                f"arming={self._arming_state_name(arming_state)}"
            )

    def _vehicle_command_ack_cb(self, msg: VehicleCommandAck) -> None:
        signature = (int(msg.command), int(msg.result), int(msg.result_param1), int(msg.result_param2))
        if signature == self.last_ack_signature:
            return
        self.last_ack_signature = signature
        self.get_logger().info(
            "fixed-wing: cmd ack "
            f"command={int(msg.command)} "
            f"result={self._command_result_name(int(msg.result))} "
            f"param1={int(msg.result_param1)} "
            f"param2={int(msg.result_param2)}"
        )

    def _global_pos_cb(self, msg: VehicleGlobalPosition) -> None:
        self.vehicle_global_position = msg

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def _position_setpoint_triplet_cb(self, msg: PositionSetpointTriplet) -> None:
        self.vehicle_setpoint_triplet = msg

    def _airspeed_cb(self, msg: AirspeedValidated) -> None:
        self.airspeed_validated = msg

    def _tecs_status_cb(self, msg: TecsStatus) -> None:
        self.tecs_status = msg

    def _timer_cb(self) -> None:
        if not self.commanded_start and not self.auto_takeoff_on_launch:
            return
        if self.vehicle_global_position is None or self.vehicle_local_position is None or self.vehicle_status is None:
            return
        if not self.takeoff_sent:
            self._send_arm()
            self._send_takeoff()
            self.takeoff_sent = True
            return

        if self.start_requested and self._should_start_glide():
            if not self.glide_active:
                phase = self.docking_status.phase.upper() if self.docking_status is not None else "IDLE"
                if phase in {"DOCKING", "COMPLETED"} and not self.glide_tangent_exit_history_valid:
                    self._arm_glide_tangent_exit()
                    self._log_docking_tangent_release_if_needed()
                self._seed_approach_axis_from_current_flight_path()
            self.glide_active = True

        if self.glide_active and self._ready_for_offboard():
            self._update_glide_tangent_exit_state()
            self._update_terminal_straight_state()
            self._update_terminal_sync_state()
            if not self.glide_activation_logged and self.docking_status is not None:
                self.get_logger().info(
                    "fixed-wing: glide window accepted "
                    f"mode={self.glide_release_mode} "
                    f"phase={self.docking_status.phase} "
                    f"distance={self.docking_status.relative_distance:.3f} "
                    f"rel=({self.docking_status.relative_position.x:.3f},"
                    f"{self.docking_status.relative_position.y:.3f},"
                    f"{self.docking_status.relative_position.z:.3f}) "
                    f"vel=({self.docking_status.relative_velocity.x:.3f},"
                    f"{self.docking_status.relative_velocity.y:.3f},"
                    f"{self.docking_status.relative_velocity.z:.3f})"
                )
                self.glide_activation_logged = True
            self.offboard_active = True
            selected_speed, speed_mode = self._select_terminal_speed()
            self._publish_speed_command_if_needed(selected_speed, mode=speed_mode)
            self._publish_glide_setpoint()
        else:
            self._update_terminal_straight_state()
            self._initialize_orbit_hold_if_needed()
            orbit_hold_ready = self._ready_for_orbit_hold()
            wait_loiter_enabled = self.wait_loiter_active or orbit_hold_ready
            if wait_loiter_enabled and self.use_offboard_orbit_hold:
                if not self.wait_loiter_active and self.vehicle_local_position is not None:
                    current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)
                    self.wait_orbit_altitude = max(self.takeoff_altitude, current_world_z)
                self.wait_loiter_active = True
                self.orbit_hold_committed = True
                self.offboard_active = True
                self._publish_speed_command_if_needed(self.loiter_speed_command, mode="loiter")
                self._publish_orbit_setpoint()
                self._publish_status_debug_if_needed("wait_orbit_offboard")
            elif wait_loiter_enabled:
                if not self.wait_loiter_active and self.vehicle_local_position is not None:
                    current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)
                    self.wait_orbit_altitude = max(self.takeoff_altitude, current_world_z)
                self.wait_loiter_active = True
                self.orbit_hold_committed = False
                self.offboard_active = False
                self.offboard_mode_sent = False
                self.offboard_counter = 0
                self._publish_loiter_command_if_needed()
                # Keep the active loiter setpoint fresh so PX4 fixed-wing uses our desired
                # orbit altitude/radius/cruising_speed instead of falling back to defaults.
                self._publish_wait_loiter_triplet()
                self._publish_status_debug_if_needed("wait_orbit_native")
            else:
                self.wait_loiter_active = False
                self.orbit_hold_committed = False
                self.offboard_active = False
                self.offboard_mode_sent = False
                self.offboard_counter = 0
                self._publish_status_debug_if_needed("climb_wait")

    def _update_terminal_straight_state(self) -> None:
        phase = self.docking_status.phase.upper() if self.docking_status is not None else "IDLE"
        if self.vehicle_local_position is None:
            self._clear_terminal_sync_debug()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.terminal_straight_active:
            if self.terminal_straight_start_time_sec is None:
                self.terminal_straight_start_time_sec = now_sec
            current_distance = (
                float(self.docking_status.relative_distance)
                if self.docking_status is not None else math.inf
            )
            straight_elapsed = max(0.0, now_sec - self.terminal_straight_start_time_sec)
            reopen_distance = max(
                self.terminal_slowdown_start_distance + 5.0,
                self.terminal_entry_distance + 8.0,
                12.0,
            )
            if (
                phase == "APPROACH" and
                math.isfinite(current_distance) and
                straight_elapsed >= 4.0 and
                current_distance >= reopen_distance
            ):
                self.glide_tangent_exit_history_valid = False
                self._seed_approach_axis_from_current_flight_path()
                self._clear_terminal_straight_state(
                    reason=(
                        f"reopen distance={current_distance:.3f} "
                        f"entry={self.terminal_entry_distance:.3f} "
                        f"elapsed={straight_elapsed:.3f}"
                    )
                )
                return

        if phase not in {"DOCKING", "COMPLETED"}:
            self.terminal_sync_active = False
            self._clear_terminal_sync_debug()

        if phase in {"DOCKING", "COMPLETED"} and not self.terminal_straight_active:
            current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
            current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
            current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)
            if self.glide_tangent_exit_history_valid:
                exit_axis = self._normalize_xy(
                    self.glide_tangent_exit_axis_world[0],
                    self.glide_tangent_exit_axis_world[1],
                )
                anchor_world = [
                    self.glide_tangent_exit_anchor_world[0],
                    self.glide_tangent_exit_anchor_world[1],
                ]
                anchor_altitude = self.glide_tangent_exit_altitude
            else:
                current_ground_vx = float(self.vehicle_local_position.vx)
                current_ground_vy = float(self.vehicle_local_position.vy)
                if math.hypot(current_ground_vx, current_ground_vy) > 3.0:
                    exit_axis = self._normalize_xy(current_ground_vx, current_ground_vy)
                else:
                    exit_axis = self._normalize_xy(
                        self.approach_axis_world[0], self.approach_axis_world[1]
                    )
                anchor_world = [current_world_x, current_world_y]
                anchor_altitude = max(self.takeoff_altitude, current_world_z)
            self.terminal_straight_active = True
            self.terminal_straight_start_time_sec = now_sec
            self.terminal_exit_axis_world = [exit_axis[0], exit_axis[1]]
            self.terminal_exit_anchor_world = anchor_world
            self.terminal_exit_altitude = anchor_altitude
            self.terminal_entry_distance = float(self.docking_status.relative_distance)
            self.get_logger().info(
                "fixed-wing: terminal straight-line mode armed "
                f"phase={phase} "
                f"anchor=({self.terminal_exit_anchor_world[0]:.1f},{self.terminal_exit_anchor_world[1]:.1f},{self.terminal_exit_altitude:.1f}) "
                f"axis=({exit_axis[0]:.3f},{exit_axis[1]:.3f})"
            )

    def _update_terminal_sync_state(self) -> None:
        phase = self.docking_status.phase.upper() if self.docking_status is not None else "IDLE"
        if self.glide_tangent_exit_active and phase in {"TRACKING", "DOCKING"}:
            self.terminal_sync_active = False
            self._clear_terminal_sync_debug()
            return
        if (
            phase not in {"APPROACH", "TRACKING", "DOCKING", "COMPLETED"} or
            not self.terminal_straight_active or
            self.carrier_odom is None or
            self.vehicle_local_position is None
        ):
            if phase not in {"APPROACH", "TRACKING", "DOCKING", "COMPLETED"}:
                self.terminal_sync_active = False
                self._reset_terminal_sync_state()
            return

        if phase == "COMPLETED":
            self.terminal_sync_active = True
            self.terminal_sync_score = 1.0
            self.terminal_sync_best_score = max(self.terminal_sync_best_score, 1.0)
            self.terminal_sync_release_ready = True
            return

        metrics = self._compute_terminal_sync_metrics()
        if metrics is None:
            self._reset_terminal_sync_state()
            return

        self.terminal_sync_debug.update(metrics)
        score = float(metrics["score"])
        improved = score > self.terminal_sync_best_score + self.terminal_sync_score_improvement_margin
        if improved:
            self.terminal_sync_best_score = score
            self.terminal_sync_plateau_count = 0
            self.get_logger().info(
                "fixed-wing: terminal sync best "
                f"score={score:.3f} "
                f"distance={float(metrics['distance_xy']):.3f} "
                f"line_lat={float(metrics['line_lateral_error']):.3f} "
                f"height_error={float(metrics['height_error']):.3f}"
            )
        elif score >= max(
            self.terminal_sync_score_hold,
            self.terminal_sync_best_score - self.terminal_sync_score_rebound_margin,
        ):
            self.terminal_sync_plateau_count += 1
        else:
            self.terminal_sync_plateau_count = 0

        if score >= self.terminal_sync_score_accept:
            self.terminal_sync_good_count += 1
        elif (
            self.terminal_sync_best_score >= self.terminal_sync_score_accept and
            score >= self.terminal_sync_score_hold
        ):
            self.terminal_sync_good_count += 1
        else:
            self.terminal_sync_good_count = 0

        self.terminal_sync_good_count = min(
            self.terminal_sync_good_count,
            self.terminal_sync_score_good_samples,
        )
        plateau_release = (
            self.terminal_sync_best_score >= self.terminal_sync_score_accept and
            self.terminal_sync_plateau_count >= self.terminal_sync_score_plateau_samples and
            score >= self.terminal_sync_score_hold
        )
        self.terminal_sync_score = score
        self.terminal_sync_release_ready = (
            self.terminal_sync_good_count >= self.terminal_sync_score_good_samples or
            plateau_release
        )
        self.terminal_sync_debug["sync_score"] = self.terminal_sync_score
        self.terminal_sync_debug["sync_best_score"] = self.terminal_sync_best_score
        self.terminal_sync_debug["sync_good_count"] = float(self.terminal_sync_good_count)
        self.terminal_sync_debug["sync_plateau_count"] = float(self.terminal_sync_plateau_count)
        self.terminal_sync_debug["sync_release_ready"] = 1.0 if self.terminal_sync_release_ready else 0.0

        sync_latch_distance = max(
            self.terminal_slowdown_start_distance + 2.3,
            self.capture_distance + 4.0,
            self.terminal_sync_start_distance,
        )
        sync_latch_abs_y = max(
            self.terminal_slowdown_max_abs_y + 0.25,
            self.terminal_sync_max_abs_y + 0.80,
        )
        sync_latch_height_error = max(
            self.terminal_sync_max_height_offset + 0.75,
            1.15,
        )
        if (
            not self.terminal_sync_active and
            float(metrics["distance_xy"]) <= sync_latch_distance and
            abs(float(metrics["line_lateral_error"])) <= sync_latch_abs_y and
            abs(float(metrics["height_error"])) <= sync_latch_height_error and
            float(metrics["line_progress"]) >= 0.0
        ):
            self.terminal_sync_active = True
            self.get_logger().info(
                "fixed-wing: terminal sync latched "
                f"score={score:.3f} "
                f"distance={float(metrics['distance_xy']):.3f} "
                f"line_lat={float(metrics['line_lateral_error']):.3f} "
                f"height_error={float(metrics['height_error']):.3f} "
                f"progress={float(metrics['line_progress']):.3f}"
            )

    def _initialize_orbit_hold_if_needed(self) -> None:
        if self.orbit_hold_initialized or self.vehicle_local_position is None:
            return
        current_climb_alt = self._current_climb_altitude()
        if current_climb_alt is None:
            return
        init_altitude = min(
            max(self.orbit_hold_ready_altitude - 1.0, self.takeoff_altitude * 0.88, 6.0),
            max(self.takeoff_altitude - 0.6, 6.0),
        )
        if current_climb_alt < init_altitude:
            return

        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        configured_distance = math.hypot(
            current_world_x - self.orbit_center[0],
            current_world_y - self.orbit_center[1],
        )
        current_phase = math.atan2(
            current_world_y - self.orbit_center[1],
            current_world_x - self.orbit_center[0],
        )
        phase_error = self._wrap_angle(self.orbit_start_phase - current_phase)
        if (
            self.allow_orbit_recentering and
            (
                configured_distance > self.orbit_radius * self.orbit_recenter_distance_ratio or
                abs(phase_error) > self.max_orbit_entry_phase_correction
            )
        ):
            current_speed_x = float(self.vehicle_local_position.vx)
            current_speed_y = float(self.vehicle_local_position.vy)
            current_speed_norm = math.hypot(current_speed_x, current_speed_y)

            if current_speed_norm > 1.0:
                tangent_x = current_speed_x / current_speed_norm
                tangent_y = current_speed_y / current_speed_norm
                radial_x = -tangent_y
                radial_y = tangent_x
                self.orbit_center = [
                    current_world_x - self.orbit_radius * radial_x,
                    current_world_y - self.orbit_radius * radial_y,
                ]
                self.orbit_phase = math.atan2(radial_y, radial_x)
                recenter_mode = "tangent"
            else:
                self.orbit_center = [
                    current_world_x - self.orbit_radius * math.cos(self.orbit_start_phase),
                    current_world_y - self.orbit_radius * math.sin(self.orbit_start_phase),
                ]
                self.orbit_phase = self.orbit_start_phase
                recenter_mode = "phase"
            self.last_orbit_update_time = self.get_clock().now().nanoseconds * 1e-9
            self.orbit_hold_initialized = True
            self.loiter_command_sent = False
            self.last_loiter_command_time = None
            self.get_logger().info(
                "fixed-wing: orbit center recentered "
                f"mode={recenter_mode} "
                f"center=({self.orbit_center[0]:.1f},{self.orbit_center[1]:.1f}) "
                f"phase_deg={math.degrees(self.orbit_phase):.1f} "
                f"phase_error_deg={math.degrees(phase_error):.1f} "
                f"climb_alt={current_climb_alt:.1f}"
            )
            return

        bounded_correction = max(
            -self.max_orbit_entry_phase_correction,
            min(self.max_orbit_entry_phase_correction, phase_error),
        )
        self.orbit_phase = current_phase + bounded_correction
        self.last_orbit_update_time = self.get_clock().now().nanoseconds * 1e-9
        self.orbit_hold_initialized = True
        self.loiter_command_sent = False
        self.last_loiter_command_time = None
        self.get_logger().info(
            "fixed-wing: orbit hold initialized "
            f"current_phase_deg={math.degrees(current_phase):.1f} "
            f"target_phase_deg={math.degrees(self.orbit_start_phase):.1f} "
            f"applied_correction_deg={math.degrees(bounded_correction):.1f} "
            f"climb_alt={current_climb_alt:.1f}"
        )

    def _empty_terminal_sync_debug(self) -> dict[str, float]:
        return {
            "distance_xy": math.nan,
            "line_progress": math.nan,
            "line_lateral_error": math.nan,
            "height_error": math.nan,
            "distance_score": 0.0,
            "progress_score": 0.0,
            "lateral_score": 0.0,
            "height_score": 0.0,
            "score": 0.0,
            "sync_score": 0.0,
            "sync_best_score": 0.0,
            "sync_release_ready": 0.0,
            "sync_good_count": 0.0,
            "sync_plateau_count": 0.0,
            "cluster_code": 0.0,
        }

    def _empty_handoff_debug(self) -> dict[str, float]:
        return {
            "tangent_active": 0.0,
            "distance_xy": math.nan,
            "line_progress": math.nan,
            "line_lateral_error": math.nan,
            "height_error": math.nan,
            "geom_ready": 0.0,
        }

    def _clear_terminal_sync_debug(self) -> None:
        self.terminal_sync_debug = self._empty_terminal_sync_debug()
        self.terminal_sync_debug["sync_score"] = self.terminal_sync_score
        self.terminal_sync_debug["sync_best_score"] = self.terminal_sync_best_score
        self.terminal_sync_debug["sync_release_ready"] = 1.0 if self.terminal_sync_release_ready else 0.0
        self.terminal_sync_debug["sync_good_count"] = float(self.terminal_sync_good_count)
        self.terminal_sync_debug["sync_plateau_count"] = float(self.terminal_sync_plateau_count)
        self.terminal_sync_debug["cluster_code"] = self._terminal_sync_cluster_code()

    def _reset_terminal_sync_state(self) -> None:
        self.terminal_sync_score = 0.0
        self.terminal_sync_best_score = 0.0
        self.terminal_sync_good_count = 0
        self.terminal_sync_plateau_count = 0
        self.terminal_sync_release_ready = False
        self._clear_terminal_sync_debug()

    def _clear_terminal_straight_state(self, reason: Optional[str] = None) -> None:
        if self.terminal_straight_active and reason is not None:
            self.get_logger().info(f"fixed-wing: terminal straight cleared reason={reason}")
        self.terminal_straight_active = False
        self.terminal_straight_start_time_sec = None
        self.terminal_sync_active = False
        self.terminal_exit_axis_world = [1.0, 0.0]
        self.terminal_exit_anchor_world = [0.0, 0.0]
        self.terminal_exit_altitude = self.takeoff_altitude
        self.terminal_entry_distance = math.inf
        self._reset_terminal_sync_state()

    def _terminal_sync_cluster_code(self) -> float:
        if self.glide_score_best_cluster == "A":
            return 1.0
        if self.glide_score_best_cluster == "B":
            return 2.0
        return 0.0

    def _compute_terminal_sync_metrics(self) -> Optional[dict[str, float]]:
        if self.carrier_odom is None or self.vehicle_local_position is None:
            return None

        carrier_pos = self.carrier_odom.pose.pose.position
        deck_x = carrier_pos.x + self.final_relative_position[0]
        deck_y = carrier_pos.y + self.final_relative_position[1]
        actual_deck_z = carrier_pos.z + self.final_relative_position[2]
        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)

        delta_x = deck_x - current_world_x
        delta_y = deck_y - current_world_y
        distance_xy = math.hypot(delta_x, delta_y)
        axis_x, axis_y = self.terminal_exit_axis_world
        lateral_x = -axis_y
        lateral_y = axis_x
        rel_anchor_x = current_world_x - self.terminal_exit_anchor_world[0]
        rel_anchor_y = current_world_y - self.terminal_exit_anchor_world[1]
        line_progress = rel_anchor_x * axis_x + rel_anchor_y * axis_y
        line_lateral_error = rel_anchor_x * lateral_x + rel_anchor_y * lateral_y
        height_error = current_world_z - actual_deck_z

        sync_gate_distance = max(
            self.terminal_slowdown_start_distance + 2.3,
            self.capture_distance + 4.0,
            self.terminal_sync_start_distance,
        )
        sync_gate_lateral = max(
            self.terminal_slowdown_max_abs_y + 0.25,
            self.terminal_sync_max_abs_y + 0.80,
        )
        sync_gate_height = max(
            self.terminal_sync_max_height_offset + 0.75,
            1.15,
        )
        score_distance_ref = max(sync_gate_distance + 1.4, self.capture_distance + 4.8)
        score_lateral_ref = max(sync_gate_lateral + 1.15, 2.40)
        score_height_ref = max(sync_gate_height + 0.20, 1.35)
        score_progress_ref = max(self.capture_distance + 1.4, 2.2)

        distance_score = max(
            0.0,
            min(
                1.0,
                (score_distance_ref - distance_xy) /
                max(score_distance_ref - self.capture_distance, 0.1),
            ),
        )
        progress_score = max(
            0.0,
            min(1.0, (line_progress + 0.35) / score_progress_ref),
        )
        lateral_score = max(
            0.0,
            min(1.0, (score_lateral_ref - abs(line_lateral_error)) / score_lateral_ref),
        )
        height_score = max(
            0.0,
            min(1.0, (score_height_ref - abs(height_error)) / score_height_ref),
        )
        score = (
            0.28 * distance_score +
            0.18 * progress_score +
            0.34 * lateral_score +
            0.20 * height_score
        )
        return {
            "distance_xy": distance_xy,
            "line_progress": line_progress,
            "line_lateral_error": line_lateral_error,
            "height_error": height_error,
            "distance_score": distance_score,
            "progress_score": progress_score,
            "lateral_score": lateral_score,
            "height_score": height_score,
            "score": score,
            "cluster_code": self._terminal_sync_cluster_code(),
        }

    def _publish_terminal_sync_debug(self) -> None:
        debug_msg = Float64MultiArray()
        debug_msg.data = [
            1.0 if self.terminal_straight_active else 0.0,
            1.0 if self.terminal_sync_active else 0.0,
            float(self.terminal_sync_score),
            float(self.terminal_sync_best_score),
            1.0 if self.terminal_sync_release_ready else 0.0,
            float(self.terminal_sync_good_count),
            float(self.terminal_sync_plateau_count),
            float(self.terminal_sync_debug["cluster_code"]),
            float(self.terminal_sync_debug["distance_xy"]),
            float(self.terminal_sync_debug["line_progress"]),
            float(self.terminal_sync_debug["line_lateral_error"]),
            float(self.terminal_sync_debug["height_error"]),
            float(self.terminal_sync_debug["distance_score"]),
            float(self.terminal_sync_debug["progress_score"]),
            float(self.terminal_sync_debug["lateral_score"]),
            float(self.terminal_sync_debug["height_score"]),
        ]
        self.terminal_sync_debug_pub.publish(debug_msg)

        handoff_msg = Float64MultiArray()
        handoff_msg.data = [
            float(self.handoff_debug["tangent_active"]),
            float(self.handoff_debug["distance_xy"]),
            float(self.handoff_debug["line_progress"]),
            float(self.handoff_debug["line_lateral_error"]),
            float(self.handoff_debug["height_error"]),
            float(self.handoff_debug["geom_ready"]),
        ]
        self.handoff_debug_pub.publish(handoff_msg)

        energy_msg = Float64MultiArray()
        energy_active = 1.0 if self._energy_guard_active() else 0.0
        true_airspeed = (
            float(self.airspeed_validated.true_airspeed_m_s)
            if self.airspeed_validated is not None else math.nan
        )
        underspeed_ratio = (
            float(self.tecs_status.underspeed_ratio)
            if self.tecs_status is not None else math.nan
        )
        bad_airspeed = (
            1.0 if math.isfinite(true_airspeed) and
            true_airspeed < self.energy_guard_min_true_airspeed_mps else 0.0
        )
        bad_underspeed = (
            1.0 if math.isfinite(underspeed_ratio) and
            underspeed_ratio > self.energy_guard_max_underspeed_ratio else 0.0
        )
        recover_remaining = max(
            0.0,
            self.energy_guard_recover_until_sec - self.get_clock().now().nanoseconds * 1e-9,
        )
        energy_msg.data = [
            energy_active,
            bad_airspeed,
            bad_underspeed,
            true_airspeed,
            underspeed_ratio,
            recover_remaining,
        ]
        self.energy_debug_pub.publish(energy_msg)

    def _send_arm(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _send_takeoff(self) -> None:
        assert self.vehicle_global_position is not None
        self.takeoff_reference_alt_m = float(self.vehicle_global_position.alt)
        takeoff_target_world = [self.orbit_center[0], self.orbit_center[1], self.takeoff_altitude]
        target_lat, target_lon, target_alt = self._world_point_to_global(takeoff_target_world)
        self.takeoff_target_alt_m = float(target_alt)
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param5=float(target_lat),
            param6=float(target_lon),
            param7=float(target_alt),
        )
        self.takeoff_sent_time_sec = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info("fixed-wing: takeoff command sent")

    def _ready_for_offboard(self) -> bool:
        current_climb_alt = self._current_climb_altitude()
        if current_climb_alt is None:
            return False
        return current_climb_alt >= max(4.0, self.takeoff_altitude * 0.6)

    def _ready_for_orbit_hold(self) -> bool:
        if self.takeoff_sent_time_sec is None:
            return False
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        delay_elapsed = (now_sec - self.takeoff_sent_time_sec) >= self.orbit_hold_enable_delay_sec
        current_climb_alt = self._current_climb_altitude()
        if current_climb_alt is None:
            return False
        enter_altitude = max(self.orbit_hold_ready_altitude, self.takeoff_altitude * 0.65)
        exit_altitude = max(enter_altitude - 0.8, enter_altitude * 0.88)
        if not delay_elapsed:
            self.orbit_hold_ready_active = False
            return False
        if self.orbit_hold_ready_active:
            self.orbit_hold_ready_active = current_climb_alt >= exit_altitude
        else:
            self.orbit_hold_ready_active = current_climb_alt >= enter_altitude
        return self.orbit_hold_ready_active

    def _current_relative_metrics(self) -> Optional[dict[str, float | str]]:
        if self.docking_status is None:
            return None
        rel_pos = self.docking_status.relative_position
        rel_vel = self.docking_status.relative_velocity
        phase = self.docking_status.phase.upper()
        distance = float(self.docking_status.relative_distance)
        rel_x = float(rel_pos.x)
        rel_y = float(rel_pos.y)
        rel_z = float(rel_pos.z)
        rel_vx = float(rel_vel.x)
        rel_vy = float(rel_vel.y)
        rel_vz = float(rel_vel.z)
        relative_speed = math.sqrt(rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz)
        if distance <= 1e-6:
            closing_rate = 0.0
        else:
            closing_rate = (
                rel_x * rel_vx +
                rel_y * rel_vy +
                rel_z * rel_vz
            ) / distance
        if rel_x < 0.0 and rel_vx > 0.2:
            horizon = min(max(-rel_x / rel_vx, 0.0), 2.5)
        else:
            horizon = min(max(distance / max(relative_speed, 1e-3), 0.0), 2.5)
        return {
            "phase": phase,
            "relative_distance": distance,
            "rel_x": rel_x,
            "rel_y": rel_y,
            "rel_z": rel_z,
            "rel_vx": rel_vx,
            "rel_vy": rel_vy,
            "rel_vz": rel_vz,
            "relative_speed": relative_speed,
            "closing_rate": closing_rate,
            "projected_abs_y": abs(rel_y + rel_vy * horizon),
        }

    def _current_flight_path_axis_world(self) -> tuple[float, float]:
        if self.vehicle_local_position is None:
            return self._normalize_xy(self.approach_axis_world[0], self.approach_axis_world[1])

        current_ground_vx = float(self.vehicle_local_position.vx)
        current_ground_vy = float(self.vehicle_local_position.vy)
        if math.hypot(current_ground_vx, current_ground_vy) > 3.0:
            return self._normalize_xy(current_ground_vx, current_ground_vy)

        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        radial_x = current_world_x - self.orbit_center[0]
        radial_y = current_world_y - self.orbit_center[1]
        radial_norm = math.hypot(radial_x, radial_y)
        if radial_norm <= 1e-6:
            return self._normalize_xy(self.approach_axis_world[0], self.approach_axis_world[1])
        return self._normalize_xy(-radial_y, radial_x)

    def _current_tangent_release_metrics(self) -> Optional[dict[str, float]]:
        if self.docking_status is None or self.carrier_odom is None or self.vehicle_local_position is None:
            return None

        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        carrier_pos = self.carrier_odom.pose.pose.position
        deck_x = carrier_pos.x + self.final_relative_position[0]
        deck_y = carrier_pos.y + self.final_relative_position[1]
        axis_x, axis_y = self._current_flight_path_axis_world()
        lateral_x = -axis_y
        lateral_y = axis_x
        delta_x = deck_x - current_world_x
        delta_y = deck_y - current_world_y
        return {
            "distance_xy": math.hypot(delta_x, delta_y),
            "forward_progress": delta_x * axis_x + delta_y * axis_y,
            "lateral_error": delta_x * lateral_x + delta_y * lateral_y,
            "rel_y": float(self.docking_status.relative_position.y),
            "axis_x": axis_x,
            "axis_y": axis_y,
        }

    def _seed_approach_axis_from_current_flight_path(self) -> None:
        axis_x, axis_y = self._current_flight_path_axis_world()
        self.approach_axis_world = [axis_x, axis_y]

    def _arm_glide_tangent_exit(self, metrics: Optional[dict[str, float]] = None) -> None:
        if not self.glide_tangent_exit_enable or self.vehicle_local_position is None:
            self.glide_tangent_exit_active = False
            self.glide_tangent_exit_start_time_sec = None
            return

        if metrics is None:
            metrics = self._current_tangent_release_metrics()
        axis_x, axis_y = self._current_flight_path_axis_world()
        if metrics is not None:
            axis_x = float(metrics["axis_x"])
            axis_y = float(metrics["axis_y"])

        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        self.glide_tangent_exit_active = True
        self.glide_tangent_exit_start_time_sec = now_sec
        self.glide_tangent_exit_axis_world = [axis_x, axis_y]
        self.glide_tangent_exit_anchor_world = [current_world_x, current_world_y]
        self.glide_tangent_exit_altitude = current_world_z
        self.glide_tangent_exit_history_valid = True
        self.approach_axis_world = [axis_x, axis_y]
        self.get_logger().info(
            "fixed-wing: tangent exit armed "
            f"anchor=({current_world_x:.3f},{current_world_y:.3f},{current_world_z:.3f}) "
            f"axis=({axis_x:.3f},{axis_y:.3f})"
        )

    def _update_glide_tangent_exit_state(self) -> None:
        if not self.glide_tangent_exit_active:
            self.handoff_debug = self._empty_handoff_debug()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.glide_tangent_exit_start_time_sec is None:
            self.glide_tangent_exit_active = False
            self.handoff_debug = self._empty_handoff_debug()
            return

        phase = self.docking_status.phase.upper() if self.docking_status is not None else "IDLE"
        elapsed = now_sec - self.glide_tangent_exit_start_time_sec
        handoff_metrics = self._compute_terminal_sync_metrics()
        sync_gate_distance = max(
            self.terminal_slowdown_start_distance + 3.2,
            self.capture_distance + 5.5,
            self.terminal_sync_start_distance + 1.6,
        )
        if self.glide_tangent_exit_sync_gate_distance_cap > 0.0:
            sync_gate_distance = min(
                sync_gate_distance,
                self.glide_tangent_exit_sync_gate_distance_cap,
            )
        sync_gate_lateral = max(
            self.terminal_sync_max_abs_y + 0.90,
            self.terminal_slowdown_max_abs_y + 0.40,
        )
        sync_gate_height = max(
            self.terminal_sync_max_height_offset + 0.90,
            1.35,
        )
        geom_ready = False
        if handoff_metrics is not None:
            geom_ready = (
                float(handoff_metrics["distance_xy"]) <= sync_gate_distance and
                abs(float(handoff_metrics["line_lateral_error"])) <= sync_gate_lateral and
                abs(float(handoff_metrics["height_error"])) <= sync_gate_height and
                float(handoff_metrics["line_progress"]) >= -0.5
            )
            self.handoff_debug = {
                "tangent_active": 1.0,
                "distance_xy": float(handoff_metrics["distance_xy"]),
                "line_progress": float(handoff_metrics["line_progress"]),
                "line_lateral_error": float(handoff_metrics["line_lateral_error"]),
                "height_error": float(handoff_metrics["height_error"]),
                "geom_ready": 1.0 if geom_ready else 0.0,
            }
        else:
            self.handoff_debug = {
                "tangent_active": 1.0,
                "distance_xy": math.nan,
                "line_progress": math.nan,
                "line_lateral_error": math.nan,
                "height_error": math.nan,
                "geom_ready": 0.0,
            }
        reason = None
        if phase == "COMPLETED":
            reason = f"phase={phase}"
        elif elapsed < self.glide_tangent_exit_min_hold_sec:
            return
        else:
            handoff_ready = (
                handoff_metrics is not None and
                geom_ready
            )
            if handoff_ready and handoff_metrics is not None:
                if self.glide_tangent_exit_release_distance_max > 0.0:
                    handoff_ready = (
                        handoff_ready and
                        float(handoff_metrics["distance_xy"]) <=
                        self.glide_tangent_exit_release_distance_max
                    )
                if self.glide_tangent_exit_release_height_error_max > 0.0:
                    handoff_ready = (
                        handoff_ready and
                        abs(float(handoff_metrics["height_error"])) <=
                        self.glide_tangent_exit_release_height_error_max
                    )
                if self.glide_tangent_exit_release_progress_min >= 0.0:
                    handoff_ready = (
                        handoff_ready and
                        float(handoff_metrics["line_progress"]) >=
                        self.glide_tangent_exit_release_progress_min
                    )
            if handoff_ready:
                reason = (
                    "handoff_ready "
                    f"distance={float(handoff_metrics['distance_xy']):.3f} "
                    f"line_lat={float(handoff_metrics['line_lateral_error']):.3f} "
                    f"height_error={float(handoff_metrics['height_error']):.3f} "
                    f"progress={float(handoff_metrics['line_progress']):.3f}"
                )
            elif phase not in {"APPROACH", "TRACKING", "DOCKING"}:
                reason = f"phase={phase}"
            else:
                max_hold_sec = max(
                    self.glide_tangent_exit_hold_sec * 3.0,
                    self.glide_tangent_exit_min_hold_sec + 10.0,
                )
                if elapsed >= max_hold_sec:
                    reason = f"safety_timeout elapsed={elapsed:.3f}"

        should_clear = reason is not None
        if not should_clear:
            return

        self.glide_tangent_exit_active = False
        self.glide_tangent_exit_start_time_sec = None
        self.handoff_debug["tangent_active"] = 0.0
        self.get_logger().info(f"fixed-wing: tangent exit cleared reason={reason}")

    def _reset_glide_score_state(self, reason: Optional[str] = None) -> None:
        if self.glide_score_observing and reason is not None:
            self.get_logger().info(
                "fixed-wing: glide score observe cleared "
                f"reason={reason} "
                f"best={self.glide_score_best:.3f} "
                f"cluster={self.glide_score_best_cluster or 'n/a'}"
            )
        self.glide_score_observing = False
        self.glide_score_good_count = 0
        self.glide_score_plateau_count = 0
        self.glide_score_best = math.inf
        self.glide_score_best_cluster = ""
        self.glide_score_last_score = math.inf
        self.glide_score_last_log_bucket = None
        self.glide_tangent_exit_active = False
        self.glide_tangent_exit_start_time_sec = None

    def _glide_score_observe_gate(
        self,
        metrics: dict[str, float | str],
    ) -> tuple[bool, str]:
        phase = str(metrics["phase"])
        if phase not in {"APPROACH", "TRACKING"}:
            return False, f"phase={phase}"
        distance = float(metrics["relative_distance"])
        rel_x = float(metrics["rel_x"])
        rel_y = float(metrics["rel_y"])
        rel_vy = float(metrics["rel_vy"])
        closing_rate = float(metrics["closing_rate"])
        projected_abs_y = float(metrics["projected_abs_y"])
        if distance < self.glide_score_observe_distance_min:
            return False, f"distance_low={distance:.3f}"
        if distance > self.glide_score_observe_distance_max:
            return False, f"distance_high={distance:.3f}"
        if rel_x < self.glide_score_observe_min_rel_x or rel_x > self.glide_score_observe_max_rel_x:
            return False, f"rel_x={rel_x:.3f}"
        if rel_y < self.glide_score_observe_min_rel_y or rel_y > self.glide_score_observe_max_rel_y:
            return False, f"rel_y={rel_y:.3f}"
        if projected_abs_y > self.glide_score_observe_max_projected_abs_y:
            return False, f"projected_abs_y={projected_abs_y:.3f}"
        if abs(rel_vy) > self.glide_score_observe_max_abs_vy:
            return False, f"abs_rel_vy={abs(rel_vy):.3f}"
        if closing_rate < self.glide_score_observe_min_closing_rate:
            return False, f"closing_rate_low={closing_rate:.3f}"
        if closing_rate > self.glide_score_observe_max_closing_rate:
            return False, f"closing_rate_high={closing_rate:.3f}"
        if rel_x >= 0.0 or rel_y <= 0.0 or rel_vy >= 0.0 or closing_rate >= 0.0:
            return False, "sign_pattern"
        return True, "observe"

    def _glide_score_cluster_score(
        self,
        metrics: dict[str, float | str],
        center: list[float],
        spread: list[float],
    ) -> float:
        total = 0.0
        for index, key in enumerate(GLIDE_SCORE_FEATURE_ORDER):
            total += (
                self.glide_score_weights[index] *
                abs(float(metrics[key]) - center[index]) /
                max(spread[index], 1e-6)
            )
        return total

    def _glide_score_candidate(
        self,
        metrics: dict[str, float | str],
    ) -> tuple[float, str]:
        score_a = self._glide_score_cluster_score(
            metrics,
            self.glide_score_cluster_a_center,
            self.glide_score_cluster_a_spread,
        )
        score_b = self._glide_score_cluster_score(
            metrics,
            self.glide_score_cluster_b_center,
            self.glide_score_cluster_b_spread,
        )
        if score_a <= score_b:
            return score_a, "A"
        return score_b, "B"

    def _close_tracking_release_gate(
        self,
        metrics: dict[str, float | str],
    ) -> tuple[bool, str]:
        if not self.close_tracking_release_enabled:
            return False, "disabled"
        phase = str(metrics["phase"])
        if phase != "TRACKING":
            return False, f"phase={phase}"

        distance = float(metrics["relative_distance"])
        rel_x = float(metrics["rel_x"])
        rel_y = float(metrics["rel_y"])
        rel_vx = float(metrics["rel_vx"])
        rel_vy = float(metrics["rel_vy"])
        window_ok = (
            self.close_tracking_release_min_distance <= distance <= self.close_tracking_release_max_distance and
            self.close_tracking_release_min_rel_x <= rel_x <= self.close_tracking_release_max_rel_x and
            self.close_tracking_release_min_rel_y <= rel_y <= self.close_tracking_release_max_rel_y and
            self.close_tracking_release_min_rel_vx <= rel_vx <= self.close_tracking_release_max_rel_vx and
            self.close_tracking_release_min_rel_vy <= rel_vy <= self.close_tracking_release_max_rel_vy
        )
        if window_ok:
            return True, "close_tracking"

        return False, (
            f"distance={distance:.3f},rel_x={rel_x:.3f},rel_y={rel_y:.3f},"
            f"rel_vx={rel_vx:.3f},rel_vy={rel_vy:.3f}"
        )

    def _should_start_glide_score_state_machine(self) -> bool:
        metrics = self._current_relative_metrics()
        if metrics is None:
            self._reset_glide_score_state()
            return False

        close_tracking_ok, close_tracking_reason = self._close_tracking_release_gate(metrics)
        if close_tracking_ok:
            self._arm_glide_tangent_exit()
            self.get_logger().info(
                "fixed-wing: glide close-tracking release accepted "
                f"distance={float(metrics['relative_distance']):.3f} "
                f"rel=({float(metrics['rel_x']):.3f},{float(metrics['rel_y']):.3f},{float(metrics['rel_z']):.3f}) "
                f"rel_v=({float(metrics['rel_vx']):.3f},{float(metrics['rel_vy']):.3f},{float(metrics['rel_vz']):.3f})"
            )
            return True

        observe_ok, observe_reason = self._glide_score_observe_gate(metrics)
        if not observe_ok:
            self._reset_glide_score_state(observe_reason)
            return False

        score, cluster = self._glide_score_candidate(metrics)
        if not self.glide_score_observing:
            self.glide_score_observing = True
            self.glide_score_good_count = 0
            self.glide_score_plateau_count = 0
            self.glide_score_best = score
            self.glide_score_best_cluster = cluster
            self.glide_score_last_score = score
            self.glide_score_last_log_bucket = int(math.floor(score))
            self.get_logger().info(
                "fixed-wing: glide score observe entered "
                f"phase={metrics['phase']} "
                f"distance={float(metrics['relative_distance']):.3f} "
                f"rel=({float(metrics['rel_x']):.3f},{float(metrics['rel_y']):.3f},{float(metrics['rel_z']):.3f}) "
                f"score={score:.3f} "
                f"cluster={cluster}"
            )

        improved = score < (self.glide_score_best - self.glide_score_min_improvement)
        if improved:
            self.glide_score_best = score
            self.glide_score_best_cluster = cluster
            self.glide_score_plateau_count = 0
        else:
            self.glide_score_plateau_count += 1

        if score <= self.glide_score_hold_threshold:
            self.glide_score_good_count += 1
        else:
            self.glide_score_good_count = 0

        log_bucket = int(math.floor(score))
        if improved and (
            self.glide_score_last_log_bucket is None or
            log_bucket < self.glide_score_last_log_bucket
        ):
            self.glide_score_last_log_bucket = log_bucket
            self.get_logger().info(
                "fixed-wing: glide score best "
                f"score={score:.3f} "
                f"cluster={cluster} "
                f"distance={float(metrics['relative_distance']):.3f} "
                f"projected_abs_y={float(metrics['projected_abs_y']):.3f} "
                f"closing_rate={float(metrics['closing_rate']):.3f}"
            )

        stable_good = (
            score <= self.glide_score_accept_threshold and
            self.glide_score_good_count >= self.glide_score_good_samples
        )
        plateau_release = (
            self.glide_score_best <= self.glide_score_accept_threshold and
            self.glide_score_plateau_count >= self.glide_score_plateau_samples and
            score <= self.glide_score_best + self.glide_score_rebound_margin
        )
        cluster_release_ok = not (
            cluster == "A" and
            float(metrics["projected_abs_y"]) < self.glide_score_cluster_a_min_projected_abs_y
        )

        self.glide_score_last_score = score
        if not cluster_release_ok:
            return False
        if not stable_good and not plateau_release:
            return False

        release_reason = "stable_good" if stable_good else "plateau"
        self._arm_glide_tangent_exit()
        self.get_logger().info(
            "fixed-wing: glide score release accepted "
            f"reason={release_reason} "
            f"cluster={cluster} "
            f"score={score:.3f} "
            f"best={self.glide_score_best:.3f} "
            f"good_samples={self.glide_score_good_count} "
            f"plateau_samples={self.glide_score_plateau_count} "
            f"distance={float(metrics['relative_distance']):.3f} "
            f"rel=({float(metrics['rel_x']):.3f},{float(metrics['rel_y']):.3f},{float(metrics['rel_z']):.3f}) "
            f"rel_v=({float(metrics['rel_vx']):.3f},{float(metrics['rel_vy']):.3f},{float(metrics['rel_vz']):.3f}) "
            f"projected_abs_y={float(metrics['projected_abs_y']):.3f} "
            f"closing_rate={float(metrics['closing_rate']):.3f} "
            f"relative_speed={float(metrics['relative_speed']):.3f}"
        )
        return True

    def _should_start_glide(self) -> bool:
        if self.glide_active:
            return True
        if not self.glide_release_enabled:
            return False
        if self.docking_status is None or self.carrier_odom is None:
            return False
        distance = float(self.docking_status.relative_distance)
        phase = self.docking_status.phase.upper()
        if phase in {"DOCKING", "COMPLETED"}:
            return True
        if self.glide_release_mode == "score_state_machine":
            return self._should_start_glide_score_state_machine()
        if phase != self.glide_trigger_phase:
            return False
        return distance <= self.glide_trigger_distance

    def _publish_orbit_setpoint(self) -> None:
        if self.vehicle_local_position is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_orbit_update_time is None:
            self.last_orbit_update_time = now_sec
        dt = max(0.0, min(now_sec - self.last_orbit_update_time, 0.1))
        self.last_orbit_update_time = now_sec

        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        radial_x = current_world_x - self.orbit_center[0]
        radial_y = current_world_y - self.orbit_center[1]
        current_radius = math.hypot(radial_x, radial_y)

        if current_radius >= 1e-3:
            current_phase = math.atan2(radial_y, radial_x)
            current_radial_unit = [radial_x / current_radius, radial_y / current_radius]
        else:
            current_phase = self.orbit_phase
            current_radial_unit = [math.cos(current_phase), math.sin(current_phase)]

        orbit_rate = self.orbit_speed / max(self.orbit_radius, 1.0)
        self.orbit_phase = (current_phase + orbit_rate * dt) % (2.0 * math.pi)
        target_phase = self.orbit_phase + self.orbit_phase_lead

        target_radial_unit = [math.cos(target_phase), math.sin(target_phase)]
        target_tangent_unit = [-math.sin(target_phase), math.cos(target_phase)]

        radial_velocity_correction = max(
            -self.orbit_radial_speed_limit,
            min(
                self.orbit_radial_speed_limit,
                self.orbit_radial_gain * (self.orbit_radius - current_radius),
            ),
        )

        desired_position_world = [
            self.orbit_center[0] + self.orbit_radius * target_radial_unit[0],
            self.orbit_center[1] + self.orbit_radius * target_radial_unit[1],
            self.wait_orbit_altitude,
        ]
        desired_velocity_world = [
            target_tangent_unit[0] * self.orbit_speed + current_radial_unit[0] * radial_velocity_correction,
            target_tangent_unit[1] * self.orbit_speed + current_radial_unit[1] * radial_velocity_correction,
            0.0,
        ]
        centripetal_accel = (self.orbit_speed * self.orbit_speed) / max(self.orbit_radius, 1.0)
        desired_acceleration_world = [
            -target_radial_unit[0] * centripetal_accel,
            -target_radial_unit[1] * centripetal_accel,
            0.0,
        ]

        self._publish_offboard_trajectory(
            position_world=desired_position_world,
            velocity_world=desired_velocity_world,
            acceleration_world=desired_acceleration_world,
        )
        self._publish_glide_visuals(desired_position_world)

    def _fill_invalid_position_setpoint(self, setpoint: PositionSetpoint, timestamp_us: int) -> None:
        setpoint.timestamp = timestamp_us
        setpoint.valid = False
        setpoint.type = PositionSetpoint.SETPOINT_TYPE_POSITION
        setpoint.vx = math.nan
        setpoint.vy = math.nan
        setpoint.vz = math.nan
        setpoint.lat = math.nan
        setpoint.lon = math.nan
        setpoint.alt = math.nan
        setpoint.yaw = math.nan
        setpoint.loiter_radius = math.nan
        setpoint.loiter_minor_radius = math.nan
        setpoint.loiter_direction_counter_clockwise = False
        setpoint.loiter_orientation = math.nan
        setpoint.loiter_pattern = PositionSetpoint.LOITER_TYPE_ORBIT
        setpoint.acceptance_radius = math.nan
        setpoint.alt_acceptance_radius = math.nan
        setpoint.cruising_speed = math.nan
        setpoint.gliding_enabled = False
        setpoint.cruising_throttle = math.nan

    def _fill_loiter_position_setpoint(self, setpoint: PositionSetpoint, timestamp_us: int) -> None:
        lat, lon, alt = self._world_point_to_global(
            [self.orbit_center[0], self.orbit_center[1], self.wait_orbit_altitude]
        )
        setpoint.timestamp = timestamp_us
        setpoint.valid = True
        setpoint.type = PositionSetpoint.SETPOINT_TYPE_LOITER
        setpoint.vx = math.nan
        setpoint.vy = math.nan
        setpoint.vz = math.nan
        setpoint.lat = lat
        setpoint.lon = lon
        setpoint.alt = alt
        setpoint.yaw = math.nan
        setpoint.loiter_radius = float(self.orbit_radius)
        setpoint.loiter_minor_radius = float(self.orbit_radius)
        setpoint.loiter_direction_counter_clockwise = False
        setpoint.loiter_orientation = 0.0
        setpoint.loiter_pattern = PositionSetpoint.LOITER_TYPE_ORBIT
        setpoint.acceptance_radius = float(self.orbit_radius)
        setpoint.alt_acceptance_radius = 2.0
        setpoint.cruising_speed = float(self.loiter_speed_command)
        setpoint.gliding_enabled = False
        setpoint.cruising_throttle = math.nan

    def _publish_wait_loiter_triplet(self) -> None:
        if self.vehicle_global_position is None or self.vehicle_local_position is None:
            return

        timestamp_us = int(self.get_clock().now().nanoseconds / 1000)
        triplet = PositionSetpointTriplet()
        triplet.timestamp = timestamp_us

        previous = PositionSetpoint()
        self._fill_invalid_position_setpoint(previous, timestamp_us)
        triplet.previous = previous

        current = PositionSetpoint()
        self._fill_loiter_position_setpoint(current, timestamp_us)
        triplet.current = current

        next_setpoint = PositionSetpoint()
        self._fill_invalid_position_setpoint(next_setpoint, timestamp_us)
        triplet.next = next_setpoint

        self.position_setpoint_triplet_pub.publish(triplet)

    def _publish_loiter_command_if_needed(self) -> None:
        if self.vehicle_global_position is None or self.vehicle_local_position is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        current_climb_alt = self._current_climb_altitude()
        if current_climb_alt is None:
            return

        if not self.orbit_hold_initialized:
            return
        min_loiter_altitude = max(self.takeoff_altitude - 1.2, self.takeoff_altitude * 0.93)

        if current_climb_alt < min_loiter_altitude:
            return

        refresh_period_sec = (
            self.refresh_orbit_period_sec if self.use_offboard_orbit_hold
            else self.native_wait_orbit_refresh_sec
        )

        if (
            self.loiter_command_sent and self.last_loiter_command_time is not None
            and (now_sec - self.last_loiter_command_time) < refresh_period_sec
        ):
            return

        lat, lon, alt = self._world_point_to_global(
            [self.orbit_center[0], self.orbit_center[1], self.wait_orbit_altitude]
        )
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_ORBIT,
            param1=float(self.orbit_radius),
            param5=lat,
            param6=lon,
            param7=alt,
        )
        # PX4 fixed-wing loiter resets cruising_speed on orbit activation, so reapply speed after DO_ORBIT.
        self._publish_speed_command_if_needed(self.loiter_speed_command, mode="loiter", force=True)
        if not self.loiter_command_sent:
            self.get_logger().info(
                "fixed-wing: orbit command sent "
                f"mode={'loiter_backup' if self.use_offboard_orbit_hold else 'loiter_native'} "
                f"center=({self.orbit_center[0]:.1f},{self.orbit_center[1]:.1f}) "
                f"radius={self.orbit_radius:.1f} "
                f"climb_alt={current_climb_alt:.1f}"
            )
        self.loiter_command_sent = True
        self.last_loiter_command_time = now_sec

    def _publish_speed_command_if_needed(self, speed: float, mode: str, force: bool = False) -> None:
        if (
            not force and
            self.current_speed_mode == mode and
            self.last_speed_command is not None and
            abs(self.last_speed_command - float(speed)) < 0.05
        ):
            return
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED,
            param1=0.0,
            param2=float(speed),
            param3=-1.0,
        )
        self.get_logger().info(
            "fixed-wing: speed command "
            f"mode={mode} speed={float(speed):.2f} force={str(force).lower()}"
        )
        self.current_speed_mode = mode
        self.last_speed_command = float(speed)

    def _publish_glide_setpoint(self) -> None:
        if self.carrier_odom is None or self.vehicle_local_position is None:
            return

        desired_point_world, desired_velocity_world = self._compute_glide_target_world()
        self._publish_offboard_trajectory(
            position_world=desired_point_world,
            velocity_world=desired_velocity_world,
            acceleration_world=[math.nan, math.nan, math.nan],
        )

        self._publish_glide_visuals(desired_point_world)

    def _select_terminal_speed(self) -> tuple[float, str]:
        phase = self.docking_status.phase.upper() if self.docking_status is not None else "APPROACH"
        distance = float(self.docking_status.relative_distance) if self.docking_status is not None else math.inf
        corridor_distance, corridor_along_error, corridor_lateral_error = self._terminal_approach_geometry()
        terminal_distance = corridor_distance if math.isfinite(corridor_distance) else distance
        abs_lateral_error = abs(corridor_lateral_error) if math.isfinite(corridor_lateral_error) else math.inf
        terminal_straight_guidance_active = (
            self.terminal_straight_active and
            phase in {"APPROACH", "TRACKING", "DOCKING"}
        )
        terminal_straight_elapsed = 0.0
        if terminal_straight_guidance_active and self.glide_tangent_exit_start_time_sec is not None:
            terminal_straight_elapsed = max(
                0.0,
                self.get_clock().now().nanoseconds * 1e-9 - self.glide_tangent_exit_start_time_sec,
            )
        sync_distance = max(
            self.terminal_slowdown_start_distance + 2.3,
            self.capture_distance + 4.0,
            self.terminal_sync_start_distance,
        )
        sync_ready = (
            terminal_straight_guidance_active and
            self.terminal_sync_active and
            math.isfinite(terminal_distance) and
            terminal_distance <= sync_distance and
            abs_lateral_error <= max(
                self.terminal_sync_max_abs_y + 0.85,
                self.terminal_slowdown_max_abs_y,
            ) and
            corridor_along_error > -1.0
        )

        if phase == "COMPLETED":
            selected_speed = self.capture_speed_command
            speed_mode = "capture"
        elif terminal_straight_guidance_active:
            if phase == "APPROACH":
                cruise_tracking_speed = min(
                    self.tracking_speed_command,
                    self.docking_speed_command + 0.75 * max(
                        0.0,
                        self.tracking_speed_command - self.docking_speed_command,
                    ),
                )
            else:
                cruise_tracking_speed = self.tracking_speed_command
            straight_slowdown_alpha = max(
                0.0,
                min(1.0, terminal_straight_elapsed / 10.0),
            )
            terminal_cruise_speed = cruise_tracking_speed + straight_slowdown_alpha * (
                self.docking_speed_command - cruise_tracking_speed
            )
            terminal_decel_alpha = max(
                0.0,
                min(1.0, terminal_straight_elapsed / 5.0),
            )
            monotone_terminal_speed = self.docking_speed_command + terminal_decel_alpha * (
                self.capture_speed_command - self.docking_speed_command
            )
            if self.terminal_straight_active:
                if sync_ready:
                    span = max(
                        sync_distance - self.capture_distance,
                        0.1,
                    )
                    alpha = max(
                        0.0,
                        min(1.0, (terminal_distance - self.capture_distance) / span),
                    )
                    selected_speed = self.capture_speed_command + alpha * (
                        self.docking_speed_command - self.capture_speed_command
                    )
                    speed_mode = "terminal_sync_straight"
                else:
                    entry_distance = max(
                        self.terminal_entry_distance,
                        self.terminal_slowdown_start_distance + 6.0,
                        self.capture_distance + 1.0,
                    )
                    if distance >= self.terminal_slowdown_start_distance:
                        span = max(
                            entry_distance - self.terminal_slowdown_start_distance,
                            0.1,
                        )
                        alpha = max(
                            0.0,
                            min(1.0, (distance - self.terminal_slowdown_start_distance) / span),
                        )
                        selected_speed = self.docking_speed_command + alpha * (
                            terminal_cruise_speed - self.docking_speed_command
                        )
                        speed_mode = "docking_straight" if phase == "DOCKING" else "tracking_straight"
                    elif distance <= self.capture_distance:
                        span = max(
                            self.capture_distance - self.terminal_slowdown_finish_distance,
                            0.1,
                        )
                        alpha = max(
                            0.0,
                            min(1.0, (distance - self.terminal_slowdown_finish_distance) / span),
                        )
                        selected_speed = self.capture_speed_command + alpha * (
                            self.docking_speed_command - self.capture_speed_command
                        )
                        speed_mode = "terminal_capture_straight"
                    elif distance <= self.terminal_slowdown_start_distance:
                        span = max(
                            self.terminal_slowdown_start_distance - self.capture_distance,
                            0.1,
                        )
                        alpha = max(
                            0.0,
                            min(1.0, (distance - self.capture_distance) / span),
                        )
                        selected_speed = self.docking_speed_command + alpha * (
                            terminal_cruise_speed - self.docking_speed_command
                        )
                        speed_mode = "terminal_slow_straight"
                    else:
                        selected_speed = monotone_terminal_speed
                        speed_mode = "terminal_hold_straight"
                selected_speed = max(
                    self.capture_speed_command,
                    min(selected_speed, monotone_terminal_speed),
                )
            else:
                cruise_speed = self.docking_speed_command + 0.45 * max(
                    0.0,
                    self.tracking_speed_command - self.docking_speed_command,
                )
                slowdown_ready = (
                    terminal_distance <= self.terminal_slowdown_start_distance and
                    abs_lateral_error <= self.terminal_slowdown_max_abs_y and
                    corridor_along_error > -0.6
                )
                if not slowdown_ready:
                    selected_speed = cruise_speed
                    speed_mode = "docking"
                elif terminal_distance <= self.capture_distance:
                    span = max(
                        self.capture_distance - self.terminal_slowdown_finish_distance,
                        0.1,
                    )
                    alpha = max(
                        0.0,
                        min(1.0, (terminal_distance - self.terminal_slowdown_finish_distance) / span),
                    )
                    selected_speed = self.capture_speed_command + alpha * (
                        cruise_speed - self.capture_speed_command
                    )
                    speed_mode = "terminal_capture"
                else:
                    span = max(
                        self.terminal_slowdown_start_distance - self.capture_distance,
                        0.1,
                    )
                    alpha = max(
                        0.0,
                        min(1.0, (terminal_distance - self.capture_distance) / span),
                    )
                    selected_speed = self.docking_speed_command + alpha * (
                        cruise_speed - self.docking_speed_command
                    )
                    speed_mode = "terminal_slow"
        elif phase == "TRACKING":
            selected_speed = self.tracking_speed_command
            speed_mode = "tracking"
        elif phase == "DOCKING":
            selected_speed = self.docking_speed_command
            speed_mode = "docking"
        else:
            selected_speed = self.glide_speed_command
            speed_mode = "glide"

        if self._energy_guard_active():
            recovery_speed = max(self.glide_speed_command, self.tracking_speed_command)
            if recovery_speed > selected_speed:
                selected_speed = recovery_speed
                speed_mode = f"{speed_mode}_energy_guard"

        return selected_speed, speed_mode

    def _energy_guard_active(self) -> bool:
        if not self.energy_guard_enable:
            return False

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        bad_airspeed = False
        bad_underspeed = False
        if self.airspeed_validated is not None:
            bad_airspeed = (
                float(self.airspeed_validated.true_airspeed_m_s) <
                self.energy_guard_min_true_airspeed_mps
            )
        if self.tecs_status is not None:
            bad_underspeed = (
                float(self.tecs_status.underspeed_ratio) >
                self.energy_guard_max_underspeed_ratio
            )

        if bad_airspeed or bad_underspeed:
            self.energy_guard_recover_until_sec = max(
                self.energy_guard_recover_until_sec,
                now_sec + self.energy_guard_hold_sec,
            )

        active = now_sec < self.energy_guard_recover_until_sec
        if active != self.energy_guard_last_active:
            self.energy_guard_last_active = active
            if active:
                self.get_logger().info(
                    "fixed-wing: energy guard active "
                    f"tas={float(self.airspeed_validated.true_airspeed_m_s) if self.airspeed_validated is not None else float('nan'):.2f} "
                    f"underspeed={float(self.tecs_status.underspeed_ratio) if self.tecs_status is not None else float('nan'):.3f}"
                )
            else:
                self.get_logger().info("fixed-wing: energy guard released")
        return active

    def _publish_offboard_trajectory(
        self,
        position_world: list[float],
        velocity_world: list[float],
        acceleration_world: list[float],
    ) -> None:
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        px4_position = self._world_position_to_px4_local(position_world)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = now_us
        has_velocity = any(math.isfinite(value) for value in velocity_world)
        has_acceleration = any(math.isfinite(value) for value in acceleration_world)
        offboard_mode.position = True
        offboard_mode.velocity = has_velocity
        offboard_mode.acceleration = has_acceleration
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        if hasattr(offboard_mode, "thrust_and_torque"):
            offboard_mode.thrust_and_torque = False
        if hasattr(offboard_mode, "direct_actuator"):
            offboard_mode.direct_actuator = False
        self.offboard_mode_pub.publish(offboard_mode)

        trajectory = TrajectorySetpoint()
        trajectory.timestamp = now_us
        trajectory.position = px4_position
        trajectory.velocity = self._world_vector_to_px4_or_nan(velocity_world)
        trajectory.acceleration = self._world_vector_to_px4_or_nan(acceleration_world)
        trajectory.yaw = math.nan
        trajectory.yawspeed = math.nan
        self.trajectory_pub.publish(trajectory)

        self.offboard_counter += 1
        if self.offboard_counter >= 20 and not self.offboard_mode_sent:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.offboard_mode_sent = True
            self.get_logger().info("fixed-wing: requested OFFBOARD")

    def _normalize_xy(self, x: float, y: float) -> tuple[float, float]:
        norm = math.hypot(x, y)
        if norm < 1e-6:
            return 1.0, 0.0
        return x / norm, y / norm

    def _blend_xy_directions(
        self,
        primary: tuple[float, float],
        secondary: tuple[float, float],
        primary_weight: float,
        secondary_weight: float,
    ) -> tuple[float, float]:
        blend_x = primary[0] * primary_weight + secondary[0] * secondary_weight
        blend_y = primary[1] * primary_weight + secondary[1] * secondary_weight
        return self._normalize_xy(blend_x, blend_y)

    def _terminal_approach_geometry(self) -> tuple[float, float, float]:
        if self.carrier_odom is None or self.vehicle_local_position is None:
            return math.inf, math.inf, math.inf

        carrier_pos = self.carrier_odom.pose.pose.position
        deck_x = carrier_pos.x + self.final_relative_position[0]
        deck_y = carrier_pos.y + self.final_relative_position[1]
        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)

        delta_x = deck_x - current_world_x
        delta_y = deck_y - current_world_y
        distance_xy = math.hypot(delta_x, delta_y)
        if distance_xy > 1e-3:
            direct_axis = self._normalize_xy(delta_x, delta_y)
        else:
            direct_axis = self._normalize_xy(self.approach_axis_world[0], self.approach_axis_world[1])

        approach_axis = self._blend_xy_directions(
            primary=self._normalize_xy(self.approach_axis_world[0], self.approach_axis_world[1]),
            secondary=direct_axis,
            primary_weight=0.72,
            secondary_weight=0.28,
        )
        lateral_x = -approach_axis[1]
        lateral_y = approach_axis[0]
        along_error = delta_x * approach_axis[0] + delta_y * approach_axis[1]
        lateral_error = delta_x * lateral_x + delta_y * lateral_y
        return distance_xy, along_error, lateral_error

    def _compute_glide_target_world(self) -> tuple[list[float], list[float]]:
        assert self.carrier_odom is not None
        assert self.vehicle_local_position is not None
        carrier_pos = self.carrier_odom.pose.pose.position
        deck_x = carrier_pos.x + self.final_relative_position[0]
        deck_y = carrier_pos.y + self.final_relative_position[1]
        actual_deck_z = carrier_pos.z + self.final_relative_position[2]
        nominal_deck_z = self.takeoff_altitude + self.final_relative_position[2]

        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)
        current_ground_vx = float(self.vehicle_local_position.vx)
        current_ground_vy = float(self.vehicle_local_position.vy)

        delta_x = deck_x - current_world_x
        delta_y = deck_y - current_world_y
        distance_xy = math.hypot(delta_x, delta_y)
        if distance_xy > 1e-3:
            direct_axis = self._normalize_xy(delta_x, delta_y)
        else:
            direct_axis = tuple(self.approach_axis_world)

        if math.hypot(current_ground_vx, current_ground_vy) > 4.0:
            ground_track_axis = self._normalize_xy(current_ground_vx, current_ground_vy)
        else:
            ground_track_axis = tuple(self.approach_axis_world)

        phase = self.docking_status.phase.upper() if self.docking_status is not None else "APPROACH"
        self._update_terminal_sync_state()
        selected_speed, _ = self._select_terminal_speed()
        if self.glide_tangent_exit_active and phase in {"APPROACH", "TRACKING", "DOCKING"}:
            axis_x, axis_y = self.glide_tangent_exit_axis_world
            lateral_x = -axis_y
            lateral_y = axis_x
            elapsed = 0.0
            if self.glide_tangent_exit_start_time_sec is not None:
                elapsed = max(
                    0.0,
                    self.get_clock().now().nanoseconds * 1e-9 - self.glide_tangent_exit_start_time_sec,
                )
            tangent_exit_alpha = max(
                0.0,
                min(1.0, elapsed / max(self.glide_tangent_exit_hold_sec, 1e-3)),
            )
            tangent_exit_speed = self.tracking_speed_command + tangent_exit_alpha * (
                self.docking_speed_command - self.tracking_speed_command
            )
            rel_anchor_x = current_world_x - self.glide_tangent_exit_anchor_world[0]
            rel_anchor_y = current_world_y - self.glide_tangent_exit_anchor_world[1]
            line_progress = rel_anchor_x * axis_x + rel_anchor_y * axis_y
            line_lateral_error = rel_anchor_x * lateral_x + rel_anchor_y * lateral_y
            lookahead = max(
                4.5,
                min(
                    10.0,
                    max(self.glide_tangent_exit_lookahead * 0.9, tangent_exit_speed * 0.72),
                ),
            )
            target_progress = max(line_progress, 0.0) + lookahead
            target_x = self.glide_tangent_exit_anchor_world[0] + axis_x * target_progress
            target_y = self.glide_tangent_exit_anchor_world[1] + axis_y * target_progress
            target_z = max(
                nominal_deck_z + 0.45,
                self.glide_tangent_exit_altitude - min(0.90, 0.28 * elapsed),
            )
            lateral_velocity_correction = max(
                -0.35 * self.glide_tangent_exit_lateral_limit,
                min(
                    0.35 * self.glide_tangent_exit_lateral_limit,
                    0.65 * self.glide_tangent_exit_lateral_gain * line_lateral_error,
                ),
            )
            velocity_x = axis_x * tangent_exit_speed - lateral_x * lateral_velocity_correction
            velocity_y = axis_y * tangent_exit_speed - lateral_y * lateral_velocity_correction
            velocity_z = max(
                min((target_z - current_world_z) * 0.35, 0.35),
                -0.35,
            )
            self.approach_axis_world = [axis_x, axis_y]
            return [target_x, target_y, target_z], [velocity_x, velocity_y, velocity_z]

        if phase in {"APPROACH", "TRACKING", "DOCKING", "COMPLETED"} and self.terminal_straight_active:
            sync_distance = max(
                self.terminal_slowdown_start_distance + 2.3,
                self.capture_distance + 4.0,
                self.terminal_sync_start_distance,
            )
            axis_x, axis_y = self.terminal_exit_axis_world
            lateral_x = -axis_y
            lateral_y = axis_x
            rel_anchor_x = current_world_x - self.terminal_exit_anchor_world[0]
            rel_anchor_y = current_world_y - self.terminal_exit_anchor_world[1]
            line_progress = rel_anchor_x * axis_x + rel_anchor_y * axis_y
            line_lateral_error = rel_anchor_x * lateral_x + rel_anchor_y * lateral_y
            sync_ready = (
                phase in {"TRACKING", "DOCKING"} and
                self.terminal_sync_active and
                distance_xy <= sync_distance and
                abs(line_lateral_error) <= max(
                    self.terminal_sync_max_abs_y + 1.2,
                    self.terminal_slowdown_max_abs_y + 0.35,
                )
            )
            lookahead = max(6.0, min(16.0, selected_speed * 1.35))
            if sync_ready:
                lookahead = max(2.8, min(6.0, selected_speed * 0.85))
            elif phase == "COMPLETED":
                lookahead = max(4.0, min(10.0, selected_speed * 1.1))
            target_progress = line_progress + lookahead
            lateral_pull_gain = 0.55 if not sync_ready else 0.32
            lateral_pull_limit = 1.2 if not sync_ready else 0.45
            lateral_pull = max(
                -lateral_pull_limit,
                min(lateral_pull_limit, lateral_pull_gain * line_lateral_error),
            )
            target_x = (
                self.terminal_exit_anchor_world[0] +
                axis_x * target_progress -
                lateral_x * lateral_pull
            )
            target_y = (
                self.terminal_exit_anchor_world[1] +
                axis_y * target_progress -
                lateral_y * lateral_pull
            )
            if sync_ready:
                sync_span = max(sync_distance - self.capture_distance, 0.1)
                sync_alpha = max(
                    0.0,
                    min(1.0, (distance_xy - self.capture_distance) / sync_span),
                )
                target_height_offset = self.terminal_sync_min_height_offset + sync_alpha * (
                    self.terminal_sync_max_height_offset - self.terminal_sync_min_height_offset
                )
                target_z = actual_deck_z + target_height_offset
                lateral_velocity_limit = 0.55
                lateral_velocity_gain = 0.38
                velocity_z = max(
                    min((target_z - current_world_z) * 0.55, self.terminal_sync_descent_rate),
                    -self.terminal_sync_descent_rate,
                )
            else:
                target_z = self.terminal_exit_altitude
                lateral_velocity_limit = 0.9
                lateral_velocity_gain = 0.65
                velocity_z = max(
                    min((self.terminal_exit_altitude - current_world_z) * 0.25, 0.25),
                    -0.25,
                )
            velocity_x = axis_x * selected_speed - lateral_x * max(
                -lateral_velocity_limit,
                min(lateral_velocity_limit, lateral_velocity_gain * line_lateral_error),
            )
            velocity_y = axis_y * selected_speed - lateral_y * max(
                -lateral_velocity_limit,
                min(lateral_velocity_limit, lateral_velocity_gain * line_lateral_error),
            )
            return [target_x, target_y, target_z], [velocity_x, velocity_y, velocity_z]

        if phase == "DOCKING":
            direct_weight = 0.82
            smoothing_alpha = 0.34
        elif phase == "TRACKING":
            direct_weight = 0.68
            smoothing_alpha = 0.24
        else:
            direct_weight = 0.42
            smoothing_alpha = 0.10

        desired_axis = self._blend_xy_directions(
            primary=direct_axis,
            secondary=ground_track_axis,
            primary_weight=direct_weight,
            secondary_weight=1.0 - direct_weight,
        )
        blended_axis = self._blend_xy_directions(
            primary=desired_axis,
            secondary=tuple(self.approach_axis_world),
            primary_weight=smoothing_alpha,
            secondary_weight=1.0 - smoothing_alpha,
        )
        self.approach_axis_world = [blended_axis[0], blended_axis[1]]
        axis_x, axis_y = blended_axis
        lateral_x = -axis_y
        lateral_y = axis_x
        along_error = delta_x * axis_x + delta_y * axis_y
        lateral_error = delta_x * lateral_x + delta_y * lateral_y

        if phase == "DOCKING":
            deck_follow_blend = max(0.0, min(1.0, (4.0 - distance_xy) / 3.0))
        elif phase == "TRACKING":
            deck_follow_blend = max(0.0, min(0.25, (3.0 - distance_xy) / 3.0))
        else:
            deck_follow_blend = 0.0
        deck_z = nominal_deck_z + (actual_deck_z - nominal_deck_z) * deck_follow_blend

        if phase == "DOCKING":
            standoff = min(4.0, max(0.0, distance_xy - 1.2))
            speed_cmd = min(self.glide_speed, selected_speed)
            height = deck_z + min(0.8, distance_xy * 0.10)
        elif phase == "TRACKING":
            standoff = min(self.glide_entry_distance * 0.45, max(0.0, distance_xy - 1.8))
            speed_cmd = min(self.glide_speed, selected_speed)
            height = deck_z + min(self.glide_entry_height * 0.45, distance_xy * 0.16)
        else:
            standoff = min(self.glide_entry_distance, max(0.0, distance_xy - 4.0))
            speed_cmd = min(self.glide_speed, selected_speed)
            height = deck_z + min(self.glide_entry_height, distance_xy * 0.25)

        lateral_corridor_pull = 0.0
        if phase == "DOCKING":
            lateral_corridor_pull = max(-4.5, min(4.5, 0.92 * lateral_error))
        elif phase == "TRACKING":
            lateral_corridor_pull = max(-4.0, min(4.0, 0.78 * lateral_error))
        elif phase == "COMPLETED":
            lateral_corridor_pull = max(-4.0, min(4.0, 0.90 * lateral_error))

        target_x = deck_x - axis_x * standoff - lateral_x * lateral_corridor_pull
        target_y = deck_y - axis_y * standoff - lateral_y * lateral_corridor_pull
        target_z = height

        lateral_velocity_correction = 0.0
        if phase == "DOCKING":
            lateral_velocity_correction = max(-2.8, min(2.8, 0.82 * lateral_error))
        elif phase == "TRACKING":
            lateral_velocity_correction = max(-2.4, min(2.4, 0.68 * lateral_error))
        elif phase == "COMPLETED":
            lateral_velocity_correction = max(-2.4, min(2.4, 0.78 * lateral_error))
        velocity_x = axis_x * speed_cmd - lateral_x * lateral_velocity_correction
        velocity_y = axis_y * speed_cmd - lateral_y * lateral_velocity_correction
        vertical_error = target_z - current_world_z
        velocity_z = max(min(vertical_error * 0.35, self.glide_descent_rate), -self.glide_descent_rate)

        if distance_xy < 6.0:
            close_standoff = 0.0
            if phase in {"TRACKING", "DOCKING"}:
                close_standoff = max(0.28, min(2.0, 0.30 * distance_xy + 0.28))
            close_lateral_pull = max(-1.2, min(1.2, 0.45 * lateral_error))
            target_x = deck_x - axis_x * close_standoff - lateral_x * close_lateral_pull
            target_y = deck_y - axis_y * close_standoff - lateral_y * close_lateral_pull
            target_z = deck_z
            speed_cmd = min(speed_cmd, self.capture_speed_command if phase == "DOCKING" else speed_cmd)
            velocity_x = axis_x * speed_cmd - lateral_x * max(-1.8, min(1.8, 1.05 * lateral_error))
            velocity_y = axis_y * speed_cmd - lateral_y * max(-1.8, min(1.8, 1.05 * lateral_error))
            velocity_z = max(min((deck_z - current_world_z) * 0.4, 0.5), -0.5)

        if self.docking_status is not None and self.docking_status.phase.upper() == "COMPLETED":
            target_x = deck_x
            target_y = deck_y
            target_z = actual_deck_z
            velocity_z = 0.0

        if phase in {"TRACKING", "DOCKING"}:
            target_z = min(target_z, nominal_deck_z + 0.9)

        return [target_x, target_y, target_z], [velocity_x, velocity_y, velocity_z]

    def _publish_glide_visuals(self, target_world: list[float]) -> None:
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = target_world[0]
        pose.pose.position.y = target_world[1]
        pose.pose.position.z = target_world[2]
        pose.pose.orientation.w = 1.0
        self.glide_target_pub.publish(pose)

        if self.carrier_odom is None:
            return
        path = Path()
        path.header = pose.header
        for alpha in [0.0, 0.33, 0.66, 1.0]:
            point_pose = PoseStamped()
            point_pose.header = pose.header
            point_pose.pose.orientation.w = 1.0
            point_pose.pose.position.x = (
                self.carrier_odom.pose.pose.position.x * alpha + target_world[0] * (1.0 - alpha)
            )
            point_pose.pose.position.y = (
                self.carrier_odom.pose.pose.position.y * alpha + target_world[1] * (1.0 - alpha)
            )
            point_pose.pose.position.z = (
                self.carrier_odom.pose.pose.position.z * alpha + target_world[2] * (1.0 - alpha)
            )
            path.poses.append(point_pose)
        self.glide_path_pub.publish(path)

    def _world_position_to_px4_local(self, point_world: list[float]) -> list[float]:
        return [
            float(point_world[0]) - self.world_offset[0],
            float(point_world[1]) - self.world_offset[1],
            -(float(point_world[2]) - self.world_offset[2]),
        ]

    def _world_velocity_to_px4_local(self, velocity_world: list[float]) -> list[float]:
        return [float(velocity_world[0]), float(velocity_world[1]), -float(velocity_world[2])]

    def _world_vector_to_px4_or_nan(self, vector_world: list[float]) -> list[float]:
        if not all(math.isfinite(float(component)) for component in vector_world):
            return [math.nan, math.nan, math.nan]
        return self._world_velocity_to_px4_local(vector_world)

    def _world_point_to_global(self, point_world: list[float]) -> tuple[float, float, float]:
        assert self.vehicle_global_position is not None
        assert self.vehicle_local_position is not None
        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)

        d_north = float(point_world[0]) - current_world_x
        d_east = float(point_world[1]) - current_world_y
        lat = float(self.vehicle_global_position.lat) + (d_north / 6378137.0) * (180.0 / math.pi)
        lon = float(self.vehicle_global_position.lon) + (
            d_east / (6378137.0 * max(math.cos(math.radians(float(self.vehicle_global_position.lat))), 1e-6))
        ) * (180.0 / math.pi)
        alt = float(self.vehicle_global_position.alt) + (float(point_world[2]) - current_world_z)
        return lat, lon, alt

    def _publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = int(command)
        msg.target_system = self.vehicle_id
        msg.target_component = 1
        msg.source_system = self.vehicle_id
        msg.source_component = 1
        msg.from_external = True
        for index in range(1, 8):
            setattr(msg, f"param{index}", float(params.get(f"param{index}", 0.0)))
        self.vehicle_command_pub.publish(msg)

    def _publish_status_debug_if_needed(self, mode: str) -> None:
        if self.vehicle_local_position is None or self.vehicle_status is None:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            self.last_status_debug_time_sec is not None and
            (now_sec - self.last_status_debug_time_sec) < 2.0
        ):
            return
        self.last_status_debug_time_sec = now_sec
        world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        world_z = self.world_offset[2] - float(self.vehicle_local_position.z)
        current_climb_alt = self._current_climb_altitude()
        climb_text = "n/a" if current_climb_alt is None else f"{current_climb_alt:.1f}"
        xy_speed = math.hypot(
            float(self.vehicle_local_position.vx),
            float(self.vehicle_local_position.vy),
        )
        airspeed_text = "tas=n/a thr=n/a"
        if self.airspeed_validated is not None:
            airspeed_text = (
                f"tas={float(self.airspeed_validated.true_airspeed_m_s):.1f} "
                f"thr={float(self.airspeed_validated.throttle_filtered):.2f}"
            )
        setpoint_text = "sp=n/a"
        if self.vehicle_setpoint_triplet is not None:
            current_sp = self.vehicle_setpoint_triplet.current
            setpoint_text = (
                "sp="
                f"{self._position_setpoint_type_name(int(current_sp.type))}"
                f" alt={float(current_sp.alt):.1f}"
                f" speed={float(current_sp.cruising_speed):.1f}"
                f" radius={float(current_sp.loiter_radius):.1f}"
            )
        self.get_logger().info(
            "fixed-wing: wait state "
            f"mode={mode} "
            f"nav={self._nav_state_name(int(self.vehicle_status.nav_state))} "
            f"arming={self._arming_state_name(int(self.vehicle_status.arming_state))} "
            f"pos=({world_x:.1f},{world_y:.1f},{world_z:.1f}) "
            f"xy_speed={xy_speed:.1f} "
            f"{airspeed_text} "
            f"hold_z={self.wait_orbit_altitude:.1f} "
            f"climb_alt={climb_text} "
            f"target_alt={self.takeoff_altitude:.1f} "
            f"{setpoint_text}"
        )

    def _current_climb_altitude(self) -> Optional[float]:
        if self.vehicle_global_position is None or self.takeoff_reference_alt_m is None:
            return None
        return float(self.vehicle_global_position.alt) - float(self.takeoff_reference_alt_m)

    @staticmethod
    def _yaw_from_quaternion(quat) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _nav_state_name(value: int) -> str:
        names = {
            int(VehicleStatus.NAVIGATION_STATE_MANUAL): "MANUAL",
            int(VehicleStatus.NAVIGATION_STATE_AUTO_MISSION): "AUTO_MISSION",
            int(VehicleStatus.NAVIGATION_STATE_AUTO_LOITER): "AUTO_LOITER",
            int(VehicleStatus.NAVIGATION_STATE_OFFBOARD): "OFFBOARD",
            int(VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF): "AUTO_TAKEOFF",
            int(VehicleStatus.NAVIGATION_STATE_ORBIT): "ORBIT",
        }
        return names.get(value, str(value))

    @staticmethod
    def _arming_state_name(value: int) -> str:
        names = {
            int(VehicleStatus.ARMING_STATE_DISARMED): "DISARMED",
            int(VehicleStatus.ARMING_STATE_ARMED): "ARMED",
        }
        return names.get(value, str(value))

    @staticmethod
    def _command_result_name(value: int) -> str:
        names = {
            int(VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED): "ACCEPTED",
            int(VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED): "TEMP_REJECTED",
            int(VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED): "DENIED",
            int(VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED): "UNSUPPORTED",
            int(VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED): "FAILED",
            int(VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS): "IN_PROGRESS",
            int(VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED): "CANCELLED",
        }
        return names.get(value, str(value))

    @staticmethod
    def _position_setpoint_type_name(value: int) -> str:
        names = {
            int(PositionSetpoint.SETPOINT_TYPE_POSITION): "POSITION",
            int(PositionSetpoint.SETPOINT_TYPE_LOITER): "LOITER",
            int(PositionSetpoint.SETPOINT_TYPE_TAKEOFF): "TAKEOFF",
            int(PositionSetpoint.SETPOINT_TYPE_LAND): "LAND",
            int(PositionSetpoint.SETPOINT_TYPE_IDLE): "IDLE",
            int(PositionSetpoint.SETPOINT_TYPE_VELOCITY): "VELOCITY",
        }
        return names.get(value, str(value))


def main() -> None:
    rclpy.init()
    node = Px4FixedWingBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
