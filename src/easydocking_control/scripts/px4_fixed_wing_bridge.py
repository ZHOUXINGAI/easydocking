#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from easydocking_msgs.msg import DockingCommand, DockingStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import (
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
    )
except ImportError:  # pragma: no cover
    AirspeedValidated = None
    OffboardControlMode = None
    PositionSetpoint = None
    PositionSetpointTriplet = None
    TrajectorySetpoint = None
    VehicleCommand = None
    VehicleCommandAck = None
    VehicleGlobalPosition = None
    VehicleLocalPosition = None
    VehicleStatus = None


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
        self.declare_parameter("glide_trigger_phase", "TRACKING")
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
        self.declare_parameter("use_offboard_orbit_hold", True)
        self.declare_parameter("max_orbit_entry_phase_correction_deg", 45.0)
        self.declare_parameter("orbit_hold_ready_altitude", 1.5)
        self.declare_parameter("orbit_hold_enable_delay_sec", 3.0)
        self.declare_parameter("orbit_radial_gain", 0.35)
        self.declare_parameter("orbit_radial_speed_limit", 2.0)
        self.declare_parameter("orbit_phase_lead_deg", 18.0)
        self.declare_parameter("orbit_recenter_distance_ratio", 1.3)
        self.declare_parameter("allow_orbit_recentering", False)

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
        self.orbit_hold_initialized = False
        self.orbit_hold_ready_active = False
        self.orbit_hold_committed = False
        self.wait_loiter_active = False
        self.wait_orbit_altitude = self.takeoff_altitude
        self.takeoff_sent_time_sec = None
        self.takeoff_reference_alt_m = None
        self.takeoff_target_alt_m = None
        self.last_status_debug_time_sec = None
        self.last_logged_nav_state = None
        self.last_logged_arming_state = None
        self.last_ack_signature = None

        self.vehicle_status: Optional[VehicleStatus] = None
        self.vehicle_global_position: Optional[VehicleGlobalPosition] = None
        self.vehicle_local_position: Optional[VehicleLocalPosition] = None
        self.vehicle_setpoint_triplet: Optional[PositionSetpointTriplet] = None
        self.airspeed_validated: Optional[AirspeedValidated] = None
        self.carrier_odom: Optional[Odometry] = None
        self.docking_status: Optional[DockingStatus] = None

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

        self.timer = self.create_timer(0.05, self._timer_cb)

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
            self.orbit_hold_initialized = False
            self.orbit_hold_ready_active = False
            self.orbit_hold_committed = False
            self.wait_loiter_active = False
            self.wait_orbit_altitude = self.takeoff_altitude
            self.takeoff_sent_time_sec = None
            self.takeoff_reference_alt_m = None
            self.takeoff_target_alt_m = None
            self.last_status_debug_time_sec = None
            if command == "RESET":
                self.takeoff_sent = False
            self.get_logger().info(f"fixed-wing: received {command}")

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
            self.glide_active = True

        if self.glide_active and self._ready_for_offboard():
            if not self.glide_activation_logged and self.docking_status is not None:
                self.get_logger().info(
                    "fixed-wing: glide window accepted "
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
            self._initialize_orbit_hold_if_needed()
            wait_loiter_enabled = self.use_offboard_orbit_hold and (
                self.wait_loiter_active or self._ready_for_orbit_hold()
            )
            if wait_loiter_enabled:
                if not self.wait_loiter_active and self.vehicle_local_position is not None:
                    current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)
                    self.wait_orbit_altitude = max(self.takeoff_altitude, current_world_z)
                self.wait_loiter_active = True
                self.orbit_hold_committed = True
                self.offboard_active = True
                self._publish_speed_command_if_needed(self.loiter_speed_command, mode="loiter")
                self._publish_orbit_setpoint()
                self._publish_status_debug_if_needed("wait_orbit_offboard")
            else:
                self.wait_loiter_active = False
                self.orbit_hold_committed = False
                self.offboard_active = False
                self.offboard_mode_sent = False
                self.offboard_counter = 0
                self._publish_status_debug_if_needed("climb_wait")

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

    def _should_start_glide(self) -> bool:
        if self.glide_active:
            return True
        if self.docking_status is None or self.carrier_odom is None:
            return False
        distance = float(self.docking_status.relative_distance)
        phase = self.docking_status.phase.upper()
        if phase not in {self.glide_trigger_phase, "DOCKING", "COMPLETED"}:
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
        if not self.use_offboard_orbit_hold:
            return
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

        if (
            self.loiter_command_sent and self.last_loiter_command_time is not None
            and (now_sec - self.last_loiter_command_time) < self.refresh_orbit_period_sec
        ):
            return

        lat, lon, alt = self._world_point_to_global(
            [self.orbit_center[0], self.orbit_center[1], self.wait_orbit_altitude]
        )
        self._publish_speed_command_if_needed(self.loiter_speed_command, mode="loiter")
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_ORBIT,
            param1=float(self.orbit_radius),
            param5=lat,
            param6=lon,
            param7=alt,
        )
        if not self.loiter_command_sent:
            self.get_logger().info(
                "fixed-wing: orbit command sent "
                "mode=loiter backup "
                f"center=({self.orbit_center[0]:.1f},{self.orbit_center[1]:.1f}) "
                f"radius={self.orbit_radius:.1f} "
                f"climb_alt={current_climb_alt:.1f}"
            )
        self.loiter_command_sent = True
        self.last_loiter_command_time = now_sec

    def _publish_speed_command_if_needed(self, speed: float, mode: str) -> None:
        if (
            self.current_speed_mode == mode and
            self.last_speed_command is not None and
            abs(self.last_speed_command - float(speed)) < 0.05
        ):
            return
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED,
            param1=1.0,
            param2=float(speed),
            param3=-1.0,
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
        abs_rel_y = (
            abs(float(self.docking_status.relative_position.y))
            if self.docking_status is not None else math.inf
        )

        if phase == "COMPLETED":
            return self.capture_speed_command, "capture"
        if phase == "TRACKING":
            return self.tracking_speed_command, "tracking"
        if phase == "DOCKING":
            cruise_speed = max(self.docking_speed_command, self.tracking_speed_command - 0.15)
            slowdown_ready = (
                distance <= self.terminal_slowdown_start_distance and
                abs_rel_y <= self.terminal_slowdown_max_abs_y
            )
            if not slowdown_ready:
                return cruise_speed, "docking"

            if distance <= self.capture_distance:
                span = max(
                    self.capture_distance - self.terminal_slowdown_finish_distance,
                    0.1,
                )
                alpha = max(
                    0.0,
                    min(1.0, (distance - self.terminal_slowdown_finish_distance) / span),
                )
                speed = self.capture_speed_command + alpha * (cruise_speed - self.capture_speed_command)
                return speed, "terminal_capture"

            span = max(
                self.terminal_slowdown_start_distance - self.capture_distance,
                0.1,
            )
            alpha = max(
                0.0,
                min(1.0, (distance - self.capture_distance) / span),
            )
            speed = self.docking_speed_command + alpha * (cruise_speed - self.docking_speed_command)
            return speed, "terminal_slow"
        return self.glide_speed_command, "glide"

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

    def _compute_glide_target_world(self) -> tuple[list[float], list[float]]:
        assert self.carrier_odom is not None
        assert self.vehicle_local_position is not None
        carrier_pos = self.carrier_odom.pose.pose.position
        deck_x = carrier_pos.x + self.final_relative_position[0]
        deck_y = carrier_pos.y + self.final_relative_position[1]
        deck_z = carrier_pos.z + self.final_relative_position[2]

        current_world_x = self.world_offset[0] + float(self.vehicle_local_position.x)
        current_world_y = self.world_offset[1] + float(self.vehicle_local_position.y)
        current_world_z = self.world_offset[2] - float(self.vehicle_local_position.z)

        carrier_yaw = self._yaw_from_quaternion(self.carrier_odom.pose.pose.orientation)
        corridor_x = math.cos(carrier_yaw)
        corridor_y = math.sin(carrier_yaw)
        lateral_x = -corridor_y
        lateral_y = corridor_x

        delta_x = deck_x - current_world_x
        delta_y = deck_y - current_world_y
        along_error = delta_x * corridor_x + delta_y * corridor_y
        lateral_error = delta_x * lateral_x + delta_y * lateral_y
        distance_xy = math.hypot(delta_x, delta_y)
        if distance_xy > 1e-3:
            direct_x = delta_x / distance_xy
            direct_y = delta_y / distance_xy
        else:
            direct_x = corridor_x
            direct_y = corridor_y

        phase = self.docking_status.phase.upper() if self.docking_status is not None else "APPROACH"
        selected_speed, _ = self._select_terminal_speed()
        corridor_blend = 0.0
        if phase in {"TRACKING", "DOCKING", "COMPLETED"}:
            corridor_blend = 0.82 if abs(lateral_error) > 2.0 else 0.72
            axis_x = corridor_x * corridor_blend + direct_x * (1.0 - corridor_blend)
            axis_y = corridor_y * corridor_blend + direct_y * (1.0 - corridor_blend)
        else:
            axis_x = corridor_x * 0.45 + direct_x * 0.55
            axis_y = corridor_y * 0.45 + direct_y * 0.55
        axis_norm = max(math.hypot(axis_x, axis_y), 1e-6)
        axis_x /= axis_norm
        axis_y /= axis_norm

        if phase == "DOCKING":
            standoff = min(8.0, max(0.0, distance_xy - 1.5))
            speed_cmd = min(self.glide_speed, selected_speed)
            height = deck_z + min(1.2, distance_xy * 0.12)
        elif phase == "TRACKING":
            standoff = min(self.glide_entry_distance * 0.55, max(0.0, distance_xy - 2.5))
            speed_cmd = min(self.glide_speed, selected_speed)
            height = deck_z + min(self.glide_entry_height * 0.55, distance_xy * 0.18)
        else:
            standoff = min(self.glide_entry_distance, max(0.0, distance_xy - 4.0))
            speed_cmd = min(self.glide_speed, selected_speed)
            height = deck_z + min(self.glide_entry_height, distance_xy * 0.25)

        lateral_corridor_pull = 0.0
        if phase in {"TRACKING", "DOCKING", "COMPLETED"}:
            lateral_corridor_pull = max(-3.5, min(3.5, 0.65 * lateral_error))

        target_x = deck_x - axis_x * standoff - lateral_x * lateral_corridor_pull
        target_y = deck_y - axis_y * standoff - lateral_y * lateral_corridor_pull
        target_z = height

        lateral_velocity_correction = 0.0
        if phase in {"TRACKING", "DOCKING", "COMPLETED"}:
            lateral_velocity_correction = max(-2.0, min(2.0, 0.55 * lateral_error))
        velocity_x = axis_x * speed_cmd - lateral_x * lateral_velocity_correction
        velocity_y = axis_y * speed_cmd - lateral_y * lateral_velocity_correction
        vertical_error = target_z - current_world_z
        velocity_z = max(min(vertical_error * 0.35, self.glide_descent_rate), -self.glide_descent_rate)

        if distance_xy < 6.0:
            target_x = deck_x
            target_y = deck_y
            target_z = deck_z
            speed_cmd = min(speed_cmd, self.capture_speed_command if phase == "DOCKING" else speed_cmd)
            velocity_x = corridor_x * speed_cmd - lateral_x * max(-1.3, min(1.3, 0.8 * lateral_error))
            velocity_y = corridor_y * speed_cmd - lateral_y * max(-1.3, min(1.3, 0.8 * lateral_error))
            velocity_z = max(min((deck_z - current_world_z) * 0.4, 0.5), -0.5)

        if self.docking_status is not None and self.docking_status.phase.upper() == "COMPLETED":
            target_x = deck_x
            target_y = deck_y
            target_z = deck_z
            velocity_z = 0.0

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
            f"hold_z={world_z:.1f} "
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
