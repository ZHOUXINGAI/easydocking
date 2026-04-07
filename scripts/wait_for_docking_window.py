#!/usr/bin/env python3

import math
import sys
from typing import Optional

import rclpy
from easydocking_msgs.msg import DockingCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import AirspeedValidated, TecsStatus
except ImportError:  # pragma: no cover
    AirspeedValidated = None
    TecsStatus = None


class DockingWindowStarter(Node):
    def __init__(self) -> None:
        super().__init__("docking_window_starter")
        self.declare_parameter("alignment_min", 0.65)
        self.declare_parameter("tca_min_sec", 4.6)
        self.declare_parameter("tca_max_sec", 5.4)
        self.declare_parameter("relative_z_min_m", 20.0)
        self.declare_parameter("relative_distance_min_m", 35.0)
        self.declare_parameter("relative_distance_max_m", 95.0)
        self.declare_parameter("relative_speed_min_mps", 8.0)
        self.declare_parameter("hold_count", 4)
        self.declare_parameter("timeout_sec", 60.0)
        self.declare_parameter("publish_repeats", 2)
        self.declare_parameter("fallback_immediate", True)
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
        self.declare_parameter("enable_geometry_cluster_gate", False)
        self.declare_parameter("geometry_cluster_score_threshold", 1.0)
        self.declare_parameter("geometry_cluster_min_rel_z", 33.15)
        self.declare_parameter("geometry_cluster_max_rel_z", 33.40)
        self.declare_parameter("geometry_cluster_a_center", [-4.166384, 105.1457845, -6.485333, -10.2991625])
        self.declare_parameter("geometry_cluster_a_spread", [1.6, 1.2, 0.25, 0.25])
        self.declare_parameter("geometry_cluster_b_center", [-5.395847, 102.9623125, -6.2016955, -10.510685])
        self.declare_parameter("geometry_cluster_b_spread", [0.9, 1.2, 0.22, 0.18])
        self.declare_parameter("require_mini_energy_healthy", False)
        self.declare_parameter("mini_px4_namespace", "/px4_2")
        self.declare_parameter("health_min_true_airspeed_mps", 7.0)
        self.declare_parameter("health_max_underspeed_ratio", 0.35)
        self.declare_parameter("health_min_samples", 5)

        self.alignment_min = float(self.get_parameter("alignment_min").value)
        self.tca_min_sec = float(self.get_parameter("tca_min_sec").value)
        self.tca_max_sec = float(self.get_parameter("tca_max_sec").value)
        self.relative_z_min_m = float(self.get_parameter("relative_z_min_m").value)
        self.relative_distance_min_m = float(self.get_parameter("relative_distance_min_m").value)
        self.relative_distance_max_m = float(self.get_parameter("relative_distance_max_m").value)
        self.relative_speed_min_mps = float(self.get_parameter("relative_speed_min_mps").value)
        self.hold_count_required = int(self.get_parameter("hold_count").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.publish_repeats = int(self.get_parameter("publish_repeats").value)
        self.fallback_immediate = bool(self.get_parameter("fallback_immediate").value)
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
        self.enable_geometry_cluster_gate = bool(
            self.get_parameter("enable_geometry_cluster_gate").value
        )
        self.geometry_cluster_score_threshold = float(
            self.get_parameter("geometry_cluster_score_threshold").value
        )
        self.geometry_cluster_min_rel_z = float(
            self.get_parameter("geometry_cluster_min_rel_z").value
        )
        self.geometry_cluster_max_rel_z = float(
            self.get_parameter("geometry_cluster_max_rel_z").value
        )
        self.geometry_cluster_a_center = [
            float(v) for v in self.get_parameter("geometry_cluster_a_center").value
        ]
        self.geometry_cluster_a_spread = [
            float(v) for v in self.get_parameter("geometry_cluster_a_spread").value
        ]
        self.geometry_cluster_b_center = [
            float(v) for v in self.get_parameter("geometry_cluster_b_center").value
        ]
        self.geometry_cluster_b_spread = [
            float(v) for v in self.get_parameter("geometry_cluster_b_spread").value
        ]
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

        self.carrier_odom: Optional[Odometry] = None
        self.mini_odom: Optional[Odometry] = None
        self.hold_count = 0
        self.command_sent = False
        self.start_time = self.get_clock().now()
        self.orbit_gate_ready_samples = 0
        self.orbit_gate_tracking_active = not self.require_orbit_completion_before_start
        self.orbit_gate_completed = not self.require_orbit_completion_before_start
        self.orbit_gate_initial_angle: Optional[float] = None
        self.orbit_gate_previous_angle: Optional[float] = None
        self.orbit_gate_accumulated_angle = 0.0
        self.orbit_gate_last_reported_quarter = -1
        self.orbit_gate_completed_elapsed_sec: Optional[float] = 0.0 if self.orbit_gate_completed else None
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
        self.command_latched_pub = self.create_publisher(
            DockingCommand, "/docking/command_latched", latched_qos
        )
        self.create_subscription(Odometry, "/carrier/odom", self._carrier_cb, 10)
        self.create_subscription(Odometry, "/mini/odom", self._mini_cb, 10)
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
        self.timer = self.create_timer(0.05, self._tick)

    def _carrier_cb(self, msg: Odometry) -> None:
        self.carrier_odom = msg

    def _mini_cb(self, msg: Odometry) -> None:
        self.mini_odom = msg

    def _mini_airspeed_cb(self, msg: AirspeedValidated) -> None:
        self.mini_true_airspeed_mps = float(msg.true_airspeed_m_s)

    def _mini_tecs_cb(self, msg: TecsStatus) -> None:
        self.mini_underspeed_ratio = float(msg.underspeed_ratio)

    def _tick(self) -> None:
        if self.command_sent:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        timeout_elapsed = elapsed
        if (
            self.require_orbit_completion_before_start and
            self.orbit_gate_completed_elapsed_sec is not None
        ):
            timeout_elapsed = elapsed - self.orbit_gate_completed_elapsed_sec
        if self.carrier_odom is None or self.mini_odom is None:
            if timeout_elapsed > self.timeout_sec and self.fallback_immediate:
                self.get_logger().warn("Timeout before odom ready, publishing START anyway")
                self._publish_start("timeout_no_odom")
            return

        rel_x = self.mini_odom.pose.pose.position.x - self.carrier_odom.pose.pose.position.x
        rel_y = self.mini_odom.pose.pose.position.y - self.carrier_odom.pose.pose.position.y
        rel_z = self.mini_odom.pose.pose.position.z - self.carrier_odom.pose.pose.position.z
        rel_vx = self.mini_odom.twist.twist.linear.x - self.carrier_odom.twist.twist.linear.x
        rel_vy = self.mini_odom.twist.twist.linear.y - self.carrier_odom.twist.twist.linear.y

        self._update_orbit_gate()
        if not self._mini_energy_healthy():
            self.hold_count = 0
            if timeout_elapsed > self.timeout_sec:
                orbit_ready_for_fallback = (
                    not self.require_orbit_completion_before_start or
                    self.orbit_gate_completed
                )
                if self.fallback_immediate and orbit_ready_for_fallback:
                    self.get_logger().warn(
                        "Timeout waiting for mini-energy health, publishing START anyway"
                    )
                    self._publish_start("timeout_energy_health")
                else:
                    self.get_logger().error("Timeout waiting for mini-energy health")
                    rclpy.shutdown()
                    sys.exit(1)
            return
        rel_distance_xy = math.hypot(rel_x, rel_y)
        rel_speed_xy = math.hypot(rel_vx, rel_vy)
        if rel_distance_xy < 1e-6 or rel_speed_xy < 1e-6:
            alignment = -1.0
            tca_sec = math.inf
        else:
            alignment = (-rel_x * rel_vx - rel_y * rel_vy) / (rel_distance_xy * rel_speed_xy)
            tca_sec = -(rel_x * rel_vx + rel_y * rel_vy) / max(rel_speed_xy ** 2, 1e-6)

        geometry_cluster_score = min(
            self._geometry_cluster_score(
                (rel_x, rel_y, rel_vx, rel_vy),
                self.geometry_cluster_a_center,
                self.geometry_cluster_a_spread,
            ),
            self._geometry_cluster_score(
                (rel_x, rel_y, rel_vx, rel_vy),
                self.geometry_cluster_b_center,
                self.geometry_cluster_b_spread,
            ),
        )
        geometry_cluster_ok = (
            not self.enable_geometry_cluster_gate or
            (
                geometry_cluster_score <= self.geometry_cluster_score_threshold and
                self.geometry_cluster_min_rel_z <= rel_z <= self.geometry_cluster_max_rel_z
            )
        )

        window_ok = (
            rel_z >= self.relative_z_min_m and
            rel_speed_xy >= self.relative_speed_min_mps and
            rel_distance_xy >= self.relative_distance_min_m and
            rel_distance_xy <= self.relative_distance_max_m and
            alignment >= self.alignment_min and
            self.tca_min_sec <= tca_sec <= self.tca_max_sec and
            geometry_cluster_ok
        )

        if window_ok:
            self.hold_count += 1
        else:
            self.hold_count = 0

        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(
                "window_check "
                f"dist_xy={rel_distance_xy:.2f} rel_z={rel_z:.2f} "
                f"speed_xy={rel_speed_xy:.2f} align={alignment:.3f} tca={tca_sec:.2f} "
                f"cluster={geometry_cluster_score:.3f} "
                f"hold={self.hold_count}/{self.hold_count_required}"
            )

        if self.require_orbit_completion_before_start and not self.orbit_gate_completed:
            self.hold_count = 0
            if timeout_elapsed > self.timeout_sec:
                if self.fallback_immediate:
                    self.get_logger().warn(
                        "Timeout before orbit gate completion, publishing START anyway"
                    )
                    self._publish_start("timeout_before_orbit_gate")
                else:
                    self.get_logger().error("Timeout before orbit gate completion")
                    rclpy.shutdown()
                    sys.exit(1)
            return

        if self.hold_count >= self.hold_count_required:
            self.get_logger().info(
                "Window accepted "
                f"dist_xy={rel_distance_xy:.2f} rel_z={rel_z:.2f} "
                f"speed_xy={rel_speed_xy:.2f} align={alignment:.3f} tca={tca_sec:.2f}"
            )
            self._publish_start("window")
            return

        if timeout_elapsed > self.timeout_sec:
            if self.fallback_immediate:
                self.get_logger().warn(
                    "Timeout waiting for window, publishing START anyway"
                )
                self._publish_start("timeout")
            else:
                self.get_logger().error("Timeout waiting for docking window")
                rclpy.shutdown()
                sys.exit(1)

    def _publish_start(self, reason: str) -> None:
        msg = DockingCommand()
        msg.command = "START"
        for _ in range(max(self.publish_repeats, 1)):
            self.command_pub.publish(msg)
            self.command_latched_pub.publish(msg)
        self.command_sent = True
        self.get_logger().info(f"START published reason={reason}")
        self.create_timer(0.5, self._shutdown_once)

    def _shutdown_once(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def _update_orbit_gate(self) -> None:
        if self.orbit_gate_completed or self.mini_odom is None:
            return

        x = float(self.mini_odom.pose.pose.position.x)
        y = float(self.mini_odom.pose.pose.position.y)
        z = float(self.mini_odom.pose.pose.position.z)
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
            self.hold_count = 0
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.orbit_gate_completed_elapsed_sec = elapsed
            self.get_logger().info(
                "Orbit gate completed "
                f"laps={completed_laps:.2f} altitude={z:.2f} radius={radius:.2f}"
            )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _geometry_cluster_score(
        sample: tuple[float, float, float, float],
        center: list[float],
        spread: list[float],
    ) -> float:
        score = 0.0
        for value, target, scale in zip(sample, center, spread):
            score += ((value - target) / max(scale, 1e-6)) ** 2
        return score

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
                    "Window starter mini-energy healthy "
                    f"tas={self.mini_true_airspeed_mps:.2f} "
                    f"underspeed={self.mini_underspeed_ratio:.3f}"
                )
            else:
                self.get_logger().info(
                    "Window starter waiting for mini-energy health "
                    f"tas={self.mini_true_airspeed_mps:.2f} "
                    f"underspeed={self.mini_underspeed_ratio:.3f}"
                )
        return ready


def main() -> None:
    rclpy.init()
    node = DockingWindowStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
