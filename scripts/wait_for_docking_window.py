#!/usr/bin/env python3

import math
import sys
from typing import Optional

import rclpy
from easydocking_msgs.msg import DockingCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


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

        self.carrier_odom: Optional[Odometry] = None
        self.mini_odom: Optional[Odometry] = None
        self.hold_count = 0
        self.command_sent = False
        self.start_time = self.get_clock().now()

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_pub = self.create_publisher(DockingCommand, "/docking/command", 10)
        self.command_latched_pub = self.create_publisher(
            DockingCommand, "/docking/command_latched", latched_qos
        )
        self.create_subscription(Odometry, "/carrier/odom", self._carrier_cb, 10)
        self.create_subscription(Odometry, "/mini/odom", self._mini_cb, 10)
        self.timer = self.create_timer(0.05, self._tick)

    def _carrier_cb(self, msg: Odometry) -> None:
        self.carrier_odom = msg

    def _mini_cb(self, msg: Odometry) -> None:
        self.mini_odom = msg

    def _tick(self) -> None:
        if self.command_sent:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.carrier_odom is None or self.mini_odom is None:
            if elapsed > self.timeout_sec and self.fallback_immediate:
                self.get_logger().warn("Timeout before odom ready, publishing START anyway")
                self._publish_start("timeout_no_odom")
            return

        rel_x = self.mini_odom.pose.pose.position.x - self.carrier_odom.pose.pose.position.x
        rel_y = self.mini_odom.pose.pose.position.y - self.carrier_odom.pose.pose.position.y
        rel_z = self.mini_odom.pose.pose.position.z - self.carrier_odom.pose.pose.position.z
        rel_vx = self.mini_odom.twist.twist.linear.x - self.carrier_odom.twist.twist.linear.x
        rel_vy = self.mini_odom.twist.twist.linear.y - self.carrier_odom.twist.twist.linear.y

        rel_distance_xy = math.hypot(rel_x, rel_y)
        rel_speed_xy = math.hypot(rel_vx, rel_vy)
        if rel_distance_xy < 1e-6 or rel_speed_xy < 1e-6:
            alignment = -1.0
            tca_sec = math.inf
        else:
            alignment = (-rel_x * rel_vx - rel_y * rel_vy) / (rel_distance_xy * rel_speed_xy)
            tca_sec = -(rel_x * rel_vx + rel_y * rel_vy) / max(rel_speed_xy ** 2, 1e-6)

        window_ok = (
            rel_z >= self.relative_z_min_m and
            rel_speed_xy >= self.relative_speed_min_mps and
            rel_distance_xy >= self.relative_distance_min_m and
            rel_distance_xy <= self.relative_distance_max_m and
            alignment >= self.alignment_min and
            self.tca_min_sec <= tca_sec <= self.tca_max_sec
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
                f"hold={self.hold_count}/{self.hold_count_required}"
            )

        if self.hold_count >= self.hold_count_required:
            self.get_logger().info(
                "Window accepted "
                f"dist_xy={rel_distance_xy:.2f} rel_z={rel_z:.2f} "
                f"speed_xy={rel_speed_xy:.2f} align={alignment:.3f} tca={tca_sec:.2f}"
            )
            self._publish_start("window")
            return

        if elapsed > self.timeout_sec:
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
