#!/usr/bin/env python3

import csv
import os
import time
from pathlib import Path

import rclpy
from easydocking_msgs.msg import DockingStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ExperimentLogger(Node):
    def __init__(self) -> None:
        super().__init__("experiment_logger")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("duration_sec", 30.0)
        self.declare_parameter("post_completed_settle_sec", 0.6)
        self.declare_parameter("post_completed_rows", 12)

        output_dir = self.get_parameter("output_dir").value
        if not output_dir:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_dir = f"/home/hw/easydocking/results/{timestamp}"
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.post_completed_settle_sec = float(
            self.get_parameter("post_completed_settle_sec").value
        )
        self.post_completed_rows = int(self.get_parameter("post_completed_rows").value)
        self.start_wall = time.time()
        self.start_ros_time = None
        self.latest_phase = "IDLE"
        self.rows_written = 0
        self.completed_wall_time = None
        self.completed_rows_written = 0

        self.csv_path = self.output_dir / "docking_log.csv"
        self.metadata_path = self.output_dir / "metadata.txt"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "t",
            "phase",
            "relative_distance",
            "rel_x",
            "rel_y",
            "rel_z",
            "rel_vx",
            "rel_vy",
            "rel_vz",
            "carrier_x",
            "carrier_y",
            "carrier_z",
            "mini_x",
            "mini_y",
            "mini_z",
            "carrier_sp_x",
            "carrier_sp_y",
            "carrier_sp_z",
            "mini_target_x",
            "mini_target_y",
            "mini_target_z",
            "measured_rel_x",
            "measured_rel_y",
            "measured_rel_z",
            "measured_rel_distance",
            "measured_rel_vx",
            "measured_rel_vy",
            "measured_rel_vz",
            "measured_rel_speed",
        ])

        self.carrier_position = [0.0, 0.0, 0.0]
        self.mini_position = [0.0, 0.0, 0.0]
        self.carrier_velocity = [0.0, 0.0, 0.0]
        self.mini_velocity = [0.0, 0.0, 0.0]
        self.carrier_odom_received = False
        self.mini_odom_received = False
        self.carrier_setpoint = [0.0, 0.0, 0.0]
        self.mini_target = [0.0, 0.0, 0.0]
        self.latest_status = None

        self.create_subscription(DockingStatus, "/docking/status", self._status_cb, 10)
        self.create_subscription(Odometry, "/carrier/odom", self._carrier_cb, 10)
        self.create_subscription(Odometry, "/mini/odom", self._mini_cb, 10)
        self.create_subscription(PoseStamped, "/carrier/setpoint/pose", self._carrier_sp_cb, 10)
        self.create_subscription(PoseStamped, "/mini/glide_target", self._mini_target_cb, 10)
        self.timer = self.create_timer(0.05, self._flush_row)

    def _carrier_cb(self, msg: Odometry) -> None:
        self.carrier_odom_received = True
        self.carrier_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.carrier_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

    def _mini_cb(self, msg: Odometry) -> None:
        self.mini_odom_received = True
        self.mini_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.mini_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

    def _carrier_sp_cb(self, msg: PoseStamped) -> None:
        self.carrier_setpoint = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def _mini_target_cb(self, msg: PoseStamped) -> None:
        self.mini_target = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def _status_cb(self, msg: DockingStatus) -> None:
        self.latest_status = msg
        self.latest_phase = msg.phase
        if self.start_ros_time is None:
            self.start_ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _flush_row(self) -> None:
        if (
            self.latest_status is None or
            not self.carrier_odom_received or
            not self.mini_odom_received
        ):
            self._check_timeout()
            return

        ros_time = (
            self.latest_status.header.stamp.sec +
            self.latest_status.header.stamp.nanosec * 1e-9
        )
        t = 0.0 if self.start_ros_time is None else ros_time - self.start_ros_time
        measured_rel = [
            self.mini_position[0] - self.carrier_position[0],
            self.mini_position[1] - self.carrier_position[1],
            self.mini_position[2] - self.carrier_position[2],
        ]
        measured_rel_distance = sum(value * value for value in measured_rel) ** 0.5
        measured_rel_velocity = [
            self.mini_velocity[0] - self.carrier_velocity[0],
            self.mini_velocity[1] - self.carrier_velocity[1],
            self.mini_velocity[2] - self.carrier_velocity[2],
        ]
        measured_rel_speed = sum(value * value for value in measured_rel_velocity) ** 0.5
        self.writer.writerow([
            f"{t:.3f}",
            self.latest_status.phase,
            f"{self.latest_status.relative_distance:.6f}",
            f"{self.latest_status.relative_position.x:.6f}",
            f"{self.latest_status.relative_position.y:.6f}",
            f"{self.latest_status.relative_position.z:.6f}",
            f"{self.latest_status.relative_velocity.x:.6f}",
            f"{self.latest_status.relative_velocity.y:.6f}",
            f"{self.latest_status.relative_velocity.z:.6f}",
            f"{self.carrier_position[0]:.6f}",
            f"{self.carrier_position[1]:.6f}",
            f"{self.carrier_position[2]:.6f}",
            f"{self.mini_position[0]:.6f}",
            f"{self.mini_position[1]:.6f}",
            f"{self.mini_position[2]:.6f}",
            f"{self.carrier_setpoint[0]:.6f}",
            f"{self.carrier_setpoint[1]:.6f}",
            f"{self.carrier_setpoint[2]:.6f}",
            f"{self.mini_target[0]:.6f}",
            f"{self.mini_target[1]:.6f}",
            f"{self.mini_target[2]:.6f}",
            f"{measured_rel[0]:.6f}",
            f"{measured_rel[1]:.6f}",
            f"{measured_rel[2]:.6f}",
            f"{measured_rel_distance:.6f}",
            f"{measured_rel_velocity[0]:.6f}",
            f"{measured_rel_velocity[1]:.6f}",
            f"{measured_rel_velocity[2]:.6f}",
            f"{measured_rel_speed:.6f}",
        ])
        self.rows_written += 1

        if self.latest_status.phase == "COMPLETED":
            if self.completed_wall_time is None:
                self.completed_wall_time = time.time()
                self.completed_rows_written = 0
            self.completed_rows_written += 1
            elapsed_after_completed = time.time() - self.completed_wall_time
            if (
                elapsed_after_completed >= self.post_completed_settle_sec and
                self.completed_rows_written >= self.post_completed_rows
            ):
                self._finalize("completed")
                return

        self._check_timeout()

    def _check_timeout(self) -> None:
        if time.time() - self.start_wall >= self.duration_sec:
            self._finalize("timeout")

    def _finalize(self, reason: str) -> None:
        if self.csv_file.closed:
            return
        self.csv_file.flush()
        self.csv_file.close()
        with self.metadata_path.open("w", encoding="utf-8") as file:
            file.write(f"reason={reason}\n")
            file.write(f"rows={self.rows_written}\n")
            file.write(f"phase={self.latest_phase}\n")
            file.write(f"completed_rows={self.completed_rows_written}\n")
            file.write(
                f"post_completed_settle_sec={self.post_completed_settle_sec:.3f}\n"
            )
            file.write(f"post_completed_rows={self.post_completed_rows}\n")
            file.write(f"csv={self.csv_path}\n")
        self.get_logger().info(f"Experiment finished: {reason}, rows={self.rows_written}")
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if not node.csv_file.closed:
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
