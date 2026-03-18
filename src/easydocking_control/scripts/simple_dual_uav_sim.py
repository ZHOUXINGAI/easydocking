#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from easydocking_msgs.msg import DockingStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SimpleDualUavSim(Node):
    def __init__(self) -> None:
        super().__init__("simple_dual_uav_sim")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("carrier_initial", [0.0, 0.0, 1.5])
        self.declare_parameter("mini_initial", [10.0, 0.0, 0.0])
        self.declare_parameter("mini_mode", "fixed_wing_orbit")
        self.declare_parameter("mini_takeoff_altitude", 6.0)
        self.declare_parameter("mini_orbit_center", [4.0, 0.0, 6.0])
        self.declare_parameter("mini_orbit_radius", 8.0)
        self.declare_parameter("mini_orbit_speed", 3.5)
        self.declare_parameter("mini_tracking_speed", 3.0)
        self.declare_parameter("mini_docking_speed", 2.6)
        self.declare_parameter("mini_capture_speed", 2.2)
        self.declare_parameter("mini_slowdown_start_distance", 8.0)
        self.declare_parameter("mini_slowdown_finish_distance", 2.0)
        self.declare_parameter("terminal_straight_trigger_distance", 1.35)
        self.declare_parameter("mini_max_accel", 1.2)
        self.declare_parameter("carrier_max_accel", 1.8)
        self.declare_parameter("carrier_max_speed_xy", 4.5)
        self.declare_parameter("carrier_max_speed_z", 1.5)
        self.declare_parameter("attach_relative_position", [0.0, 0.0, 0.8])
        self.declare_parameter("attach_distance", 0.24)
        self.declare_parameter("attach_speed_threshold", 0.8)
        self.declare_parameter("soft_attach_distance", 0.34)
        self.declare_parameter("soft_attach_xy_tolerance", 0.16)
        self.declare_parameter("soft_attach_z_min", 0.12)
        self.declare_parameter("soft_attach_z_max", 0.32)
        self.declare_parameter("soft_attach_vx_threshold", 0.9)
        self.declare_parameter("soft_attach_vy_threshold", 0.9)
        self.declare_parameter("soft_attach_vz_threshold", 0.35)

        self.world_frame = self.get_parameter("world_frame").value
        self.dt = float(self.get_parameter("dt").value)
        self.carrier_pos = list(self.get_parameter("carrier_initial").value)
        self.mini_pos = list(self.get_parameter("mini_initial").value)
        self.mini_mode = self.get_parameter("mini_mode").value
        self.mini_takeoff_altitude = float(self.get_parameter("mini_takeoff_altitude").value)
        self.mini_orbit_center = list(self.get_parameter("mini_orbit_center").value)
        self.mini_orbit_radius = float(self.get_parameter("mini_orbit_radius").value)
        self.mini_orbit_speed = float(self.get_parameter("mini_orbit_speed").value)
        self.mini_tracking_speed = float(self.get_parameter("mini_tracking_speed").value)
        self.mini_docking_speed = float(self.get_parameter("mini_docking_speed").value)
        self.mini_capture_speed = float(self.get_parameter("mini_capture_speed").value)
        self.mini_slowdown_start_distance = float(
            self.get_parameter("mini_slowdown_start_distance").value
        )
        self.mini_slowdown_finish_distance = float(
            self.get_parameter("mini_slowdown_finish_distance").value
        )
        self.terminal_straight_trigger_distance = float(
            self.get_parameter("terminal_straight_trigger_distance").value
        )
        self.mini_max_accel = float(self.get_parameter("mini_max_accel").value)
        self.carrier_max_accel = float(self.get_parameter("carrier_max_accel").value)
        self.carrier_max_speed_xy = float(self.get_parameter("carrier_max_speed_xy").value)
        self.carrier_max_speed_z = float(self.get_parameter("carrier_max_speed_z").value)
        self.attach_relative_position = list(self.get_parameter("attach_relative_position").value)
        self.attach_distance = float(self.get_parameter("attach_distance").value)
        self.attach_speed_threshold = float(self.get_parameter("attach_speed_threshold").value)
        self.soft_attach_distance = float(self.get_parameter("soft_attach_distance").value)
        self.soft_attach_xy_tolerance = float(self.get_parameter("soft_attach_xy_tolerance").value)
        self.soft_attach_z_min = float(self.get_parameter("soft_attach_z_min").value)
        self.soft_attach_z_max = float(self.get_parameter("soft_attach_z_max").value)
        self.soft_attach_vx_threshold = float(self.get_parameter("soft_attach_vx_threshold").value)
        self.soft_attach_vy_threshold = float(self.get_parameter("soft_attach_vy_threshold").value)
        self.soft_attach_vz_threshold = float(self.get_parameter("soft_attach_vz_threshold").value)
        self.carrier_vel = [0.0, 0.0, 0.0]
        self.mini_vel = [0.0, 0.0, 0.0]
        self.carrier_yaw = 0.0
        self.mini_yaw = 0.0
        self.sim_time = 0.0
        self.carrier_pose_sp = None
        self.mini_pose_sp = None
        self.carrier_vel_sp = None
        self.mini_vel_sp = None
        self.latest_status: Optional[DockingStatus] = None
        self.docking_phase = "IDLE"
        self.docking_active = False
        self.relative_distance = math.inf
        self.relative_speed = math.inf
        self.relative_position = [math.inf, math.inf, math.inf]
        self.relative_velocity = [math.inf, math.inf, math.inf]
        self.mini_attached = False
        self.terminal_straight_active = False
        self.terminal_direction = [1.0, 0.0]
        self.last_phase = "IDLE"

        self.create_subscription(PoseStamped, "/carrier/setpoint/pose", self._carrier_pose_cb, 10)
        self.create_subscription(PoseStamped, "/mini/setpoint/pose", self._mini_pose_cb, 10)
        self.create_subscription(TwistStamped, "/carrier/setpoint/velocity", self._carrier_vel_cb, 10)
        self.create_subscription(TwistStamped, "/mini/setpoint/velocity", self._mini_vel_cb, 10)
        self.create_subscription(DockingStatus, "/docking/status", self._status_cb, 10)

        self.carrier_pub = self.create_publisher(Odometry, "/carrier/odom", 10)
        self.mini_pub = self.create_publisher(Odometry, "/mini/odom", 10)
        self.timer = self.create_timer(self.dt, self._step)

    def _carrier_pose_cb(self, msg: PoseStamped) -> None:
        self.carrier_pose_sp = msg

    def _mini_pose_cb(self, msg: PoseStamped) -> None:
        self.mini_pose_sp = msg

    def _carrier_vel_cb(self, msg: TwistStamped) -> None:
        self.carrier_vel_sp = msg

    def _mini_vel_cb(self, msg: TwistStamped) -> None:
        self.mini_vel_sp = msg

    def _status_cb(self, msg: DockingStatus) -> None:
        self.latest_status = msg
        self.docking_phase = msg.phase.upper()
        self.docking_active = bool(msg.is_active)
        self.relative_distance = float(msg.relative_distance)
        self.relative_position = [
            float(msg.relative_position.x),
            float(msg.relative_position.y),
            float(msg.relative_position.z),
        ]
        self.relative_velocity = [
            float(msg.relative_velocity.x),
            float(msg.relative_velocity.y),
            float(msg.relative_velocity.z),
        ]
        self.relative_speed = math.sqrt(
            msg.relative_velocity.x ** 2 +
            msg.relative_velocity.y ** 2 +
            msg.relative_velocity.z ** 2
        )
        if self.docking_phase not in {"DOCKING", "COMPLETED"} and self.last_phase == "DOCKING":
            self.terminal_straight_active = False
        self.last_phase = self.docking_phase

    def _step(self) -> None:
        self.sim_time += self.dt
        self._update_carrier()
        self._update_mini()
        self.carrier_pub.publish(self._make_odom("carrier", self.carrier_pos, self.carrier_vel, self.carrier_yaw))
        self.mini_pub.publish(self._make_odom("mini", self.mini_pos, self.mini_vel, self.mini_yaw))

    def _update_carrier(self) -> None:
        pos = self.carrier_pos
        vel = self.carrier_vel
        pose_sp = self.carrier_pose_sp
        vel_sp = self.carrier_vel_sp
        commanded_vel = [0.0, 0.0, 0.0]
        if pose_sp is not None:
            commanded_vel = [
                max(min((pose_sp.pose.position.x - pos[0]) * 1.2, self.carrier_max_speed_xy), -self.carrier_max_speed_xy),
                max(min((pose_sp.pose.position.y - pos[1]) * 1.2, self.carrier_max_speed_xy), -self.carrier_max_speed_xy),
                max(min((pose_sp.pose.position.z - pos[2]) * 1.0, self.carrier_max_speed_z), -self.carrier_max_speed_z),
            ]

        if vel_sp is not None:
            commanded_vel = [
                commanded_vel[0] + vel_sp.twist.linear.x,
                commanded_vel[1] + vel_sp.twist.linear.y,
                commanded_vel[2] + vel_sp.twist.linear.z,
            ]

        commanded_vel[0] = max(min(commanded_vel[0], self.carrier_max_speed_xy), -self.carrier_max_speed_xy)
        commanded_vel[1] = max(min(commanded_vel[1], self.carrier_max_speed_xy), -self.carrier_max_speed_xy)
        commanded_vel[2] = max(min(commanded_vel[2], self.carrier_max_speed_z), -self.carrier_max_speed_z)

        for i in range(3):
            dv = commanded_vel[i] - vel[i]
            dv = max(min(dv, self.carrier_max_accel * self.dt), -self.carrier_max_accel * self.dt)
            vel[i] += dv
            pos[i] += vel[i] * self.dt

        if abs(vel[0]) + abs(vel[1]) > 1e-3:
            self.carrier_yaw = math.atan2(vel[1], vel[0])

    def _update_mini(self) -> None:
        if self.mini_attached:
            self._update_attached_mini()
            return
        if self.mini_mode == "fixed_wing_orbit":
            self._update_fixed_wing_orbit()
            return

        pos = self.mini_pos
        vel = self.mini_vel
        pose_sp = self.mini_pose_sp
        vel_sp = self.mini_vel_sp
        commanded_vel = [0.0, 0.0, 0.0]
        if pose_sp is not None:
            commanded_vel = [
                max(min((pose_sp.pose.position.x - pos[0]) * 1.5, 2.0), -2.0),
                max(min((pose_sp.pose.position.y - pos[1]) * 1.5, 2.0), -2.0),
                max(min((pose_sp.pose.position.z - pos[2]) * 1.5, 1.5), -1.5),
            ]

        if vel_sp is not None:
            commanded_vel = [
                commanded_vel[0] + vel_sp.twist.linear.x,
                commanded_vel[1] + vel_sp.twist.linear.y,
                commanded_vel[2] + vel_sp.twist.linear.z,
            ]

        for i in range(3):
            vel[i] = 0.8 * vel[i] + 0.2 * commanded_vel[i]
            pos[i] += vel[i] * self.dt

        if abs(vel[0]) + abs(vel[1]) > 1e-3:
            self.mini_yaw = math.atan2(vel[1], vel[0])

    def _update_fixed_wing_orbit(self) -> None:
        cx, cy, cz = self.mini_orbit_center
        pos = self.mini_pos
        vel = self.mini_vel

        if self._should_attach():
            self.mini_attached = True
            self._update_attached_mini()
            return

        target_speed = self._compute_mini_target_speed()
        dx = pos[0] - cx
        dy = pos[1] - cy
        radius = max(math.hypot(dx, dy), 1e-3)
        radial_x = dx / radius
        radial_y = dy / radius
        tangential_x = -radial_y
        tangential_y = radial_x
        radial_error = radius - self.mini_orbit_radius
        climb_rate_limit = 1.2 if pos[2] < self.mini_takeoff_altitude else 0.8

        if (
            not self.terminal_straight_active and
            self.docking_phase in {"DOCKING", "COMPLETED"} and
            math.isfinite(self.relative_distance) and
            self.relative_distance <= self.terminal_straight_trigger_distance
        ):
            speed_xy = math.hypot(self.mini_vel[0], self.mini_vel[1])
            if speed_xy > 1e-6:
                self.terminal_direction = [self.mini_vel[0] / speed_xy, self.mini_vel[1] / speed_xy]
            self.terminal_straight_active = True

        if self.terminal_straight_active and self.docking_phase in {"DOCKING", "COMPLETED"}:
            commanded_vel = [
                self.terminal_direction[0] * target_speed,
                self.terminal_direction[1] * target_speed,
                max(min((cz - pos[2]) * 0.8, 0.4), -0.4),
            ]
        else:
            commanded_vel = [
                tangential_x * target_speed - radial_x * radial_error * 0.6,
                tangential_y * target_speed - radial_y * radial_error * 0.6,
                max(min((cz - pos[2]) * 0.8, climb_rate_limit), -0.8),
            ]

        desired_xy_speed = math.hypot(commanded_vel[0], commanded_vel[1])
        speed_limit = max(target_speed, self.mini_capture_speed)
        if desired_xy_speed > speed_limit and desired_xy_speed > 1e-6:
            scale = speed_limit / desired_xy_speed
            commanded_vel[0] *= scale
            commanded_vel[1] *= scale

        for i in range(3):
            dv = commanded_vel[i] - vel[i]
            dv = max(min(dv, self.mini_max_accel * self.dt), -self.mini_max_accel * self.dt)
            vel[i] += dv
            pos[i] += vel[i] * self.dt

        xy_speed = math.hypot(vel[0], vel[1])
        if xy_speed > 0.5:
            self.mini_yaw = math.atan2(vel[1], vel[0])

    def _compute_mini_target_speed(self) -> float:
        target_speed = self.mini_orbit_speed
        if not self.docking_active or not math.isfinite(self.relative_distance):
            return target_speed

        terminal_error = self.relative_distance
        if all(math.isfinite(value) for value in self.relative_position):
            dz = self.relative_position[2] - self.attach_relative_position[2]
            terminal_error = math.sqrt(
                self.relative_position[0] ** 2 +
                self.relative_position[1] ** 2 +
                dz ** 2
            )

        if self.docking_phase in {"TRACKING", "DOCKING", "COMPLETED"}:
            target_speed = self.mini_tracking_speed

        if self.relative_distance <= self.mini_slowdown_start_distance:
            span = max(
                self.mini_slowdown_start_distance - self.mini_slowdown_finish_distance,
                1e-3,
            )
            blend = 1.0 - max(
                0.0,
                min(
                    (self.relative_distance - self.mini_slowdown_finish_distance) / span,
                    1.0,
                ),
            )
            target_speed += (self.mini_docking_speed - target_speed) * blend

        if self.docking_phase in {"DOCKING", "COMPLETED"}:
            span = max(self.mini_slowdown_finish_distance, 1e-3)
            capture_blend = 1.0 - max(
                0.0,
                min(self.relative_distance / span, 1.0),
            )
            alignment_blend = 1.0 - max(
                0.0,
                min((abs(self.relative_position[1]) - 0.20) / 0.65, 1.0),
            )
            terminal_capture_blend = 1.0 - max(
                0.0,
                min(terminal_error / span, 1.0),
            )
            target_speed += (
                self.mini_capture_speed - target_speed
            ) * capture_blend * alignment_blend * terminal_capture_blend

        return max(self.mini_capture_speed, target_speed)

    def _should_attach(self) -> bool:
        if self.docking_phase == "COMPLETED":
            return True
        return (
            self.docking_phase == "DOCKING" and
            math.isfinite(self.relative_distance) and
            (
                (
                    math.isfinite(self.relative_speed) and
                    self.relative_distance <= self.attach_distance and
                    self.relative_speed <= self.attach_speed_threshold
                ) or (
                    self.relative_distance <= self.soft_attach_distance and
                    abs(self.relative_position[0]) <= self.soft_attach_xy_tolerance and
                    abs(self.relative_position[1]) <= self.soft_attach_xy_tolerance and
                    self.soft_attach_z_min <= self.relative_position[2] <= self.soft_attach_z_max and
                    abs(self.relative_velocity[0]) <= self.soft_attach_vx_threshold and
                    abs(self.relative_velocity[1]) <= self.soft_attach_vy_threshold and
                    abs(self.relative_velocity[2]) <= self.soft_attach_vz_threshold
                )
            )
        )

    def _update_attached_mini(self) -> None:
        target_pos = [
            self.carrier_pos[0] + self.attach_relative_position[0],
            self.carrier_pos[1] + self.attach_relative_position[1],
            self.carrier_pos[2] + self.attach_relative_position[2],
        ]
        self.mini_vel = list(self.carrier_vel)
        self.mini_pos = target_pos
        self.mini_yaw = self.carrier_yaw

    def _make_odom(self, name: str, pos: list[float], vel: list[float], yaw: float) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = f"{name}/base_link"
        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        msg.twist.twist.linear.x = vel[0]
        msg.twist.twist.linear.y = vel[1]
        msg.twist.twist.linear.z = vel[2]
        return msg


def main() -> None:
    rclpy.init()
    node = SimpleDualUavSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
