#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from easydocking_msgs.msg import DockingCommand, DockingStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


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

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.command_pub = self.create_publisher(DockingCommand, "/docking/command", 10)
        self.command_latched_pub = self.create_publisher(DockingCommand, "/docking/command_latched", latched_qos)
        self.create_subscription(DockingStatus, "/docking/status", self._status_cb, 10)
        self.timer = self.create_timer(0.1, self._timeout_cb)

    def _status_cb(self, msg: DockingStatus) -> None:
        if msg.is_active:
            self.get_logger().info("Auto START confirmed active")
            raise SystemExit(0)

        if self.sent:
            return
        elapsed = time.time() - self.start_wall
        if elapsed < self.min_wait_sec:
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
            if not self.trigger_candidate_announced:
                candidate = self.trigger_best_candidate
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

        if time.time() - self.start_wall >= self.timeout_sec:
            self.get_logger().info(
                "Auto START timed out without finding a valid geometry window"
            )
            raise SystemExit(1)


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
