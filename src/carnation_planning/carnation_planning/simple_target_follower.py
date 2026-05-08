from dataclasses import dataclass
from tkinter import SEL
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


@dataclass
class JointMapping:
    name: str
    source_axis: str
    zero_world_value: float
    scale: float
    minimum: float
    maximum: float
    speed: float
    initial: float


@dataclass
class MotionPhase:
    name: str
    targets: Dict[str, float]
    pause_seconds: float = 0.0


class SimpleTargetFollower(Node):
    def __init__(self) -> None:
        super().__init__("simple_target_follower")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("target_topic", "/target_pose")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("target_marker_topic", "/target_marker")
        self.declare_parameter("expected_target_frame", "world")
        self.declare_parameter("marker_scale", 0.05)
        self.declare_parameter("phase_tolerance", 0.002)
        self.declare_parameter("release_pause_seconds", 2.0)
        self.declare_parameter("cut_pause_seconds", 2.0)
        self.declare_parameter("first_cycle_start_delay_seconds", 2.0)
        self.declare_parameter("cycle_start_delay_seconds", 1.0)
        self.declare_parameter("verbose_motion_logs", False)
        self.declare_parameter("cycle_busy_topic", "/harvest_cycle_busy")
        self.declare_parameter("cycle_busy_heartbeat_hz", 2.0)

        joint_defaults = {
            "base_move_joint": {
                "source_axis": "x",
                "zero_world_value": 0.0,
                "scale": 1.0,
                "min": 0.0,
                "max": 7.0,
                "speed": 0.60,
                "initial": 0.0,
            },
            "upper_lift_joint": {
                "source_axis": "z",
                "zero_world_value": 0.82,
                "scale": 1.0,
                "min": 0.0,
                "max": 0.23,
                "speed": 0.10,
                "initial": 0.0,
            },
            "upper_extend_joint": {
                "source_axis": "y",
                "zero_world_value": 0.35,
                "scale": -1.0,
                "min": -0.37,
                "max": 0.0,
                "speed": 0.08,
                "initial": 0.0,
            },
            "lower_lift_joint": {
                "source_axis": "z",
                "zero_world_value": 0.82,
                "scale": 1.0,
                "min": 0.0,
                "max": 0.23,
                "speed": 0.10,
                "initial": 0.0,
            },
            "lower_extend_joint": {
                "source_axis": "y",
                "zero_world_value": 0.35,
                "scale": -1.0,
                "min": -0.37,
                "max": 0.0,
                "speed": 0.08,
                "initial": 0.0,
            },
        }

        self.joint_mappings: List[JointMapping] = []
        for joint_name, defaults in joint_defaults.items():
            self.declare_parameter(f"{joint_name}.source_axis", defaults["source_axis"])
            self.declare_parameter(f"{joint_name}.zero_world_value", defaults["zero_world_value"])
            self.declare_parameter(f"{joint_name}.scale", defaults["scale"])
            self.declare_parameter(f"{joint_name}.min", defaults["min"])
            self.declare_parameter(f"{joint_name}.max", defaults["max"])
            self.declare_parameter(f"{joint_name}.speed", defaults["speed"])
            self.declare_parameter(f"{joint_name}.initial", defaults["initial"])

            self.joint_mappings.append(
                JointMapping(
                    name=joint_name,
                    source_axis=str(self.get_parameter(f"{joint_name}.source_axis").value),
                    zero_world_value=float(self.get_parameter(f"{joint_name}.zero_world_value").value),
                    scale=float(self.get_parameter(f"{joint_name}.scale").value),
                    minimum=float(self.get_parameter(f"{joint_name}.min").value),
                    maximum=float(self.get_parameter(f"{joint_name}.max").value),
                    speed=float(self.get_parameter(f"{joint_name}.speed").value),
                    initial=float(self.get_parameter(f"{joint_name}.initial").value),
                )
            )

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.expected_target_frame = str(self.get_parameter("expected_target_frame").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.phase_tolerance = float(self.get_parameter("phase_tolerance").value)
        self.release_pause_seconds = float(self.get_parameter("release_pause_seconds").value)
        self.cut_phase_seconds = float(self.get_parameter("cut_pause_seconds").value)
        self.first_cycle_start_delay_seconds = float(
            self.get_parameter("first_cycle_start_delay_seconds").value
        )
        self.cycle_start_delay_seconds = float(
            self.get_parameter("cycle_start_delay_seconds").value
        )
        self.verbose_motion_logs = bool(self.get_parameter("verbose_motion_logs").value)
        self.cycle_busy_topic = str(self.get_parameter("cycle_busy_topic").value)
        self.cycle_busy_heartbeat_hz = float(
            self.get_parameter("cycle_busy_heartbeat_hz").value
        )

        self.mapping_by_name: Dict[str, JointMapping] = {
            mapping.name: mapping for mapping in self.joint_mappings
        }

        self.current_positions: Dict[str, float] = {
            mapping.name: mapping.initial for mapping in self.joint_mappings
        }
        self.target_positions: Dict[str, float] = dict(self.current_positions)
        self.sequence_active = False
        self.phase_queue: List[MotionPhase] = []
        self.phase_index = -1
        self.phase_hold_until: Optional[float] = None
        self.cycle_start_until: Optional[float] = None
        self.phase_logged_complete = False
        self.harvest_cycle_count = 0
        self.cycle_busy_state = False
        self.last_cycle_busy_heartbeat_sec = 0.0

        target_topic = str(self.get_parameter("target_topic").value)
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        marker_topic = str(self.get_parameter("target_marker_topic").value)

        self.create_subscription(PoseStamped, target_topic, self._target_callback, 10)
        self.joint_state_publisher = self.create_publisher(JointState, joint_state_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, marker_topic, 10)
        self.cycle_busy_publisher = self.create_publisher(Bool, self.cycle_busy_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self._publish_cycle_busy(False)

        self.get_logger().debug(
            "simple_target_follower started. Publish PoseStamped to %s in frame '%s'."
            % (target_topic, self.expected_target_frame)
        )

    def _target_callback(self, message: PoseStamped) -> None:
        frame_id = message.header.frame_id or self.expected_target_frame
        if frame_id != self.expected_target_frame:
            self.get_logger().warning(
                "Ignoring target in frame '%s'; expected '%s'."
                % (frame_id, self.expected_target_frame)
            )
            return

        if self.sequence_active:
            return

        coordinates = {
            "x": float(message.pose.position.x),
            "y": float(message.pose.position.y),
            "z": float(message.pose.position.z),
        }
        joint_targets = self._compute_joint_targets(coordinates)
        self._build_phase_queue(joint_targets)

        self._publish_target_marker(message)

    def _publish_target_marker(self, message: PoseStamped) -> None:
        marker = Marker()
        marker.header = message.header
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = message.pose
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 0.95
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

    def _timer_callback(self) -> None:
        dt = 1.0 / self.publish_rate_hz
        self._publish_cycle_busy_heartbeat()

        if self.sequence_active:
            self._update_phase_progress()

        for mapping in self.joint_mappings:
            current = self.current_positions[mapping.name]
            target = self.target_positions[mapping.name]
            max_step = mapping.speed * dt
            delta = target - current
            if abs(delta) <= max_step:
                self.current_positions[mapping.name] = target
            else:
                direction = 1.0 if delta > 0.0 else -1.0
                self.current_positions[mapping.name] = current + max_step * direction

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [mapping.name for mapping in self.joint_mappings]
        message.position = [self.current_positions[mapping.name] for mapping in self.joint_mappings]
        self.joint_state_publisher.publish(message)

    def _compute_joint_targets(self, coordinates: Dict[str, float]) -> Dict[str, float]:
        joint_targets: Dict[str, float] = {}
        for mapping in self.joint_mappings:
            raw_value = mapping.scale * (coordinates[mapping.source_axis] - mapping.zero_world_value)
            lower = min(mapping.minimum, mapping.maximum)
            upper = max(mapping.minimum, mapping.maximum)
            joint_targets[mapping.name] = min(max(raw_value, lower), upper)
        return joint_targets

    def _home_targets_with_base(self, base_value: float) -> Dict[str, float]:
        targets = {mapping.name: mapping.initial for mapping in self.joint_mappings}
        targets["base_move_joint"] = base_value
        return targets

    def _build_phase_queue(self, joint_targets: Dict[str, float]) -> None:
        base_target = joint_targets["base_move_joint"]
        home_targets = self._home_targets_with_base(base_target)

        base_align = dict(home_targets)
        base_align["base_move_joint"] = base_target

        lift_align = dict(base_align)
        lift_align["upper_lift_joint"] = joint_targets["upper_lift_joint"]
        lift_align["lower_lift_joint"] = joint_targets["lower_lift_joint"]

        upper_hold = dict(lift_align)
        upper_hold["upper_extend_joint"] = joint_targets["upper_extend_joint"]

        lower_cut = dict(upper_hold)
        lower_cut["lower_extend_joint"] = joint_targets["lower_extend_joint"]

        lower_retract = dict(lower_cut)
        lower_retract["lower_extend_joint"] = self.mapping_by_name["lower_extend_joint"].initial

        upper_lift_raise = dict(lower_retract)
        upper_lift_raise["upper_lift_joint"] = max(
            self.mapping_by_name["upper_lift_joint"].minimum,
            self.mapping_by_name["upper_lift_joint"].maximum,
        )

        upper_retract = dict(upper_lift_raise)
        upper_retract["upper_extend_joint"] = self.mapping_by_name["upper_extend_joint"].initial

        lift_reset = dict(home_targets)
        lift_reset["base_move_joint"] = base_target

        self.phase_queue = [
            MotionPhase("base_align", base_align),
            MotionPhase("lift_align", lift_align),
            MotionPhase("upper_hold_extend", upper_hold),
            MotionPhase("lower_cut_extend", lower_cut, pause_seconds=self.cut_phase_seconds),
            MotionPhase("lower_retract", lower_retract),
            MotionPhase("upper_lift_raise", upper_lift_raise),
            MotionPhase("upper_retract", upper_retract, pause_seconds=self.release_pause_seconds),
            MotionPhase("lift_reset", lift_reset),
        ]
        self.phase_index = -1
        self.phase_hold_until = None
        start_delay = (
            self.first_cycle_start_delay_seconds
            if self.harvest_cycle_count == 0
            else self.cycle_start_delay_seconds
        )
        self.harvest_cycle_count += 1

        if start_delay > 0.0:
            self.cycle_start_until = (
                self.get_clock().now().nanoseconds / 1e9 + start_delay
            )
        else:
            self.cycle_start_until = None
        self.phase_logged_complete = False
        self.sequence_active = True
        self._publish_cycle_busy(True)
        if self.cycle_start_until is None:
            self._advance_phase()

    def _advance_phase(self) -> None:
        self.phase_index += 1
        self.phase_hold_until = None
        self.phase_logged_complete = False

        if self.phase_index >= len(self.phase_queue):
            self.sequence_active = False
            self.phase_queue = []
            self._publish_cycle_busy(False)
            if self.verbose_motion_logs:
                self.get_logger().info("Harvest cycle finished. Base remains at the aligned x position.")
            return

        phase = self.phase_queue[self.phase_index]
        self.target_positions = dict(phase.targets)
        if self.verbose_motion_logs:
            target_summary = ", ".join(
                f"{joint}={value:.3f}" for joint, value in phase.targets.items()
            )
            self.get_logger().info(f"Starting phase '{phase.name}': {target_summary}")

    def _update_phase_progress(self) -> None:
        if not self.sequence_active or self.phase_index >= len(self.phase_queue):
            return
        if self.phase_index < 0:
            if self.cycle_start_until is not None:
                now_seconds = self.get_clock().now().nanoseconds / 1e9
                if now_seconds < self.cycle_start_until:
                    return
                self.cycle_start_until = None
            self._advance_phase()
            return

        phase = self.phase_queue[self.phase_index]
        if self._targets_reached(phase.targets):
            if phase.pause_seconds > 0.0:
                if self.phase_hold_until is None:
                    self.phase_hold_until = self.get_clock().now().nanoseconds / 1e9 + phase.pause_seconds
                    if self.verbose_motion_logs:
                        self.get_logger().info(
                            f"Phase '{phase.name}' reached. Holding for {phase.pause_seconds:.1f} s."
                        )
                    return
                if self.get_clock().now().nanoseconds / 1e9 < self.phase_hold_until:
                    return

            if not self.phase_logged_complete:
                if self.verbose_motion_logs:
                    self.get_logger().info(f"Phase '{phase.name}' completed.")
                self.phase_logged_complete = True
            self._advance_phase()

    def _targets_reached(self, targets: Dict[str, float]) -> bool:
        return all(
            abs(self.current_positions[joint_name] - target_value) <= self.phase_tolerance
            for joint_name, target_value in targets.items()
        )

    def _publish_cycle_busy(self, is_busy: bool) -> None:
        self.cycle_busy_state = is_busy
        self.last_cycle_busy_heartbeat_sec = self.get_clock().now().nanoseconds / 1e9
        message = Bool()
        message.data = is_busy
        self.cycle_busy_publisher.publish(message)

    def _publish_cycle_busy_heartbeat(self) -> None:
        if self.cycle_busy_heartbeat_hz <= 0.0:
            return

        now_seconds = self.get_clock().now().nanoseconds / 1e9
        if now_seconds - self.last_cycle_busy_heartbeat_sec < 1.0 / self.cycle_busy_heartbeat_hz:
            return

        self.last_cycle_busy_heartbeat_sec = now_seconds
        message = Bool()
        message.data = self.cycle_busy_state
        self.cycle_busy_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = SimpleTargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
