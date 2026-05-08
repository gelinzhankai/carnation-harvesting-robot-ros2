from dataclasses import dataclass
from typing import Optional

from carnation_interfaces.msg import FlowerDetection
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


@dataclass
class DetectionSample:
    stamp_sec: float
    message: FlowerDetection


class StereoPixelToWorldNode(Node):
    def __init__(self) -> None:
        super().__init__("stereo_pixel_to_world_node")

        self.declare_parameter("left_input_topic", "/carnation/left/pixel_center")
        self.declare_parameter("right_input_topic", "/carnation/right/pixel_center")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("camera_point_topic", "/carnation/stereo_target_point_camera")
        self.declare_parameter("world_point_topic", "/carnation/stereo_target_point_world")
        self.declare_parameter("marker_topic", "/carnation/stereo_target_marker_world")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("camera_frame", "left_camera_optical_frame")

        self.declare_parameter("fx", 1000.0)
        self.declare_parameter("fy", 1000.0)
        self.declare_parameter("cx", 640.0)
        self.declare_parameter("cy", 360.0)
        self.declare_parameter("baseline_m", 0.06)
        self.declare_parameter("max_sync_delta_sec", 0.15)
        self.declare_parameter("min_disparity_px", 1.0)

        self.declare_parameter("world_translation_xyz", [0.0, 0.11, 1.20])
        self.declare_parameter(
            "world_rotation_matrix",
            [1.0, 0.0, 0.0,
             0.0, 0.0, 1.0,
             0.0, -1.0, 0.0],
        )
        self.declare_parameter("target_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("clamp_to_workspace", False)
        self.declare_parameter("workspace_min_xyz", [-3.5, 0.39, 0.82])
        self.declare_parameter("workspace_max_xyz", [3.5, 0.73, 1.05])
        self.declare_parameter("marker_scale", 0.05)
        self.declare_parameter("log_target_coordinates", True)

        self.left_input_topic = str(self.get_parameter("left_input_topic").value)
        self.right_input_topic = str(self.get_parameter("right_input_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.camera_point_topic = str(self.get_parameter("camera_point_topic").value)
        self.world_point_topic = str(self.get_parameter("world_point_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)

        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.baseline_m = float(self.get_parameter("baseline_m").value)
        self.max_sync_delta_sec = float(self.get_parameter("max_sync_delta_sec").value)
        self.min_disparity_px = float(self.get_parameter("min_disparity_px").value)

        self.world_translation = np.array(
            list(self.get_parameter("world_translation_xyz").value),
            dtype=float,
        ).reshape(3)
        self.world_rotation = np.array(
            list(self.get_parameter("world_rotation_matrix").value),
            dtype=float,
        ).reshape(3, 3)
        self.target_offset = np.array(
            list(self.get_parameter("target_offset_xyz").value),
            dtype=float,
        ).reshape(3)
        self.clamp_to_workspace = bool(self.get_parameter("clamp_to_workspace").value)
        self.workspace_min = np.array(
            list(self.get_parameter("workspace_min_xyz").value),
            dtype=float,
        ).reshape(3)
        self.workspace_max = np.array(
            list(self.get_parameter("workspace_max_xyz").value),
            dtype=float,
        ).reshape(3)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.log_target_coordinates = bool(self.get_parameter("log_target_coordinates").value)

        self.pose_publisher = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.camera_point_publisher = self.create_publisher(PointStamped, self.camera_point_topic, 10)
        self.world_point_publisher = self.create_publisher(PointStamped, self.world_point_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, self.marker_topic, 10)

        self.left_detection: Optional[DetectionSample] = None
        self.right_detection: Optional[DetectionSample] = None

        self.create_subscription(FlowerDetection, self.left_input_topic, self._left_callback, 10)
        self.create_subscription(FlowerDetection, self.right_input_topic, self._right_callback, 10)

        self.get_logger().info(
            "Stereo pixel-to-world node ready. "
            f"left_input_topic={self.left_input_topic}, right_input_topic={self.right_input_topic}, "
            f"baseline_m={self.baseline_m:.3f}"
        )

    def _left_callback(self, message: FlowerDetection) -> None:
        self.left_detection = DetectionSample(self._stamp_to_sec(message), message)
        self._try_process_pair()

    def _right_callback(self, message: FlowerDetection) -> None:
        self.right_detection = DetectionSample(self._stamp_to_sec(message), message)
        self._try_process_pair()

    def _try_process_pair(self) -> None:
        if self.left_detection is None or self.right_detection is None:
            return

        # 将时间上最接近的一对左右检测结果视为同一时刻的双目观测。
        dt = abs(self.left_detection.stamp_sec - self.right_detection.stamp_sec)
        if dt > self.max_sync_delta_sec:
            if self.left_detection.stamp_sec > self.right_detection.stamp_sec:
                self.right_detection = None
            else:
                self.left_detection = None
            return

        left_msg = self.left_detection.message
        right_msg = self.right_detection.message

        # 视差d的计算 对应式4-1
        disparity = float(left_msg.center_u) - float(right_msg.center_u)
        if disparity <= self.min_disparity_px:
            self.get_logger().warning(
                f"Stereo disparity too small: d={disparity:.3f}px. Skipping this pair."
            )
            self.left_detection = None
            self.right_detection = None
            return

        # 反解出左相机坐标系下的三维坐标 对应式4-5  4-8  4-9
        z_camera = self.fx * self.baseline_m / disparity
        x_camera = (float(left_msg.center_u) - self.cx) * z_camera / self.fx
        y_camera = (float(left_msg.center_v) - self.cy) * z_camera / self.fy

        camera_point = np.array([x_camera, y_camera, z_camera], dtype=float)

        # 将相机坐标转换到世界坐标：对应式4-13  4-14
        raw_world_point = self.world_rotation @ camera_point + self.world_translation
        shaped_world_point = raw_world_point + self.target_offset

        # 可选的工作空间裁剪：保证最终发布的目标点落在机器人设定的可达范围内，
        if self.clamp_to_workspace:
            shaped_world_point = np.minimum(
                np.maximum(shaped_world_point, self.workspace_min),
                self.workspace_max,
            )

        self._publish_outputs(camera_point, raw_world_point, shaped_world_point, left_msg)

        self.left_detection = None
        self.right_detection = None

    def _publish_outputs(
        self,
        camera_point: np.ndarray,
        raw_world_point: np.ndarray,
        shaped_world_point: np.ndarray,
        left_msg: FlowerDetection,
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        camera_msg = PointStamped()
        camera_msg.header.stamp = stamp
        camera_msg.header.frame_id = self.camera_frame
        camera_msg.point.x = float(camera_point[0])
        camera_msg.point.y = float(camera_point[1])
        camera_msg.point.z = float(camera_point[2])
        self.camera_point_publisher.publish(camera_msg)

        world_msg = PointStamped()
        world_msg.header.stamp = stamp
        world_msg.header.frame_id = self.target_frame
        world_msg.point.x = float(shaped_world_point[0])
        world_msg.point.y = float(shaped_world_point[1])
        world_msg.point.z = float(shaped_world_point[2])
        self.world_point_publisher.publish(world_msg)

        pose_msg = PoseStamped()
        pose_msg.header = world_msg.header
        pose_msg.pose.position = world_msg.point
        pose_msg.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose_msg)

        marker = Marker()
        marker.header = world_msg.header
        marker.ns = "stereo_localized_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = world_msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

        if self.log_target_coordinates:
            disparity = float(self.fx * self.baseline_m / camera_point[2]) if camera_point[2] != 0.0 else 0.0
            self.get_logger().info(
                "stereo_disparity=%.3f target_camera=(%.3f, %.3f, %.3f) "
                "target_world_raw=(%.3f, %.3f, %.3f) "
                "target_world_shaped=(%.3f, %.3f, %.3f)"
                % (
                    disparity,
                    camera_point[0],
                    camera_point[1],
                    camera_point[2],
                    raw_world_point[0],
                    raw_world_point[1],
                    raw_world_point[2],
                    shaped_world_point[0],
                    shaped_world_point[1],
                    shaped_world_point[2],
                )
            )

    @staticmethod
    def _stamp_to_sec(message: FlowerDetection) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def main() -> None:
    rclpy.init()
    node = StereoPixelToWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
