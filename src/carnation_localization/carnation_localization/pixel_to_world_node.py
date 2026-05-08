import math

from carnation_interfaces.msg import FlowerDetection
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


class PixelToWorldNode(Node):
    def __init__(self) -> None:
        super().__init__("pixel_to_world_node")

        self.declare_parameter("input_topic", "/carnation/pixel_center")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("camera_point_topic", "/carnation/target_point_camera")
        self.declare_parameter("world_point_topic", "/carnation/target_point_world")
        self.declare_parameter("marker_topic", "/carnation/target_marker_world")
        self.declare_parameter("target_rejected_topic", "/carnation/target_rejected")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("camera_frame", "camera_optical_frame")

        self.declare_parameter("fx", 1000.0)
        self.declare_parameter("fy", 1000.0)
        self.declare_parameter("cx", 640.0)
        self.declare_parameter("cy", 360.0)

        self.declare_parameter("depth_mode", "fixed_depth")
        self.declare_parameter("fixed_depth_m", 0.52)
        self.declare_parameter("target_real_height_m", 0.067)
        self.declare_parameter("reference_bbox_height_px", 150.00)
        self.declare_parameter("bbox_height_depth_weight", 0.0015)
        self.declare_parameter("clamp_near_depth_to_min", True)
        self.declare_parameter("camera_depth_min_m", 0.28)
        self.declare_parameter("camera_depth_max_m", 0.58)
        self.declare_parameter("reject_too_near_targets", False)
        self.declare_parameter("reject_out_of_depth_range", True)
        self.declare_parameter("use_tf_extrinsics", True)
        self.declare_parameter("camera_translation_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_rotation_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("target_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("clamp_to_workspace", True)
        self.declare_parameter("workspace_min_xyz", [-3.5, 0.39, 0.82])
        self.declare_parameter("workspace_max_xyz", [3.5, 0.73, 1.05])
        self.declare_parameter("marker_scale", 0.05)
        self.declare_parameter("log_target_coordinates", True)

        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.depth_mode = str(self.get_parameter("depth_mode").value)
        self.fixed_depth_m = float(self.get_parameter("fixed_depth_m").value)
        self.target_real_height_m = float(self.get_parameter("target_real_height_m").value)
        self.reference_bbox_height_px = float(self.get_parameter("reference_bbox_height_px").value)
        self.bbox_height_depth_weight = float(self.get_parameter("bbox_height_depth_weight").value)
        self.clamp_near_depth_to_min = bool(self.get_parameter("clamp_near_depth_to_min").value)
        self.camera_depth_min_m = float(self.get_parameter("camera_depth_min_m").value)
        self.camera_depth_max_m = float(self.get_parameter("camera_depth_max_m").value)
        self.reject_too_near_targets = bool(self.get_parameter("reject_too_near_targets").value)
        self.reject_out_of_depth_range = bool(self.get_parameter("reject_out_of_depth_range").value)
        self.use_tf_extrinsics = bool(self.get_parameter("use_tf_extrinsics").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.log_target_coordinates = bool(self.get_parameter("log_target_coordinates").value)

        translation = list(self.get_parameter("camera_translation_xyz").value)
        rotation_rpy = list(self.get_parameter("camera_rotation_rpy").value)
        self.camera_translation = np.array(translation, dtype=float).reshape(3)
        self.camera_rotation = rpy_to_rotation_matrix(
            float(rotation_rpy[0]),
            float(rotation_rpy[1]),
            float(rotation_rpy[2]),
        )
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

        input_topic = str(self.get_parameter("input_topic").value)
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        camera_point_topic = str(self.get_parameter("camera_point_topic").value)
        world_point_topic = str(self.get_parameter("world_point_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        target_rejected_topic = str(self.get_parameter("target_rejected_topic").value)

        self.pose_publisher = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.camera_point_publisher = self.create_publisher(PointStamped, camera_point_topic, 10)
        self.world_point_publisher = self.create_publisher(PointStamped, world_point_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, marker_topic, 10)
        self.target_rejected_publisher = self.create_publisher(Bool, target_rejected_topic, 10)
        self.create_subscription(FlowerDetection, input_topic, self._detection_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.logged_tf_success = False
        self.logged_manual_fallback = False
        self.logged_workspace_config = False

        self.get_logger().debug(
            f"Pixel-to-world node ready. input_topic={input_topic}, target_pose_topic={target_pose_topic}, "
            f"depth_mode={self.depth_mode}, fixed_depth_m={self.fixed_depth_m:.3f}, "
            f"use_tf_extrinsics={self.use_tf_extrinsics}"
        )

    def _detection_callback(self, message: FlowerDetection) -> None:
        z_camera = self._estimate_camera_depth(message)
        if self._depth_is_rejected(z_camera):
            self._publish_target_rejected()
            if self.log_target_coordinates:
                self.get_logger().info(
                    "target_camera_depth_rejected=(%.3f), allowed=(%.3f, %.3f)"
                    % (z_camera, self.camera_depth_min_m, self.camera_depth_max_m)
                )
            return

        x_camera = (float(message.center_u) - self.cx) * z_camera / self.fx
        y_camera = (float(message.center_v) - self.cy) * z_camera / self.fy
        camera_point = np.array([x_camera, y_camera, z_camera], dtype=float)

        stamp = self.get_clock().now().to_msg()
        camera_header_frame = message.header.frame_id or self.camera_frame
        raw_world_point = self._camera_point_to_world(camera_point, camera_header_frame)
        world_point = self._shape_world_point(raw_world_point)

        camera_msg = PointStamped()
        camera_msg.header.stamp = stamp
        camera_msg.header.frame_id = camera_header_frame
        camera_msg.point.x = float(camera_point[0])
        camera_msg.point.y = float(camera_point[1])
        camera_msg.point.z = float(camera_point[2])
        self.camera_point_publisher.publish(camera_msg)

        world_msg = PointStamped()
        world_msg.header.stamp = stamp
        world_msg.header.frame_id = self.target_frame
        world_msg.point.x = float(world_point[0])
        world_msg.point.y = float(world_point[1])
        world_msg.point.z = float(world_point[2])
        self.world_point_publisher.publish(world_msg)

        pose_msg = PoseStamped()
        pose_msg.header = world_msg.header
        pose_msg.pose.position = world_msg.point
        pose_msg.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose_msg)

        marker = Marker()
        marker.header = world_msg.header
        marker.ns = "localized_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = world_msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 0.1
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

        if self.log_target_coordinates:
            self.get_logger().info(
                "target_camera=(%.3f, %.3f, %.3f) "
                "target_world_raw=(%.3f, %.3f, %.3f) "
                "target_world_shaped=(%.3f, %.3f, %.3f)"
                % (
                    camera_point[0],
                    camera_point[1],
                    camera_point[2],
                    raw_world_point[0],
                    raw_world_point[1],
                    raw_world_point[2],
                    world_point[0],
                    world_point[1],
                    world_point[2],
                )
            )

    def _estimate_camera_depth(self, message: FlowerDetection) -> float:
        if self.depth_mode == "fixed_depth":
            return self.fixed_depth_m

        if self.depth_mode == "bbox_height":
            bbox_height_px = float(message.bbox_ymax) - float(message.bbox_ymin)
            if bbox_height_px <= 1.0:
                return self.fixed_depth_m
            return self.fy * self.target_real_height_m / bbox_height_px

        if self.depth_mode == "fixed_plus_bbox_height":
            bbox_height_px = float(message.bbox_ymax) - float(message.bbox_ymin)
            if bbox_height_px <= 1.0:
                return self.fixed_depth_m

            estimated_depth = self.fixed_depth_m + self.bbox_height_depth_weight * (
                self.reference_bbox_height_px - bbox_height_px
            )
            if self.clamp_near_depth_to_min:
                estimated_depth = max(estimated_depth, self.camera_depth_min_m)
            return estimated_depth

        self.get_logger().warning(
            f"Unsupported depth_mode '{self.depth_mode}', falling back to fixed_depth."
        )
        return self.fixed_depth_m

    def _depth_is_rejected(self, z_camera: float) -> bool:
        if not self.reject_out_of_depth_range:
            return False
        if z_camera > self.camera_depth_max_m:
            return True
        return self.reject_too_near_targets and z_camera < self.camera_depth_min_m

    def _publish_target_rejected(self) -> None:
        message = Bool()
        message.data = True
        self.target_rejected_publisher.publish(message)

    def _shape_world_point(self, world_point: np.ndarray) -> np.ndarray:
        shaped_point = world_point + self.target_offset

        if self.clamp_to_workspace:
            shaped_point = np.minimum(np.maximum(shaped_point, self.workspace_min), self.workspace_max)

        if not self.logged_workspace_config:
            self.get_logger().debug(
                "Workspace shaping enabled: "
                f"offset={self.target_offset.tolist()}, "
                f"clamp={self.clamp_to_workspace}, "
                f"min={self.workspace_min.tolist()}, "
                f"max={self.workspace_max.tolist()}"
            )
            self.logged_workspace_config = True

        return shaped_point

    def _camera_point_to_world(self, camera_point: np.ndarray, source_frame: str) -> np.ndarray:
        if self.use_tf_extrinsics:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )

                translation = transform.transform.translation
                rotation = transform.transform.rotation
                rotation_matrix = quaternion_to_rotation_matrix(
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                )
                world_point = rotation_matrix @ camera_point + np.array(
                    [translation.x, translation.y, translation.z],
                    dtype=float,
                )

                if not self.logged_tf_success:
                    self.get_logger().debug(
                        f"Using TF extrinsics from {self.target_frame} <- {source_frame}."
                    )
                    self.logged_tf_success = True
                return world_point
            except TransformException as exc:
                if not self.logged_manual_fallback:
                    self.get_logger().warning(
                        f"TF lookup failed for {self.target_frame} <- {source_frame}: {exc}. "
                        "Falling back to manual extrinsics."
                    )
                    self.logged_manual_fallback = True

        return self.camera_rotation @ camera_point + self.camera_translation


def main() -> None:
    rclpy.init()
    node = PixelToWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
