from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import ImageGrab
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ImageSourceNode(Node):
    def __init__(self) -> None:
        super().__init__("image_source_node")

        self.declare_parameter("source_type", "image")
        self.declare_parameter("source_path", "")
        self.declare_parameter("trigger_file_path", "")
        self.declare_parameter("trigger_wait_timeout_sec", 2.0)
        self.declare_parameter("use_stale_image_on_trigger_timeout", False)
        self.declare_parameter("trigger_publish_burst_count", 5)
        self.declare_parameter("trigger_idle_refresh_sec", 3.0)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("output_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("screen_region_width", 1000)
        self.declare_parameter("screen_region_height", 1000)
        self.declare_parameter("screen_region_right_margin", 0)
        self.declare_parameter("screen_region_bottom_margin", 0)
        self.declare_parameter("loop_video", True)
        self.declare_parameter("loop_images", False)
        self.declare_parameter("advance_images_on_cycle_complete", True)
        self.declare_parameter("advance_images_on_frame_exhausted", True)
        self.declare_parameter("gate_on_cycle_complete", True)
        self.declare_parameter("cycle_busy_topic", "/harvest_cycle_busy")
        self.declare_parameter("image_done_topic", "/carnation/image_targets_done")
        self.declare_parameter("target_rejected_topic", "/carnation/target_rejected")
        self.declare_parameter("frame_exhausted_topic", "/carnation/frame_exhausted")
        self.declare_parameter("publish_on_startup", True)

        self.source_type = str(self.get_parameter("source_type").value).lower()
        self.source_path = str(self.get_parameter("source_path").value)
        self.trigger_file_path = str(self.get_parameter("trigger_file_path").value)
        self.trigger_wait_timeout_sec = float(
            self.get_parameter("trigger_wait_timeout_sec").value
        )
        self.use_stale_image_on_trigger_timeout = bool(
            self.get_parameter("use_stale_image_on_trigger_timeout").value
        )
        self.trigger_publish_burst_count = int(
            self.get_parameter("trigger_publish_burst_count").value
        )
        self.trigger_idle_refresh_sec = float(
            self.get_parameter("trigger_idle_refresh_sec").value
        )
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.screen_region_width = int(self.get_parameter("screen_region_width").value)
        self.screen_region_height = int(self.get_parameter("screen_region_height").value)
        self.screen_region_right_margin = int(
            self.get_parameter("screen_region_right_margin").value
        )
        self.screen_region_bottom_margin = int(
            self.get_parameter("screen_region_bottom_margin").value
        )
        self.loop_video = bool(self.get_parameter("loop_video").value)
        self.loop_images = bool(self.get_parameter("loop_images").value)
        self.advance_images_on_cycle_complete = bool(
            self.get_parameter("advance_images_on_cycle_complete").value
        )
        self.advance_images_on_frame_exhausted = bool(
            self.get_parameter("advance_images_on_frame_exhausted").value
        )
        self.gate_on_cycle_complete = bool(self.get_parameter("gate_on_cycle_complete").value)
        self.cycle_busy_topic = str(self.get_parameter("cycle_busy_topic").value)
        self.image_done_topic = str(self.get_parameter("image_done_topic").value)
        self.target_rejected_topic = str(self.get_parameter("target_rejected_topic").value)
        self.frame_exhausted_topic = str(self.get_parameter("frame_exhausted_topic").value)
        self.publish_on_startup = bool(self.get_parameter("publish_on_startup").value)

        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.bridge = CvBridge()
        self.capture = None
        self.static_frame = None
        self.screen_region_bbox = None
        self.image_paths = []
        self.current_image_index = 0
        self.cycle_busy = False
        self.pending_publish = self.publish_on_startup
        self.cycle_started_once = False
        self.waiting_for_triggered_frame = False
        self.trigger_publish_burst_remaining = 0
        self.last_triggered_frame = None
        self.trigger_request_time = 0.0
        self.last_trigger_request_time = 0.0
        self.last_publish_time = 0.0
        self.trigger_timeout_warned = False
        self.last_source_mtime = 0.0
        self.published_frame_count = 0

        if self.source_type == "image":
            if not self.source_path:
                raise ValueError("image source_type requires a non-empty source_path.")
            image_path = Path(self.source_path)
            if not image_path.is_file():
                raise FileNotFoundError(f"Image file not found: {image_path}")
            self.static_frame = cv2.imread(str(image_path))
            if self.static_frame is None:
                raise RuntimeError(f"Failed to read image: {image_path}")
            self.get_logger().debug(f"Publishing static image from {image_path}")
        elif self.source_type == "image_directory":
            if not self.source_path:
                raise ValueError("image_directory source_type requires a non-empty source_path.")
            image_dir = Path(self.source_path)
            if not image_dir.is_dir():
                raise FileNotFoundError(f"Image directory not found: {image_dir}")
            suffixes = {".jpg", ".jpeg", ".png", ".bmp"}
            self.image_paths = sorted(
                path for path in image_dir.iterdir() if path.is_file() and path.suffix.lower() in suffixes
            )
            if not self.image_paths:
                raise FileNotFoundError(f"No image files found in directory: {image_dir}")
            self.get_logger().debug(
                f"Publishing image directory from {image_dir}; count={len(self.image_paths)}"
            )
        elif self.source_type == "latest_image":
            if not self.source_path:
                raise ValueError("latest_image source_type requires a non-empty source_path.")
            self.get_logger().debug(f"Publishing latest image file from {self.source_path}")
        elif self.source_type == "triggered_latest_image":
            if not self.source_path:
                raise ValueError("triggered_latest_image source_type requires a non-empty source_path.")
            if not self.trigger_file_path:
                raise ValueError("triggered_latest_image requires a non-empty trigger_file_path.")
            self._request_triggered_frame()
            self.get_logger().debug(
                f"Publishing triggered latest image from {self.source_path}; "
                f"trigger_file={self.trigger_file_path}"
            )
        elif self.source_type == "video":
            if not self.source_path:
                raise ValueError("video source_type requires a non-empty source_path.")
            self.capture = cv2.VideoCapture(self.source_path)
            if not self.capture.isOpened():
                raise RuntimeError(f"Failed to open video: {self.source_path}")
            self.get_logger().debug(f"Publishing video from {self.source_path}")
        elif self.source_type == "camera":
            self.capture = cv2.VideoCapture(self.camera_index)
            if not self.capture.isOpened():
                raise RuntimeError(f"Failed to open camera index {self.camera_index}")
            self.get_logger().debug(f"Publishing camera stream from index {self.camera_index}")
        elif self.source_type == "screen_region":
            self.screen_region_bbox = self._compute_screen_region_bbox()
            self.get_logger().debug(f"Publishing screen region: bbox={self.screen_region_bbox}")
        else:
            raise ValueError(
                "source_type must be one of: image, image_directory, latest_image, "
                "triggered_latest_image, video, camera, screen_region"
            )

        if self.gate_on_cycle_complete:
            self.create_subscription(Bool, self.cycle_busy_topic, self._cycle_busy_callback, 10)
            self.get_logger().debug(
                f"Image source gating enabled. Waiting for idle events on {self.cycle_busy_topic}"
            )
        if self.source_type == "image_directory":
            self.create_subscription(Bool, self.image_done_topic, self._image_done_callback, 10)
        if self.source_type in ("triggered_latest_image", "image_directory"):
            self.create_subscription(Bool, self.frame_exhausted_topic, self._frame_exhausted_callback, 10)

        self.create_timer(1.0 / max(self.publish_rate_hz, 0.1), self._timer_callback)

    def _cycle_busy_callback(self, message: Bool) -> None:
        previous_busy = self.cycle_busy
        self.cycle_busy = bool(message.data)

        if self.cycle_busy:
            self.cycle_started_once = True
            self.pending_publish = False

        if (
            previous_busy
            and not self.cycle_busy
            and self.source_type == "image_directory"
            and self.advance_images_on_cycle_complete
        ):
            self._advance_image_directory()
        elif previous_busy and not self.cycle_busy and self.source_type != "image_directory":
            self.pending_publish = True
            if self.source_type == "triggered_latest_image":
                self._request_triggered_frame()
        elif (
            self.source_type == "triggered_latest_image"
            and not self.cycle_busy
            and not self.pending_publish
            and not self.waiting_for_triggered_frame
            and self._trigger_idle_refresh_due()
        ):
            self.pending_publish = True
            self._request_triggered_frame()

    def _image_done_callback(self, message: Bool) -> None:
        if not message.data or self.source_type != "image_directory":
            return

        self._advance_image_directory()

    def _frame_exhausted_callback(self, message: Bool) -> None:
        if not message.data:
            return

        if self.source_type == "triggered_latest_image":
            self.pending_publish = True
            self._request_triggered_frame()
            return

        if self.source_type == "image_directory" and self.advance_images_on_frame_exhausted:
            self._advance_image_directory()

    def _timer_callback(self) -> None:
        if self.gate_on_cycle_complete:
            if self.cycle_busy:
                return
            if self.source_type == "triggered_latest_image" and not self.pending_publish:
                return
            if self.cycle_started_once and not self.pending_publish:
                return

        if self.source_type == "image":
            frame = self.static_frame.copy()
        elif self.source_type == "image_directory":
            image_path = self.image_paths[self.current_image_index]
            frame = cv2.imread(str(image_path))
            if frame is None:
                self.get_logger().warning(f"Failed to read image: {image_path}")
                self.pending_publish = False
                return
        elif self.source_type == "latest_image":
            frame = cv2.imread(self.source_path)
            if frame is None:
                self.get_logger().warning(f"Latest image file is not readable yet: {self.source_path}")
                return
        elif self.source_type == "triggered_latest_image":
            frame = self._read_triggered_latest_image()
            if frame is None:
                return
        elif self.source_type == "screen_region":
            frame = self._read_screen_region_frame()
        else:
            ok, frame = self.capture.read()
            if not ok or frame is None:
                if self.source_type == "video" and self.loop_video:
                    self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = self.capture.read()
                if not ok or frame is None:
                    self.get_logger().warning("Image source has no frame to publish.")
                    return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        self.published_frame_count += 1
        self.last_publish_time = time.monotonic()
        self.get_logger().info(
            f"Published image frame #{self.published_frame_count} from {self.source_type}."
        )
        if self.source_type == "triggered_latest_image" and self.trigger_publish_burst_remaining > 0:
            self.trigger_publish_burst_remaining -= 1
        if (
            self.source_type == "triggered_latest_image"
            and self.trigger_publish_burst_remaining > 0
        ):
            self.pending_publish = True
        elif self.cycle_started_once or self.source_type == "triggered_latest_image":
            self.pending_publish = False
        if self.source_type == "triggered_latest_image":
            self.waiting_for_triggered_frame = False

    def _compute_screen_region_bbox(self) -> tuple[int, int, int, int]:
        if self.screen_region_width <= 0 or self.screen_region_height <= 0:
            raise ValueError("screen_region_width and screen_region_height must be positive.")
        if self.screen_region_right_margin < 0 or self.screen_region_bottom_margin < 0:
            raise ValueError("screen region margins must be zero or positive.")

        try:
            screen = ImageGrab.grab()
        except Exception as exc:
            raise RuntimeError(
                "Failed to capture the screen. In WSL this depends on GUI/ImageGrab support; "
                "if Windows desktop capture is needed, run from a Windows-side bridge or use a camera/video source."
            ) from exc

        screen_width, screen_height = screen.size
        right = screen_width - self.screen_region_right_margin
        bottom = screen_height - self.screen_region_bottom_margin
        left = max(0, right - self.screen_region_width)
        top = max(0, bottom - self.screen_region_height)
        return (left, top, right, bottom)

    def _read_screen_region_frame(self):
        image = ImageGrab.grab(bbox=self.screen_region_bbox)
        return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

    def _advance_image_directory(self) -> None:
        if self.source_type != "image_directory":
            return

        if self.current_image_index + 1 < len(self.image_paths):
            self.current_image_index += 1
            self.pending_publish = True
            self.get_logger().info(
                f"Advancing to image {self.current_image_index + 1}/{len(self.image_paths)}: "
                f"{self.image_paths[self.current_image_index]}"
            )
            return

        if self.loop_images:
            self.current_image_index = 0
            self.pending_publish = True
            self.get_logger().info(
                f"Looping to image 1/{len(self.image_paths)}: {self.image_paths[0]}"
            )
            return

        self.pending_publish = False
        self.get_logger().info("Image directory is exhausted; no more images to publish.")

    def _request_triggered_frame(self) -> None:
        source_path = Path(self.source_path)
        self.last_source_mtime = source_path.stat().st_mtime if source_path.exists() else 0.0
        trigger_path = Path(self.trigger_file_path)
        trigger_path.parent.mkdir(parents=True, exist_ok=True)
        trigger_path.write_text(str(self.get_clock().now().nanoseconds), encoding="utf-8")
        self.waiting_for_triggered_frame = True
        self.trigger_publish_burst_remaining = 0
        self.last_triggered_frame = None
        self.trigger_request_time = time.monotonic()
        self.last_trigger_request_time = self.trigger_request_time
        self.trigger_timeout_warned = False
        self.get_logger().info(f"Requested triggered image: {trigger_path}")

    def _trigger_idle_refresh_due(self) -> bool:
        if self.trigger_idle_refresh_sec <= 0.0:
            return False

        now = time.monotonic()
        last_activity = max(self.last_trigger_request_time, self.last_publish_time)
        if last_activity <= 0.0:
            return True
        return now - last_activity >= self.trigger_idle_refresh_sec

    def _read_triggered_latest_image(self):
        source_path = Path(self.source_path)
        if self.waiting_for_triggered_frame:
            if not source_path.exists():
                return None
            if source_path.stat().st_mtime <= self.last_source_mtime:
                elapsed = time.monotonic() - self.trigger_request_time
                if elapsed < self.trigger_wait_timeout_sec:
                    return None
                if not self.trigger_timeout_warned:
                    if self.use_stale_image_on_trigger_timeout:
                        self.get_logger().warning(
                            "Timed out waiting for a new triggered image; "
                            "using the existing image file instead."
                        )
                    else:
                        self.get_logger().warning(
                            "Timed out waiting for a new triggered image; "
                            "not publishing the stale image file."
                        )
                    self.trigger_timeout_warned = True
                if not self.use_stale_image_on_trigger_timeout:
                    return None

        frame = cv2.imread(str(source_path))
        if frame is None:
            self.get_logger().warning(f"Triggered latest image file is not readable yet: {self.source_path}")
            return None
        if self.waiting_for_triggered_frame or self.last_triggered_frame is None:
            self.last_triggered_frame = frame
            self.trigger_publish_burst_remaining = max(self.trigger_publish_burst_count, 1)
        elif self.trigger_publish_burst_remaining > 0:
            frame = self.last_triggered_frame.copy()
        return frame


def main() -> None:
    rclpy.init()
    node = ImageSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.capture is not None:
            node.capture.release()
        node.destroy_node()
        rclpy.shutdown()
