from pathlib import Path
import hashlib
import json

from carnation_interfaces.msg import FlowerDetection
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        self.declare_parameter("model_path", "/root/carnation_harvest/models/carnation_yolov8m2_best.pt")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("pixel_center_topic", "/carnation/pixel_center")
        self.declare_parameter("annotated_image_topic", "/carnation/detections/image_annotated")
        self.declare_parameter("image_done_topic", "/carnation/image_targets_done")
        self.declare_parameter("target_rejected_topic", "/carnation/target_rejected")
        self.declare_parameter("frame_exhausted_topic", "/carnation/frame_exhausted")
        self.declare_parameter("runtime_config_path", "")
        self.declare_parameter("target_class_name", "carnation_flower")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("annotated_republish_rate_hz", 1.0)
        self.declare_parameter("image_size", 960)
        self.declare_parameter("device", "")
        self.declare_parameter("sequence_detections", True)
        self.declare_parameter("cycle_busy_topic", "/harvest_cycle_busy")
        self.declare_parameter("wait_for_cycle_ready", True)
        self.declare_parameter("first_target_delay_sec", 1.0)
        self.declare_parameter("target_feedback_timeout_sec", 3.0)
        self.declare_parameter("allow_sequence_repeat", False)
        self.declare_parameter("sequence_sort_axis", "u")
        self.declare_parameter("sequence_sort_reverse", False)
        self.declare_parameter("simulate_recenter_after_target", False)
        self.declare_parameter("virtual_principal_u", 640.0)
        self.declare_parameter("reset_virtual_principal_on_new_image", True)

        model_path = Path(str(self.get_parameter("model_path").value))
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model file not found: {model_path}")

        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)
        self.annotated_republish_rate_hz = float(
            self.get_parameter("annotated_republish_rate_hz").value
        )
        self.image_size = int(self.get_parameter("image_size").value)
        self.runtime_config_path_text = str(self.get_parameter("runtime_config_path").value).strip()
        self.runtime_config_path = (
            Path(self.runtime_config_path_text) if self.runtime_config_path_text else None
        )
        self.runtime_config_mtime = 0.0
        self.device = str(self.get_parameter("device").value).strip()
        self.sequence_detections = bool(self.get_parameter("sequence_detections").value)
        self.cycle_busy_topic = str(self.get_parameter("cycle_busy_topic").value)
        self.wait_for_cycle_ready = bool(self.get_parameter("wait_for_cycle_ready").value)
        self.first_target_delay_sec = float(self.get_parameter("first_target_delay_sec").value)
        self.target_feedback_timeout_sec = float(
            self.get_parameter("target_feedback_timeout_sec").value
        )
        self.allow_sequence_repeat = bool(self.get_parameter("allow_sequence_repeat").value)
        self.sequence_sort_axis = str(self.get_parameter("sequence_sort_axis").value).lower()
        self.sequence_sort_reverse = bool(self.get_parameter("sequence_sort_reverse").value)
        self.simulate_recenter_after_target = bool(
            self.get_parameter("simulate_recenter_after_target").value
        )
        self.configured_virtual_principal_u = float(
            self.get_parameter("virtual_principal_u").value
        )
        self.reset_virtual_principal_on_new_image = bool(
            self.get_parameter("reset_virtual_principal_on_new_image").value
        )

        image_topic = str(self.get_parameter("image_topic").value)
        pixel_center_topic = str(self.get_parameter("pixel_center_topic").value)
        annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        self.image_done_topic = str(self.get_parameter("image_done_topic").value)
        self.target_rejected_topic = str(self.get_parameter("target_rejected_topic").value)
        self.frame_exhausted_topic = str(self.get_parameter("frame_exhausted_topic").value)

        self.bridge = CvBridge()
        self.model = YOLO(str(model_path))
        self.detection_publisher = self.create_publisher(FlowerDetection, pixel_center_topic, 10)
        self.annotated_publisher = self.create_publisher(Image, annotated_image_topic, 10)
        self.image_done_publisher = self.create_publisher(Bool, self.image_done_topic, 10)
        self.target_rejected_publisher = self.create_publisher(Bool, self.target_rejected_topic, 10)
        self.frame_exhausted_publisher = self.create_publisher(Bool, self.frame_exhausted_topic, 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.create_subscription(Bool, self.target_rejected_topic, self._target_rejected_callback, 10)
        self.detection_queue = []
        self.realtime_detection_queue = []
        self.current_detection = None
        self.last_image_message = None
        self.last_clean_frame = None
        self.realtime_image_message = None
        self.realtime_clean_frame = None
        self.realtime_published_count = 0
        self.realtime_target_publish_time_sec = None
        self.current_image_signature = ""
        self.last_finished_image_signature = ""
        self.waiting_for_new_image = False
        self.nominal_principal_u = self.configured_virtual_principal_u
        self.current_virtual_principal_u = self.configured_virtual_principal_u
        self.sequence_initialized = False
        self.sequence_completed = False
        self.sequence_total = 0
        self.sequence_published_count = 0
        self.cycle_busy = False
        self.cycle_state_seen = False
        self.first_target_timer = None
        self.last_annotated_image_msg = None

        self.create_subscription(Bool, self.cycle_busy_topic, self._cycle_busy_callback, 10)
        if self.publish_annotated_image and self.annotated_republish_rate_hz > 0.0:
            self.create_timer(
                1.0 / self.annotated_republish_rate_hz,
                self._republish_last_annotated_image,
            )
        if not self.sequence_detections and self.target_feedback_timeout_sec > 0.0:
            self.create_timer(0.2, self._check_realtime_target_timeout)

        self.get_logger().debug(
            f"YOLO detector ready. model={model_path}, image_topic={image_topic}, "
            f"pixel_center_topic={pixel_center_topic}"
        )

    def _image_callback(self, message: Image) -> None:
        if self.sequence_detections and self.sequence_initialized:
            return

        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        image_signature = self._image_signature(frame)
        if self.waiting_for_new_image and image_signature == self.last_finished_image_signature:
            return
        if not self.sequence_detections and image_signature == self.current_image_signature:
            return
        self.waiting_for_new_image = False
        self.current_image_signature = image_signature
        self._reload_runtime_config_if_needed()

        predict_kwargs = {"verbose": False}
        if self.image_size > 0:
            predict_kwargs["imgsz"] = self.image_size
        if self.confidence_threshold > 0.0:
            predict_kwargs["conf"] = self.confidence_threshold
        if self.device:
            predict_kwargs["device"] = self.device

        results = self.model.predict(source=frame, **predict_kwargs)
        if not results:
            if self.publish_annotated_image:
                self._publish_annotated_image(frame, message.header)
            if self.sequence_detections:
                self._finish_current_image_sequence()
            else:
                self._finish_realtime_frame()
                self._publish_frame_exhausted()
            return

        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            if self.publish_annotated_image:
                self._publish_annotated_image(frame, message.header)
            if self.sequence_detections:
                self._finish_current_image_sequence()
            else:
                self._finish_realtime_frame()
                self._publish_frame_exhausted()
            return

        names = result.names if hasattr(result, "names") else self.model.names

        detections = []

        for box in boxes:
            cls_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            class_name = str(names.get(cls_id, cls_id))

            if confidence < self.confidence_threshold:
                continue
            if self.target_class_name and class_name != self.target_class_name:
                continue

            xyxy = box.xyxy[0].tolist()
            xmin, ymin, xmax, ymax = [float(v) for v in xyxy]
            center_u = 0.5 * (xmin + xmax)
            center_v = 0.5 * (ymin + ymax)

            detection = {
                "class_name": class_name,
                "confidence": confidence,
                "xmin": xmin,
                "ymin": ymin,
                "xmax": xmax,
                "ymax": ymax,
                "center_u": center_u,
                "center_v": center_v,
            }
            detections.append(detection)

        if self.sequence_detections:
            self._start_detection_sequence(detections, message, frame)
        else:
            self._start_realtime_candidates(detections, message, frame)

    def _cycle_busy_callback(self, message: Bool) -> None:
        previous_busy = self.cycle_busy
        self.cycle_busy = bool(message.data)
        self.cycle_state_seen = True

        if not self.sequence_detections:
            if self.cycle_busy:
                # 实时截图模式下一帧只服务一次采收；目标已被规划层接受后，
                # 丢弃同一帧剩余候选，采收完成后由图像源触发下一帧。
                self.realtime_detection_queue = []
                self.realtime_target_publish_time_sec = None
            elif previous_busy:
                self._finish_realtime_frame()
            return

        if (
            self.sequence_initialized
            and self.sequence_published_count == 0
            and self.detection_queue
            and not self.cycle_busy
        ):
            self._cancel_first_target_timer()
            self._publish_next_queued_detection()
            return

        if previous_busy and not self.cycle_busy:
            self._recenter_to_completed_target()
            self._publish_next_queued_detection()

    def _target_rejected_callback(self, message: Bool) -> None:
        if not message.data:
            return

        self.current_detection = None
        self.realtime_target_publish_time_sec = None
        if self.sequence_initialized:
            self._publish_next_queued_detection()
            return

        if not self.sequence_detections:
            self.get_logger().info("Current target was rejected; trying next candidate in this frame.")
            self._publish_next_realtime_detection()

    def _start_detection_sequence(self, detections: list, image_message: Image, clean_frame) -> None:
        self.sequence_initialized = True
        self.sequence_completed = False
        self.last_image_message = image_message
        self.last_clean_frame = clean_frame.copy()
        self.current_detection = None
        self.sequence_published_count = 0
        self._reset_virtual_principal_for_image(image_message)

        self.detection_queue = self._sort_detections(detections)
        self.sequence_total = len(self.detection_queue)

        if self.sequence_total == 0:
            if self.publish_annotated_image:
                self._publish_annotated_image(self.last_clean_frame, image_message.header)
            self._finish_current_image_sequence()
            self.get_logger().debug("No target detections found in the current image.")
            return

        self.get_logger().debug(
            f"Queued {self.sequence_total} target detections from the static image."
        )
        if self.wait_for_cycle_ready and not self.cycle_state_seen:
            self.get_logger().debug(
                f"Waiting up to {self.first_target_delay_sec:.1f}s for cycle state on "
                f"{self.cycle_busy_topic} before publishing the first target."
            )
            self.first_target_timer = self.create_timer(
                max(self.first_target_delay_sec, 0.1),
                self._publish_first_target_after_delay,
            )
            return

        self._publish_next_queued_detection()

    def _publish_first_target_after_delay(self) -> None:
        self._cancel_first_target_timer()
        if self.sequence_published_count == 0 and self.detection_queue:
            self._publish_next_queued_detection()

    def _cancel_first_target_timer(self) -> None:
        if self.first_target_timer is not None:
            self.first_target_timer.cancel()
            self.first_target_timer = None

    def _publish_next_queued_detection(self) -> None:
        if self.cycle_busy:
            return

        if not self.detection_queue:
            if not self.sequence_completed:
                self.get_logger().debug("All queued target detections have been published.")
                self._finish_current_image_sequence()
            return

        detection = self.detection_queue.pop(0)
        self.current_detection = detection
        self.sequence_published_count += 1
        if self.publish_annotated_image:
            self._publish_current_target_annotated_image(detection)
        self._publish_detection(detection, self.last_image_message)
        self.get_logger().debug(
            "Published target %d/%d: center=(%.1f, %.1f), confidence=%.2f"
            % (
                self.sequence_published_count,
                self.sequence_total,
                detection["center_u"],
                detection["center_v"],
                detection["confidence"],
            )
        )

    def _finish_current_image_sequence(self) -> None:
        if self.sequence_completed:
            return

        self.sequence_completed = True
        self.sequence_initialized = False
        self.current_detection = None
        self.detection_queue = []
        self.last_finished_image_signature = self.current_image_signature
        self.waiting_for_new_image = True
        if self.publish_annotated_image and self.last_clean_frame is not None and self.last_image_message is not None:
            self._publish_annotated_image(self.last_clean_frame, self.last_image_message.header)

        done_msg = Bool()
        done_msg.data = True
        self.image_done_publisher.publish(done_msg)

    def _publish_target_rejected(self) -> None:
        message = Bool()
        message.data = True
        self.target_rejected_publisher.publish(message)

    def _publish_frame_exhausted(self) -> None:
        message = Bool()
        message.data = True
        self.frame_exhausted_publisher.publish(message)

    def _start_realtime_candidates(self, detections: list, image_message: Image, clean_frame) -> None:
        self.realtime_image_message = image_message
        self.realtime_clean_frame = clean_frame.copy()
        self.realtime_published_count = 0
        self.current_detection = None
        self.realtime_target_publish_time_sec = None
        self._reset_virtual_principal_for_image(image_message)
        self.realtime_detection_queue = sorted(
            detections,
            key=lambda detection: detection["confidence"],
            reverse=True,
        )

        if not self.realtime_detection_queue:
            if self.publish_annotated_image:
                self._publish_annotated_image(self.realtime_clean_frame, image_message.header)
            self._finish_realtime_frame()
            self._publish_frame_exhausted()
            return

        self._publish_next_realtime_detection()

    def _publish_next_realtime_detection(self) -> None:
        if not self.realtime_detection_queue:
            self._finish_realtime_frame()
            self.get_logger().info("Current frame has no remaining valid candidates; requesting a new frame.")
            self._publish_frame_exhausted()
            return

        detection = self.realtime_detection_queue.pop(0)
        self.current_detection = detection
        self.realtime_published_count += 1
        self.get_logger().info(
            "Published candidate %d: center=(%.1f, %.1f), bbox_h=%.1f, confidence=%.2f"
            % (
                self.realtime_published_count,
                detection["center_u"],
                detection["center_v"],
                detection["ymax"] - detection["ymin"],
                detection["confidence"],
            )
        )
        if self.publish_annotated_image:
            self._publish_realtime_target_annotated_image(detection)
        if self.realtime_image_message is not None:
            self._publish_detection(detection, self.realtime_image_message)
            self.realtime_target_publish_time_sec = self._now_seconds()

    def _clear_realtime_candidates(self) -> None:
        self.realtime_detection_queue = []
        self.realtime_image_message = None
        self.realtime_clean_frame = None
        self.realtime_published_count = 0
        self.realtime_target_publish_time_sec = None
        self.current_detection = None

    def _finish_realtime_frame(self) -> None:
        self.last_finished_image_signature = self.current_image_signature
        self.waiting_for_new_image = True
        self._clear_realtime_candidates()

    def _check_realtime_target_timeout(self) -> None:
        if self.sequence_detections or self.cycle_busy:
            return
        if self.current_detection is None or self.realtime_target_publish_time_sec is None:
            return

        elapsed = self._now_seconds() - self.realtime_target_publish_time_sec
        if elapsed < self.target_feedback_timeout_sec:
            return

        self.get_logger().info(
            "No accepted/rejected feedback for %.1fs after publishing the current target; "
            "requesting a new frame." % elapsed
        )
        self._finish_realtime_frame()
        self._publish_frame_exhausted()

    def _publish_current_target_annotated_image(self, detection: dict) -> None:
        if self.last_clean_frame is None or self.last_image_message is None:
            return

        annotated_frame = self.last_clean_frame.copy()
        self._draw_detection(annotated_frame, detection, index=self.sequence_published_count)
        self._publish_annotated_image(annotated_frame, self.last_image_message.header)

    def _publish_realtime_target_annotated_image(self, detection: dict) -> None:
        if self.realtime_clean_frame is None or self.realtime_image_message is None:
            return

        annotated_frame = self.realtime_clean_frame.copy()
        self._draw_detection(annotated_frame, detection, index=self.realtime_published_count)
        self._publish_annotated_image(annotated_frame, self.realtime_image_message.header)

    def _sort_detections(self, detections: list) -> list:
        if self.sequence_sort_axis == "v":
            key_name = "center_v"
        elif self.sequence_sort_axis == "confidence":
            key_name = "confidence"
        else:
            key_name = "center_u"
        return sorted(
            detections,
            key=lambda detection: detection[key_name],
            reverse=self.sequence_sort_reverse,
        )

    def _draw_detection(self, frame, detection: dict, index: int | None = None) -> None:
        cv2.rectangle(
            frame,
            (int(detection["xmin"]), int(detection["ymin"])),
            (int(detection["xmax"]), int(detection["ymax"])),
            (0, 255, 0),
            2,
        )
        cv2.circle(
            frame,
            (int(detection["center_u"]), int(detection["center_v"])),
            5,
            (0, 0, 255),
            -1,
        )
        label_prefix = f"{index}: " if index is not None else ""
        cv2.putText(
            frame,
            f'{label_prefix}{detection["class_name"]} {detection["confidence"]:.2f}',
            (int(detection["xmin"]), max(int(detection["ymin"]) - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    def _publish_detection(self, detection: dict, image_message: Image) -> None:
        center_u = float(detection["center_u"])
        if self.simulate_recenter_after_target:
            center_u = self.nominal_principal_u + center_u - self.current_virtual_principal_u

        msg = FlowerDetection()
        msg.header = image_message.header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.class_name = detection["class_name"]
        msg.confidence = float(detection["confidence"])
        msg.center_u = center_u
        msg.center_v = float(detection["center_v"])
        msg.bbox_xmin = float(detection["xmin"])
        msg.bbox_ymin = float(detection["ymin"])
        msg.bbox_xmax = float(detection["xmax"])
        msg.bbox_ymax = float(detection["ymax"])
        msg.image_width = int(image_message.width)
        msg.image_height = int(image_message.height)
        self.detection_publisher.publish(msg)

    def _publish_annotated_image(self, frame, header) -> None:
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header = header
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.last_annotated_image_msg = image_msg
        self.annotated_publisher.publish(image_msg)

    def _republish_last_annotated_image(self) -> None:
        if self.last_annotated_image_msg is None:
            return
        self.last_annotated_image_msg.header.stamp = self.get_clock().now().to_msg()
        self.annotated_publisher.publish(self.last_annotated_image_msg)

    def _image_signature(self, frame) -> str:
        return hashlib.sha1(frame.tobytes()).hexdigest()

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _reload_runtime_config_if_needed(self) -> None:
        if self.runtime_config_path is None:
            return
        if not self.runtime_config_path.exists():
            return
        if not self.runtime_config_path.is_file():
            return

        mtime = self.runtime_config_path.stat().st_mtime
        if mtime <= self.runtime_config_mtime:
            return

        try:
            with self.runtime_config_path.open("r", encoding="utf-8") as file:
                data = json.load(file)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Failed to read runtime config: {exc}")
            return

        self.runtime_config_mtime = mtime
        if "confidence_threshold" in data:
            self.confidence_threshold = float(data["confidence_threshold"])
        if "image_size" in data:
            self.image_size = int(data["image_size"])

    def _reset_virtual_principal_for_image(self, image_message: Image) -> None:
        if not self.reset_virtual_principal_on_new_image:
            return

        if self.configured_virtual_principal_u > 0.0:
            self.nominal_principal_u = self.configured_virtual_principal_u
        else:
            self.nominal_principal_u = 0.5 * float(image_message.width)
        self.current_virtual_principal_u = self.nominal_principal_u

    def _recenter_to_completed_target(self) -> None:
        if not self.simulate_recenter_after_target or self.current_detection is None:
            return

        self.current_virtual_principal_u = float(self.current_detection["center_u"])


def main() -> None:
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
