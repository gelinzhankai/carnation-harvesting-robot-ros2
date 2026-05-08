#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, Tuple

import cv2
import numpy as np
from ultralytics import YOLO

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is expected in the current ROS env.
    yaml = None


def load_stereo_params(config_path: Path) -> Dict[str, Any]:
    defaults = {
        "fx": 1000.0,
        "fy": 1000.0,
        "cx": 640.0,
        "cy": 360.0,
        "baseline_m": 0.06,
        "world_translation_xyz": [0.0, 0.11, 1.20],
        "world_rotation_matrix": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0],
        "target_offset_xyz": [0.0, 0.0, 0.0],
        "clamp_to_workspace": False,
        "workspace_min_xyz": [-3.5, 0.39, 0.82],
        "workspace_max_xyz": [3.5, 0.73, 1.05],
    }

    if yaml is None or not config_path.is_file():
        return defaults

    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    params = data.get("stereo_pixel_to_world_node", {}).get("ros__parameters", {})
    defaults.update(params)
    return defaults


def make_stereo_crops(image: np.ndarray, shift_px: int) -> Tuple[np.ndarray, np.ndarray]:
    height, width = image.shape[:2]
    if shift_px <= 0 or shift_px >= width // 2:
        raise ValueError(f"shift_px must be in (0, {width // 2}), got {shift_px}")

    crop_width = width - shift_px
    left = image[:, 0:crop_width]
    right = image[:, shift_px:shift_px + crop_width]
    return left, right


def detect_best_flower(
    model: YOLO,
    image: np.ndarray,
    target_class_name: str,
    confidence_threshold: float,
    image_size: int,
) -> Tuple[Dict[str, float], np.ndarray]:
    results = model.predict(source=image, imgsz=image_size, verbose=False)
    if not results:
        raise RuntimeError("YOLO returned no result.")

    result = results[0]
    names = result.names if hasattr(result, "names") else model.names
    best = None
    best_confidence = -1.0

    if result.boxes is None or len(result.boxes) == 0:
        raise RuntimeError("No detection boxes found.")

    for box in result.boxes:
        cls_id = int(box.cls[0].item())
        class_name = str(names.get(cls_id, cls_id))
        confidence = float(box.conf[0].item())

        if target_class_name and class_name != target_class_name:
            continue
        if confidence < confidence_threshold:
            continue

        xmin, ymin, xmax, ymax = [float(v) for v in box.xyxy[0].tolist()]
        center_u = 0.5 * (xmin + xmax)
        center_v = 0.5 * (ymin + ymax)

        if confidence > best_confidence:
            best_confidence = confidence
            best = {
                "class_name": class_name,
                "confidence": confidence,
                "xmin": xmin,
                "ymin": ymin,
                "xmax": xmax,
                "ymax": ymax,
                "center_u": center_u,
                "center_v": center_v,
            }

    if best is None:
        raise RuntimeError(
            f"No '{target_class_name}' detection above confidence {confidence_threshold:.2f}."
        )

    annotated = image.copy()
    cv2.rectangle(
        annotated,
        (int(best["xmin"]), int(best["ymin"])),
        (int(best["xmax"]), int(best["ymax"])),
        (0, 255, 0),
        2,
    )
    cv2.circle(annotated, (int(best["center_u"]), int(best["center_v"])), 5, (0, 0, 255), -1)
    cv2.putText(
        annotated,
        f'{best["class_name"]} {best["confidence"]:.2f}',
        (int(best["xmin"]), max(int(best["ymin"]) - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )
    return best, annotated


def compute_stereo_coordinates(
    left_detection: Dict[str, float],
    right_detection: Dict[str, float],
    params: Dict[str, Any],
) -> Tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    fx = float(params["fx"])
    fy = float(params["fy"])
    cx = float(params["cx"])
    cy = float(params["cy"])
    baseline_m = float(params["baseline_m"])

    u_left = float(left_detection["center_u"])
    v_left = float(left_detection["center_v"])
    u_right = float(right_detection["center_u"])
    disparity = u_left - u_right
    if disparity <= 0.0:
        raise RuntimeError(
            f"Invalid simulated disparity d={disparity:.3f}; expected u_left > u_right."
        )

    z_camera = fx * baseline_m / disparity
    x_camera = (u_left - cx) * z_camera / fx
    y_camera = (v_left - cy) * z_camera / fy
    camera_point = np.array([x_camera, y_camera, z_camera], dtype=float)

    rotation = np.array(params["world_rotation_matrix"], dtype=float).reshape(3, 3)
    translation = np.array(params["world_translation_xyz"], dtype=float).reshape(3)
    offset = np.array(params["target_offset_xyz"], dtype=float).reshape(3)
    raw_world_point = rotation @ camera_point + translation
    shaped_world_point = raw_world_point + offset

    if bool(params["clamp_to_workspace"]):
        workspace_min = np.array(params["workspace_min_xyz"], dtype=float).reshape(3)
        workspace_max = np.array(params["workspace_max_xyz"], dtype=float).reshape(3)
        shaped_world_point = np.minimum(np.maximum(shaped_world_point, workspace_min), workspace_max)

    return disparity, camera_point, raw_world_point, shaped_world_point


def write_result_file(
    output_path: Path,
    left_detection: Dict[str, float],
    right_detection: Dict[str, float],
    disparity: float,
    camera_point: np.ndarray,
    raw_world_point: np.ndarray,
    shaped_world_point: np.ndarray,
) -> None:
    lines = [
        "双目坐标转换演示结果",
        "",
        f"左目中心像素坐标: u_L={left_detection['center_u']:.3f}, v_L={left_detection['center_v']:.3f}",
        f"右目中心像素坐标: u_R={right_detection['center_u']:.3f}, v_R={right_detection['center_v']:.3f}",
        f"视差: d=u_L-u_R={disparity:.3f} px",
        "",
        "左相机坐标系下目标点:",
        f"X_c={camera_point[0]:.6f} m, Y_c={camera_point[1]:.6f} m, Z_c={camera_point[2]:.6f} m",
        "",
        "世界坐标系下原始目标点:",
        f"X_w={raw_world_point[0]:.6f} m, Y_w={raw_world_point[1]:.6f} m, Z_w={raw_world_point[2]:.6f} m",
        "",
        "最终输出世界坐标:",
        f"X={shaped_world_point[0]:.6f} m, Y={shaped_world_point[1]:.6f} m, Z={shaped_world_point[2]:.6f} m",
        "",
    ]
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a stereo localization demo from one image.")
    parser.add_argument("--image", default="/mnt/d/桌面/test1.jpg", help="Input image path.")
    parser.add_argument(
        "--model",
        default="/root/carnation_harvest/models/carnation_yolov8m2_best.pt",
        help="YOLO model path.",
    )
    parser.add_argument(
        "--config",
        default="/root/carnation_harvest/src/carnation_localization/config/stereo_pixel_to_world.yaml",
        help="Stereo localization config path.",
    )
    parser.add_argument("--output-dir", default="/root/carnation_harvest/demo_outputs/stereo_test1")
    parser.add_argument("--shift-px", type=int, default=60, help="Horizontal crop shift for fake stereo.")
    parser.add_argument("--target-class-name", default="carnation_flower")
    parser.add_argument("--confidence-threshold", type=float, default=0.25)
    parser.add_argument("--image-size", type=int, default=960)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    image_path = Path(args.image)
    model_path = Path(args.model)
    config_path = Path(args.config)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"Failed to read image: {image_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"YOLO model file not found: {model_path}")

    params = load_stereo_params(config_path)
    left_image, right_image = make_stereo_crops(image, args.shift_px)

    cv2.imwrite(str(output_dir / "left_crop.jpg"), left_image)
    cv2.imwrite(str(output_dir / "right_crop.jpg"), right_image)

    model = YOLO(str(model_path))
    left_detection, left_annotated = detect_best_flower(
        model,
        left_image,
        args.target_class_name,
        args.confidence_threshold,
        args.image_size,
    )
    right_detection, right_annotated = detect_best_flower(
        model,
        right_image,
        args.target_class_name,
        args.confidence_threshold,
        args.image_size,
    )

    disparity, camera_point, raw_world_point, shaped_world_point = compute_stereo_coordinates(
        left_detection,
        right_detection,
        params,
    )

    cv2.imwrite(str(output_dir / "left_detection.jpg"), left_annotated)
    cv2.imwrite(str(output_dir / "right_detection.jpg"), right_annotated)
    write_result_file(
        output_dir / "stereo_result.txt",
        left_detection,
        right_detection,
        disparity,
        camera_point,
        raw_world_point,
        shaped_world_point,
    )

    print("双目坐标转换演示结果")
    print(f"左目中心像素坐标: u_L={left_detection['center_u']:.3f}, v_L={left_detection['center_v']:.3f}")
    print(f"右目中心像素坐标: u_R={right_detection['center_u']:.3f}, v_R={right_detection['center_v']:.3f}")
    print(f"视差: d={disparity:.3f} px")
    print(
        "相机坐标: "
        f"X_c={camera_point[0]:.6f} m, Y_c={camera_point[1]:.6f} m, Z_c={camera_point[2]:.6f} m"
    )
    print(
        "世界坐标 raw: "
        f"X_w={raw_world_point[0]:.6f} m, Y_w={raw_world_point[1]:.6f} m, Z_w={raw_world_point[2]:.6f} m"
    )
    print(
        "最终世界坐标: "
        f"X={shaped_world_point[0]:.6f} m, Y={shaped_world_point[1]:.6f} m, Z={shaped_world_point[2]:.6f} m"
    )
    print(f"结果文件: {output_dir / 'stereo_result.txt'}")
    print(f"左目检测图: {output_dir / 'left_detection.jpg'}")
    print(f"右目检测图: {output_dir / 'right_detection.jpg'}")


if __name__ == "__main__":
    main()
