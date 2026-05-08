from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import ImageGrab
from ultralytics import YOLO


WINDOW_NAME = "YOLOv8m2 Screen Region Detection"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLOv8m2 detection on a Windows screen region and write ROS bridge config."
    )
    parser.add_argument(
        "--model",
        default=r"H:\carnation_detection\carnation_yolov8m2_best.pt",
        help="Path to the trained YOLOv8m2 .pt weight file.",
    )
    parser.add_argument(
        "--config",
        default=r"H:\carnation_detection\ros_screen_region_config.json",
        help="Path to the runtime config JSON read by the ROS frame bridge.",
    )
    return parser.parse_args()


def read_int(prompt: str, default: int) -> int:
    raw = input(f"{prompt} [default {default}]: ").strip()
    if not raw:
        return default
    return int(raw)


def read_float(prompt: str, default: float) -> float:
    raw = input(f"{prompt} [default {default}]: ").strip()
    if not raw:
        return default
    return float(raw)


def write_runtime_config(
    path: Path,
    left: int,
    top: int,
    right: int,
    bottom: int,
    confidence_threshold: float,
    image_size: int,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "left": left,
                "top": top,
                "right": right,
                "bottom": bottom,
                "confidence_threshold": confidence_threshold,
                "image_size": image_size,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    model_path = Path(args.model)
    config_path = Path(args.config)

    if not model_path.exists():
        print(f"Model checkpoint not found: {model_path}")
        return 1

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    screen = ImageGrab.grab()
    screen_width, screen_height = screen.size

    print("Capture region is anchored to the bottom-right corner of the screen.")
    width = read_int("Capture width", 1000)
    height = read_int("Capture height", 1000)
    right_margin = read_int("Right margin", 0)
    bottom_margin = read_int("Bottom margin", 0)
    conf = read_float("Confidence threshold", 0.50)
    imgsz = read_int("Image size", 640)

    if width <= 0 or height <= 0:
        print("Width and height must be positive.")
        return 1
    if right_margin < 0 or bottom_margin < 0:
        print("Margins must be zero or positive.")
        return 1

    right = screen_width - right_margin
    bottom = screen_height - bottom_margin
    left = max(0, right - width)
    top = max(0, bottom - height)
    bbox = (left, top, right, bottom)

    write_runtime_config(config_path, left, top, right, bottom, conf, imgsz)
    model = YOLO(str(model_path))

    print()
    print(f"Model: {model_path}")
    print(f"Screen size: {screen_width}x{screen_height}")
    print(f"Capture region: left={left}, top={top}, right={right}, bottom={bottom}")
    print(f"ROS config written to: {config_path}")
    print("Press q to quit.")
    print()

    prev_ticks = cv2.getTickCount()

    while True:
        image = ImageGrab.grab(bbox=bbox)
        frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

        results = model.predict(frame, conf=conf, imgsz=imgsz, verbose=False)
        plotted = results[0].plot()

        current_ticks = cv2.getTickCount()
        fps = cv2.getTickFrequency() / max(current_ticks - prev_ticks, 1)
        prev_ticks = current_ticks

        cv2.putText(
            plotted,
            f"Region: ({left}, {top}) - ({right}, {bottom})",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            plotted,
            f"FPS: {fps:.1f}",
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(WINDOW_NAME, plotted)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        raise SystemExit(0)
