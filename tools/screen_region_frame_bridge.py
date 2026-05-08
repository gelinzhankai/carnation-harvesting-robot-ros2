from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import ImageGrab


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture a Windows screen region and write the latest frame for ROS2 WSL use."
    )
    parser.add_argument("--output", default=r"H:\carnation_detection\ros_screen_region_frame.jpg")
    parser.add_argument("--config", default=r"H:\carnation_detection\ros_screen_region_config.json")
    parser.add_argument("--width", type=int, default=1000)
    parser.add_argument("--height", type=int, default=1000)
    parser.add_argument("--right-margin", type=int, default=0)
    parser.add_argument("--bottom-margin", type=int, default=0)
    parser.add_argument("--left", type=int, default=None)
    parser.add_argument("--top", type=int, default=None)
    parser.add_argument("--right", type=int, default=None)
    parser.add_argument("--bottom", type=int, default=None)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--quality", type=int, default=90)
    parser.add_argument("--mode", choices=["continuous", "triggered"], default="triggered")
    parser.add_argument("--trigger-file", default=r"H:\carnation_detection\ros_screen_region_trigger.txt")
    parser.add_argument("--poll-sec", type=float, default=0.1)
    return parser.parse_args()


def compute_bbox(width: int, height: int, right_margin: int, bottom_margin: int) -> tuple[int, int, int, int]:
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive.")
    if right_margin < 0 or bottom_margin < 0:
        raise ValueError("margins must be zero or positive.")

    screen = ImageGrab.grab()
    screen_width, screen_height = screen.size
    right = screen_width - right_margin
    bottom = screen_height - bottom_margin
    left = max(0, right - width)
    top = max(0, bottom - height)
    return (left, top, right, bottom)


def compute_explicit_bbox(
    left: int | None,
    top: int | None,
    right: int | None,
    bottom: int | None,
    width: int,
    height: int,
    right_margin: int,
    bottom_margin: int,
) -> tuple[int, int, int, int]:
    if None not in (left, top, right, bottom):
        if right <= left or bottom <= top:
            raise ValueError("explicit region must satisfy right > left and bottom > top.")
        return (left, top, right, bottom)

    return compute_bbox(width, height, right_margin, bottom_margin)


def load_region_config(path: str) -> dict:
    config_path = Path(path)
    if not path or not config_path.exists():
        return {}

    with config_path.open("r", encoding="utf-8") as file:
        data = json.load(file)
    return data if isinstance(data, dict) else {}


def resolve_bbox(args: argparse.Namespace) -> tuple[int, int, int, int]:
    region_config = load_region_config(args.config)
    return compute_explicit_bbox(
        region_config.get("left", args.left),
        region_config.get("top", args.top),
        region_config.get("right", args.right),
        region_config.get("bottom", args.bottom),
        args.width,
        args.height,
        args.right_margin,
        args.bottom_margin,
    )


def main() -> int:
    args = parse_args()
    output_path = Path(args.output)
    trigger_path = Path(args.trigger_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    trigger_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = output_path.with_name(f"{output_path.stem}.tmp{output_path.suffix}")
    bbox = resolve_bbox(args)
    frame_period = 1.0 / max(args.fps, 0.1)

    print(f"Capture region: left={bbox[0]}, top={bbox[1]}, right={bbox[2]}, bottom={bbox[3]}")
    print(f"Writing latest frame to: {output_path}")
    if args.mode == "triggered":
        print(f"Waiting for trigger file: {trigger_path}")
    print("Press Ctrl+C to stop.")
    capture_once(bbox, temp_path, output_path, args.quality)

    last_trigger_mtime = trigger_path.stat().st_mtime if trigger_path.exists() else 0.0
    while True:
        started = time.perf_counter()
        if args.mode == "continuous":
            bbox = resolve_bbox(args)
            capture_once(bbox, temp_path, output_path, args.quality)
            sleep_seconds = max(0.0, frame_period - (time.perf_counter() - started))
        else:
            if trigger_path.exists():
                trigger_mtime = trigger_path.stat().st_mtime
                if trigger_mtime > last_trigger_mtime:
                    last_trigger_mtime = trigger_mtime
                    bbox = resolve_bbox(args)
                    capture_once(bbox, temp_path, output_path, args.quality)
                    print(
                        f"Triggered capture: left={bbox[0]}, top={bbox[1]}, "
                        f"right={bbox[2]}, bottom={bbox[3]} -> {output_path}",
                        flush=True,
                    )
            sleep_seconds = max(args.poll_sec, 0.02)

        time.sleep(sleep_seconds)


def capture_once(
    bbox: tuple[int, int, int, int],
    temp_path: Path,
    output_path: Path,
    quality: int,
) -> None:
    image = ImageGrab.grab(bbox=bbox)
    frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    ok = cv2.imwrite(str(temp_path), frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if ok:
        temp_path.replace(output_path)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
