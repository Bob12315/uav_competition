#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def _build_camera_info(width: int, height: int, camera_name: str) -> CameraInfo:
    """
    Build a minimal CameraInfo with rough intrinsics so cameracalibrator can run
    even without existing calibration.
    """
    info = CameraInfo()
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    # Rough focal guess: pixels ~ 0.9 * larger dimension
    fx = fy = 0.9 * float(max(width, height))
    cx = width / 2.0
    cy = height / 2.0
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.header.frame_id = f"{camera_name}_optical_frame"
    return info


class VideoPublisher(Node):
    def __init__(
        self,
        device: str | int,
        width: int,
        height: int,
        fps: float,
        camera_name: str,
        namespace: str,
    ) -> None:
        super().__init__("video_publisher")
        self.namespace = namespace.rstrip("/")
        self.frame_id = f"{camera_name}_optical_frame"
        self.info = _build_camera_info(width, height, camera_name)

        # Publishers
        self.pub_img = self.create_publisher(Image, f"{self.namespace}/image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, f"{self.namespace}/camera_info", 10)

        # Camera
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {device}")

        self.get_logger().info(
            f"Camera {device} opened at {width}x{height} {fps:.1f} FPS, publishing to {self.namespace}/image_raw"
        )

        self.timer = self.create_timer(1.0 / fps, self._on_timer)

    def _on_timer(self) -> None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning("Camera read failed")
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()

        self.info.header.stamp = msg.header.stamp
        self.pub_img.publish(msg)
        self.pub_info.publish(self.info)

    def destroy_node(self) -> bool:
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simple ROS2 video publisher for calibration/testing.")
    parser.add_argument("--device", default="/dev/video2", help="Camera device path or index (default: /dev/video2)")
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
    parser.add_argument("--fps", type=float, default=30.0, help="Publish FPS (default: 30)")
    parser.add_argument("--camera-name", default="my_camera", help="Camera name for CameraInfo/frame_id (default: my_camera)")
    parser.add_argument(
        "--namespace",
        default="/my_camera",
        help="ROS namespace for topics (image_raw, camera_info) (default: /my_camera)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    try:
        node = VideoPublisher(
            device=args.device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            camera_name=args.camera_name,
            namespace=args.namespace,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Failed to start video publisher: {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
