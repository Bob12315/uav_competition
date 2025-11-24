from __future__ import annotations

import time
from logging import Logger

import cv2
import numpy as np

from app.core.models import VisionResult


class VisionSystem:
    """
    占位视觉模块：
    - 默认打开 /dev/video0
    - 不做真实检测，只输出 has_target=False
    - 你以后在这里接入自己的检测 / 跟踪
    """

    def __init__(self, device: str, logger: Logger) -> None:
        self.log = logger.getChild("Vision")
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.log.warning("Failed to open camera %s", device)

    def read(self) -> tuple[VisionResult, "cv2.Mat | None"]:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return VisionResult(has_target=False), None

        # TODO: 替换为真正的检测算法
        vr = VisionResult(has_target=False, dx=0.0, dy=0.0, confidence=0.0)
        return vr, frame

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()


class FakeVisionSystem:
    """
    纯软件 Demo 用的假视觉：
    - 640x480 暗底
    - 每 10 秒出现 5 秒目标，dx/dy 逐步逼近中心
    """

    def __init__(self, logger: Logger) -> None:
        self.log = logger.getChild("FakeVision")
        self._start_time = time.time()
        self._dx = 150.0
        self._dy = -100.0

    def read(self) -> tuple[VisionResult, np.ndarray]:
        now = time.time() - self._start_time
        h, w = 480, 640
        frame = np.full((h, w, 3), 40, dtype=np.uint8)

        phase = now % 10.0
        has_target = 3.0 <= phase <= 8.0

        if has_target:
            self._dx *= 0.9
            self._dy *= 0.9
        else:
            self._dx = 150.0
            self._dy = -100.0

        cx, cy = w // 2, h // 2
        tx = int(cx + self._dx)
        ty = int(cy + self._dy)

        cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        if has_target:
            cv2.circle(frame, (tx, ty), 15, (0, 0, 255), 2)

        vr = VisionResult(
            has_target=has_target,
            dx=float(self._dx),
            dy=float(self._dy),
            confidence=0.9 if has_target else 0.0,
        )
        return vr, frame
