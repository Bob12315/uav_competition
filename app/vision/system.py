from __future__ import annotations

import cv2
from logging import Logger

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
