# AlignController placeholder
from __future__ import annotations

from logging import Logger

from app.config import AlignConfig
from app.core.models import ControlCommand, VisionResult


class AlignController:
    def __init__(self, cfg: AlignConfig, logger: Logger) -> None:
        self.cfg = cfg
        self.log = logger.getChild("Align")

    def compute(self, vision: VisionResult) -> ControlCommand:
        """
        简单 P 控制：
        - dx, dy -> gimbal 微调 + 机体 yaw_rate
        这里纯示例，你可以按自己的需要复杂化。
        """
        cmd = ControlCommand()
        if not vision.has_target:
            return cmd

        dx, dy = vision.dx, vision.dy

        # 云台微调
        cmd.gimbal_yaw = -self.cfg.kx * dx
        cmd.gimbal_pitch = -self.cfg.ky * dy

        # 简单用 dx 控制 yaw_rate（示意）
        cmd.yaw_rate = -self.cfg.kx * dx
        return cmd

    def is_aligned(self, vision: VisionResult) -> bool:
        if not vision.has_target:
            return False
        return (
            abs(vision.dx) < self.cfg.dx_threshold
            and abs(vision.dy) < self.cfg.dy_threshold
        )
