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
        - 优先使用 ArUco 姿态的 tvec (x 右, y 下, z 前) 做平移对齐
        - 若 tvec 不可用，则回退 dx/dy 像素误差 -> 云台/yaw 微调
        """
        cmd = ControlCommand()
        if not vision.has_target:
            return cmd

        # 1) 如果有 tvec，直接用米级误差做平移控制（假设机体坐标 ~ 相机坐标）
        if vision.tvec is not None:
            x, y, _ = vision.tvec
            cmd.vx = -self.cfg.kx_pos * x  # 向右为正，反向纠偏
            cmd.vy = -self.cfg.ky_pos * y  # 向下为正，反向纠偏

        # 2) 仍然使用像素误差微调云台/yaw，便于目标居中
        dx, dy = vision.dx, vision.dy
        cmd.gimbal_yaw = -self.cfg.kx * dx
        cmd.gimbal_pitch = -self.cfg.ky * dy
        cmd.yaw_rate = -self.cfg.kx * dx
        return cmd

    def is_aligned(self, vision: VisionResult) -> bool:
        if not vision.has_target:
            return False

        if vision.tvec is not None:
            x, y, _ = vision.tvec
            if abs(x) > self.cfg.x_threshold_m or abs(y) > self.cfg.y_threshold_m:
                return False

        return (
            abs(vision.dx) < self.cfg.dx_threshold
            and abs(vision.dy) < self.cfg.dy_threshold
        )
