# GimbalController placeholder
from __future__ import annotations

from logging import Logger

from app.config import GimbalConfig
from .fc_client import FcClient


class GimbalController:
    def __init__(self, fc: FcClient, cfg: GimbalConfig, logger: Logger) -> None:
        self.fc = fc
        self.cfg = cfg
        self.log = logger.getChild("Gimbal")
        self._pitch = cfg.pitch_scan
        self._yaw = cfg.yaw_scan

    def set_angles(self, pitch_deg: float, yaw_deg: float) -> None:
        self._pitch = pitch_deg
        self._yaw = yaw_deg
        self.fc.send_gimbal_angles(pitch_deg, yaw_deg)

    def nudge(self, d_pitch: float, d_yaw: float) -> None:
        self.set_angles(self._pitch + d_pitch, self._yaw + d_yaw)

    def set_scan_pose(self) -> None:
        """斜下视搜索姿态"""
        self.set_angles(self.cfg.pitch_scan, self.cfg.yaw_scan)

    @property
    def angles(self) -> tuple[float, float]:
        return self._pitch, self._yaw
