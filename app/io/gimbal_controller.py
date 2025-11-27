from __future__ import annotations

from logging import Logger

from app.config import GimbalConfig
from app.io.fc_client import FcClient


class GimbalController:
    """Control gimbal pitch/yaw via MAVLink mount commands."""

    def __init__(self, fc: FcClient, cfg: GimbalConfig, logger: Logger) -> None:
        self.fc = fc
        self.cfg = cfg
        self.log = logger.getChild("Gimbal")
        self.pitch = 0.0
        self.yaw = 0.0

    def set_angles(self, pitch_deg: float, yaw_deg: float) -> None:
        self.pitch = pitch_deg
        self.yaw = yaw_deg
        self.fc.send_gimbal_angles(pitch_deg, yaw_deg)

    def nudge(self, d_pitch: float, d_yaw: float) -> None:
        self.set_angles(self.pitch + d_pitch, self.yaw + d_yaw)

    def set_scan_pose(self) -> None:
        self.set_angles(self.cfg.pitch_scan, self.cfg.yaw_scan)
