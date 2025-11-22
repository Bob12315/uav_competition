# RcInput wrapper placeholder
from __future__ import annotations

from logging import Logger

from app.config import RcSwitchConfig
from .fc_client import FcClient


class RcInput:
    """
    简单把 PWM 范围映射到逻辑状态：
    - mode: MANUAL / SEMI / AUTO
    - drop: OFF / ON
    """

    def __init__(self, fc: FcClient, cfg: RcSwitchConfig, logger: Logger) -> None:
        self.fc = fc
        self.cfg = cfg
        self.log = logger.getChild("RcInput")

    def get_mode(self) -> str:
        pwm = self.fc.read_rc_channels().get(self.cfg.mode_channel, 1500)
        if pwm < 1300:
            return "MANUAL"
        elif pwm > 1700:
            return "AUTO"
        return "SEMI"

    def get_drop_switch(self) -> str:
        pwm = self.fc.read_rc_channels().get(self.cfg.drop_channel, 1000)
        return "ON" if pwm > 1700 else "OFF"
