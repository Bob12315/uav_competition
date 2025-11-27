from __future__ import annotations

from logging import Logger
from typing import Dict

from app.config import RcSwitchConfig
from app.io.fc_client import FcClient


class RcInput:
    """Parse RC switch states from MAVLink RC channels."""

    def __init__(self, fc: FcClient, cfg: RcSwitchConfig, logger: Logger) -> None:
        self.fc = fc
        self.cfg = cfg
        self.log = logger.getChild("RcInput")
        self._last_rc: Dict[int, int] = {}

    def _read(self) -> Dict[int, int]:
        self._last_rc = self.fc.read_rc_channels()
        return self._last_rc

    def get_mode_switch(self) -> str:
        rc = self._read()
        pwm = rc.get(self.cfg.mode_channel, 0)
        if pwm > 1700:
            return "AUTO"
        if pwm > 1300:
            return "SEMI"
        return "MANUAL"

    def get_drop_switch(self) -> str:
        rc = self._read()
        pwm = rc.get(self.cfg.drop_channel, 0)
        return "ON" if pwm > 1700 else "OFF"
