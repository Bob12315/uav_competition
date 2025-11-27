from __future__ import annotations

import time
from logging import Logger

from app.config import DropperConfig
from app.io.fc_client import FcClient


class DropperController:
    """Operate dropper servo via MAVLink servo command."""

    def __init__(self, fc: FcClient, cfg: DropperConfig, logger: Logger) -> None:
        self.fc = fc
        self.cfg = cfg
        self.log = logger.getChild("Dropper")
        self._last_drop_ts: float | None = None

    def is_ready(self) -> bool:
        if self._last_drop_ts is None:
            return True
        return (time.time() - self._last_drop_ts) > (self.cfg.pulse_ms / 1000.0)

    def drop_once(self) -> None:
        self.log.info("Dropper: sending drop pulse (ch=%d pwm=%d)", self.cfg.channel, self.cfg.pwm_drop)
        self.fc.send_servo(self.cfg.channel, self.cfg.pwm_drop)
        time.sleep(self.cfg.pulse_ms / 1000.0)
        self.fc.send_servo(self.cfg.channel, self.cfg.pwm_safe)
        self._last_drop_ts = time.time()
