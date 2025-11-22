# DropperController placeholder
from __future__ import annotations

import time
from logging import Logger

from app.config import DropperConfig
from .fc_client import FcClient


class DropperController:
    def __init__(self, fc: FcClient, cfg: DropperConfig, logger: Logger) -> None:
        self.fc = fc
        self.cfg = cfg
        self.log = logger.getChild("Dropper")
        self._last_drop_time: float | None = None
        # 初始化为安全位置
        self.fc.send_servo(self.cfg.channel, self.cfg.pwm_safe)

    def drop_once(self) -> None:
        """简单阻塞式投弹，比赛中可改为非阻塞状态机。"""
        self.log.info("Dropping payload...")
        self.fc.send_servo(self.cfg.channel, self.cfg.pwm_drop)
        time.sleep(self.cfg.pulse_ms / 1000.0)
        self.fc.send_servo(self.cfg.channel, self.cfg.pwm_safe)
        self._last_drop_time = time.time()

    def is_ready(self) -> bool:
        # 示例：0.5 秒内不允许再次投
        if self._last_drop_time is None:
            return True
        return (time.time() - self._last_drop_time) > 0.5
