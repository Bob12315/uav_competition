# Sensors wrapper placeholder
from __future__ import annotations

from logging import Logger

from app.core.models import TelemetrySnapshot
from .fc_client import FcClient


class Sensors:
    def __init__(self, fc: FcClient, logger: Logger) -> None:
        self.fc = fc
        self.log = logger.getChild("Sensors")

    def read(self) -> TelemetrySnapshot:
        raw = self.fc.read_telemetry_raw()
        ts = TelemetrySnapshot(
            time=raw["time"],
            mode=raw["mode"],
            armed=raw["armed"],
            voltage=raw["voltage"],
            alt=raw["alt"],
            vx=raw["vx"],
            vy=raw["vy"],
            yaw=raw["yaw"],
            lat=raw["lat"],
            lon=raw["lon"],
            sats=raw["sats"],
        )
        return ts
