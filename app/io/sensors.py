from __future__ import annotations

from logging import Logger

from app.core.models import TelemetrySnapshot
from app.io.fc_client import FcClient


class Sensors:
    """Read telemetry snapshot from FC client."""

    def __init__(self, fc: FcClient, logger: Logger) -> None:
        self.fc = fc
        self.log = logger.getChild("Sensors")

    def read(self) -> TelemetrySnapshot:
        return self.fc.read_telemetry()
