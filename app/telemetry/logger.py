# TelemetryLogger placeholder
from __future__ import annotations

import csv
from pathlib import Path
from logging import Logger

from app.core.models import TelemetrySnapshot, VisionResult, ControlCommand


class TelemetryLogger:
    def __init__(self, path: Path, logger: Logger) -> None:
        self.log = logger.getChild("TLog")
        self.path = path
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                "time",
                "alt",
                "voltage",
                "vx",
                "vy",
                "yaw",
                "lat",
                "lon",
                "sats",
                "has_target",
                "dx",
                "dy",
                "conf",
                "vx_cmd",
                "vy_cmd",
                "yaw_rate",
                "drop",
            ]
        )

    def log_step(
        self,
        telem: TelemetrySnapshot,
        vis: VisionResult,
        cmd: ControlCommand,
    ) -> None:
        self._writer.writerow(
            [
                telem.time,
                telem.alt,
                telem.voltage,
                telem.vx,
                telem.vy,
                telem.yaw,
                telem.lat,
                telem.lon,
                telem.sats,
                int(vis.has_target),
                vis.dx,
                vis.dy,
                vis.confidence,
                cmd.vx,
                cmd.vy,
                cmd.yaw_rate,
                int(cmd.drop),
            ]
        )

    def close(self) -> None:
        self._file.close()
