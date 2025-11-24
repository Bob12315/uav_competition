# HudRenderer placeholder
from __future__ import annotations

import cv2
from logging import Logger

from app.core.models import HudData


class HudRenderer:
    def __init__(self, logger: Logger) -> None:
        self.log = logger.getChild("HUD")

    def render(self, frame, hud: HudData):
        if frame is None:
            return None

        h, w = frame.shape[:2]

        # 中心十字
        cv2.drawMarker(
            frame,
            (w // 2, h // 2),
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=1,
        )

        # 文本信息
        t = hud.telemetry
        v = hud.vision
        text_lines = [
            f"MODE: {t.mode}  STATE: {hud.mission_state}",
            f"ALT: {t.alt:.1f}m  V: {t.voltage:.1f}V  SATS: {t.sats}",
            f"TARGET: {'YES' if v.has_target else 'NO'}  dx={v.dx:.1f} dy={v.dy:.1f} conf={v.confidence:.2f}",
        ]

        if v.tvec is not None:
            x, y, z = v.tvec
            text_lines.append(f"Tvec(m): x={x:.3f} y={y:.3f} z={z:.3f}")
        if v.rvec is not None:
            rx, ry, rz = v.rvec
            text_lines.append(f"Rvec(rad): rx={rx:.3f} ry={ry:.3f} rz={rz:.3f}")

        y = 20
        for line in text_lines:
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            y += 18

        return frame
