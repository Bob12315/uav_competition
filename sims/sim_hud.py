# sim_hud placeholder
from __future__ import annotations

import cv2
import numpy as np

from app.config import load_config
from app.logging_config import setup_logging
from app.core.models import TelemetrySnapshot, VisionResult, HudData
from app.hud.renderer import HudRenderer


def main():
    cfg = load_config()
    logger = setup_logging(cfg)

    hud = HudRenderer(logger)

    # 白底图代替相机画面
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)

    telem = TelemetrySnapshot(
        time=0.0,
        mode="GUIDED",
        armed=True,
        voltage=15.3,
        alt=12.5,
        yaw=90.0,
        sats=12,
    )
    vis = VisionResult(has_target=True, dx=15.0, dy=-10.0, confidence=0.9)
    hd = HudData(telemetry=telem, vision=vis, mission_state="ALIGN")

    while True:
        f = frame.copy()
        f = hud.render(f, hd)
        cv2.imshow("sim_hud", f)
        if cv2.waitKey(30) & 0xFF == 27:
            break


if __name__ == "__main__":
    main()
