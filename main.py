from __future__ import annotations

import time

import cv2

from app.config import load_config
from app.core.models import HudData
from app.hud.renderer import HudRenderer
from app.io.fc_client import FakeFcClient
from app.logging_config import setup_logging
from app.mission.align_controller import AlignController
from app.mission.mission_manager import MissionManager
from app.vision.system import VisionSystem


def main() -> None:
    cfg = load_config()
    logger = setup_logging(cfg)
    logger.info("Starting fake UAV competition loop (no real hardware).")

    fc = FakeFcClient(logger)
    # 真相机：device/width/height/pixel_format 来自 config.yaml -> vision.*
    device = int(cfg.vision.device) if str(cfg.vision.device).isdigit() else cfg.vision.device
    vision = VisionSystem(
        device=device,
        logger=logger,
        width=cfg.vision.width,
        height=cfg.vision.height,
        pixel_format=cfg.vision.pixel_format,
        aruco_dict=cfg.vision.aruco_dict,
        marker_length_m=cfg.vision.marker_length_m,
        camera_matrix=cfg.vision.camera_matrix,
        dist_coeffs=cfg.vision.dist_coeffs,
    )
    align = AlignController(cfg.align, logger)
    mission = MissionManager(align=align, logger=logger, cfg=cfg)
    hud = HudRenderer(logger)

    loop_hz = cfg.flight.loop_hz
    dt = 1.0 / loop_hz

    try:
        while True:
            t0 = time.time()

            telem = fc.read_telemetry()
            vr, frame = vision.read()
            cmd, state = mission.update(telem, vr)

            fc.send_control(cmd)

            hud_data = HudData(telemetry=telem, vision=vr, mission_state=state)
            out = hud.render(frame, hud_data)
            if out is not None:
                cv2.imshow("UAV Fake HUD", out)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    finally:
        if hasattr(vision, "release"):
            vision.release()
        cv2.destroyAllWindows()
        logger.info("Exit.")


if __name__ == "__main__":
    main()
