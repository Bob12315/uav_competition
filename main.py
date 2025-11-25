from __future__ import annotations

import os
import sys
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


def _can_display() -> bool:
    """粗略判断是否有图形环境（headless 时避免 GTK 崩溃）。"""
    if sys.platform.startswith("win"):
        return True
    if sys.platform == "darwin":
        return True
    # Linux: 需要 DISPLAY/Wayland/MIR 之一
    return bool(
        os.environ.get("DISPLAY")
        or os.environ.get("WAYLAND_DISPLAY")
        or os.environ.get("MIR_SOCKET")
    )


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
    use_fbdev = bool(cfg.hud.use_fbdev)
    hud = HudRenderer(
        logger,
        use_fbdev=use_fbdev,
        fbdev_path=cfg.hud.fbdev_path,
        fps=cfg.flight.loop_hz,
    )
    display_enabled = bool(cfg.hud.display) and not use_fbdev
    if display_enabled and not _can_display():
        logger.warning("No GUI environment detected; disabling HUD display (headless).")
        display_enabled = False
    if use_fbdev and cfg.hud.display:
        logger.info("fbdev 已启用，已自动关闭窗口显示（只输出到 fbdev）。")

    display_failed = False

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
            if display_enabled and out is not None and not display_failed:
                try:
                    cv2.imshow("UAV Fake HUD", out)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC
                        break
                except cv2.error as exc:
                    display_failed = True
                    logger.warning("cv2.imshow failed (%s); disabling display for headless run.", exc)

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        logger.info("Interrupted by user (Ctrl+C), exiting main loop.")
    finally:
        if hasattr(vision, "release"):
            vision.release()
        if display_enabled and not display_failed:
            cv2.destroyAllWindows()
        hud.release()
        logger.info("Exit.")


if __name__ == "__main__":
    main()
