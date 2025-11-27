from __future__ import annotations

import os
import time

import cv2

from app.config import load_config
from app.io.fc_client import FakeFcClient, FcClient, FcClientConfig
from app.logging_config import setup_logging
from app.mission.align_controller import AlignController
from app.mission.mission_manager import MissionManager
from app.vision.hud import draw_hud, init_fb_writer
from app.vision.system import VisionSystem


def main() -> None:
    cfg = load_config()
    logger = setup_logging(cfg)
    if cfg.fc.use_fake:
        logger.info("Starting UAV loop with FakeFcClient (no real hardware).")
        fc = FakeFcClient(logger)
    else:
        logger.info("Starting UAV loop with FC endpoint %s", cfg.fc.endpoint)
        fc = FcClient(
            FcClientConfig(endpoint=cfg.fc.endpoint, baud=cfg.fc.baud),
            logger,
        )
        fc.connect()

    device = int(cfg.vision.device) if str(cfg.vision.device).isdigit() else cfg.vision.device
    vision = VisionSystem(
        device=device,
        logger=logger,
        width=cfg.vision.width,
        height=cfg.vision.height,
        pixel_format=cfg.vision.pixel_format,
        fps=cfg.vision.fps,
        aruco_dict=cfg.vision.aruco_dict,
        marker_length_m=cfg.vision.marker_length_m,
        camera_matrix=cfg.vision.camera_matrix,
        dist_coeffs=cfg.vision.dist_coeffs,
    )
    align = AlignController(cfg.align, logger)
    mission = MissionManager(align=align, logger=logger, cfg=cfg)

    use_fbdev = bool(int(os.environ.get("USE_FB", "1")))
    fbdev_path = os.environ.get("FBDEV", "/dev/fb0")
    fb_writer = None
    fb_attempted = False

    loop_hz = cfg.flight.loop_hz
    dt = 1.0 / loop_hz

    try:
        while True:
            t0 = time.time()

            telem = fc.read_telemetry()
            vr, frame = vision.read()
            aligned = align.is_aligned(vr) if vr.has_target else False

            cmd, state = mission.update(telem, vr)

            if use_fbdev and frame is not None:
                hud = draw_hud(frame, vr, state, aligned, telem=telem)
                if fb_writer is None and not fb_attempted:
                    fb_writer = init_fb_writer(
                        fbdev_path=fbdev_path,
                        width=hud.shape[1],
                        height=hud.shape[0],
                        fps=cfg.vision.fps or cfg.flight.loop_hz,
                        logger=logger,
                    )
                    fb_attempted = True
                if fb_writer is not None:
                    try:
                        fb_writer.write(hud)
                    except cv2.error as exc:
                        logger.warning("fbdev 写入失败（%s），关闭 fbdev 输出。", exc)
                        fb_writer.release()
                        fb_writer = None

            fc.send_control(cmd)

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        logger.info("Interrupted by user (Ctrl+C), exiting main loop.")
    finally:
        if hasattr(vision, "release"):
            vision.release()
        if fb_writer is not None:
            fb_writer.release()
        logger.info("Exit.")


if __name__ == "__main__":
    main()
