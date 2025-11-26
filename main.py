from __future__ import annotations

import os
import time
from typing import Optional

import cv2
import numpy as np

from app.config import load_config
from app.io.fc_client import FakeFcClient
from app.logging_config import setup_logging
from app.mission.align_controller import AlignController
from app.mission.mission_manager import MissionManager
from app.vision.system import VisionSystem


def _has_gstreamer() -> bool:
    try:
        info = cv2.getBuildInformation()
        return ("GStreamer" in info) and ("YES" in info.split("GStreamer")[-1][:60])
    except Exception:
        return False


def _init_fb_writer(fbdev_path: str, width: int, height: int, fps: int, logger):
    if not _has_gstreamer():
        logger.warning("OpenCV 未启用 GStreamer，无法输出到 fbdev。")
        return None
    if not os.path.exists(fbdev_path):
        logger.warning("未找到 fbdev 设备 %s，无法输出到 fbdev。", fbdev_path)
        return None

    pipeline = (
        "appsrc is-live=true block=true format=time ! "
        "videoconvert ! videoscale ! "
        f"video/x-raw,format=BGRx,width={width},height={height},framerate={fps}/1 ! "
        f"fbdevsink device={fbdev_path} sync=false"
    )
    writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(fps), (width, height), True)
    if writer.isOpened():
        logger.info("fbdev 输出启用：%s (%dx%d @ %dfps)", fbdev_path, width, height, fps)
        return writer

    logger.warning("fbdevsink 打开失败（%s），关闭 fbdev 输出。", fbdev_path)
    return None


def _draw_hud(frame, vision, state: str, aligned: Optional[bool], telem=None, panel_width: int = 260):
    img = frame.copy()
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    thickness = max(1, min(w, h) // 300)

    cv2.line(img, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), thickness, cv2.LINE_AA)
    cv2.line(img, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), thickness, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), thickness + 1, (0, 255, 0), -1, cv2.LINE_AA)

    if vision.has_target:
        ex, ey = int(cx + vision.dx), int(cy + vision.dy)
        cv2.arrowedLine(img, (cx, cy), (ex, ey), (0, 255, 255), thickness + 1, tipLength=0.18)

    panel = np.full((h, panel_width, 3), 20, dtype=np.uint8)
    y = 26

    def put(text: str):
        nonlocal y
        cv2.putText(panel, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
        y += 26

    put(f"State: {state}")
    put(f"dx={vision.dx:.1f}px dy={vision.dy:.1f}px")
    put(f"conf={vision.confidence:.2f}")
    if aligned is not None:
        put(f"Aligned: {'YES' if aligned else 'NO'}")
    if vision.tvec is not None:
        tx, ty, tz = vision.tvec
        put(f"tvec x={tx:.2f}m")
        put(f"     y={ty:.2f}m")
        put(f"     z={tz:.2f}m")
    if telem is not None:
        put(f"Voltage: {telem.voltage:.2f}V  Sats: {telem.sats}")
        put(f"Alt: {telem.alt:.1f}m  Yaw: {telem.yaw:.1f}")
        put(f"Mode: {telem.mode}  Armed: {'Y' if telem.armed else 'N'}")
        put(f"Vel: vx={telem.vx:.2f} vy={telem.vy:.2f}")
        put(f"Lat: {telem.lat:.6f}")
        put(f"Lon: {telem.lon:.6f}")

    composite = np.concatenate([img, panel], axis=1)
    return composite


def main() -> None:
    cfg = load_config()
    logger = setup_logging(cfg)
    logger.info("Starting fake UAV competition loop (no real hardware).")

    fc = FakeFcClient(logger)

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
                hud = _draw_hud(frame, vr, state, aligned)
                if fb_writer is None and not fb_attempted:
                    fb_writer = _init_fb_writer(
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
