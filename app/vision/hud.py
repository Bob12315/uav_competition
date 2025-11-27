from __future__ import annotations

import os
from logging import Logger
from typing import Optional

import cv2
import numpy as np

from app.core.models import TelemetrySnapshot, VisionResult


def has_gstreamer() -> bool:
    """
    Quick capability probe for OpenCV GStreamer support.
    """
    try:
        info = cv2.getBuildInformation()
    except Exception:  # noqa: BLE001
        return False
    return ("GStreamer" in info) and ("YES" in info.split("GStreamer")[-1][:60])


def init_fb_writer(
    fbdev_path: str, width: int, height: int, fps: int, logger: Logger
) -> cv2.VideoWriter | None:
    """
    Initialize a GStreamer fbdevsink writer if supported by the current OpenCV build.
    """
    if not has_gstreamer():
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


def draw_hud(
    frame: "cv2.Mat",
    vision: VisionResult,
    state: str,
    aligned: Optional[bool],
    telem: TelemetrySnapshot | None = None,
    panel_width: int = 260,
) -> np.ndarray:
    """
    Render HUD with reticle, target arrow, telemetry sidebar.
    """
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

    def put(text: str) -> None:
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
