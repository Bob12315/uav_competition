# HudRenderer placeholder
from __future__ import annotations

import os
import cv2
import numpy as np
from logging import Logger

from app.core.models import HudData


class HudRenderer:
    def __init__(self, logger: Logger, use_fbdev: bool = False, fbdev_path: str | None = None, fps: int = 30) -> None:
        self.log = logger.getChild("HUD")
        self.target_width = 640
        self.target_height = 480
        self.panel_width = 240
        self.output_width = self.target_width + self.panel_width
        self.output_height = self.target_height

        self.use_fbdev = use_fbdev
        self.fbdev_path = fbdev_path or "/dev/fb0"
        self.fps = fps
        self.fb_writer: cv2.VideoWriter | None = None

        if self.use_fbdev:
            self._init_fb_writer()

    def render(self, frame, hud: HudData):
        if frame is None:
            return None

        # 确保主画面为 640x480 居中显示
        resized = cv2.resize(frame, (self.target_width, self.target_height))
        h, w = resized.shape[:2]

        # 中心十字
        cv2.drawMarker(
            resized,
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

        # 右侧 OSD 面板
        panel = self._build_osd_panel(text_lines)

        # 拼接：摄像头画面居中 + 右侧 OSD
        out = cv2.hconcat([resized, panel])

        if self.fb_writer is not None:
            self.fb_writer.write(out)

        return out

    def _build_osd_panel(self, lines: list[str]):
        panel_height = self.target_height
        panel = 40 * np.ones((panel_height, self.panel_width, 3), dtype=np.uint8)
        x0, y = 10, 30
        for line in lines:
            cv2.putText(panel, line, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            y += 22
        return panel

    def _has_gstreamer(self) -> bool:
        try:
            info = cv2.getBuildInformation()
            return ("GStreamer" in info) and ("YES" in info.split("GStreamer")[-1][:60])
        except Exception:
            return False

    def _init_fb_writer(self) -> None:
        if not self._has_gstreamer():
            self.log.error("OpenCV 未启用 GStreamer，无法使用 fbdev 输出。")
            return
        if not os.path.exists(self.fbdev_path):
            self.log.error("未找到 fbdev 设备 %s，无法使用 fbdev 输出。", self.fbdev_path)
            return

        pipeline = (
            # 与 vision_adapter 相同的 fbdev 推流方式
            "appsrc ! queue max-size-buffers=2 leaky=downstream ! "
            "videoconvert ! video/x-raw,format=BGRx ! "
            f"fbdevsink device={self.fbdev_path} sync=false"
        )
        writer = cv2.VideoWriter(
            pipeline,
            cv2.CAP_GSTREAMER,
            0,
            float(self.fps),
            (self.output_width, self.output_height),
            True,
        )
        if writer.isOpened():
            self.fb_writer = writer
            self.log.info("HUD 输出到 %s via fbdev (%dx%d @ %dfps)", self.fbdev_path, self.output_width, self.output_height, self.fps)
        else:
            self.log.warning("fbdevsink 打开失败（%s），HUD fbdev 输出关闭。", self.fbdev_path)
            self.fb_writer = None

    def release(self) -> None:
        if self.fb_writer is not None:
            try:
                self.fb_writer.release()
            except Exception:
                pass
            self.fb_writer = None
