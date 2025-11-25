#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_adapter.py  — 完整版（Zero3 × ArduPilot）
-------------------------------------------------
功能：
  • 从 USB 摄像头读取 640x480@30 帧（优先 GStreamer v4l2src）
  • 检测 ArUco 标记（默认 DICT_4X_50；可指定目标 ID）
  • 输出中心偏差 (dx, dy) 与置信度 conf（右正，上正）
  • HUD 叠加并通过 GStreamer → fbdevsink 推到 /dev/fb0（无桌面环境可用）
  • 兼容主控：默认 read() 返回 3 个值 (dx, dy, conf)；需要帧时用 return_frame=True
"""

from __future__ import annotations
import os, time
from typing import Optional, Tuple, Dict, Any, List

import cv2
import numpy as np

# ---------- ArUco 可用性 ----------
try:
    from cv2 import aruco
    _ARUCO_OK = True
except Exception:
    aruco = None
    _ARUCO_OK = False


def _has_gstreamer() -> bool:
    try:
        info = cv2.getBuildInformation()
        return ("GStreamer" in info) and ("YES" in info.split("GStreamer")[-1][:60])
    except Exception:
        return False


class VisionAdapter:
    """
    统一接口（与主控兼容）：
        dx, dy, conf = read(stage=None, aligned=None)
        dx, dy, conf, frame = read(..., return_frame=True)
    """
    def __init__(
        self,
        source_cfg: Optional[Dict[str, Any]] = None,
        hud: bool = True,
        fbdev_path: str = "/dev/fb0",
        aruco_dict: str = "DICT_4X4_50",
        target_ids: Optional[List[int]] = None,
    ) -> None:
        if not _ARUCO_OK:
            raise RuntimeError("OpenCV 未启用 aruco 模块，请安装带 contrib 的 OpenCV 包。")

        self.src = source_cfg or self._auto_source_from_env()
        self.hud_enabled = bool(hud) and bool(int(os.environ.get("USE_FB", "1")))
        self.fbdev_path = fbdev_path or os.environ.get("FBDEV", "/dev/fb0")
        self.target_ids = target_ids[:] if target_ids else self._ids_from_env()
        self.w = int(self.src.get('width', 640)); self.h = int(self.src.get('height', 480))
        self.fps = int(self.src.get('fps', 30))

        # ArUco
        self.aruco_dict = self._get_aruco_dict(aruco_dict or os.environ.get("ARUCO_DICT", "DICT_4X4_50"))
        self.aruco_params = aruco.DetectorParameters_create()

        # 输入：优先 GStreamer v4l2src（指定 device=/dev/videoX）
        self.cap = self._open_input(self.src)

        # 输出：GStreamer fbdevsink（若可用）；否则禁用 HUD
        self.writer = None
        if self.hud_enabled and os.path.exists(self.fbdev_path) and _has_gstreamer():
            self.writer = self._open_fbdev_writer(self.fbdev_path, self.w, self.h, self.fps)
            if not (self.writer and self.writer.isOpened()):
                print("[VisionAdapter] 警告：fbdevsink 打开失败，关闭 HUD。")
                self.writer = None
                self.hud_enabled = False
            else:
                print(f"[VisionAdapter] HDMI HUD → {self.fbdev_path}  {self.w}x{self.h}@{self.fps}")
        else:
            if not os.path.exists(self.fbdev_path):
                print(f"[VisionAdapter] 提示：找不到 {self.fbdev_path}，HUD 关闭。")
            elif not _has_gstreamer():
                print("[VisionAdapter] 提示：OpenCV 未启用 GStreamer，HUD 关闭。")
            self.hud_enabled = False

        self.font = cv2.FONT_HERSHEY_SIMPLEX

    # ---------- Source 自动推断 ----------
    def _auto_source_from_env(self) -> Dict[str, Any]:
        dev = os.environ.get("CAM_DEV", "").strip()
        idx = os.environ.get("CAM_INDEX", "").strip()
        w = int(os.environ.get("CAP_W", "640"))
        h = int(os.environ.get("CAP_H", "480"))
        fps = int(os.environ.get("CAP_FPS", "30"))

        if dev and os.path.exists(dev):
            return {'type':'gstreamer', 'device':dev, 'width':w, 'height':h, 'fps':fps}
        if not idx:
            idx = "1"  # 默认 /dev/video1
        return {'type':'v4l2', 'index':int(idx), 'width':w, 'height':h, 'fps':fps}

    def _ids_from_env(self) -> Optional[List[int]]:
        s = os.environ.get("TARGET_IDS", "").strip()
        if not s:
            return None
        out = []
        for t in s.split(","):
            t = t.strip()
            if t.isdigit(): out.append(int(t))
        return out or None

    # ---------- Open ----------
    def _get_aruco_dict(self, name: str):
        name = name.upper().strip()
        if not name.startswith("DICT_"):
            name = "DICT_" + name
        if not hasattr(aruco, name):
            raise ValueError(f"不支持的 ArUco 字典：{name}")
        return aruco.getPredefinedDictionary(getattr(aruco, name))

    def _open_input(self, cfg: Dict[str, Any]) -> cv2.VideoCapture:
        t = cfg.get('type', 'gstreamer')
        if t == 'gstreamer':
            device = cfg.get('device', '/dev/video1')
            w, h, fps = int(cfg.get('width',640)), int(cfg.get('height',480)), int(cfg.get('fps',30))
            pipeline = (
                f"v4l2src device={device} io-mode=2 ! "
                f"video/x-raw,format=YUY2,width={w},height={h},framerate={fps}/1 ! "
                f"queue max-size-buffers=2 leaky=downstream ! "
                f"videoconvert ! video/x-raw,format=BGR ! "
                f"appsink drop=true max-buffers=2"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                raise RuntimeError(f"无法打开摄像头（GStreamer）：{device}")
            print(f"[VisionAdapter] 输入：{device} {w}x{h}@{fps} via GStreamer")
            return cap
        elif t == 'v4l2':
            idx = int(cfg.get('index', 1))
            w, h, fps = int(cfg.get('width',640)), int(cfg.get('height',480)), int(cfg.get('fps',30))
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            if not cap.isOpened():
                raise RuntimeError(f"无法打开摄像头（V4L2 index）：/dev/video{idx}")
            print(f"[VisionAdapter] 输入：/dev/video{idx} {w}x{h}@{fps} via V4L2")
            return cap
        else:
            raise ValueError(f"未知输入类型：{t}")

    def _open_fbdev_writer(self, fb: str, w: int, h: int, fps: int) -> Optional[cv2.VideoWriter]:
        pipeline = (
            "appsrc ! videoconvert ! video/x-raw,format=BGRx ! "
            f"fbdevsink device={fb} sync=false"
        )
        wr = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(fps), (w, h), True)
        return wr

    # ---------- 主接口 ----------
    def read(self, stage: Optional[str]=None, aligned: Optional[bool]=None,
             return_frame: bool=False):
        ok, frame = (self.cap.read() if self.cap is not None else (False, None))
        if (not ok) or frame is None:
            if return_frame:
                return 0.0, 0.0, 0.0, None
            return 0.0, 0.0, 0.0

        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_LINEAR)

        dx, dy, conf, bbox, info = self._detect_aruco(frame)

        if self.hud_enabled and self.writer is not None:
            hud = self._draw_hud(frame.copy(), dx, dy, conf, bbox, info, stage, aligned)
            self.writer.write(hud)

        if return_frame:
            return dx, dy, conf, frame
        else:
            return dx, dy, conf

    # ---------- 检测 ----------
    def _detect_aruco(self, frame: np.ndarray):
        h, w = frame.shape[:2]; cx, cy = w*0.5, h*0.5
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rej = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(corners) == 0:
            return 0.0, 0.0, 0.0, None, None

        sel = -1
        if self.target_ids:
            id_list = ids.flatten().tolist()
            for i, mid in enumerate(id_list):
                if int(mid) in self.target_ids:
                    sel = i; break
        if sel < 0:
            areas = [cv2.contourArea(c.astype(np.float32)) for c in corners]
            sel = int(np.argmax(areas))

        c = corners[sel].reshape(-1,2)
        mid = int(ids[sel][0])
        mx, my = c[:,0].mean(), c[:,1].mean()

        dx = float(mx - cx)          # 右为正
        dy = float(cy - my)          # 上为正

        area = cv2.contourArea(c.astype(np.float32))
        conf = float(max(0.0, min(1.0, (area/(w*h))*8.0)))

        x,y,bw,bh = cv2.boundingRect(c.astype(np.float32))
        bbox = np.array([x, y, x+bw, y+bh], dtype=np.int32)

        return dx, dy, conf, bbox, (mid, int(mx), int(my))

    # ---------- HUD ----------
    def _draw_hud(self, img: np.ndarray, dx: float, dy: float, conf: float,
                  bbox, info, stage, aligned) -> np.ndarray:
        h, w = img.shape[:2]; cx, cy = w//2, h//2
        th = max(1, min(w,h)//300)

        cv2.line(img, (cx-20, cy), (cx+20, cy), (0,255,0), th, cv2.LINE_AA)
        cv2.line(img, (cx, cy-20), (cx, cy+20), (0,255,0), th, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), th+1, (0,255,0), -1, cv2.LINE_AA)

        if abs(dx) > 1 or abs(dy) > 1:
            ex, ey = int(cx + dx), int(cy - dy)
            cv2.arrowedLine(img, (cx, cy), (ex, ey), (0,255,255), th+1, tipLength=0.18)

        if bbox is not None:
            x1,y1,x2,y2 = bbox.tolist()
            cv2.rectangle(img, (x1,y1), (x2,y2), (0,180,255), th+1)
        if info is not None:
            mid, mx, my = info
            cv2.circle(img, (mx,my), 4, (255,255,0), -1)
            cv2.putText(img, f"id={mid}", (mx+6, my-6), self.font, 0.5, (255,255,0), 1, cv2.LINE_AA)

        y = 22
        def put(s):
            nonlocal y
            cv2.putText(img, s, (8,y), self.font, 0.6, (255,255,255), 1, cv2.LINE_AA); y += 24

        if stage is not None: put(f"Stage: {stage}")
        put(f"dx={dx:.1f}px  dy={dy:.1f}px  conf={conf:.2f}")
        if aligned is not None: put(f"Aligned: {'YES' if aligned else 'NO'}")
        return img

    # ---------- 资源 ----------
    def release(self) -> None:
        try:
            if self.cap is not None: self.cap.release()
        except Exception: pass
        try:
            if self.writer is not None: self.writer.release()
        except Exception: pass


# ========== 独立运行 ==========
def _standalone():
    dev = os.environ.get("CAM_DEV", "").strip()
    idx = os.environ.get("CAM_INDEX", "").strip()
    w = int(os.environ.get("CAP_W", "640"))
    h = int(os.environ.get("CAP_H", "480"))
    fps = int(os.environ.get("CAP_FPS", "30"))
    use_fb = bool(int(os.environ.get("USE_FB", "1")))

    if dev and os.path.exists(dev):
        src = {'type':'gstreamer','device':dev,'width':w,'height':h,'fps':fps}
    else:
        if not idx: idx = "1"
        src = {'type':'v4l2','index':int(idx),'width':w,'height':h,'fps':fps}

    va = VisionAdapter(source_cfg=src, hud=use_fb, fbdev_path=os.environ.get("FBDEV","/dev/fb0"))
    print("[VisionAdapter] Standalone running. Ctrl+C to exit.")
    t0 = time.time()
    try:
        while True:
            dx, dy, conf = va.read(stage="ALIGN", aligned=None, return_frame=False)
            if time.time() - t0 > 0.5:
                print(f"dx={dx:.1f}  dy={dy:.1f}  conf={conf:.2f}")
                t0 = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        va.release()


if __name__ == "__main__":
    _standalone()
