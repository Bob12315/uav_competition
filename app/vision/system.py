from __future__ import annotations

import os
import time
from logging import Logger

import cv2
import numpy as np

from app.core.models import VisionResult


class VisionSystem:
    """
    摄像头读取 + ArUco 检测（管道方式与 vision_adapter.py 保持一致）。
    优先使用 GStreamer v4l2src + appsink，失败再回退 V4L2/default。
    """

    def __init__(
        self,
        device: int | str,
        logger: Logger,
        width: int | None = None,
        height: int | None = None,
        pixel_format: str | None = None,
        fps: int | None = None,
        aruco_dict: str | None = None,
        marker_length_m: float = 0.05,
        camera_matrix: list[list[float]] | None = None,
        dist_coeffs: list[float] | None = None,
        prefer_gst_input: bool | None = None,
    ) -> None:
        self.log = logger.getChild("Vision")
        self.cap: cv2.VideoCapture | None = None
        self.device = device
        self.width = width
        self.height = height
        self.pixel_format = (pixel_format or "MJPG").upper()
        self.fps = int(fps) if fps else 30
        self.marker_length_m = float(marker_length_m)
        env_prefer_gst = os.environ.get("USE_GST_INPUT")
        if prefer_gst_input is not None:
            self.prefer_gst_input = prefer_gst_input
        else:
            self.prefer_gst_input = env_prefer_gst == "1"

        self._camera_matrix = (
            np.array(camera_matrix, dtype=np.float32) if camera_matrix is not None else None
        )
        self._dist_coeffs = (
            np.array(dist_coeffs, dtype=np.float32).reshape(-1, 1)
            if dist_coeffs is not None
            else None
        )

        self._aruco_dict = None
        self._aruco_params = None
        self._aruco_detector = None
        self._init_aruco(aruco_dict or "DICT_4X4_50")

        self._opened = self._init_capture()

    def read(self) -> tuple[VisionResult, "cv2.Mat | None"]:
        if self.cap is None or not self.cap.isOpened():
            return VisionResult(has_target=False), None

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.log.warning("Camera read failed")
            return VisionResult(has_target=False), None

        try:
            vr = self._detect_aruco(frame)
        except Exception as exc:  # noqa: BLE001
            self.log.exception("ArUco detection failed: %s", exc)
            vr = VisionResult(has_target=False, dx=0.0, dy=0.0, confidence=0.0)

        return vr, frame

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()

    @staticmethod
    def _build_gst_pipeline(
        device: str, width: int | None, height: int | None, fmt: str, fps: int
    ) -> str:
        w = width or 640
        h = height or 480
        fmt_upper = fmt.upper()
        if fmt_upper == "YUYV":
            caps = f"video/x-raw,format=YUY2,width={w},height={h},framerate={fps}/1"
            decode = "videoconvert ! video/x-raw,format=BGR"
        else:
            caps = f"image/jpeg,width={w},height={h},framerate={fps}/1"
            decode = "jpegdec ! videoconvert ! video/x-raw,format=BGR"
        return (
            f"v4l2src device={device} io-mode=2 ! {caps} ! "
            "queue max-size-buffers=2 leaky=downstream ! "
            f"{decode} ! appsink drop=true max-buffers=2"
        )

    def _init_capture(self) -> bool:
        dev = self.device
        w, h, fmt = self.width, self.height, self.pixel_format
        attempts: list[tuple[str, cv2.VideoCapture]] = []

        def gst_attempts() -> list[tuple[str, cv2.VideoCapture]]:
            out: list[tuple[str, cv2.VideoCapture]] = []
            if isinstance(dev, str) and dev.startswith('/dev/video'):
                pipeline = self._build_gst_pipeline(dev, w, h, fmt, self.fps)
                out.append((f"GSTREAMER-{fmt}", cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)))
                if fmt != 'YUYV':
                    pipeline_yuyv = self._build_gst_pipeline(dev, w, h, 'YUYV', self.fps)
                    out.append((f"GSTREAMER-YUYV", cv2.VideoCapture(pipeline_yuyv, cv2.CAP_GSTREAMER)))
            return out

        gst_first = bool(getattr(self, 'prefer_gst_input', False))
        if gst_first:
            attempts.extend(gst_attempts())
        attempts.append(('V4L2', cv2.VideoCapture(dev, cv2.CAP_V4L2)))
        attempts.append(('DEFAULT', cv2.VideoCapture(dev)))

        for name, cap in attempts:
            if not cap.isOpened():
                self.log.debug('Open attempt %s failed for %s', name, dev)
                cap.release()
                continue

            if fmt:
                fourcc = cv2.VideoWriter_fourcc(*fmt)
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            if w:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            if h:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, self.fps)

            ok, frame = cap.read()
            if ok and frame is not None:
                self.cap = cap
                self.log.info('Camera %s opened via %s (%dx%d %s)', dev, name, frame.shape[1], frame.shape[0], fmt)
                return True

            self.log.debug('Open attempt %s gave no frame, releasing', name)
            cap.release()

        self.log.error('Failed to open camera %s after all attempts', dev)
        return False

    def _init_aruco(self, dict_name: str) -> None:
        if not hasattr(cv2, "aruco"):
            self.log.error("OpenCV aruco module not found; ArUco detection disabled.")
            return

        try:
            dictionary_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.log.warning("Unknown ArUco dictionary %s, fallback to DICT_4X4_50", dict_name)
            dictionary_id = cv2.aruco.DICT_4X4_50

        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self._aruco_params = cv2.aruco.DetectorParameters()
        else:
            self._aruco_params = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
        else:
            self._aruco_detector = None
        self.log.info("ArUco detection ready with dictionary %s", dict_name)

    def _ensure_camera_matrix(self, width: int, height: int) -> bool:
        if self._camera_matrix is None:
            fx = fy = 0.9 * float(max(width, height))
            cx = width / 2.0
            cy = height / 2.0
            self._camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
            self.log.debug(
                "Using default camera matrix for pose: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy
            )

        if self._dist_coeffs is None:
            self._dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        return self._camera_matrix is not None and self._dist_coeffs is not None

    def _detect_aruco(self, frame: "cv2.Mat") -> VisionResult:
        if self._aruco_dict is None:
            return VisionResult(has_target=False, dx=0.0, dy=0.0, confidence=0.0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._aruco_detector is not None:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_params)

        if ids is None or len(ids) == 0:
            return VisionResult(has_target=False, dx=0.0, dy=0.0, confidence=0.0)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        h, w = gray.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        marker_corners = corners[0].reshape(-1, 2)
        mx, my = marker_corners.mean(axis=0)

        vr = VisionResult(
            has_target=True,
            dx=float(mx - cx),
            dy=float(my - cy),
            confidence=1.0,
        )

        pose = self._estimate_pose(frame, corners, w, h)
        if pose is not None:
            rvec, tvec, distance = pose
            vr.confidence = max(0.0, min(1.0, 1.0 / (1.0 + distance)))
            flat_tvec = np.asarray(tvec).reshape(-1)
            flat_rvec = np.asarray(rvec).reshape(-1)
            if flat_tvec.size >= 3 and flat_rvec.size >= 3:
                vr.tvec = (float(flat_tvec[0]), float(flat_tvec[1]), float(flat_tvec[2]))
                vr.rvec = (float(flat_rvec[0]), float(flat_rvec[1]), float(flat_rvec[2]))
            else:
                self.log.debug("Pose vectors returned with unexpected shape: rvec=%s tvec=%s", rvec, tvec)

        return vr

    def _estimate_pose(
        self, frame: "cv2.Mat", corners, width: int, height: int
    ) -> tuple[np.ndarray, np.ndarray, float] | None:
        if not self._ensure_camera_matrix(width, height):
            return None

        estimator = getattr(cv2.aruco, "estimatePoseSingleMarkers", None)

        if estimator is not None:
            rvecs, tvecs, _ = estimator(corners, self.marker_length_m, self._camera_matrix, self._dist_coeffs)
            if rvecs is None or len(rvecs) == 0:
                return None
            rvec = rvecs[0]
            tvec = tvecs[0]
        else:
            pts_img = corners[0].reshape(-1, 2).astype(np.float32)
            half = self.marker_length_m / 2.0
            pts_obj = np.array(
                [
                    [-half, half, 0.0],
                    [half, half, 0.0],
                    [half, -half, 0.0],
                    [-half, -half, 0.0],
                ],
                dtype=np.float32,
            )
            success, rvec, tvec = cv2.solvePnP(pts_obj, pts_img, self._camera_matrix, self._dist_coeffs)
            if not success:
                return None

        if hasattr(cv2, "drawFrameAxes"):
            cv2.drawFrameAxes(
                frame,
                self._camera_matrix,
                self._dist_coeffs,
                rvec,
                tvec,
                self.marker_length_m * 0.5,
            )

        distance = float(np.linalg.norm(tvec))
        return rvec, tvec, distance


class FakeVisionSystem:
    """
    纯软件 Demo 用的假视觉：
    - 640x480 暗底
    - 每 10 秒出现 5 秒目标，dx/dy 逐步逼近中心
    """

    def __init__(self, logger: Logger) -> None:
        self.log = logger.getChild("FakeVision")
        self._start_time = time.time()
        self._dx = 150.0
        self._dy = -100.0

    def read(self) -> tuple[VisionResult, np.ndarray]:
        now = time.time() - self._start_time
        h, w = 480, 640
        frame = np.full((h, w, 3), 40, dtype=np.uint8)

        phase = now % 10.0
        has_target = 3.0 <= phase <= 8.0

        if has_target:
            self._dx *= 0.9
            self._dy *= 0.9
        else:
            self._dx = 150.0
            self._dy = -100.0

        cx, cy = w // 2, h // 2
        tx = int(cx + self._dx)
        ty = int(cy + self._dy)

        cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        if has_target:
            cv2.circle(frame, (tx, ty), 15, (0, 0, 255), 2)

        vr = VisionResult(
            has_target=has_target,
            dx=float(self._dx),
            dy=float(self._dy),
            confidence=0.9 if has_target else 0.0,
        )
        return vr, frame
