from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.yaml"


@dataclass
class LoggingConfig:
    level: str = "INFO"


@dataclass
class FlightConfig:
    loop_hz: int = 20
    scan_alt: float = 30.0
    drop_alt: float = 10.0


@dataclass
class AlignConfig:
    dx_threshold: float = 10.0
    dy_threshold: float = 10.0
    kx: float = 0.001
    ky: float = 0.001
    x_threshold_m: float = 0.05
    y_threshold_m: float = 0.05
    kx_pos: float = 0.5  # m -> m/s
    ky_pos: float = 0.5  # m -> m/s


@dataclass
class GimbalConfig:
    pitch_scan: float = -35.0
    yaw_scan: float = 0.0


@dataclass
class VisionConfig:
    device: str = "0"  # 摄像头索引或路径
    width: int = 640
    height: int = 480
    pixel_format: str = "MJPG"  # 或 YUYV
    fps: int = 30
    calibration_file: str | None = None
    aruco_dict: str = "DICT_4X4_50"
    marker_length_m: float = 0.05
    camera_matrix: list[list[float]] | None = None
    dist_coeffs: list[float] | None = None


@dataclass
class RcSwitchConfig:
    mode_channel: int = 5
    drop_channel: int = 6


@dataclass
class DropperConfig:
    channel: int = 15
    pwm_safe: int = 1100
    pwm_drop: int = 1900
    pulse_ms: int = 500


@dataclass
class AppConfig:
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    flight: FlightConfig = field(default_factory=FlightConfig)
    align: AlignConfig = field(default_factory=AlignConfig)
    gimbal: GimbalConfig = field(default_factory=GimbalConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    rc_switch: RcSwitchConfig = field(default_factory=RcSwitchConfig)
    dropper: DropperConfig = field(default_factory=DropperConfig)


def _load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _load_calibration_from_file(path: str | None) -> tuple[list[list[float]] | None, list[float] | None]:
    if not path:
        return None, None

    cal_path = Path(path)
    if not cal_path.is_absolute():
        cal_path = PROJECT_ROOT / cal_path

    if not cal_path.exists():
        return None, None

    try:
        with cal_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return None, None

    cam_data = (data.get("camera_matrix") or {}).get("data")
    dist_data = (data.get("distortion_coefficients") or {}).get("data")

    camera_matrix = None
    if isinstance(cam_data, list) and len(cam_data) >= 9:
        camera_matrix = [
            [float(cam_data[0]), float(cam_data[1]), float(cam_data[2])],
            [float(cam_data[3]), float(cam_data[4]), float(cam_data[5])],
            [float(cam_data[6]), float(cam_data[7]), float(cam_data[8])],
        ]

    dist_coeffs = None
    if isinstance(dist_data, list) and len(dist_data) >= 4:
        dist_coeffs = [float(x) for x in dist_data]

    return camera_matrix, dist_coeffs


def load_config(config_path: Path | None = None) -> AppConfig:
    path = config_path or DEFAULT_CONFIG_PATH
    data = _load_yaml(path)

    logging_cfg = data.get("logging", {}) or {}
    flight_cfg = data.get("flight", {}) or {}
    align_cfg = data.get("align", {}) or {}
    gimbal_cfg = data.get("gimbal", {}) or {}
    vision_cfg = data.get("vision", {}) or {}
    rc_cfg = data.get("rc_switch", {}) or {}
    drop_cfg = data.get("dropper", {}) or {}

    calibration_file = vision_cfg.get("calibration_file")
    camera_matrix = vision_cfg.get("camera_matrix")
    dist_coeffs = vision_cfg.get("dist_coeffs")

    if (camera_matrix is None or dist_coeffs is None) and calibration_file:
        cal_cam, cal_dist = _load_calibration_from_file(calibration_file)
        if camera_matrix is None:
            camera_matrix = cal_cam
        if dist_coeffs is None:
            dist_coeffs = cal_dist

    cfg = AppConfig(
        logging=LoggingConfig(level=logging_cfg.get("level", "INFO")),
        flight=FlightConfig(**flight_cfg),
        align=AlignConfig(**align_cfg),
        gimbal=GimbalConfig(**gimbal_cfg),
        vision=VisionConfig(
            device=str(vision_cfg.get("device", "0")),
            width=int(vision_cfg.get("width", 640)),
            height=int(vision_cfg.get("height", 480)),
            pixel_format=str(vision_cfg.get("pixel_format", "MJPG")),
            fps=int(vision_cfg.get("fps", 30)),
            calibration_file=str(calibration_file) if calibration_file else None,
            aruco_dict=str(vision_cfg.get("aruco_dict", "DICT_4X4_50")),
            marker_length_m=float(vision_cfg.get("marker_length_m", 0.05)),
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        ),
        rc_switch=RcSwitchConfig(**rc_cfg),
        dropper=DropperConfig(**drop_cfg),
    )
    return cfg
