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


@dataclass
class GimbalConfig:
    pitch_scan: float = -35.0
    yaw_scan: float = 0.0


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
    rc_switch: RcSwitchConfig = field(default_factory=RcSwitchConfig)
    dropper: DropperConfig = field(default_factory=DropperConfig)


def _load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_config(config_path: Path | None = None) -> AppConfig:
    path = config_path or DEFAULT_CONFIG_PATH
    data = _load_yaml(path)

    logging_cfg = data.get("logging", {}) or {}
    flight_cfg = data.get("flight", {}) or {}
    align_cfg = data.get("align", {}) or {}
    gimbal_cfg = data.get("gimbal", {}) or {}
    rc_cfg = data.get("rc_switch", {}) or {}
    drop_cfg = data.get("dropper", {}) or {}

    cfg = AppConfig(
        logging=LoggingConfig(level=logging_cfg.get("level", "INFO")),
        flight=FlightConfig(**flight_cfg),
        align=AlignConfig(**align_cfg),
        gimbal=GimbalConfig(**gimbal_cfg),
        rc_switch=RcSwitchConfig(**rc_cfg),
        dropper=DropperConfig(**drop_cfg),
    )
    return cfg
