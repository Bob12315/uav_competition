from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TelemetrySnapshot:
    time: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    voltage: float = 0.0
    alt: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    sats: int = 0


@dataclass
class VisionResult:
    has_target: bool = False
    dx: float = 0.0
    dy: float = 0.0
    confidence: float = 0.0
    tvec: tuple[float, float, float] | None = None  # meters, camera coords (x right, y down, z forward)
    rvec: tuple[float, float, float] | None = None  # Rodrigues rotation vector (rad)


@dataclass
class ControlCommand:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    gimbal_pitch: float | None = None
    gimbal_yaw: float | None = None
    drop: bool = False


@dataclass
class HudData:
    telemetry: TelemetrySnapshot
    vision: VisionResult
    mission_state: str = "IDLE"
