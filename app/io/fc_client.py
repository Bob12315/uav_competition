from __future__ import annotations

import time
from dataclasses import dataclass
from logging import Logger
from math import sin
from typing import Any, Dict

from app.core.models import ControlCommand, TelemetrySnapshot
from app.io.link_mav import MavLink, mavutil

ARMED_FLAG = 0b1000_0000  # MAV_MODE_FLAG_SAFETY_ARMED


@dataclass
class FcClientConfig:
    endpoint: str = "udp:192.168.10.14:15001"
    baud: int = 115200
    dry_run: bool = False


class FcClient:
    """MAVLink flight-controller client built on top of MavLink wrapper."""

    def __init__(self, cfg: FcClientConfig, logger: Logger) -> None:
        self.cfg = cfg
        self.log = logger.getChild("FcClient")
        self.mav = MavLink(cfg.endpoint, cfg.baud, dry_run=cfg.dry_run)
        self._last_raw: Dict[str, Any] = {
            "time": time.time(),
            "mode": "UNKNOWN",
            "armed": False,
            "voltage": 0.0,
            "alt": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "yaw": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "sats": 0,
        }
        self._last_rc: Dict[int, int] = {}

    def connect(self) -> None:
        self.log.info("Connecting to FC endpoint=%s (baud=%d)", self.cfg.endpoint, self.cfg.baud)
        self.mav.connect()
        self.log.info("FC connected (MAVLink ready).")

    def heartbeat_ok(self) -> bool:
        return self.mav.heartbeat_ok()

    def _drain_messages(self) -> None:
        """Drain MAVLink queue and update cached telemetry/RC."""
        master = self.mav.master
        if master is None:
            return

        while True:
            msg = master.recv_match(blocking=False)
            if msg is None:
                break

            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                base_mode = getattr(msg, "base_mode", 0)
                self._last_raw["armed"] = bool(base_mode & ARMED_FLAG)
                try:
                    mode_str = mavutil.mode_string_v10(msg)
                except Exception:
                    mode_str = None
                if mode_str:
                    self._last_raw["mode"] = mode_str
                    master.flightmode = mode_str
            elif mtype == "ATTITUDE":
                self._last_raw.update(
                    roll=msg.roll * 57.2958,
                    pitch=msg.pitch * 57.2958,
                    yaw=msg.yaw * 57.2958,
                )
            elif mtype == "GLOBAL_POSITION_INT":
                self._last_raw.update(
                    vx=msg.vx / 100.0,
                    alt=msg.relative_alt / 1000.0,
                    lat=msg.lat / 1e7,
                    lon=msg.lon / 1e7,
                )
            elif mtype == "GPS_RAW_INT":
                lat = getattr(msg, "lat", None)
                lon = getattr(msg, "lon", None)
                if lat not in (None, 0, 0x7FFFFFFF):
                    self._last_raw["lat"] = lat / 1e7
                if lon not in (None, 0, 0x7FFFFFFF):
                    self._last_raw["lon"] = lon / 1e7
                sats = getattr(msg, "satellites_visible", None)
                if sats is not None:
                    self._last_raw["sats"] = int(sats)
            elif mtype == "VFR_HUD":
                gs = getattr(msg, "groundspeed", None)
                if gs is not None:
                    self._last_raw["vx"] = gs
                hdg = getattr(msg, "heading", None)
                if hdg is not None:
                    self._last_raw["yaw"] = float(hdg)
                alt = getattr(msg, "alt", None)
                if alt is not None:
                    self._last_raw["alt"] = float(alt)
            elif mtype == "SYS_STATUS":
                vb = getattr(msg, "voltage_battery", 65535)
                if vb not in (0, 65535):
                    self._last_raw["voltage"] = vb / 1000.0
            elif mtype == "BATTERY_STATUS":
                volts = getattr(msg, "voltages", None)
                if volts:
                    v0 = volts[0]
                    if v0 not in (0, 0xFFFF):
                        self._last_raw["voltage"] = v0 / 1000.0
            elif mtype in ("RC_CHANNELS", "RC_CHANNELS_OVERRIDE"):
                for i in range(1, 9):
                    val = getattr(msg, f"ch{i}", None)
                    if val is not None:
                        self._last_rc[i] = val

    # ===== Telemetry =====
    def read_telemetry_raw(self) -> Dict[str, Any]:
        """
        Return raw telemetry dict assembled from MAVLink messages.
        Keys: time, mode, armed, voltage, alt, vx, vy, yaw, lat, lon, sats.
        """
        self._drain_messages()
        raw = dict(self._last_raw)
        raw["time"] = time.time()
        return raw

    def read_telemetry(self) -> TelemetrySnapshot:
        raw = self.read_telemetry_raw()
        return TelemetrySnapshot(
            time=raw["time"],
            mode=raw["mode"],
            armed=raw["armed"],
            voltage=raw["voltage"],
            alt=raw["alt"],
            vx=raw["vx"],
            vy=raw["vy"],
            yaw=raw["yaw"],
            lat=raw["lat"],
            lon=raw["lon"],
            sats=raw["sats"],
        )

    def read_rc_channels(self) -> Dict[int, int]:
        self._drain_messages()
        return dict(self._last_rc)

    # ===== Commands =====
    def send_velocity_body(self, vx: float, vy: float, vz: float) -> None:
        self.mav.send_velocity_body(vx, vy, vz)

    def send_yaw_rate(self, yaw_rate: float) -> None:
        self.mav.send_yaw_rate(yaw_rate)

    def send_gimbal_angles(self, pitch_deg: float, yaw_deg: float) -> None:
        self.mav.send_gimbal_angles(pitch_deg, yaw_deg)

    def send_servo(self, channel: int, pwm: int) -> None:
        self.mav.do_set_servo(channel, pwm)

    def send_control(self, cmd: ControlCommand) -> None:
        """Aggregate control command from mission loop."""
        self.send_velocity_body(cmd.vx, cmd.vy, cmd.vz)
        if abs(cmd.yaw_rate) > 1e-6:
            self.send_yaw_rate(cmd.yaw_rate)
        if cmd.gimbal_pitch is not None or cmd.gimbal_yaw is not None:
            self.send_gimbal_angles(cmd.gimbal_pitch or 0.0, cmd.gimbal_yaw or 0.0)
        if cmd.drop:
            self.log.info("Drop requested (send servo via dropper).")


class FakeFcClient:
    """Software-only FC used for desktop demo/testing."""

    def __init__(self, logger: Logger) -> None:
        self.log = logger.getChild("FakeFc")
        self._start_time = time.time()

    def connect(self) -> None:
        self.log.info("Fake FC connected (no hardware).")

    def heartbeat_ok(self) -> bool:
        return True

    def read_telemetry(self) -> TelemetrySnapshot:
        now = time.time() - self._start_time
        alt = 15.0 + 10.0 * sin(now * 0.2)
        voltage = 16.8 - 0.01 * now
        return TelemetrySnapshot(
            time=time.time(),
            mode="GUIDED",
            armed=True,
            voltage=max(voltage, 13.0),
            alt=alt,
            vx=0.0,
            vy=0.0,
            yaw=(now * 10.0) % 360,
            lat=0.0,
            lon=0.0,
            sats=10,
        )

    def read_rc_channels(self) -> Dict[int, int]:
        return {5: 1500, 6: 1000}

    def send_control(self, cmd: ControlCommand) -> None:
        if (
            abs(cmd.vx) > 1e-3
            or abs(cmd.vy) > 1e-3
            or abs(cmd.yaw_rate) > 1e-3
            or cmd.drop
        ):
            self.log.debug(
                "control: vx=%.2f vy=%.2f yaw_rate=%.3f drop=%s",
                cmd.vx,
                cmd.vy,
                cmd.yaw_rate,
                cmd.drop,
            )

    # Stub helpers for compatibility with real client
    def send_velocity_body(self, vx: float, vy: float, vz: float) -> None:
        self.log.debug("fake velocity: %.2f %.2f %.2f", vx, vy, vz)

    def send_yaw_rate(self, yaw_rate: float) -> None:
        self.log.debug("fake yaw_rate: %.3f", yaw_rate)

    def send_gimbal_angles(self, pitch_deg: float, yaw_deg: float) -> None:
        self.log.debug("fake gimbal pitch=%.1f yaw=%.1f", pitch_deg, yaw_deg)

    def send_servo(self, channel: int, pwm: int) -> None:
        self.log.debug("fake servo ch=%d pwm=%d", channel, pwm)
