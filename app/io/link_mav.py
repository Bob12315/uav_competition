# -*- coding: utf-8 -*-
"""
Lightweight MAVLink helper for connecting to a flight controller via pymavlink.

Features:
- UDP or serial connection
- Data stream request (status/attitude/position)
- Heartbeat check
- Mode get/set
- Body-frame velocity commands
- Servo command
- Read attitude/altitude/battery
- Read RC channels
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

try:
    from pymavlink import mavutil
except Exception:  # noqa: BLE001
    mavutil = None


class MavLink:
    """Small wrapper around pymavlink for quick desktop/hardware tests."""

    def __init__(self, device: str, baud: int = 921600, dry_run: bool = False) -> None:
        """
        Parameters
        ----------
        device : str
            Connection string, e.g. "/dev/ttyAMA0" or "udp:0.0.0.0:15001".
        baud : int
            Baudrate for serial connections (ignored for UDP).
        dry_run : bool
            If True or pymavlink missing, skip real connection and emit fake data.
        """
        self.device = device
        self.baud = baud
        self.dry = bool(dry_run or (mavutil is None))

        self.master: Optional[Any] = None
        self._last_hb: float = 0.0
        self._target_sys: int = 1
        self._target_comp: int = 1

    # ================== Connection & streams ==================

    def connect(self, wait_heartbeat: bool = True, timeout: float = 10.0) -> None:
        """Establish MAVLink connection and request common data streams."""
        if self.dry:
            print("[MavLink] dry_run mode; skip real connection.")
            return

        if self.device.startswith(("udp:", "udpin:", "udpout:", "tcp:")):
            self.master = mavutil.mavlink_connection(self.device, source_system=255)
        else:
            self.master = mavutil.mavlink_connection(
                self.device,
                baud=self.baud,
                source_system=255,
            )

        if wait_heartbeat:
            self.master.wait_heartbeat(timeout=timeout)
            self._last_hb = time.time()
            print(
                f"[MavLink] Heartbeat from system {self.master.target_system} "
                f"component {self.master.target_component}"
            )

        self._target_sys = int(self.master.target_system or 1)
        self._target_comp = int(self.master.target_component or 1)
        self._request_streams()

    def _request_streams(self) -> None:
        """Request common data streams (status/position/attitude)."""
        if self.dry or self.master is None:
            return

        try:
            streams = [
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,  # SYS_STATUS / battery
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # GLOBAL_POSITION_INT
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
            ]

            for sid in streams:
                self.master.mav.request_data_stream_send(
                    self._target_sys,
                    self._target_comp,
                    sid,
                    4,  # Hz
                    1,  # start
                )

            print("[MavLink] Requested data streams (status/position/attitude).")
        except Exception as exc:  # noqa: BLE001
            print("[MavLink] request_data_stream failed:", exc)

    def heartbeat_ok(self, max_interval: float = 2.0) -> bool:
        """Return True if a heartbeat arrived within max_interval seconds."""
        if self.dry:
            return True
        if self.master is None:
            return False

        msg = self.master.recv_match(type="HEARTBEAT", blocking=False)
        if msg is not None:
            self._last_hb = time.time()
        return (time.time() - self._last_hb) < max_interval

    def poll_heartbeat(self) -> Dict[str, Any]:
        """Poll one HEARTBEAT; return armed/mode/system_status info (may be empty)."""
        info: Dict[str, Any] = {"armed": False, "mode": None, "system_status": None}
        if self.dry or self.master is None:
            return info

        while True:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=False)
            if msg is None:
                break

            self._last_hb = time.time()
            info["system_status"] = getattr(msg, "system_status", info["system_status"])

            try:
                mode_flag = getattr(msg, "base_mode", 0)
                info["armed"] = bool(mode_flag & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            except Exception:
                pass

            try:
                mode_str = mavutil.mode_string_v10(msg)
            except Exception:
                mode_str = None

            if mode_str:
                info["mode"] = mode_str
                self.master.flightmode = mode_str

        return info

    # ================== Modes ==================

    def set_mode(
        self,
        mode: str,
        wait: bool = True,
        timeout: float = 5.0,
        retries: int = 3,
    ) -> bool:
        """Set FC mode (GUIDED/LOITER/AUTO/STABILIZE...)."""
        mode = mode.upper()

        if self.dry:
            print(f"[MavLink] dry_run: set_mode({mode})")
            return True
        if self.master is None:
            print("[MavLink] set_mode failed: not connected.")
            return False

        try:
            mapping = self.master.mode_mapping()
        except Exception:
            mapping = None

        if mapping:
            if mode not in mapping:
                print(f"[MavLink] mode {mode} not supported; options: {list(mapping.keys())}")
                return False
            mode_id = mapping[mode]
            setter = self.master.set_mode
        else:
            fallback = {
                "STABILIZE": 0,
                "ACRO": 1,
                "ALT_HOLD": 2,
                "AUTO": 3,
                "GUIDED": 4,
                "LOITER": 5,
                "RTL": 6,
                "CIRCLE": 7,
                "LAND": 9,
                "DRIFT": 11,
                "SPORT": 13,
            }
            mode_id = fallback.get(mode, 5)
            setter = self.master.set_mode_apm

        for _ in range(retries):
            setter(mode_id)
            if not wait:
                return True

            t0 = time.time()
            while time.time() - t0 < timeout:
                hb = self.master.recv_match(type="HEARTBEAT", blocking=False)
                if hb is not None:
                    try:
                        self.master.flightmode = mavutil.mode_string_v10(hb)
                    except Exception:
                        pass

                cur = getattr(self.master, "flightmode", None)
                if cur == mode:
                    return True
                time.sleep(0.1)

        print(f"[MavLink] set_mode({mode}) timeout; current={self.get_mode()}")
        return False

    def get_mode(self) -> Optional[str]:
        """Return current FC mode as string."""
        if self.dry:
            return "DRY"
        if self.master is None:
            return None

        mode = getattr(self.master, "flightmode", None)
        if mode:
            return mode

        hb = self.master.recv_match(type="HEARTBEAT", blocking=False)
        if hb is not None:
            try:
                self.master.flightmode = mavutil.mode_string_v10(hb)
            except Exception:
                pass

        return getattr(self.master, "flightmode", None)

    # ================== Velocity / servo ==================

    def send_velocity_body(self, vx: float, vy: float, vz: float = 0.0) -> None:
        """Send BODY_NED velocity (m/s)."""
        if self.dry:
            print(f"[MavLink] dry_run: send_velocity_body({vx}, {vy}, {vz})")
            return
        if self.master is None:
            print("[MavLink] send_velocity_body skipped: not connected.")
            return

        type_mask = 0b0000_1111_1100_0111  # ignore pos/accel/yaw
        self.master.mav.set_position_target_local_ned_send(
            int(self.master.time_since("boot") * 1e3),
            self._target_sys,
            self._target_comp,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0,
            0,
            0,
            0,
            0,
        )

    def send_yaw_rate(self, yaw_rate: float) -> None:
        """Send yaw rate command via CONDITION_YAW (deg/s, sign = direction)."""
        if self.dry:
            print(f"[MavLink] dry_run: send_yaw_rate({yaw_rate})")
            return
        if self.master is None:
            print("[MavLink] send_yaw_rate skipped: not connected.")
            return

        direction = 1 if yaw_rate >= 0 else -1
        self.master.mav.command_long_send(
            self._target_sys,
            self._target_comp,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            0,  # target yaw (ignored when relative=1)
            abs(float(yaw_rate)),  # speed deg/s
            direction,
            1,  # relative
            0,
            0,
            0,
        )

    def do_set_servo(self, servo_num: int, pwm: int) -> None:
        """Send MAV_CMD_DO_SET_SERVO for a given channel."""
        if self.dry:
            print(f"[MavLink] dry_run: do_set_servo({servo_num}, {pwm})")
            return
        if self.master is None:
            print("[MavLink] do_set_servo skipped: not connected.")
            return

        self.master.mav.command_long_send(
            self._target_sys,
            self._target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(servo_num),
            float(pwm),
            0,
            0,
            0,
            0,
            0,
        )

    def send_gimbal_angles(self, pitch_deg: float, yaw_deg: float) -> None:
        """Control gimbal angles via MAV_CMD_DO_MOUNT_CONTROL."""
        if self.dry:
            print(f"[MavLink] dry_run: send_gimbal_angles({pitch_deg}, {yaw_deg})")
            return
        if self.master is None:
            print("[MavLink] send_gimbal_angles skipped: not connected.")
            return

        self.master.mav.command_long_send(
            self._target_sys,
            self._target_comp,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            0,
            float(pitch_deg),
            0.0,  # roll
            float(yaw_deg),
            0,
            0,
            0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
        )

    # ================== Telemetry helpers ==================

    def read_attitude_speed_alt(self) -> Dict[str, Optional[float]]:
        """Return latest attitude/ground_speed/relative_alt/lat/lon/sats by draining queue."""
        if self.dry or self.master is None:
            return {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "ground_speed": 0.0,
                "rel_alt": 30.0,
                "lat": 0.0,
                "lon": 0.0,
                "sats": 10,
            }

        out: Dict[str, Optional[float]] = {
            "roll": None,
            "pitch": None,
            "yaw": None,
            "ground_speed": None,
            "rel_alt": None,
            "lat": None,
            "lon": None,
            "sats": None,
        }

        while True:
            msg = self.master.recv_match(
                type=["ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "VFR_HUD"],
                blocking=False,
            )
            if msg is None:
                break

            mtype = msg.get_type()
            if mtype == "ATTITUDE":
                out.update(
                    roll=msg.roll * 57.2958,
                    pitch=msg.pitch * 57.2958,
                    yaw=msg.yaw * 57.2958,
                )
            elif mtype == "GLOBAL_POSITION_INT":
                out.update(
                    ground_speed=msg.vx / 100.0,  # cm/s -> m/s
                    rel_alt=msg.relative_alt / 1000.0,  # mm -> m
                    lat=msg.lat / 1e7,
                    lon=msg.lon / 1e7,
                )
            elif mtype == "GPS_RAW_INT":
                lat = getattr(msg, "lat", None)
                lon = getattr(msg, "lon", None)
                if lat not in (None, 0, 0x7FFFFFFF):
                    out["lat"] = lat / 1e7
                if lon not in (None, 0, 0x7FFFFFFF):
                    out["lon"] = lon / 1e7
                sats = getattr(msg, "satellites_visible", None)
                if sats is not None:
                    out["sats"] = sats
            elif mtype == "VFR_HUD":
                gs = getattr(msg, "groundspeed", None)
                if gs is not None:
                    out["ground_speed"] = gs
                hdg = getattr(msg, "heading", None)
                if hdg is not None:
                    out["yaw"] = float(hdg)
                alt = getattr(msg, "alt", None)
                if alt is not None:
                    out["rel_alt"] = float(alt)

        return out

    def read_battery(
        self,
        wait: bool = False,
        timeout: float = 0.5,
    ) -> Dict[str, Optional[float]]:
        """Read battery info (voltage/current/remaining). Optionally wait briefly."""
        if self.dry or self.master is None:
            return {"voltage": 16.0, "current": 0.0, "remaining": 80.0}

        info: Dict[str, Optional[float]] = {"voltage": None, "current": None, "remaining": None}
        t0 = time.time()
        while True:
            msg = self.master.recv_match(type=["SYS_STATUS", "BATTERY_STATUS"], blocking=False)
            if msg is not None:
                mtype = msg.get_type()
                if mtype == "SYS_STATUS":
                    vb = getattr(msg, "voltage_battery", 65535)
                    if vb not in (0, 65535):
                        info["voltage"] = vb / 1000.0

                    cb = getattr(msg, "current_battery", -1)
                    if cb != -1:
                        info["current"] = cb / 100.0

                    br = getattr(msg, "battery_remaining", -1)
                    if br != -1:
                        info["remaining"] = float(br)

                elif mtype == "BATTERY_STATUS":
                    volts = getattr(msg, "voltages", None)
                    if volts:
                        v0 = volts[0]
                        if v0 not in (0, 0xFFFF):
                            info["voltage"] = v0 / 1000.0

                    cb = getattr(msg, "current_battery", -1)
                    if cb != -1:
                        info["current"] = cb / 100.0

                    br = getattr(msg, "battery_remaining", -1)
                    if br != -1:
                        info["remaining"] = float(br)

                # keep draining in this call to get freshest info
                continue

            if any(v is not None for v in info.values()):
                return info
            if not wait:
                return info
            if (time.time() - t0) > timeout:
                return info
            time.sleep(0.02)

    def read_rc_channels(self) -> Dict[int, Optional[int]]:
        """Non-blocking read of RC channels 1..8."""
        if self.dry or self.master is None:
            return {}

        msg = None
        while True:
            cur = self.master.recv_match(type=["RC_CHANNELS", "RC_CHANNELS_OVERRIDE"], blocking=False)
            if cur is None:
                break
            msg = cur  # keep the latest
        rc: Dict[int, Optional[int]] = {}
        if msg is not None:
            for i in range(1, 9):
                rc[i] = getattr(msg, f"ch{i}", None)
        return rc


if __name__ == "__main__":
    # Simple self-test: python app/io/link_mav.py
    try:
        from config import MAVLINK_DEVICE, MAVLINK_BAUD, DRY_RUN
    except Exception:
        MAVLINK_DEVICE = "udp:0.0.0.0:15001"
        MAVLINK_BAUD = 921600
        DRY_RUN = False

    mav = MavLink(MAVLINK_DEVICE, MAVLINK_BAUD, dry_run=DRY_RUN)
    print(f"[MavLink] Connect to {MAVLINK_DEVICE} (dry_run={mav.dry})")

    try:
        mav.connect()
        print("Heartbeat OK:", mav.heartbeat_ok())
        print("Mode:", mav.get_mode())
        print("Set LOITER:", mav.set_mode("LOITER"))
        print("Mode after set:", mav.get_mode())

        batt = mav.read_battery(wait=True, timeout=1.0)
        print("Battery:", batt)
    except Exception as exc:  # noqa: BLE001
        print("Connection failed:", exc)
