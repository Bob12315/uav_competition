from __future__ import annotations

import time
from dataclasses import dataclass
from logging import Logger
from math import sin

from app.core.models import ControlCommand, TelemetrySnapshot


@dataclass
class FcClientConfig:
    device: str = "/dev/ttyS1"
    baud: int = 921600


class FcClient:
    """
    MAVLink 客户端占位实现：
    - 实际工程中，你在这里接 pymavlink 或 mavsdk。
    """

    def __init__(self, cfg: FcClientConfig, logger: Logger) -> None:
        self.cfg = cfg
        self.log = logger.getChild("FcClient")
        self._connected = False

    def connect(self) -> None:
        # TODO: 替换为真实串口/UDP 连接
        self.log.info("Connecting to FC at %s baud %d", self.cfg.device, self.cfg.baud)
        time.sleep(0.5)
        self._connected = True
        self.log.info("FC connected (placeholder).")

    def heartbeat_ok(self) -> bool:
        # TODO: 检查最近一次 HEARTBEAT 时间
        return self._connected

    # ===== 下面这些函数都要用 MAVLink 实现 =====

    def read_telemetry_raw(self) -> dict:
        """
        返回一个简单 dict，包含姿态/位置/电池信息。
        这里只是示例，实际应从 MAVLink 消息解析。
        """
        now = time.time()
        # 假数据：高度缓慢上升
        alt = 10.0 + 5.0 * (0.5 - (now % 1.0))
        return {
            "time": now,
            "mode": "GUIDED",
            "armed": True,
            "voltage": 15.5,
            "alt": alt,
            "vx": 0.0,
            "vy": 0.0,
            "yaw": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "sats": 10,
        }

    def read_rc_channels(self) -> dict[int, int]:
        """
        返回 {通道号: pwm}，这里用假数据。
        """
        return {
            5: 1500,  # mode channel
            6: 1000,  # drop channel
        }

    def send_velocity_body(self, vx: float, vy: float, vz: float) -> None:
        self.log.debug("send_velocity_body vx=%.3f vy=%.3f vz=%.3f", vx, vy, vz)
        # TODO: 发送 SET_POSITION_TARGET_LOCAL_NED

    def send_yaw_rate(self, yaw_rate: float) -> None:
        self.log.debug("send_yaw_rate rate=%.3f", yaw_rate)
        # TODO: 发送 yaw rate 命令

    def send_gimbal_angles(self, pitch_deg: float, yaw_deg: float) -> None:
        self.log.debug("send_gimbal pitch=%.1f yaw=%.1f", pitch_deg, yaw_deg)
        # TODO: MAV_CMD_DO_MOUNT_CONTROL 或 DO_SET_SERVO

    def send_servo(self, channel: int, pwm: int) -> None:
        self.log.debug("send_servo ch=%d pwm=%d", channel, pwm)
        # TODO: MAV_CMD_DO_SET_SERVO


class FakeFcClient:
    """
    纯软件 Demo 用的假 FC：
    - read_telemetry() 生成随时间变化的高度、电压、yaw
    - send_control() 只打印 yaw_rate / drop 行为
    """

    def __init__(self, logger: Logger) -> None:
        self.log = logger.getChild("FakeFc")
        self._start_time = time.time()

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

    def send_control(self, cmd: ControlCommand) -> None:
        if abs(cmd.yaw_rate) > 1e-3 or cmd.drop:
            self.log.debug(
                "control: yaw_rate=%.3f vx=%.2f vy=%.2f drop=%s",
                cmd.yaw_rate,
                cmd.vx,
                cmd.vy,
                cmd.drop,
            )
