import time
import logging
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


# ======================
# 1. 数据结构（黑盒之间传的“包”）
# ======================

@dataclass
class TelemetrySnapshot:
    time: float = 0.0
    mode: str = "GUIDED"
    armed: bool = True
    voltage: float = 15.5
    alt: float = 10.0
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    sats: int = 10


@dataclass
class VisionResult:
    has_target: bool = False
    dx: float = 0.0
    dy: float = 0.0
    confidence: float = 0.0


@dataclass
class ControlCommand:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    gimbal_pitch_delta: float = 0.0
    gimbal_yaw_delta: float = 0.0
    drop: bool = False


@dataclass
class HudData:
    telemetry: TelemetrySnapshot
    vision: VisionResult
    mission_state: str = "IDLE"


# ======================
# 2. Logger
# ======================

def setup_logging(level: str = "DEBUG") -> logging.Logger:
    logger = logging.getLogger("uav_comp")
    logger.setLevel(getattr(logging, level.upper(), logging.DEBUG))
    if not logger.handlers:
        h = logging.StreamHandler()
        fmt = logging.Formatter("[%(asctime)s] [%(levelname)s] %(name)s - %(message)s")
        h.setFormatter(fmt)
        logger.addHandler(h)
    return logger


# ======================
# 3. 假飞控 / 传感器黑盒
# ======================

class FakeFcClient:
    """假 FC：不连真飞控，只生成一点假数据。"""

    def __init__(self, logger: logging.Logger) -> None:
        self.log = logger.getChild("FakeFc")
        self._start_time = time.time()

    def read_telemetry(self) -> TelemetrySnapshot:
        now = time.time() - self._start_time
        # 高度在 5~25m 来回摆
        alt = 15.0 + 10.0 * np.sin(now * 0.2)
        # 电压缓慢下降
        voltage = 16.8 - 0.01 * now
        ts = TelemetrySnapshot(
            time=time.time(),
            alt=alt,
            voltage=max(voltage, 13.0),
            yaw=(now * 10.0) % 360,
        )
        return ts

    def send_control(self, cmd: ControlCommand) -> None:
        # 真飞控这里应该发 MAVLink，这里只打印一下
        if abs(cmd.yaw_rate) > 1e-3 or cmd.drop:
            self.log.debug(
                "control: yaw_rate=%.3f vx=%.2f vy=%.2f drop=%s",
                cmd.yaw_rate,
                cmd.vx,
                cmd.vy,
                cmd.drop,
            )


# ======================
# 4. 假视觉黑盒
# ======================

class FakeVisionSystem:
    """
    假视觉：
    - 屏幕 640x480
    - 每隔几秒“随机出现一个目标”，dx/dy 慢慢逼近 0
    """

    def __init__(self, logger: logging.Logger) -> None:
        self.log = logger.getChild("FakeVision")
        self._start_time = time.time()
        self._dx = 150.0
        self._dy = -100.0

    def read(self) -> tuple[VisionResult, np.ndarray]:
        now = time.time() - self._start_time
        h, w = 480, 640
        frame = np.full((h, w, 3), 40, dtype=np.uint8)  # 暗底背景

        # 每 10 秒有 5 秒出现目标
        phase = now % 10.0
        has_target = 3.0 <= phase <= 8.0

        if has_target:
            # 逐渐往中心靠
            self._dx *= 0.9
            self._dy *= 0.9
        else:
            # 重置到远处
            self._dx = 150.0
            self._dy = -100.0

        # 屏幕中心
        cx, cy = w // 2, h // 2
        tx = int(cx + self._dx)
        ty = int(cy + self._dy)

        # 画十字中心
        cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)

        # 画目标
        if has_target:
            cv2.circle(frame, (tx, ty), 15, (0, 0, 255), 2)

        vr = VisionResult(
            has_target=has_target,
            dx=float(self._dx),
            dy=float(self._dy),
            confidence=0.9 if has_target else 0.0,
        )
        return vr, frame


# ======================
# 5. 对准控制 + 状态机黑盒
# ======================

class AlignController:
    def __init__(self, logger: logging.Logger) -> None:
        self.log = logger.getChild("Align")
        self.kx = 0.001  # yaw_rate ~ -kx * dx
        self.ky = 0.001
        self.dx_thresh = 10.0
        self.dy_thresh = 10.0

    def compute(self, vision: VisionResult) -> ControlCommand:
        cmd = ControlCommand()
        if not vision.has_target:
            return cmd
        dx, dy = vision.dx, vision.dy
        cmd.gimbal_yaw_delta = -self.kx * dx
        cmd.gimbal_pitch_delta = -self.ky * dy
        cmd.yaw_rate = -self.kx * dx
        return cmd

    def is_aligned(self, vision: VisionResult) -> bool:
        if not vision.has_target:
            return False
        return abs(vision.dx) < self.dx_thresh and abs(vision.dy) < self.dy_thresh


class MissionManager:
    """
    简化任务状态机：
    IDLE -> SEARCH -> ALIGN -> READY_TO_DROP -> DROP -> EXIT
    """

    def __init__(self, align: AlignController, logger: logging.Logger) -> None:
        self.align = align
        self.log = logger.getChild("Mission")
        self.state = "IDLE"
        self._drop_done = False

    def update(
        self, telem: TelemetrySnapshot, vision: VisionResult
    ) -> tuple[ControlCommand, str]:
        cmd = ControlCommand()

        if self.state == "IDLE":
            self.state = "SEARCH"
            self.log.info("State -> SEARCH")

        elif self.state == "SEARCH":
            if vision.has_target:
                self.state = "ALIGN"
                self.log.info("State -> ALIGN")

        elif self.state == "ALIGN":
            if not vision.has_target:
                self.state = "SEARCH"
                self.log.info("Target lost, back to SEARCH")
            else:
                cmd = self.align.compute(vision)
                if self.align.is_aligned(vision) and 8.0 <= telem.alt <= 20.0:
                    self.state = "READY_TO_DROP"
                    self.log.info("State -> READY_TO_DROP")

        elif self.state == "READY_TO_DROP":
            # 假设自动投弹（不等遥控开关）
            self.state = "DROP"
            self.log.info("State -> DROP")

        elif self.state == "DROP":
            if not self._drop_done:
                cmd.drop = True
                self._drop_done = True
            self.state = "EXIT"
            self.log.info("State -> EXIT")

        elif self.state == "EXIT":
            # 退出阶段什么都不做
            pass

        return cmd, self.state


# ======================
# 6. HUD / OSD 黑盒
# ======================

class HudRenderer:
    def __init__(self, logger: logging.Logger) -> None:
        self.log = logger.getChild("HUD")

    def render(self, frame: np.ndarray, hud: HudData) -> np.ndarray:
        if frame is None:
            return frame

        h, w = frame.shape[:2]

        # 中心十字
        cv2.drawMarker(
            frame,
            (w // 2, h // 2),
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=1,
        )

        t = hud.telemetry
        v = hud.vision
        lines = [
            f"MODE: {t.mode}  STATE: {hud.mission_state}",
            f"ALT: {t.alt:5.1f} m  V: {t.voltage:4.1f} V  SATS: {t.sats}",
            f"TARGET: {'YES' if v.has_target else 'NO '}  dx={v.dx:6.1f}  dy={v.dy:6.1f}  conf={v.confidence:.2f}",
        ]

        y = 20
        for line in lines:
            cv2.putText(
                frame,
                line,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
            y += 18

        return frame


# ======================
# 7. 主循环（双击就跑的假模拟）
# ======================

def main() -> None:
    logger = setup_logging("DEBUG")
    logger.info("Starting fake UAV competition loop (no real hardware).")

    fc = FakeFcClient(logger)
    vision = FakeVisionSystem(logger)
    align = AlignController(logger)
    mission = MissionManager(align, logger)
    hud = HudRenderer(logger)

    loop_hz = 20
    dt = 1.0 / loop_hz

    try:
        while True:
            t0 = time.time()

            telem = fc.read_telemetry()
            vr, frame = vision.read()
            cmd, state = mission.update(telem, vr)

            # 假执行控制命令
            fc.send_control(cmd)

            # HUD 显示
            hud_data = HudData(telemetry=telem, vision=vr, mission_state=state)
            out = hud.render(frame, hud_data)

            cv2.imshow("UAV Fake HUD", out)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC 退出
                break

            # 固定循环频率
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    finally:
        cv2.destroyAllWindows()
        logger.info("Exit.")


if __name__ == "__main__":
    main()
