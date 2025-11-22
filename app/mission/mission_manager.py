# MissionManager placeholder
from __future__ import annotations

from logging import Logger

from app.config import AppConfig
from app.core.models import TelemetrySnapshot, VisionResult, ControlCommand
from app.io.rc_input import RcInput
from app.io.gimbal_controller import GimbalController
from app.io.dropper_controller import DropperController
from .align_controller import AlignController


class MissionManager:
    """
    简化版状态机：
    IDLE -> SEARCH -> ALIGN -> READY_TO_DROP -> DROP -> EXIT
    """

    def __init__(
        self,
        cfg: AppConfig,
        rc: RcInput,
        gimbal: GimbalController,
        dropper: DropperController,
        align: AlignController,
        logger: Logger,
    ) -> None:
        self.cfg = cfg
        self.rc = rc
        self.gimbal = gimbal
        self.dropper = dropper
        self.align = align
        self.log = logger.getChild("Mission")
        self.state = "IDLE"

    def update(
        self,
        telemetry: TelemetrySnapshot,
        vision: VisionResult,
    ) -> tuple[ControlCommand, str]:
        cmd = ControlCommand()

        mode = self.rc.get_mode()

        if self.state == "IDLE":
            if mode != "MANUAL":
                self.state = "SEARCH"
                self.log.info("State -> SEARCH")
                self.gimbal.set_scan_pose()

        elif self.state == "SEARCH":
            # 简单示例：如果看到目标就进入 ALIGN
            if vision.has_target:
                self.state = "ALIGN"
                self.log.info("State -> ALIGN")

        elif self.state == "ALIGN":
            if not vision.has_target:
                # 目标丢失，回 SEARCH
                self.state = "SEARCH"
                self.log.info("Target lost, back to SEARCH")
            else:
                # 调用 AlignController
                cmd = self.align.compute(vision)
                if self.align.is_aligned(vision) and abs(telemetry.alt - self.cfg.flight.drop_alt) < 2.0:
                    self.state = "READY_TO_DROP"
                    self.log.info("State -> READY_TO_DROP")

        elif self.state == "READY_TO_DROP":
            # 等待遥控开关确认
            if self.rc.get_drop_switch() == "ON" and self.dropper.is_ready():
                self.state = "DROP"
                self.log.info("State -> DROP")

        elif self.state == "DROP":
            cmd.drop = True
            self.state = "EXIT"
            self.log.info("State -> EXIT")

        elif self.state == "EXIT":
            # 简单示意：退出时不做动作，你可以改成拉升 / RTL
            pass

        return cmd, self.state
