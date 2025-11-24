from __future__ import annotations

from logging import Logger

from app.config import AppConfig
from app.core.models import ControlCommand, TelemetrySnapshot, VisionResult
from app.io.dropper_controller import DropperController
from app.io.gimbal_controller import GimbalController
from app.io.rc_input import RcInput
from .align_controller import AlignController


class MissionManager:
    """
    Demo 版状态机：
    IDLE -> SEARCH -> ALIGN -> READY_TO_DROP -> DROP -> EXIT
    """

    def __init__(
        self,
        align: AlignController,
        logger: Logger,
        cfg: AppConfig | None = None,
        rc: RcInput | None = None,
        gimbal: GimbalController | None = None,
        dropper: DropperController | None = None,
    ) -> None:
        self.align = align
        self.cfg = cfg
        self.rc = rc
        self.gimbal = gimbal
        self.dropper = dropper
        self.log = logger.getChild("Mission")
        self.state = "IDLE"
        self._drop_done = False

        if cfg:
            self.alt_low = cfg.flight.drop_alt - 2.0
            self.alt_high = cfg.flight.drop_alt + 10.0
        else:
            self.alt_low = 8.0
            self.alt_high = 20.0

    def update(
        self,
        telemetry: TelemetrySnapshot,
        vision: VisionResult,
    ) -> tuple[ControlCommand, str]:
        cmd = ControlCommand()

        if self.state == "IDLE":
            self.state = "SEARCH"
            self.log.info("State -> SEARCH")
            if self.gimbal:
                self.gimbal.set_scan_pose()

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
                if self.align.is_aligned(vision) and self.alt_low <= telemetry.alt <= self.alt_high:
                    self.state = "READY_TO_DROP"
                    self.log.info("State -> READY_TO_DROP")

        elif self.state == "READY_TO_DROP":
            drop_switch_on = self.rc.get_drop_switch() == "ON" if self.rc else True
            dropper_ready = self.dropper.is_ready() if self.dropper else True
            if drop_switch_on and dropper_ready:
                self.state = "DROP"
                self.log.info("State -> DROP")

        elif self.state == "DROP":
            if not self._drop_done:
                cmd.drop = True
                if self.dropper:
                    self.dropper.drop_once()
                self._drop_done = True
            self.state = "EXIT"
            self.log.info("State -> EXIT")

        elif self.state == "EXIT":
            pass

        return cmd, self.state
