# sim_mission placeholder
from __future__ import annotations

import time

from app.config import load_config
from app.logging_config import setup_logging
from app.core.models import TelemetrySnapshot, VisionResult
from app.io.fc_client import FcClient, FcClientConfig
from app.io.sensors import Sensors
from app.io.rc_input import RcInput
from app.io.gimbal_controller import GimbalController
from app.io.dropper_controller import DropperController
from app.mission.align_controller import AlignController
from app.mission.mission_manager import MissionManager


def main():
    cfg = load_config()
    logger = setup_logging(cfg)

    fc = FcClient(FcClientConfig(), logger)
    fc.connect()

    sensors = Sensors(fc, logger)
    rc = RcInput(fc, cfg.rc_switch, logger)
    gimbal = GimbalController(fc, cfg.gimbal, logger)
    dropper = DropperController(fc, cfg.dropper, logger)
    align = AlignController(cfg.align, logger)

    mission = MissionManager(cfg, rc, gimbal, dropper, align, logger)

    t0 = time.time()
    while True:
        telem: TelemetrySnapshot = sensors.read()

        # 构造一个假 VisionResult：每隔两秒“出现目标”
        now = time.time() - t0
        has_target = int(now) % 4 in (2, 3)
        vr = VisionResult(has_target=has_target, dx=5.0, dy=-3.0, confidence=0.8 if has_target else 0.0)

        cmd, state = mission.update(telem, vr)

        logger.info(
            "state=%s alt=%.1f has_target=%s drop=%s yaw_rate=%.3f",
            state,
            telem.alt,
            vr.has_target,
            cmd.drop,
            cmd.yaw_rate,
        )

        time.sleep(0.5)


if __name__ == "__main__":
    main()
