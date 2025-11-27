from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.config import load_config  # noqa: E402
from app.io.fc_client import FcClient, FcClientConfig  # noqa: E402
from app.logging_config import setup_logging  # noqa: E402


def main() -> None:
    cfg = load_config()
    logger = setup_logging(cfg)

    fc = FcClient(FcClientConfig(endpoint=cfg.fc.endpoint, baud=cfg.fc.baud), logger)
    fc.connect()

    print(f"[telemetry] listening on {cfg.fc.endpoint} (Ctrl+C to stop)")
    try:
        while True:
            tel = fc.read_telemetry()
            rc = fc.read_rc_channels()
            print(
                f"time={tel.time:.1f} mode={tel.mode} armed={tel.armed} "
                f"V={tel.voltage:.2f}V alt={tel.alt:.1f}m yaw={tel.yaw:.1f} "
                f"lat={tel.lat:.7f} lon={tel.lon:.7f} sats={tel.sats} "
                f"vx={tel.vx:.2f} vy={tel.vy:.2f} rc={rc}"
            )
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopped by user.")


if __name__ == "__main__":
    main()
