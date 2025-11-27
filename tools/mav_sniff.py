from __future__ import annotations

import time
from collections import Counter
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.config import load_config
from app.io.link_mav import MavLink


def main() -> None:
    cfg = load_config()
    mav = MavLink(cfg.fc.endpoint, cfg.fc.baud, dry_run=cfg.fc.use_fake)
    print(f"[sniff] connecting to {cfg.fc.endpoint} (dry_run={mav.dry})")
    mav.connect()

    counts: Counter[str] = Counter()
    start = time.time()
    last_print = start

    # last known values
    hb_info = {"mode": None, "armed": False}
    att_info = {}
    batt_info = {}
    rc_info = {}

    while True:
        # drain queue and keep latest messages
        if mav.master:
            while True:
                msg = mav.master.recv_match(blocking=False)
                if msg is None:
                    break
                counts[msg.get_type()] += 1

                mtype = msg.get_type()
                if mtype == "HEARTBEAT":
                    hb = mav.poll_heartbeat()
                    hb_info.update({k: v for k, v in hb.items() if v is not None})
                elif mtype in ("ATTITUDE", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "VFR_HUD"):
                    att = mav.read_attitude_speed_alt()
                    att_info.update({k: v for k, v in att.items() if v is not None})
                elif mtype in ("SYS_STATUS", "BATTERY_STATUS"):
                    batt = mav.read_battery(wait=False)
                    batt_info.update({k: v for k, v in batt.items() if v is not None})
                elif mtype in ("RC_CHANNELS", "RC_CHANNELS_OVERRIDE"):
                    rc = mav.read_rc_channels()
                    rc_info.update(rc)

        now = time.time()
        if now - last_print > 1.0:
            print(
                f"[sniff] t={now-start:5.1f}s "
                f"HB mode={hb_info.get('mode')} armed={hb_info.get('armed')} "
                f"v={att_info.get('ground_speed')} yaw={att_info.get('yaw')} alt={att_info.get('rel_alt')} "
                f"lat={att_info.get('lat')} lon={att_info.get('lon')} sats={att_info.get('sats')} "
                f"Vbat={batt_info.get('voltage')}"
            )
            top = counts.most_common(8)
            if top:
                print("       msg counts:", ", ".join(f"{k}:{v}" for k, v in top))
            if rc_info:
                print("       RC ch:", rc_info)
            last_print = now

        time.sleep(0.05)


if __name__ == "__main__":
    main()
