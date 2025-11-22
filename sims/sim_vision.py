# sim_vision placeholder
from __future__ import annotations

import cv2
from app.logging_config import setup_logging
from app.config import load_config
from app.vision.system import VisionSystem


def main():
    cfg = load_config()
    logger = setup_logging(cfg)

    vision = VisionSystem(0, logger)

    while True:
        vr, frame = vision.read()
        if frame is None:
            continue
        # 简单画出中心
        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w // 2, h // 2), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        cv2.imshow("sim_vision", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    vision.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
