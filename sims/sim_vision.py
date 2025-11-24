# sim_vision placeholder
from __future__ import annotations

import cv2
from app.logging_config import setup_logging
from app.config import load_config
from app.vision.system import VisionSystem


def main():
    cfg = load_config()
    logger = setup_logging(cfg)

    vision_cfg = cfg.vision
    device = int(vision_cfg.device) if str(vision_cfg.device).isdigit() else vision_cfg.device
    vision = VisionSystem(
        device=device,
        logger=logger,
        width=vision_cfg.width,
        height=vision_cfg.height,
        pixel_format=vision_cfg.pixel_format,
        aruco_dict=vision_cfg.aruco_dict,
        marker_length_m=vision_cfg.marker_length_m,
        camera_matrix=vision_cfg.camera_matrix,
        dist_coeffs=vision_cfg.dist_coeffs,
    )

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
