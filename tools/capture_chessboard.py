#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture chessboard images for calibration.")
    parser.add_argument("--device", default="/dev/video2", help="Camera device path or index (default: /dev/video2)")
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
    parser.add_argument("--out", default="calib_imgs", help="Output folder for saved frames (default: calib_imgs)")
    parser.add_argument("--max", type=int, default=60, help="Max images to capture (default: 60)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.device}")

    print("Press SPACE to save a frame with chessboard, ESC to quit.")
    print(f"Saving to {out_dir} (up to {args.max} images).")

    idx = len(list(out_dir.glob("img_*.png")))
    last_save = 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Camera read failed.")
            break

        cv2.putText(
            frame,
            f"Saved: {idx}/{args.max}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        cv2.imshow("capture_chessboard", frame)
        key = cv2.waitKey(1) & 0xFF
        now = time.time()

        if key == 27:  # ESC
            break
        if key == ord(" ") and idx < args.max and now - last_save > 0.2:
            fname = out_dir / f"img_{idx:04d}.png"
            cv2.imwrite(str(fname), frame)
            print(f"Saved {fname}")
            idx += 1
            last_save = now
        if idx >= args.max:
            print("Reached max images.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
