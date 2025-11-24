#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Auto capture + calibrate with a chessboard.")
    parser.add_argument("--device", default="/dev/video2", help="Camera device or index (default: /dev/video2)")
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
    parser.add_argument("--pattern-cols", type=int, default=10, help="Inner corners along columns (default: 10 for 11 squares)")
    parser.add_argument("--pattern-rows", type=int, default=7, help="Inner corners along rows (default: 7 for 8 squares)")
    parser.add_argument("--square-m", type=float, default=0.018, help="Square size in meters (default: 0.018 m)")
    parser.add_argument("--save-dir", default="calib_imgs", help="Directory to save captured images")
    parser.add_argument("--max-images", type=int, default=80, help="Max images to capture (default: 80)")
    parser.add_argument("--min-interval", type=float, default=0.5, help="Seconds between saves (default: 0.5)")
    parser.add_argument("--min-move-px", type=float, default=50.0, help="Min mean corner shift to accept a new sample (px)")
    parser.add_argument("--output", default="calibration/ost.yaml", help="Path to write calibration YAML")
    return parser.parse_args(argv)


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def draw_status(frame: np.ndarray, text: str, y: int = 20) -> None:
    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)


def save_frame(path: Path, frame: np.ndarray) -> None:
    cv2.imwrite(str(path), frame)


def should_save(last_corners: np.ndarray | None, corners: np.ndarray, min_move_px: float) -> bool:
    if last_corners is None:
        return True
    mean_shift = np.linalg.norm(last_corners.reshape(-1, 2) - corners.reshape(-1, 2), axis=1).mean()
    return mean_shift >= min_move_px


def calibrate_from_images(
    images_glob: str, pattern_size: tuple[int, int], square_size: float, out_path: Path
) -> None:
    files = sorted(glob.glob(images_glob))
    if not files:
        raise SystemExit(f"No images matched {images_glob}")

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    gray_shape = None

    for fname in files:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if gray_shape is None:
            gray_shape = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not ok:
            print(f"Skip {fname}: chessboard not found")
            continue
        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        objpoints.append(objp)
        imgpoints.append(corners2)
        print(f"Use {fname}")

    if not objpoints:
        raise SystemExit("No valid chessboard detections to calibrate.")

    ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)
    print(f"RMS reprojection error: {ret:.6f}")
    print("camera_matrix:\n", mtx)
    print("dist_coeffs:\n", dist.ravel())

    out_path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "image_width": gray_shape[0],
        "image_height": gray_shape[1],
        "camera_matrix": {"rows": 3, "cols": 3, "data": mtx.flatten().tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": dist.flatten().tolist()},
    }
    with out_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f)
    print(f"Saved calibration to {out_path}")


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv or sys.argv[1:])
    ensure_dir(Path(args.save_dir))

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        raise SystemExit(f"Failed to open camera {args.device}")

    pattern_size = (args.pattern_cols, args.pattern_rows)
    last_corners = None
    last_save_ts = 0.0
    saved = 0

    print("Auto capture started. Move the chessboard around the view.")
    print("ESC to finish and calibrate. Space forces a save when detected.")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Camera read failed.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        # 兼容棋盘方向相反的情况，尝试交换行列
        if not found:
            alt_size = (pattern_size[1], pattern_size[0])
            found, corners = cv2.findChessboardCorners(gray, alt_size)
            if found:
                pattern_size = alt_size

        draw_status(frame, f"Detected: {found}  Saved: {saved}/{args.max_images}")

        if found:
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            cv2.drawChessboardCorners(frame, pattern_size, corners2, True)

            now = time.time()
            can_save = (now - last_save_ts) >= args.min_interval and (
                last_corners is None or should_save(last_corners, corners2, args.min_move_px)
            )
            key = cv2.waitKey(1) & 0xFF

            if can_save or key == ord(" "):
                fname = Path(args.save_dir) / f"img_{saved:04d}.png"
                save_frame(fname, frame)
                print(f"Saved {fname}")
                saved += 1
                last_save_ts = now
                last_corners = corners2.copy()
            elif last_corners is not None:
                draw_status(frame, "Move board more for next sample", y=40)

        cv2.imshow("auto_calibrate", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        if saved >= args.max_images:
            print("Reached max images, calibrating...")
            break

    cap.release()
    cv2.destroyAllWindows()

    if saved < 5:
        print("Not enough images to calibrate.")
        return

    calibrate_from_images(
        images_glob=str(Path(args.save_dir) / "img_*.png"),
        pattern_size=pattern_size,
        square_size=args.square_m,
        out_path=Path(args.output),
    )


if __name__ == "__main__":
    main()
