#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Chessboard calibration to ost.yaml")
    parser.add_argument("--images-glob", default="calib_imgs/*.png", help="Glob pattern for chessboard images")
    parser.add_argument("--pattern-cols", type=int, default=10, help="Inner corners along columns (squares-1)")
    parser.add_argument("--pattern-rows", type=int, default=7, help="Inner corners along rows (squares-1)")
    parser.add_argument("--square-m", type=float, default=0.018, help="Square size in meters (default: 0.018 m)")
    parser.add_argument("--output", default="calibration/ost.yaml", help="Output YAML path")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv or sys.argv[1:])

    pattern_size = (args.pattern_cols, args.pattern_rows)
    square_size = args.square_m

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []

    files = sorted(glob.glob(args.images_glob))
    if not files:
        raise SystemExit(f"No images found for pattern {args.images_glob}")

    gray_shape = None
    for fname in files:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if gray_shape is None:
            gray_shape = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not ret:
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
        raise SystemExit("No valid chessboard detections.")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_shape, None, None
    )
    print(f"RMS reprojection error: {ret:.6f}")
    print("camera_matrix:\n", mtx)
    print("dist_coeffs:\n", dist.ravel())

    out_path = Path(args.output)
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


if __name__ == "__main__":
    main()
