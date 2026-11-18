"""相机内参标定：从一组标定板照片算出 K / 畸变系数 / 重投影误差。

用法：
    python3 tools/calib_intrinsics.py --images ~/图片            # 标定并打印
    python3 tools/calib_intrinsics.py --images ~/图片 --update   # 同时写回 configs/hik.yaml

默认按"圆点阵列 7x7、点间距 40mm"处理（DFOPTIX HC7-400-20）。
如果用的是棋盘格，加 --board chessboard --cols 9 --rows 6。

输出：
    calibration/result/intrinsics.yaml   新内参
    calibration/result/report.txt        标定报告（含逐图误差）
    calibration/result/previous_*.txt    被替换掉的旧值备份（--update 时生成）
"""

from __future__ import annotations

import argparse
import datetime
import glob
import pathlib
import re
import sys

import cv2
import numpy as np


def find_images(folder: str) -> list[str]:
    exts = ("*.bmp", "*.png", "*.jpg", "*.jpeg", "*.tif", "*.tiff")
    files: list[str] = []
    for e in exts:
        files.extend(glob.glob(str(pathlib.Path(folder) / e)))
    return sorted(files, key=lambda s: int("".join(c for c in pathlib.Path(s).stem if c.isdigit()) or 0))


def detect(img_gray, board: str, cols: int, rows: int):
    if board == "circles":
        ok, corners = cv2.findCirclesGrid(
            img_gray, (cols, rows), flags=cv2.CALIB_CB_SYMMETRIC_GRID
        )
    else:
        ok, corners = cv2.findChessboardCorners(
            img_gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if ok:
            corners = cv2.cornerSubPix(
                img_gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
    return ok, corners


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="标定图片目录")
    ap.add_argument("--board", default="circles", choices=["circles", "chessboard"])
    ap.add_argument("--cols", type=int, default=7, help="圆点/角点列数")
    ap.add_argument("--rows", type=int, default=7, help="圆点/角点行数")
    ap.add_argument("--spacing", type=float, default=40.0, help="点间距/方格边长（毫米），不影响内参")
    ap.add_argument("--out", default="calibration/result", help="结果输出目录")
    ap.add_argument("--config", default="configs/hik.yaml", help="要更新的配置")
    ap.add_argument("--update", action="store_true", help="把结果写回配置文件")
    args = ap.parse_args()

    files = find_images(args.images)
    if not files:
        print(f"目录里没有图片: {args.images}")
        return 1
    print(f"找到 {len(files)} 张图片")

    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2) * args.spacing

    objpoints, imgpoints, used, failed = [], [], [], []
    shape = None
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            failed.append(pathlib.Path(f).name)
            continue
        shape = img.shape
        ok, corners = detect(img, args.board, args.cols, args.rows)
        if not ok:
            failed.append(pathlib.Path(f).name)
            continue
        objpoints.append(objp)
        imgpoints.append(corners.astype(np.float32))
        used.append(pathlib.Path(f).name)

    print(f"检出成功 {len(used)} 张，失败 {len(failed)} 张" + (f"（{', '.join(failed)}）" if failed else ""))
    if len(used) < 6:
        print("有效图片太少（<6），无法可靠标定")
        return 1

    h, w = shape[:2]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)

    per_image = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        e = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / np.sqrt(len(proj))
        per_image.append((used[i], float(e)))
    per_image.sort(key=lambda x: -x[1])

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = D.ravel().tolist()

    print(f"\nRMS 重投影误差: {rms:.4f} px")
    print(f"fx={fx:.3f}  fy={fy:.3f}  cx={cx:.3f}  cy={cy:.3f}")
    print(f"dist = [{', '.join(f'{v:.6e}' for v in d)}]")
    print(f"逐图误差: 平均 {np.mean([e for _, e in per_image]):.3f} / 最大 {per_image[0][1]:.3f} px")

    # ---- 写结果 ----
    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    today = datetime.date.today().isoformat()

    (out_dir / "intrinsics.yaml").write_text(
        f"""# 相机内参标定结果（{today}）
# 板型: {'圆点阵列' if args.board == 'circles' else '棋盘格'} {args.cols}x{args.rows}，间距 {args.spacing:g} mm
# 有效图片: {len(used)} 张，重投影 RMS: {rms:.4f} px
image_width: {w}
image_height: {h}
camera_matrix: [{fx:.6f}, 0, {cx:.6f}, 0, {fy:.6f}, {cy:.6f}, 0, 0, 1]
distort_coeffs: [{d[0]:.10e}, {d[1]:.10e}, {d[2]:.10e}, {d[3]:.10e}, {d[4]:.10e}]
rms_reprojection_error_px: {rms:.4f}
"""
    )

    lines = [
        f"相机内参标定报告  {today}",
        "=" * 52,
        f"标定板      : {'圆点阵列' if args.board == 'circles' else '棋盘格'} {args.cols}x{args.rows}，间距 {args.spacing:g} mm",
        f"图像分辨率  : {w} x {h}",
        f"有效图片    : {len(used)} / {len(files)}",
        f"未检出图片  : {', '.join(failed) if failed else '无'}",
        f"重投影 RMS  : {rms:.4f} px   （<0.3 优秀，<0.5 合格）",
        "",
        "内参",
        f"  fx = {fx:.3f}   fy = {fy:.3f}   cx = {cx:.3f}   cy = {cy:.3f}",
        f"  dist = [{', '.join(f'{v:.6e}' for v in d)}]",
        "",
        "逐图重投影误差（像素）",
    ]
    for n, e in per_image:
        lines.append(f"  {n:>12s}  {e:.3f}")

    # ---- 更新配置 ----
    if args.update:
        cfg_path = pathlib.Path(args.config)
        text = cfg_path.read_text()
        m = re.search(r"camera_matrix:\s*\[[^\]]*\]", text)
        old_k = m.group(0) if m else ""
        m2 = re.search(r"distort_coeffs:\s*\[[^\]]*\]", text)
        old_d = m2.group(0) if m2 else ""

        (out_dir / "previous_intrinsics.txt").write_text(
            "# 被替换掉的旧内参（保留备查）\n" + old_k + "\n" + old_d + "\n"
        )

        text = re.sub(
            r"camera_matrix:\s*\[[^\]]*\]",
            f"camera_matrix: [{fx:.6f}, 0, {cx:.6f}, 0, {fy:.6f}, {cy:.6f}, 0, 0, 1]",
            text,
        )
        text = re.sub(
            r"distort_coeffs:\s*\[[^\]]*\]",
            f"distort_coeffs: [{d[0]:.10e}, {d[1]:.10e}, {d[2]:.10e}, {d[3]:.10e}, {d[4]:.10e}]",
            text,
        )
        cfg_path.write_text(text)
        lines += ["", f"已写回配置: {args.config}", f"旧值备份: {out_dir/'previous_intrinsics.txt'}"]

    (out_dir / "report.txt").write_text("\n".join(lines) + "\n")
    print(f"\n结果目录: {out_dir}")
    print("  intrinsics.yaml / report.txt" + (" / previous_intrinsics.txt" if args.update else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
