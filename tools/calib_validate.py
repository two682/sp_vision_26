"""内参标定结果的多项误差检验。

跑四类检验，确认内参"不只是 RMS 好看，而是真的稳、真的准"：

    1. 逐图重投影误差统计（均值/中位/95分位/最大）
    2. 随机子集重标定 20 次，看 fx/fy/cx/cy 的离散程度（稳定性）
    3. 分半验证：前一半 vs 后一半分别标定，比较 K（一致性）
    4. 距离一致性：用不同子集标出的 K 对同一张图做 PnP，比较测距差异

用法：
    python3 tools/calib_validate.py --images /home/lie/图片
    python3 tools/calib_validate.py --images /home/lie/图片 --cols 7 --rows 7 --spacing 40
"""

from __future__ import annotations

import argparse
import glob
import pathlib

import cv2
import numpy as np


def find_images(folder: str) -> list[str]:
    exts = ("*.bmp", "*.png", "*.jpg", "*.jpeg", "*.tif", "*.tiff")
    files: list[str] = []
    for e in exts:
        files.extend(glob.glob(str(pathlib.Path(folder) / e)))
    return sorted(files, key=lambda s: int("".join(c for c in pathlib.Path(s).stem if c.isdigit()) or 0))


def collect(files, cols, rows, spacing):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * spacing
    objpoints, imgpoints, used = [], [], []
    shape = None
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        shape = img.shape
        ok, corners = cv2.findCirclesGrid(img, (cols, rows), flags=cv2.CALIB_CB_SYMMETRIC_GRID)
        if not ok:
            continue
        objpoints.append(objp)
        imgpoints.append(corners.astype(np.float32))
        used.append(f)
    return objpoints, imgpoints, used, shape


def calibrate(objpoints, imgpoints, shape):
    h, w = shape[:2]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    return rms, K, D, rvecs, tvecs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--cols", type=int, default=7)
    ap.add_argument("--rows", type=int, default=7)
    ap.add_argument("--spacing", type=float, default=40.0)
    ap.add_argument("--trials", type=int, default=20)
    args = ap.parse_args()

    files = find_images(args.images)
    objpoints, imgpoints, used, shape = collect(files, args.cols, args.rows, args.spacing)
    n = len(used)
    print(f"有效图片 {n} 张\n")

    # ---------- 1. 全量标定 + 误差统计 ----------
    rms, K, D, rvecs, tvecs = calibrate(objpoints, imgpoints, shape)
    errs = []
    for i in range(n):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        errs.append(cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / np.sqrt(len(proj)))
    errs = np.array(errs)

    print("=" * 56)
    print("① 逐图重投影误差")
    print(f"   RMS(标定给出) : {rms:.4f} px")
    print(f"   平均          : {errs.mean():.4f} px")
    print(f"   中位数        : {np.median(errs):.4f} px")
    print(f"   95 分位       : {np.percentile(errs, 95):.4f} px")
    print(f"   最大          : {errs.max():.4f} px")
    print(f"   低于 0.3px 的 : {(errs < 0.3).sum()}/{n}")

    # ---------- 2. 随机子集稳定性 ----------
    rng = np.random.default_rng(0)
    subset = max(8, int(n * 0.7))
    ks = []
    for _ in range(args.trials):
        idx = rng.choice(n, subset, replace=False)
        _, Ks, Ds, _, _ = calibrate([objpoints[i] for i in idx], [imgpoints[i] for i in idx], shape)
        ks.append([Ks[0, 0], Ks[1, 1], Ks[0, 2], Ks[1, 2]])
    ks = np.array(ks)

    print("\n" + "=" * 56)
    print(f"② 随机抽 {subset} 张重标定 {args.trials} 次（稳定性）")
    names = ["fx", "fy", "cx", "cy"]
    full = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
    for i, nm in enumerate(names):
        mu, sd = ks[:, i].mean(), ks[:, i].std()
        print(f"   {nm}: 均值 {mu:9.3f}  标准差 {sd:6.3f}  (波动 {100*sd/mu:5.2f}%)   全量值 {full[i]:9.3f}")

    # ---------- 3. 分半一致性 ----------
    half = n // 2
    _, K1, _, _, _ = calibrate(objpoints[:half], imgpoints[:half], shape)
    _, K2, _, _, _ = calibrate(objpoints[half:], imgpoints[half:], shape)
    print("\n" + "=" * 56)
    print("③ 分半验证（前一半 vs 后一半）")
    for i, nm in enumerate(names):
        v1 = [K1[0, 0], K1[1, 1], K1[0, 2], K1[1, 2]][i]
        v2 = [K2[0, 0], K2[1, 1], K2[0, 2], K2[1, 2]][i]
        print(f"   {nm}: A组 {v1:9.3f}   B组 {v2:9.3f}   差 {abs(v1-v2):6.3f} ({100*abs(v1-v2)/v1:5.2f}%)")

    # ---------- 4. 测距一致性 ----------
    print("\n" + "=" * 56)
    print("④ 测距一致性（同一张图，用 A/B 两组内参做 PnP）")

    def dist_of(Kx, Dx, i):
        # 圆点阵列近似平面，直接用 IPPE
        obj = objpoints[i].reshape(-1, 1, 3)
        img = imgpoints[i].reshape(-1, 1, 2)
        ok, rvec, tvec = cv2.solvePnP(obj, img, Kx, Dx, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return float("nan")
        return float(np.linalg.norm(tvec)) / 1000.0  # mm -> m

    diffs = []
    for i in range(n):
        d1 = dist_of(K1, D, i)
        d2 = dist_of(K2, D, i)
        if np.isfinite(d1) and np.isfinite(d2) and d2 > 0:
            diffs.append(abs(d1 - d2) / d2 * 100)
    diffs = np.array(diffs)
    print(f"   两组内参测同一目标，距离相对差：平均 {diffs.mean():.3f}%  最大 {diffs.max():.3f}%")
    print(f"   距离范围：{min(dist_of(K, D, i) for i in range(n)):.2f} ~ {max(dist_of(K, D, i) for i in range(n)):.2f} m")

    print("\n" + "=" * 56)
    ok_all = (
        rms < 0.3
        and 100 * ks[:, 0].std() / ks[:, 0].mean() < 1.5
        and 100 * ks[:, 2].std() / ks[:, 2].mean() < 0.5
        and diffs.mean() < 2.0
    )
    print("总体判断:", "✅ 通过（稳定、一致）" if ok_all else "⚠️ 有项超阈值，见上面数据")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
