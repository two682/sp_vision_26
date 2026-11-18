#!/usr/bin/env bash
# 在「新标定」和「旧标定」之间切换（只改 configs/hik.yaml 的 camera_matrix/distort_coeffs）
#   bash calibration/switch.sh new    # 2026-11-17 标定（RMS 0.047）
#   bash calibration/switch.sh old    # 2026-11-16 标定
set -eu
cd "$(dirname "$0")/.."

NEW_K="[2404.048819, 0, 715.464987, 0, 2403.674096, 540.743026, 0, 0, 1]"
NEW_D="[-0.032271593414, 0.35905530601, 0.00011494981319, 0.00039870897997, -1.8532416696]"
OLD_K="[2411.6509, 0, 713.2076, 0, 2412.0543, 528.3691, 0, 0, 1]"
OLD_D="[-2.261528e-02, -2.964997e-02, -4.300408e-04, -8.991199e-05, 1.486106]"

case "${1:-}" in
  new) K="$NEW_K"; D="$NEW_D"; TAG="2026-11-17 新标定 (RMS 0.047px)" ;;
  old) K="$OLD_K"; D="$OLD_D"; TAG="2026-11-16 旧标定" ;;
  *) echo "用法: bash calibration/switch.sh new|old"; exit 1 ;;
esac

python3 - "$K" "$D" "$TAG" <<'PY'
import re, sys, pathlib
K, D, TAG = sys.argv[1], sys.argv[2], sys.argv[3]
p = pathlib.Path("configs/hik.yaml")
s = p.read_text()
s = re.sub(r"# 当前使用：.*", f"# 当前使用：{TAG}", s)
s = re.sub(r"camera_matrix:\s*\[[^\]]*\]", f"camera_matrix: {K}", s)
s = re.sub(r"distort_coeffs:\s*\[[^\]]*\]", f"distort_coeffs: {D}", s)
p.write_text(s)
print(f"已切换到: {TAG}")
print(f"  camera_matrix: {K}")
print(f"  distort_coeffs: {D}")
PY
