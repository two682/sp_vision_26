#!/usr/bin/env bash
# 开机自启 / 现场拉起脚本。
#   ./autostart.sh                 # 默认配置
#   CONFIG=configs/buff.yaml ./autostart.sh
#   MCU=/dev/ttyUSB0 ./autostart.sh
set -euo pipefail

cd "$(dirname "$0")"

BIN="${BIN:-build/spv_auto_aim}"
CONFIG="${CONFIG:-configs/auto_aim.yaml}"
CALIB="${CALIB:-calibration/camera.yaml}"
LOG_DIR="${LOG_DIR:-logs}"
mkdir -p "$LOG_DIR"

if [[ ! -x "$BIN" ]]; then
  echo "找不到可执行文件 $BIN，请先 cmake -B build && cmake --build build -j" >&2
  exit 1
fi

args=(--config "$CONFIG" --calib "$CALIB")
if [[ -n "${MCU:-}" ]]; then
  args+=(--mcu "$MCU")
fi

# 现场默认无显示器，直接 headless；需要看画面时 HEADLESS=0。
if [[ "${HEADLESS:-1}" == "1" ]]; then
  args+=(--headless)
fi

exec "$BIN" "${args[@]}" "$@" 2>&1 | tee -a "$LOG_DIR/vision-$(date +%Y%m%d).log"
