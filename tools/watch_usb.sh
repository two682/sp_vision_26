#!/usr/bin/env bash
# 实时监视海康相机的 USB 速率与端口，方便你换口试
#   bash tools/watch_usb.sh
while true; do
  line=""
  for dev in /sys/bus/usb/devices/*/; do
    [ "$(cat "$dev/idVendor" 2>/dev/null)" = "2bdf" ] || continue
    spd=$(cat "$dev/speed" 2>/dev/null)
    port=$(basename "$(readlink -f "$dev")")
    case "$spd" in
      5000|10000) tag="USB3 ✅" ;;
      480)        tag="USB2 ❌" ;;
      *)          tag="?      " ;;
    esac
    line="  ${tag}  ${spd} Mbps   端口 ${port}"
  done
  printf '\r%-60s' "${line:-  未检测到相机（插上试试）}"
  sleep 1
done
