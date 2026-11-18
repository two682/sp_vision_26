#!/usr/bin/env bash
# 相机链路自检：一次性打印所有关键信息
#   bash tools/check_hw.sh
set -u
cd "$(dirname "$0")/.." || exit 1

echo "==== 1. USB 设备 ===="
if lsusb | grep -qi 2bdf; then
  lsusb | grep -i 2bdf
else
  echo "  [FAIL] 总线上没有 2bdf 设备（海康相机没被认到）"
  echo "         -> 换 USB3 口直插，重插一次再看"
fi

echo
echo "==== 2. USB 速率（从 sysfs 读，最准） ===="
FOUND=0
for dev in /sys/bus/usb/devices/*/; do
  if [ "$(cat "$dev/idVendor" 2>/dev/null)" = "2bdf" ]; then
    FOUND=1
    spd=$(cat "$dev/speed" 2>/dev/null)
    prod=$(cat "$dev/product" 2>/dev/null)
    case "$spd" in
      5000|10000) mark="[OK]  USB3" ;;
      480)        mark="[WARN] USB2（带宽不够 120fps，换 USB3 口）" ;;
      *)          mark="[?]" ;;
    esac
    echo "  $prod  速率 ${spd} Mbps  $mark"
  fi
done
[ "$FOUND" = "0" ] && echo "  [FAIL] sysfs 里没有 2bdf 设备"

echo
echo "==== 3. SDK transport 库 ===="
for lib in libMvCameraControl.so libMvUsb3vTL.so libMvCamLVision.so; do
  if [ -f "io/lib/$lib" ]; then echo "  [OK]   io/lib/$lib"
  else echo "  [MISS] io/lib/$lib"; fi
done

echo
echo "==== 4. MVS 客户端 ===="
if [ -d /opt/MVS/lib/64 ]; then
  echo "  [OK] /opt/MVS/lib/64 存在"
  ls /opt/MVS/lib/64 2>/dev/null | grep -i "MvUsb3vTL\|MvCamLVision" | sed 's/^/       /'
else
  echo "  [MISS] 没装 MVS（海康官网下载 Linux 版）"
fi

echo
echo "==== 5. 设备节点权限 ===="
if ls /dev/bus/usb/*/* >/dev/null 2>&1; then
  ls -l /dev/bus/usb/*/* 2>/dev/null | tail -3
else
  echo "  [INFO] 看不到 /dev/bus/usb（沙箱或无权限）"
fi

echo
echo "==== 6. 占用进程 ===="
procs=$(ps -eo comm= 2>/dev/null | grep -iE "^(MVS|MvCameraControl)" | head -3)
if [ -n "$procs" ]; then echo "$procs"; else echo "  无"; fi

echo
echo "==== 7. udev 规则 ===="
if [ -f /etc/udev/rules.d/99-hikrobot.rules ]; then echo "  [OK] 已安装"; cat /etc/udev/rules.d/99-hikrobot.rules
else echo "  [MISS] 未安装（权限报错时再加）"; fi
