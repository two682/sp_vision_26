# 硬件接入说明（海康 MV-CS016-10UC）

## 设备信息

| 项目 | 值 |
| --- | --- |
| 型号 | Hikrobot MV-CS016-10UC（USB3.0，160 万像素） |
| USB ID | `2bdf:0001` |
| 分辨率 | 1440 × 1080（默认，可在 MVS 里改 ROI） |
| 传感器 | 1/2.9" CMOS，3.45 µm 像元，全局快门 |

## 相机内参

### 2026-11-16 标定（已弃用，实测距离偏小更多）

```
K = [[2411.6509,    0.0,     713.2076]
     [   0.0,    2412.0543,  528.3691]
     [   0.0,       0.0,       1.0   ]]

dist = [-2.261528e-02, -2.964997e-02, -4.300408e-04, -8.991199e-05, 1.486106]
```

### 当前使用：2026-11-17 标定

圆点阵列 7×7、间距 40 mm、20 张，重投影 RMS **0.047 px**：

```
fx = 2404.049   fy = 2403.674   cx = 715.465   cy = 540.743
dist = [-3.227159e-02, 3.590553e-01, 1.149498e-04, 3.987090e-04, -1.853242]
```

文件：`calibration/result/intrinsics.yaml`、`report.txt`、`validation.txt`。

说明：两套内参实测都比真实距离偏小，但 11-17 这套误差更小，故采用。
距离偏小是系统性偏差，主因可能是打印装甲板的实际宽度大于代码假设值
（小装甲板 135 mm / 大装甲板 230 mm），需实测灯条中心距后校正。

## 外参（还没标）

```yaml
R_gimbal2imubody: [1,0,0, 0,1,0, 0,0,1]   # 云台 -> IMU 本体：单位阵占位
R_camera2gimbal:  [1,0,0, 0,1,0, 0,0,1]   # 相机 -> 云台：单位阵占位
t_camera2gimbal:  [0,0,0]                 # 相机安装平移
```

这三个是**手眼标定**的产物，现在用单位阵占位，意思是"假设相机与云台同轴、无平移"。
在装上车、云台能动之后，必须用 `calibrate_handeye` 标出来，否则世界坐标会整体偏。

## 配置项

`configs/hik.yaml`：

```yaml
camera_name: "hikrobot"
vid_pid: "2bdf:0001"
exposure_ms: 3.0      # 室内先 3 ms，根据亮度调
gain: 16.0            # 增益，噪点多就降
```

## 第一步：确认相机能出图

```bash
cd /home/lie/文档/deepseek-harness/default-workspace/sp_vision_26

# 抓 30 帧，存一张 out/camera_check.png，并打印分辨率/帧率/亮度
./build/camera_info configs/hik.yaml 30 out
```

预期输出：

```
==== 相机自检 ====
分辨率   : 1440 x 1080
抓帧数   : 30 / 30
实测帧率 : 120.0 fps
样例图   : out/camera_check.png
画面亮度 : 均值 90.0  标准差 40.0 （建议均值 60~180）
```

- **报 `Unable to open usb` / `0x80000006`** → 权限问题，见下一节
- **亮度 < 60** → 加大 `exposure_ms`
- **亮度 > 180** → 减小 `exposure_ms` 或降 `gain`

## 常见错误：`Unable to open usb!` / `MV_CC_EnumDevices failed: 0x80000006`

这个组合几乎都是**同一个原因**：SDK 的 USB3 传输层没加载/没权限。

`libMvCameraControl.so` 内部会 `dlopen` 两个库：

```
libMvUsb3vTL.so     ← USB3 Vision 传输层（关键）
libMvCamLVision.so
```

我们的 `io/lib/` 里**只有 libMvCameraControl.so**，缺 `libMvUsb3vTL.so`，
所以它会报 "Unable to open usb!"。

### 解决办法（按顺序试）

**① 先排除权限**

```bash
sudo ./build/camera_info configs/hik.yaml 10 out
```

- `sudo` 能出图 → 纯权限问题，做下面 ②；
- `sudo` 也报同样的错 → 继续 ③（缺库）。

**② 加 udev 规则**

```bash
sudo tee /etc/udev/rules.d/99-hikrobot.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
# 拔插相机后重试
```

**③ 补 transport 库（最可能）**

从海康官网下载 **MVS 客户端（Linux 版）** 安装后，库在：

```
/opt/MVS/lib/64/libMvUsb3vTL.so
/opt/MVS/lib/64/libMvCamLVision.so
```

拷到工程里：

```bash
cp /opt/MVS/lib/64/libMvUsb3vTL.so   io/lib/
cp /opt/MVS/lib/64/libMvCamLVision.so io/lib/
```

或者直接 `export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH` 再跑。

装完 MVS 后再确认一次：

```bash
ldd io/lib/libMvCameraControl.so | grep -i mv     # 看依赖
./build/camera_info configs/hik.yaml 30 out
```

**④ 检查 USB3 速率**

```bash
lsusb -t | grep -A2 2bdf
```

出现 `5000M` 才对；如果是 `480M`，说明插在 USB2 口/线上，换主机后面的 USB3 口直插。

**⑤ 关掉占用设备的程序**

MVS 客户端开着时，相机可能被独占，关掉再跑。

## 权限问题（udev）

```bash
sudo tee /etc/udev/rules.d/99-hikrobot.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
# 拔插一次相机，然后重新跑 camera_info
```

先确认是不是权限问题：

```bash
sudo ./build/camera_info configs/hik.yaml 10 out
```

`sudo` 能跑通 = 权限问题；`sudo` 也报错 = 驱动/SDK 问题。

## 第二步：录一段视频，离线验证检测

```bash
./build/camera_info configs/hik.yaml 300 out out/camera_rec.avi
./build/detect_video out/camera_rec.avi -c configs/hik.yaml -o out/detect_rec.avi
```

这一步不需要 CAN、不需要场地，用你自己的画面验证：
- 检测框准不准
- 阈值 `threshold` / `min_confidence` 合不合适

## 第三步：全链路（需要云台 CAN）

```bash
# 先看 CAN 通不通
ip link show can0

./build/standard -c=configs/hik.yaml
```

`standard` 会：相机取图 → 检测 → 跟踪 → 解算 → 通过 CAN 发云台指令。

## 待确认

1. **畸变系数**：标定输出里没有 `k1,k2,p1,p2,k3`，现在是全 0。如果镜头畸变明显（画面边缘弯曲），PnP 解算会偏，需要补标。
2. **实际分辨率**：如果 MVS 里改过 ROI，`cx=691` 可能对应更窄的画面（如 1382 宽），需要确认相机输出的真实宽高。
3. **曝光/增益**：比赛现场光照差异大，建议白天/晚上各录一段，取中间值。
