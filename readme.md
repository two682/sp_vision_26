# sp_vision_26

基于 [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) 裁剪并修改的视觉框架，用于 RoboMaster 自瞄。

- 保留：传统 CV 检测链路、EKF 跟踪、PnP 解算、弹道补偿、CAN 云台通信、相机驱动（USB / 迈德威视 / 海康）
- 改动：剥离 OpenVINO 硬依赖（改为可选）、修复 headless 崩溃、修二值化与配色、include 路径扁平化
- 详细改动清单见 [开源修改记录.md](开源修改记录.md)

## 目录结构

```
sp_vision_26/
├── assets/tiny_resnet.onnx     # 数字分类器模型
├── configs/                    # example.yaml（主配置）、standard4.yaml
├── io/                         # 相机工厂 + USB/迈德威视/海康驱动 + CAN 云台通信
│   ├── lib/                    # 相机 SDK 预编译库（amd64）
│   └── *.h                     # 相机 SDK 头文件
├── tasks/auto_aim/             # 自瞄核心：armor/detector/classifier/solver/target/tracker/aimer/shooter/voter
├── tools/                      # logger / yaml / math_tools / EKF / crc / pid / ransac / trajectory / img_tools
├── src/                        # 各兵种入口（从上游复原，默认不编译）
├── tests/                      # 上游测试（从上游复原，默认不编译）
├── CMakeLists.txt
└── 开源修改记录.md
```

## 依赖

```bash
sudo apt install build-essential cmake libopencv-dev libeigen3-dev \
                 libfmt-dev libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev
```

## OpenVINO（可选）

本机 `apt` 源里没有 OpenVINO 包，官网 tgz 又被网络拦截，因此改用 pip 把运行时装进项目目录：

```bash
# 装在项目外，避免污染工程目录
cd /home/lie/文档/deepseek-harness/default-workspace
pip3 install --no-cache-dir --target openvino openvino
```

CMake 按此顺序查找：环境变量 `OPENVINO_ROOT` → 项目上一级 `../openvino/openvino` → 项目内 `third_party/openvino/openvino`。

安装后目录结构为 `../openvino/openvino/{include,libs}`，其中：

| 路径 | 内容 |
| --- | --- |
| `include/openvino/` | C++ 头文件 |
| `libs/libopenvino.so` | 运行时（已建好不带版本号软链） |
| `libs/libopenvino_*_plugin.so` | CPU / GPU / NPU 插件 |
| `openvino/` | Python 包（`PYTHONPATH` 指向后可 `import openvino`） |

找到就定义 `SPV_HAS_OPENVINO` 并链接；找不到就跳过（classifier 退回 `cv::dnn`）。

自检：

```bash
./build/ov_smoke assets/tiny_resnet.onnx
# OpenVINO: OpenVINO Runtime
# 输入: 1 个，输出: 1 个 ... 推理成功，单次耗时 0.6 ms
```

## 构建

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

产物：

| 目标 | 说明 |
| --- | --- |
| `libspv_tools.a` | 基础工具库 |
| `libspv_auto_aim.a` | 自瞄算法库 |
| `libspv_io.a` | 相机 / 云台通信库 |
| `ov_smoke` | OpenVINO 自检（启用时生成） |

## 配置

`configs/example.yaml` 关键项：

```yaml
classify_model: assets/tiny_resnet.onnx
enemy_color: "blue"          # 敌方颜色，tracker 按此过滤
threshold: 150               # 灯条亮度阈值（HSV V 通道）
camera_matrix: [...]         # 相机内参
distort_coeffs: [...]        # 畸变系数
```

装了 `third_party/openvino` 时自动走 OpenVINO 推理；没装则退回 `cv::dnn`，无需改配置。

## 未搬入的模块

YOLO 检测器、能量机关、哨兵感知、MPC 规划器、ROS2 接口、串口库、标定工具等仍未搬入，原因见 [开源修改记录.md](开源修改记录.md)。
