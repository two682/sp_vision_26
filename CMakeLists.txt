cmake_minimum_required(VERSION 3.16)
project(sp_vision_26 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# 依赖
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d highgui dnn)
find_package(Threads REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATHS /usr/include/eigen3 /usr/local/include/eigen3)
  if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 not found: sudo apt install libeigen3-dev")
  endif()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# ---------------------------------------------------------------------------
# OpenVINO（可选）：third_party/openvino 由 pip install --target 安装
#   启用后定义 SPV_HAS_OPENVINO，classifier 会编译 OpenVINO 推理路径
# ---------------------------------------------------------------------------
option(SPV_WITH_OPENVINO "使用 OpenVINO 运行时（默认找项目上一级的 openvino/）" ON)
set(SPV_HAS_OPENVINO FALSE)

if(SPV_WITH_OPENVINO)
  # 优先级：环境变量 OPENVINO_ROOT > 项目上一级 openvino/ > 项目内 third_party/openvino
  if(DEFINED ENV{OPENVINO_ROOT})
    set(OV_ROOT "$ENV{OPENVINO_ROOT}")
  elseif(EXISTS "${CMAKE_SOURCE_DIR}/../openvino/openvino/include/openvino/openvino.hpp")
    set(OV_ROOT "${CMAKE_SOURCE_DIR}/../openvino/openvino")
  else()
    set(OV_ROOT "${CMAKE_SOURCE_DIR}/third_party/openvino/openvino")
  endif()
  if(EXISTS "${OV_ROOT}/include/openvino/openvino.hpp" AND EXISTS "${OV_ROOT}/libs/libopenvino.so")
    add_library(openvino_runtime SHARED IMPORTED)
    set_target_properties(openvino_runtime PROPERTIES
      IMPORTED_LOCATION "${OV_ROOT}/libs/libopenvino.so"
      INTERFACE_INCLUDE_DIRECTORIES "${OV_ROOT}/include"
    )
    set(SPV_HAS_OPENVINO TRUE)
    message(STATUS "OpenVINO: 已启用 (${OV_ROOT})")
  else()
    message(STATUS "OpenVINO: 未找到 ${OV_ROOT}，跳过（classifier 仅用 cv::dnn）")
  endif()
endif()

set(SPV_CXX_WARNINGS -Wall -Wextra -Wno-unused-parameter)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/io
  ${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim
  ${CMAKE_CURRENT_SOURCE_DIR}/tools
  ${OpenCV_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 静态库
# ---------------------------------------------------------------------------
add_library(spv_tools STATIC
  tools/crc.cpp
  tools/exiter.cpp
  tools/extended_kalman_filter.cpp
  tools/img_tools.cpp
  tools/logger.cpp
  tools/math_tools.cpp
  tools/pid.cpp
  tools/plotter.cpp
  tools/recorder.cpp
  tools/ransac_sine_fitter.cpp
  tools/trajectory.cpp
)
target_link_libraries(spv_tools PUBLIC ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp Threads::Threads)
if(Eigen3_FOUND)
  target_link_libraries(spv_tools PUBLIC Eigen3::Eigen)
endif()
target_compile_options(spv_tools PRIVATE ${SPV_CXX_WARNINGS})

add_library(spv_auto_aim STATIC
  tasks/auto_aim/aimer.cpp
  tasks/auto_aim/armor.cpp
  tasks/auto_aim/classifier.cpp
  tasks/auto_aim/detector.cpp
  tasks/auto_aim/shooter.cpp
  tasks/auto_aim/solver.cpp
  tasks/auto_aim/target.cpp
  tasks/auto_aim/tracker.cpp
  tasks/auto_aim/voter.cpp
  tasks/auto_aim/yolo.cpp
  tasks/auto_aim/multithread/commandgener.cpp
  tasks/auto_aim/multithread/mt_detector.cpp
  tasks/auto_aim/yolos/yolov5.cpp
  tasks/auto_aim/yolos/yolov8.cpp
  tasks/auto_aim/yolos/yolo11.cpp
)
target_link_libraries(spv_auto_aim PUBLIC spv_tools ${OpenCV_LIBS})
if(Eigen3_FOUND)
  target_link_libraries(spv_auto_aim PUBLIC Eigen3::Eigen)
endif()
if(SPV_HAS_OPENVINO)
  target_link_libraries(spv_auto_aim PUBLIC openvino_runtime)
  target_compile_definitions(spv_auto_aim PUBLIC SPV_HAS_OPENVINO=1)
endif()
target_compile_options(spv_auto_aim PRIVATE ${SPV_CXX_WARNINGS})

# 串口库（gimbal / dm_imu 依赖）
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/io/serial ${CMAKE_BINARY_DIR}/serial EXCLUDE_FROM_ALL)

add_library(spv_io STATIC
  io/camera.cpp
  io/cboard.cpp
  io/hikrobot.cpp
  io/mindvision.cpp
  io/usbcamera.cpp
  io/gimbal/gimbal.cpp
  io/dm_imu/dm_imu.cpp
)
target_link_libraries(spv_io PUBLIC spv_tools ${OpenCV_LIBS} Threads::Threads)
target_include_directories(spv_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/io)
# 相机 SDK 预编译库
target_link_directories(spv_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/io/lib)
set_target_properties(spv_io PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/io/lib")
target_link_libraries(spv_io PUBLIC MvCameraControl MVSDK usb-1.0 serial)
target_compile_options(spv_io PRIVATE ${SPV_CXX_WARNINGS})

# ---------------------------------------------------------------------------
# 步兵主程序（src/standard.cpp）
# ---------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/standard.cpp")
  add_executable(standard src/standard.cpp)
  target_link_libraries(standard PRIVATE spv_auto_aim spv_io spv_tools)
  if(SPV_HAS_OPENVINO)
    target_link_libraries(standard PRIVATE openvino_runtime)
  endif()
endif()

# ---------------------------------------------------------------------------
# MPC 弹道规划（可选，依赖 TinyMPC；默认关闭）
# ---------------------------------------------------------------------------
option(SPV_WITH_MPC "编译 TinyMPC 规划器与 standard_mpc" OFF)
if(SPV_WITH_MPC)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim/planner/tinympc ${CMAKE_BINARY_DIR}/tinympc)
  add_library(spv_planner STATIC tasks/auto_aim/planner/planner.cpp)
  target_link_libraries(spv_planner PUBLIC tinympcstatic spv_tools Eigen3::Eigen)
  target_include_directories(spv_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_aim)
  target_compile_options(spv_planner PRIVATE ${SPV_CXX_WARNINGS})
  # standard_mpc 还依赖已删除的 auto_buff，等需要大符时再放开
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/standard_mpc.cpp" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_buff/buff_aimer.hpp")
    add_executable(standard_mpc src/standard_mpc.cpp)
    target_link_libraries(standard_mpc PRIVATE spv_planner spv_auto_aim spv_io spv_tools)
  endif()
endif()

# ---------------------------------------------------------------------------
# 标定工具（相机内参 / 手眼 / 机器人世界 / 录图 / 拆视频）
# ---------------------------------------------------------------------------
option(SPV_BUILD_CALIB "编译 calibration/ 下的标定工具" ON)
if(SPV_BUILD_CALIB AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/calibration/calibrate_camera.cpp")
  foreach(cal capture calibrate_camera calibrate_handeye calibrate_robotworld_handeye split_video)
    add_executable(${cal} calibration/${cal}.cpp)
    target_link_libraries(${cal} PRIVATE spv_auto_aim spv_io spv_tools)
  endforeach()
endif()

# ---------------------------------------------------------------------------
# 相机自检（接硬件后第一个跑）
# ---------------------------------------------------------------------------
add_executable(camera_info tools/camera_info.cpp)
target_link_libraries(camera_info PRIVATE spv_io spv_tools)

add_executable(auto_aim_probe tools/auto_aim_probe.cpp)
target_link_libraries(auto_aim_probe PRIVATE spv_auto_aim spv_io spv_tools)

add_executable(mt_probe tools/mt_probe.cpp)
target_link_libraries(mt_probe PRIVATE spv_auto_aim spv_io spv_tools)

add_executable(detect_camera tools/detect_camera.cpp)
target_link_libraries(detect_camera PRIVATE spv_auto_aim spv_io spv_tools)

# ---------------------------------------------------------------------------
# 离线视频检测工具
# ---------------------------------------------------------------------------
add_executable(detect_video tools/detect_video.cpp)
target_link_libraries(detect_video PRIVATE spv_auto_aim)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/detector_video_test.cpp")
  add_executable(detector_video_test tests/detector_video_test.cpp)
  target_link_libraries(detector_video_test PRIVATE spv_auto_aim)
endif()

# ---------------------------------------------------------------------------
# OpenVINO 自检程序
# ---------------------------------------------------------------------------
if(SPV_HAS_OPENVINO)
  add_executable(ov_smoke tools/ov_smoke.cpp)
  target_link_libraries(ov_smoke PRIVATE openvino_runtime)
endif()

# ---------------------------------------------------------------------------
# 示例程序：相机链路自检（无相机也能编译）
# ---------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tasks/apps/camera_smoke.cpp")
  add_executable(camera_smoke tasks/apps/camera_smoke.cpp)
  target_link_libraries(camera_smoke PRIVATE spv_io spv_auto_aim)
endif()

# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# 可选：编译 src/ 下的入口程序（需要先补搬 auto_buff / multithread / ros2 等模块）
# ---------------------------------------------------------------------------
option(SPV_BUILD_APPS "编译 src/ 下的入口程序（依赖尚未搬全，默认关闭）" OFF)
if(SPV_BUILD_APPS)
  file(GLOB SPV_APP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
  foreach(app_src ${SPV_APP_SOURCES})
    get_filename_component(app_name ${app_src} NAME_WE)
    add_executable(${app_name} ${app_src})
    target_link_libraries(${app_name} PRIVATE spv_auto_aim spv_io)
  endforeach()
endif()
