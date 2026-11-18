// 实时检测：相机取图 -> 检测装甲板 -> 画框显示（不接云台、不发 CAN）
//
//   ./build/detect_camera configs/hik.yaml                 # 相机实时
//   ./build/detect_camera configs/hik.yaml out/live.avi    # 同时录像
//   ./build/detect_camera configs/hik.yaml "" records/armor_test_1_match.avi   # 用视频代替相机
//
// 按键：q / ESC 退出，s 存当前帧
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <algorithm>
#include <cstdlib>
#include <string>

#include <opencv2/opencv.hpp>
#include <fmt/format.h>

#include "tasks/auto_aim/detector.hpp"
#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/hik.yaml";
  const std::string out_path = argc > 2 ? argv[2] : "";
  const std::string video_path = argc > 3 ? argv[3] : "";

  std::printf("配置: %s\n", config_path.c_str());

  auto_aim::Detector detector(config_path);

  cv::VideoCapture video;
  const bool from_video = !video_path.empty();
  if (from_video) {
    video.open(video_path);
    if (!video.isOpened()) {
      std::printf("打不开视频: %s\n", video_path.c_str());
      return 1;
    }
    std::printf("输入: 视频 %s\n", video_path.c_str());
  } else {
    std::printf("输入: 相机\n");
  }

  std::unique_ptr<io::Camera> camera;
  if (!from_video) {
    camera = std::make_unique<io::Camera>(config_path);

    // 第一帧最多等 8 秒；等不到就说明相机没开，直接给结论，不要无限重试
    cv::Mat probe;
    std::chrono::steady_clock::time_point t_probe;
    auto fut = std::async(std::launch::async, [&] { camera->read(probe, t_probe); });
    if (fut.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
      std::printf("\n[FAIL] 8 秒内没有画面，相机没打开。\n\n");
      std::printf("最常见原因是设备被占用（MV_CC_OpenDevice 0x80000203）：\n");
      std::printf("  ps aux | grep -iE \"MVS|detect_camera|camera_info\" | grep -v grep\n");
      std::printf("  pkill -f MVS        # 关掉 MVS 客户端\n");
      std::printf("  pkill -f detect_camera\n");
      std::printf("  lsusb -d 2bdf:0001  # 看设备号，例如 Bus 002 Device 006\n");
      std::printf("  sudo fuser -v /dev/bus/usb/002/006   # 查是谁占着\n");
      std::printf("\n仍不行就拔插一次相机（被强杀进程后设备可能卡在 busy 状态）。\n");
      std::exit(2);
    }
    fut.get();
  }

  cv::VideoWriter writer;
  long long frames = 0, hit = 0, armors_total = 0;
  double ms_sum = 0, lat_sum = 0, lat_max = 0;
  const auto t_start = std::chrono::steady_clock::now();

  cv::Mat img;
  for (int i = 0;; ++i) {
    double latency_ms = 0.0;
    if (from_video) {
      if (!video.read(img) || img.empty()) break;
    } else {
      std::chrono::steady_clock::time_point t;
      camera->read(img, t);
      if (img.empty()) continue;
      // 相机打时间戳 -> 现在：采集/传输/排队耗时（不含曝光的前半段）
      latency_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
      lat_sum += latency_ms;
      lat_max = std::max(lat_max, latency_ms);
    }

    if (writer.isOpened() == false && !out_path.empty() && !img.empty()) {
      writer.open(
        out_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(img.cols, img.rows));
      if (writer.isOpened()) std::printf("录像到: %s\n", out_path.c_str());
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto armors = detector.detect(img, i);
    const double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    ms_sum += ms;
    ++frames;
    armors_total += static_cast<long long>(armors.size());
    if (!armors.empty()) ++hit;

    for (const auto & armor : armors) {
      const std::vector<cv::Point> poly = {
        cv::Point(cvRound(armor.points[0].x), cvRound(armor.points[0].y)),
        cv::Point(cvRound(armor.points[1].x), cvRound(armor.points[1].y)),
        cv::Point(cvRound(armor.points[2].x), cvRound(armor.points[2].y)),
        cv::Point(cvRound(armor.points[3].x), cvRound(armor.points[3].y))};
      const cv::Scalar color =
        (armor.color == auto_aim::Color::red) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
      cv::polylines(img, poly, true, color, 2);
      tools::draw_text(
        img,
        fmt::format("{} {} {:.2f}", auto_aim::COLORS[armor.color],
                    auto_aim::ARMOR_NAMES[armor.name], armor.confidence),
        poly[0], color);
    }

    cv::putText(
      img,
      fmt::format("{} armors  proc {:.1f} ms  lat {:.1f} ms", armors.size(), ms, latency_ms),
      {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

    if (writer.isOpened()) writer.write(img);

    int key = -1;
    if (std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr) {
      cv::imshow("detect_camera", img);
      key = cv::waitKey(1);
    }
    if (key == 'q' || key == 27) break;
    if (key == 's') {
      const std::string path = fmt::format("out/snap_{}.png", frames);
      cv::imwrite(path, img);
      std::printf("已保存 %s\n", path.c_str());
    }
  }

  if (writer.isOpened()) writer.release();

  const double elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - t_start)
                           .count();
  std::printf("\n==== 检测统计 ====\n");
  std::printf("帧数     : %lld\n", frames);
  std::printf("检出帧   : %lld (%.1f%%)\n", hit, frames ? 100.0 * hit / frames : 0.0);
  std::printf("平均目标 : %.2f\n", frames ? double(armors_total) / frames : 0.0);
  std::printf("单帧耗时 : %.2f ms（处理）\n", frames ? ms_sum / frames : 0.0);
  std::printf("采集延迟 : 平均 %.2f ms / 最大 %.2f ms\n", frames ? lat_sum / frames : 0.0, lat_max);
  std::printf("总耗时   : %.2f s (平均 %.1f fps)\n", elapsed, elapsed > 0 ? frames / elapsed : 0);
  return 0;
}
