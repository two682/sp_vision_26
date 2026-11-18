// 多线程检测器（MultiThreadDetector）验证：push/pop 流水线
//
//   ./build/mt_probe configs/hik.yaml                 # 相机
//   ./build/mt_probe configs/hik.yaml 200             # 指定帧数
//   ./build/mt_probe configs/hik.yaml 200 records/armor_test_1_match.avi
//
// 说明：MultiThreadDetector 是 `standard` 使用的生产路径——
//       主线程 push 图像，OpenVINO 异步推理，另一个时刻 pop 结果，形成流水线。
//       它只支持 YOLO（传统 CV 没有多线程版本）。
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  const std::string config = argc > 1 ? argv[1] : "configs/hik.yaml";
  const int frames = argc > 2 ? std::atoi(argv[2]) : 200;
  const std::string video_path = argc > 3 ? argv[3] : "";
  const bool single_thread = argc > 4 && std::string(argv[4]) == "st";

  std::printf("配置: %s\n输入: %s\n帧数: %d\n\n", config.c_str(),
              video_path.empty() ? "相机" : video_path.c_str(), frames);

  std::unique_ptr<auto_aim::multithread::MultiThreadDetector> mt;
  std::unique_ptr<auto_aim::YOLO> yolo;
  if (single_thread) {
    yolo = std::make_unique<auto_aim::YOLO>(config, false);
    std::printf("模式: 单线程 YOLO\n\n");
  } else {
    mt = std::make_unique<auto_aim::multithread::MultiThreadDetector>(config, false);
    std::printf("模式: 多线程流水线\n\n");
  }

  cv::VideoCapture video;
  std::unique_ptr<io::Camera> camera;
  if (!video_path.empty()) {
    video.open(video_path);
    if (!video.isOpened()) {
      std::printf("打不开视频: %s\n", video_path.c_str());
      return 1;
    }
  } else {
    camera = std::make_unique<io::Camera>(config);
  }

  constexpr int PIPELINE_DEPTH = 4;  // 流水线深度：先压 4 帧再开始取结果

  long long pushed = 0, popped = 0, det_frames = 0, armor_total = 0;
  double latency_sum = 0, latency_max = 0;

  cv::Mat img;
  const auto t_start = std::chrono::steady_clock::now();

  auto drain_one = [&]() {
    auto [armors, t] = mt->pop();
    ++popped;
    if (!armors.empty()) ++det_frames;
    armor_total += static_cast<long long>(armors.size());
    const double lat =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
    latency_sum += lat;
    latency_max = std::max(latency_max, lat);
  };

  for (int i = 0; i < frames; ++i) {
    std::chrono::steady_clock::time_point t;
    if (!video_path.empty()) {
      if (!video.read(img) || img.empty()) break;
      t = std::chrono::steady_clock::now();
    } else {
      camera->read(img, t);
      if (img.empty()) continue;
    }
    if (single_thread) {
      auto armors = yolo->detect(img, i);
      ++pushed;
      ++popped;
      if (!armors.empty()) ++det_frames;
      armor_total += static_cast<long long>(armors.size());
      const double lat =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
      latency_sum += lat;
      latency_max = std::max(latency_max, lat);
    } else {
      mt->push(img.clone(), t);
      ++pushed;
      if (pushed > PIPELINE_DEPTH) drain_one();
    }
  }
  if (!single_thread) {
    while (popped < pushed) drain_one();
  }

  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

  std::printf("\n==== 多线程检测汇总 ====\n");
  std::printf("帧数        : %lld\n", pushed);
  std::printf("检出帧      : %lld (%.1f%%)\n", det_frames, pushed ? 100.0 * det_frames / pushed : 0);
  std::printf("平均装甲板  : %.2f\n", pushed ? double(armor_total) / pushed : 0);
  std::printf("流水线深度  : %d\n", PIPELINE_DEPTH);
  std::printf("总耗时      : %.2f s\n", elapsed);
  std::printf("吞吐        : %.1f fps\n", elapsed > 0 ? pushed / elapsed : 0);
  std::printf("单帧延迟    : 平均 %.2f ms / 最大 %.2f ms\n", popped ? latency_sum / popped : 0,
              latency_max);
  return 0;
}
