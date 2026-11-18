// 离线视频检测：逐帧跑传统 CV 检测器，输出带框视频 + 统计。
//
//   ./build/detect_video <video> [--config configs/example.yaml] [--out out/detect.avi]
//                        [--start 0] [--end 0] [--stride 1]
//
// end=0 表示跑到视频结束；stride>1 时跳帧（用于快速抽样）。
#include <fmt/core.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace {

struct Options
{
  std::string video;
  std::string config = "configs/example.yaml";
  std::string out = "out/detect.avi";
  int start = 0;
  int end = 0;
  int stride = 1;
  bool use_yolo = false;
  bool ok = true;
};

// 不用 cv::CommandLineParser：OpenCV 4.5.4 里 "-c value" 这种写法会返回 "true"。
Options parse(int argc, char ** argv)
{
  Options opt;
  bool has_video = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    std::string value;
    bool has_value = false;

    const auto eq = arg.find('=');
    if (eq != std::string::npos && arg.size() > 1 && arg[0] == '-') {
      value = arg.substr(eq + 1);
      arg = arg.substr(0, eq);
      has_value = true;
    }

    auto take = [&](std::string & dst) {
      if (has_value) {
        dst = value;
      } else if (i + 1 < argc) {
        dst = argv[++i];
      } else {
        opt.ok = false;
      }
    };

    if (arg == "-h" || arg == "--help" || arg == "-help") {
      opt.ok = false;
    } else if (arg == "-c" || arg == "--config" || arg == "--config-path") {
      take(opt.config);
    } else if (arg == "-o" || arg == "--out") {
      take(opt.out);
    } else if (arg == "-s" || arg == "--start") {
      std::string v;
      take(v);
      opt.start = std::atoi(v.c_str());
    } else if (arg == "-e" || arg == "--end") {
      std::string v;
      take(v);
      opt.end = std::atoi(v.c_str());
    } else if (arg == "-y" || arg == "--yolo") {
      opt.use_yolo = true;
    } else if (arg == "--stride") {
      std::string v;
      take(v);
      opt.stride = std::max(1, std::atoi(v.c_str()));
    } else if (!arg.empty() && arg[0] == '-') {
      std::printf("未知参数: %s\n", arg.c_str());
      opt.ok = false;
    } else if (!has_video) {
      opt.video = arg;
      has_video = true;
    }
  }
  return opt;
}

void usage()
{
  std::printf(
    "用法: detect_video <video> [选项]\n"
    "  -c, --config <path>   配置文件（默认 configs/example.yaml）\n"
    "  -o, --out <path>      输出视频（默认 out/detect.avi，留空不写）\n"
    "  -s, --start <n>       起始帧\n"
    "  -e, --end <n>         结束帧（0 = 到结尾）\n"
    "      --stride <n>      每隔 n 帧抽一帧\n"
    "  -y, --yolo            用 YOLO 检测（默认传统 CV）\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  const Options opt = parse(argc, argv);
  if (!opt.ok || opt.video.empty()) {
    usage();
    return opt.ok ? 0 : 1;
  }

  const std::string & video_path = opt.video;
  const std::string & config_path = opt.config;
  const std::string & out_path = opt.out;
  const int start_index = opt.start;
  const int end_index = opt.end;
  const int stride = opt.stride;

  if (video_path.empty()) {
    std::printf("缺少视频路径，用 -h 看用法\n");
    return 1;
  }

  if (video_path.find_first_of("/.") == std::string::npos) {
    // 没有扩展名、也不是路径 -> 多半是误把 "out/cam_rec.avi" 当成了图片序列
    std::printf("路径看起来不是视频文件: %s\n", video_path.c_str());
  }

  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    std::printf("打不开视频: %s\n", video_path.c_str());
    std::printf("  1) 文件是否存在:   ls -l %s\n", video_path.c_str());
    std::printf("  2) 是否录制成功:   先跑 camera_info 生成视频\n");
    std::printf("  3) 支持的格式:     .avi(.mjpg) / .mp4\n");
    return 1;
  }
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  const int width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH));
  const int height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT));
  const double fps = video.get(cv::CAP_PROP_FPS) > 1.0 ? video.get(cv::CAP_PROP_FPS) : 30.0;

  auto_aim::Detector detector(config_path);
  std::unique_ptr<auto_aim::YOLO> yolo;
  if (opt.use_yolo) {
    yolo = std::make_unique<auto_aim::YOLO>(config_path);
  }

  cv::VideoWriter writer;
  if (!out_path.empty()) {
    const std::string fourcc = out_path.size() > 4 && out_path.substr(out_path.size() - 4) == ".mp4"
                                 ? "mp4v"
                                 : "MJPG";
    writer.open(out_path, cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]), fps,
                cv::Size(width, height));
    if (!writer.isOpened()) {
      std::printf("无法写视频: %s\n", out_path.c_str());
    }
  }

  long long frames = 0;
  long long hit_frames = 0;
  long long armor_total = 0;
  double ms_sum = 0.0;

  cv::Mat img;
  for (int frame_count = start_index; !img.empty() || frames == 0; ++frame_count) {
    if (!video.read(img) || img.empty()) break;
    if (end_index > 0 && frame_count > end_index) break;
    if ((frame_count - start_index) % stride != 0) continue;

    const auto begin = std::chrono::steady_clock::now();
    auto armors = opt.use_yolo ? yolo->detect(img, frame_count) : detector.detect(img, frame_count);
    const auto end = std::chrono::steady_clock::now();
    ms_sum += std::chrono::duration<double, std::milli>(end - begin).count();

    ++frames;
    armor_total += static_cast<long long>(armors.size());
    if (!armors.empty()) ++hit_frames;

    for (const auto & armor : armors) {
      const std::vector<cv::Point> poly = {
        cv::Point(cvRound(armor.points[0].x), cvRound(armor.points[0].y)),
        cv::Point(cvRound(armor.points[1].x), cvRound(armor.points[1].y)),
        cv::Point(cvRound(armor.points[2].x), cvRound(armor.points[2].y)),
        cv::Point(cvRound(armor.points[3].x), cvRound(armor.points[3].y))};
      const cv::Scalar color = (armor.color == auto_aim::Color::red) ? cv::Scalar(0, 0, 255)
                                                                     : cv::Scalar(255, 0, 0);
      cv::polylines(img, poly, true, color, 2);
      tools::draw_text(
        img, fmt::format("{} {} {:.2f}", auto_aim::COLORS[armor.color],
                         auto_aim::ARMOR_NAMES[armor.name], armor.confidence),
        poly[0], color);
    }

    if (writer.isOpened()) writer.write(img);
  }

  if (writer.isOpened()) writer.release();

  const double rate = frames > 0 ? static_cast<double>(hit_frames) / frames : 0.0;
  const double avg_ms = frames > 0 ? ms_sum / frames : 0.0;

  std::printf("\n==== 离线检测结果 ====\n");
  std::printf("视频        : %s (%dx%d @ %.0f fps)\n", video_path.c_str(), width, height, fps);
  std::printf("处理帧数    : %lld (stride=%d)\n", frames, stride);
  std::printf("检出帧数    : %lld\n", hit_frames);
  std::printf("检出率      : %.1f%%\n", rate * 100.0);
  std::printf("平均装甲板数: %.2f / 帧\n", frames > 0 ? static_cast<double>(armor_total) / frames : 0.0);
  std::printf("平均耗时    : %.2f ms / 帧\n", avg_ms);
  if (!out_path.empty()) std::printf("输出视频    : %s\n", out_path.c_str());
  return 0;
}
