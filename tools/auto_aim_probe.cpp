// auto_aim 各模块逐级验证工具（不需要云台、不需要 CAN）
//
// 单级运行：
//   ./build/auto_aim_probe configs/hik.yaml det 200            # 只跑检测器
//   ./build/auto_aim_probe configs/hik.yaml all 200 "" yolo    # 全链路 + YOLO
//
// 分级运行（每跑完一级停下来，按 Enter 进下一级）：
//   ./build/auto_aim_probe configs/hik.yaml step 200
//
// 参数：<配置> <阶段|step> [帧数] [视频路径] [yolo]
// 按键：q/ESC 退出，s 截图，Enter 进入下一级（step 模式）
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <list>
#include <map>
#include <memory>
#include <string>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace
{

struct StageStats
{
  long long n = 0, det_frames = 0, armor_total = 0, target_frames = 0, fire_count = 0;
  double det_ms = 0, cls_ms = 0, solve_ms = 0, track_ms = 0, aim_ms = 0;

  // 跟踪质量统计
  std::map<std::string, long long> state_count;
  long long acquire = 0;      // 进入 tracking 的次数
  long long lost = 0;         // 从 tracking 掉出的次数
  long long first_track = -1; // 首次进入 tracking 的帧号

  // 瞄准统计
  long long cmd_frames = 0;
  double yaw_sum = 0, pitch_sum = 0, yaw_min = 1e9, yaw_max = -1e9, pitch_min = 1e9, pitch_max = -1e9;
};

class Source
{
public:
  Source(const std::string & config, const std::string & video_path)
  {
    if (!video_path.empty()) {
      video_.open(video_path);
      if (!video_.isOpened()) throw std::runtime_error("打不开视频: " + video_path);
    } else {
      camera_ = std::make_unique<io::Camera>(config);
    }
  }

  bool from_video() const { return camera_ == nullptr; }

  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & t)
  {
    if (from_video()) {
      if (!video_.read(img) || img.empty()) return false;
      t = std::chrono::steady_clock::now();
      return true;
    }
    camera_->read(img, t);
    return !img.empty();
  }

  void rewind()
  {
    if (from_video()) video_.set(cv::CAP_PROP_POS_FRAMES, 0);
  }

private:
  cv::VideoCapture video_;
  std::unique_ptr<io::Camera> camera_;
};

void draw_armors(cv::Mat & img, const std::list<auto_aim::Armor> & armors)
{
  for (const auto & a : armors) {
    if (a.points.size() != 4) continue;

    std::vector<cv::Point> poly;
    for (const auto & p : a.points) poly.emplace_back(cvRound(p.x), cvRound(p.y));

    // 绿色外框：把整个装甲板圈起来，最直观
    cv::Rect box = cv::boundingRect(poly);
    const int pad = 8;
    box.x -= pad;
    box.y -= pad;
    box.width += pad * 2;
    box.height += pad * 2;
    box &= cv::Rect(0, 0, img.cols, img.rows);
    cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    // 四角也画出来，便于看贴合度
    for (const auto & p : poly) cv::circle(img, p, 4, cv::Scalar(0, 255, 0), -1);

    // 颜色仍用红/蓝细线区分阵营
    const cv::Scalar color =
      (a.color == auto_aim::Color::red) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
    cv::polylines(img, poly, true, color, 1, cv::LINE_AA);

    cv::putText(
      img,
      fmt::format("{} {} {:.2f}", auto_aim::COLORS[a.color], auto_aim::ARMOR_NAMES[a.name],
                  a.confidence),
      cv::Point(box.x, std::max(16, box.y - 8)), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  }
}

void draw_hud(
  cv::Mat & img, const std::string & stage, const std::string & state, size_t n_armors,
  size_t n_targets, double det_ms, bool has_cmd, double yaw, double pitch, bool fire,
  const std::string & banner, double distance = -1.0, const Eigen::Vector3d & xyz = Eigen::Vector3d::Zero())
{
  int y = 30;
  auto put = [&](const std::string & text, cv::Scalar color, double scale = 0.62) {
    cv::putText(img, text, {10, y}, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
    y += 27;
  };

  put("stage: " + stage, {0, 255, 0}, 0.7);
  put(fmt::format("armors: {}   targets: {}", n_armors, n_targets), {0, 255, 0});
  put("state: " + state, {0, 255, 255});
  put(fmt::format("detect: {:.1f} ms", det_ms), {0, 255, 0});
  if (distance > 0) {
    put(fmt::format("dist: {:.2f} m", distance), {0, 200, 255});
    put(fmt::format("xyz: {:+.2f} {:+.2f} {:+.2f} m", xyz[0], xyz[1], xyz[2]), {0, 200, 255});
  }
  if (has_cmd) put(fmt::format("cmd: yaw {:+.3f} pitch {:+.3f} rad", yaw, pitch), {255, 255, 0});
  put(fmt::format("fire: {}", fire ? "YES" : "no"),
      fire ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 200, 200));

  if (!banner.empty()) {
    int base = 0;
    const auto size = cv::getTextSize(banner, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &base);
    cv::putText(
      img, banner, {(img.cols - size.width) / 2, img.rows - 60}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
      {0, 255, 255}, 2, cv::LINE_AA);
  }
}

bool has_display()
{
  return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
}

// 等待 Enter（窗口模式），无显示时退化为回车输入
bool wait_enter(cv::Mat & img, const std::string & hint)
{
  if (!has_display()) {
    std::printf("%s  （按 Enter 继续，输入 q 退出）\n", hint.c_str());
    std::fflush(stdout);
    const int c = std::getchar();
    return c != 'q' && c != 'Q' && c != EOF;
  }

  cv::Mat show = img.clone();
  int base = 0;
  const auto size = cv::getTextSize(hint, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &base);
  cv::putText(
    show, hint, {(show.cols - size.width) / 2, show.rows - 60}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
    {0, 255, 255}, 2, cv::LINE_AA);

  while (true) {
    cv::imshow("auto_aim_probe", show);
    const int key = cv::waitKey(30);
    if (key == 13 || key == 10) return true;  // Enter
    if (key == 'q' || key == 27 || key == 'Q') return false;
  }
}

StageStats run_stage(
  const std::string & stage, int frames, Source & source, const std::string & config, bool use_yolo,
  bool until_enter = false, bool debug_windows = false)
{
  const bool want_cls = stage == "cls" || stage == "all";
  const bool want_solve =
    stage == "solve" || stage == "track" || stage == "aim" || stage == "fire" || stage == "all";
  const bool want_track = stage == "track" || stage == "aim" || stage == "fire" || stage == "all";
  const bool want_aim = stage == "aim" || stage == "fire" || stage == "all";
  const bool want_fire = stage == "fire" || stage == "all";

  // debug=false：不弹检测器自带的 "binary_img"/"detection" 窗口，只保留 probe 自己的窗口
  auto_aim::Detector detector(config, debug_windows);
  std::unique_ptr<auto_aim::YOLO> yolo;
  if (use_yolo) yolo = std::make_unique<auto_aim::YOLO>(config, debug_windows);

  std::unique_ptr<auto_aim::Classifier> classifier;
  std::unique_ptr<auto_aim::Solver> solver;
  std::unique_ptr<auto_aim::Tracker> tracker;
  std::unique_ptr<auto_aim::Aimer> aimer;
  std::unique_ptr<auto_aim::Shooter> shooter;

  if (want_cls) classifier = std::make_unique<auto_aim::Classifier>(config);
  if (want_solve) solver = std::make_unique<auto_aim::Solver>(config);
  if (want_track) tracker = std::make_unique<auto_aim::Tracker>(config, *solver);
  if (want_aim) aimer = std::make_unique<auto_aim::Aimer>(config);
  if (want_fire) shooter = std::make_unique<auto_aim::Shooter>(config);

  StageStats st;
  cv::Mat img;
  std::string last_state = "-";
  std::list<auto_aim::Armor> armors;
  std::list<auto_aim::Target> targets;
  io::Command command{false, 0, 0, false, false};
  bool fired = false;

  bool next_stage = false;
  for (int i = 0; until_enter || i < frames; ++i) {
    std::chrono::steady_clock::time_point t;
    if (!source.read(img, t)) break;
    ++st.n;

    // ① 检测
    const auto d0 = std::chrono::steady_clock::now();
    armors = use_yolo ? yolo->detect(img, i) : detector.detect(img, i);
    const double det_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - d0).count();
    st.det_ms += det_ms;
    st.armor_total += static_cast<long long>(armors.size());
    if (!armors.empty()) ++st.det_frames;

    // ② 分类
    if (classifier && !armors.empty()) {
      const auto c0 = std::chrono::steady_clock::now();
      for (auto & a : armors) {
        if (a.pattern.empty() && a.box.width > 4 && a.box.height > 4) {
          a.pattern = img(a.box & cv::Rect(0, 0, img.cols, img.rows)).clone();
        }
        classifier->classify(a);
      }
      st.cls_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
    }

    // ③ 解算
    if (solver && !armors.empty()) {
      const auto s0 = std::chrono::steady_clock::now();
      for (auto & a : armors) solver->solve(a);
      st.solve_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s0).count();
    }

    // 解算后取最近一块装甲板的距离，用于显示
    double nearest_dist = -1.0;
    Eigen::Vector3d nearest_xyz = Eigen::Vector3d::Zero();
    if (solver && !armors.empty()) {
      for (const auto & a : armors) {
        const double d = a.xyz_in_gimbal.norm();
        if (d > 0.1 && (nearest_dist < 0 || d < nearest_dist)) {
          nearest_dist = d;
          nearest_xyz = a.xyz_in_gimbal;
        }
      }
      if ((stage == "solve" || stage == "all") && i % 15 == 0 && nearest_dist > 0) {
        // 用检测到的四点在图像里的像素宽度反推 fx：fx = 像素宽 × 距离 / 实际宽
        const auto & a = armors.front();
        double w_px = 0;
        if (a.points.size() == 4) w_px = cv::norm(a.points[0] - a.points[1]);
        const double W = (a.type == auto_aim::ArmorType::big) ? 0.230 : 0.135;
        const double fx_implied = (w_px > 1 && nearest_dist > 0) ? w_px * nearest_dist / W : 0;
        const char * type_str =
          (a.type == auto_aim::ArmorType::big) ? "big/230mm" : "small/135mm";
        std::printf(
          "  帧%4d dist=%.2f m  w_px=%.1f  期望dist=%.2f  fx_impl=%.0f  %s %s  yaw=%.1f°\n", i,
          nearest_dist, w_px, 2404.0 * ((a.type == auto_aim::ArmorType::big) ? 0.230 : 0.135) / w_px,
          fx_implied, type_str, auto_aim::ARMOR_NAMES[a.name].c_str(), a.ypr_in_gimbal[0] * 57.3);
      }
    }

    // ④ 跟踪
    size_t n_detected = armors.size();
    if (tracker) {
      const auto t0 = std::chrono::steady_clock::now();
      targets = tracker->track(armors, t);   // 注意：这里会按 enemy_color 就地过滤 armors
      st.track_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      const std::string now_state = tracker->state();
      st.state_count[now_state]++;
      if (now_state == "tracking" && last_state != "tracking") {
        ++st.acquire;
        if (st.first_track < 0) st.first_track = i;
      }
      if (last_state == "tracking" && now_state != "tracking") ++st.lost;
      last_state = now_state;
      if (!targets.empty()) ++st.target_frames;
    }

    // ⑤ 瞄准
    if (aimer) {
      const auto a0 = std::chrono::steady_clock::now();
      command = aimer->aim(targets, t, 25.0);
      st.aim_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a0).count();
      if (command.control) {
        ++st.cmd_frames;
        st.yaw_sum += command.yaw;
        st.pitch_sum += command.pitch;
        st.yaw_min = std::min(st.yaw_min, command.yaw);
        st.yaw_max = std::max(st.yaw_max, command.yaw);
        st.pitch_min = std::min(st.pitch_min, command.pitch);
        st.pitch_max = std::max(st.pitch_max, command.pitch);
      }
    }

    // ⑥ 开火
    if (shooter && aimer) {
      fired = shooter->shoot(command, *aimer, targets, Eigen::Vector3d{0.0, 0.0, 0.0});
      if (fired) ++st.fire_count;
    }

    // 可视化
    draw_armors(img, armors);
    draw_hud(
      img, stage, last_state, n_detected, targets.size(), det_ms, command.control, command.yaw,
      command.pitch, fired, until_enter ? "Running... press ENTER for next stage" : "",
      nearest_dist, nearest_xyz);

    if (tracker && n_detected > 0 && armors.empty()) {
      cv::putText(
        img, "all filtered by enemy_color!", {10, img.rows - 100}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
        {0, 0, 255}, 2, cv::LINE_AA);
    }

    if (has_display()) {
      cv::imshow("auto_aim_probe", img);
      const int key = cv::waitKey(source.from_video() ? 33 : 1);
      if (key == 13 || key == 10) {
        next_stage = true;
        break;
      }
      if (key == 'q' || key == 27) return st;
      if (key == 's') {
        const std::string path = fmt::format("out/probe_{}_{}.png", stage, i);
        cv::imwrite(path, img);
        std::printf("已保存 %s\n", path.c_str());
      }
    }
  }

  // 每级结束停一帧，方便看结果
  cv::Mat frozen;
  if (!img.empty()) {
    frozen = img.clone();
    draw_hud(frozen, stage + " 完成", last_state, armors.size(), targets.size(),
             st.n ? st.det_ms / st.n : 0, command.control, command.yaw, command.pitch, fired, "");
  }

  std::printf("\n==== %s 阶段汇总 ====\n", stage.c_str());
  std::printf("帧数        : %lld\n", st.n);
  std::printf("检出帧      : %lld (%.1f%%)\n", st.det_frames, st.n ? 100.0 * st.det_frames / st.n : 0);
  std::printf("平均装甲板  : %.2f\n", st.n ? double(st.armor_total) / st.n : 0);
  std::printf("检测耗时    : %.2f ms/帧\n", st.n ? st.det_ms / st.n : 0);
  if (want_cls) std::printf("分类耗时    : %.2f ms/帧\n", st.n ? st.cls_ms / st.n : 0);
  if (want_solve) std::printf("解算耗时    : %.2f ms/帧\n", st.n ? st.solve_ms / st.n : 0);
  if (want_track) {
    std::printf("跟踪耗时    : %.2f ms/帧，有目标帧 %lld (%.1f%%)\n", st.n ? st.track_ms / st.n : 0,
                st.target_frames, st.n ? 100.0 * st.target_frames / st.n : 0);
    std::printf("状态分布    :");
    for (const auto & kv : st.state_count)
      std::printf("  %s %lld(%.1f%%)", kv.first.c_str(), kv.second,
                  st.n ? 100.0 * kv.second / st.n : 0);
    std::printf("\n");
    std::printf("捕获/丢失   : 进入 tracking %lld 次，掉出 %lld 次", st.acquire, st.lost);
    if (st.first_track >= 0) std::printf("，首次 tracking 在第 %lld 帧", st.first_track);
    std::printf("\n");
  }
  if (want_aim) {
    std::printf("瞄准耗时    : %.2f ms/帧\n", st.n ? st.aim_ms / st.n : 0);
    std::printf("有效指令    : %lld 帧 (%.1f%%)\n", st.cmd_frames,
                st.n ? 100.0 * st.cmd_frames / st.n : 0);
    if (st.cmd_frames > 0) {
      std::printf(
        "yaw 指令    : 平均 %+.3f rad  范围 [%+.3f, %+.3f] rad (%.1f° ~ %.1f°)\n",
        st.yaw_sum / st.cmd_frames, st.yaw_min, st.yaw_max, st.yaw_min * 57.3, st.yaw_max * 57.3);
      std::printf("pitch 指令  : 平均 %+.3f rad  范围 [%+.3f, %+.3f] rad (%.1f° ~ %.1f°)\n",
                  st.pitch_sum / st.cmd_frames, st.pitch_min, st.pitch_max, st.pitch_min * 57.3,
                  st.pitch_max * 57.3);
    }
  }
  if (want_fire) std::printf("开火次数    : %lld\n", st.fire_count);

  if (!until_enter && !frozen.empty() && has_display()) {
    // 单级/视频模式：跑完停在最终画面，等 Enter 再继续
    cv::Mat keep = frozen.clone();
    wait_enter(keep, "按 Enter 进入下一级（q 退出）");
  }
  (void)next_stage;
  return st;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string config = argc > 1 ? argv[1] : "configs/hik.yaml";
  const std::string stage = argc > 2 ? argv[2] : "det";
  const int frames = argc > 3 ? std::atoi(argv[3]) : 200;
  const std::string video_path = argc > 4 ? argv[4] : "";
  const bool use_yolo = argc > 5 && std::strcmp(argv[5], "yolo") == 0;
  bool debug_windows = false;
  for (int k = 5; k < argc; ++k) {
    if (std::strcmp(argv[k], "debug") == 0) debug_windows = true;
  }

  std::printf("配置: %s\n阶段: %s\n输入: %s\n检测器: %s\n", config.c_str(), stage.c_str(),
              video_path.empty() ? "相机" : video_path.c_str(), use_yolo ? "YOLO" : "传统 CV");
  std::printf("帧数: %s\n\n", video_path.empty() ? "不限（按 Enter 结束）" : std::to_string(frames).c_str());

  try {
    Source source(config, video_path);

    if (stage == "step") {
      const std::vector<std::string> ladder = {"det", "cls", "solve", "track", "aim", "fire"};
      for (size_t i = 0; i < ladder.size(); ++i) {
        std::printf("\n########## 第 %zu/%zu 级：%s ##########\n", i + 1, ladder.size(),
                    ladder[i].c_str());
        source.rewind();
        // 相机：一直跑到按 Enter；视频：按帧数播放（到结尾自动停）
        const bool until_enter = has_display() && !source.from_video();
        run_stage(ladder[i], frames, source, config, use_yolo, until_enter, debug_windows);
      }
      std::printf("\n全部 6 级跑完。\n");
      return 0;
    }

    // 相机：一直跑，按 Enter 结束；视频：按帧数播放
    const bool until_enter = has_display() && !source.from_video();
    if (until_enter) {
      std::printf("相机模式：持续抓取，按 Enter 结束本级（q 退出）\n");
    }
    run_stage(stage, frames, source, config, use_yolo, until_enter, debug_windows);
  } catch (const std::exception & e) {
    std::printf("[错误] %s\n", e.what());
    return 1;
  }
  return 0;
}
