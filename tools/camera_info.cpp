// 工业相机自检：打开相机 -> 抓若干帧 -> 打印分辨率/帧率 -> 存一张图。
//
//   ./build/camera_info configs/hik.yaml                       # 抓 30 帧，存一张图
//   ./build/camera_info configs/hik.yaml 120 out/              # 抓 120 帧
//   ./build/camera_info configs/hik.yaml 300 out/ out/rec.avi  # 同时录成视频，供离线测试
//
// 这是接硬件后第一个该跑的程序：确认"相机能出图"再谈检测。
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "io/camera.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/hik.yaml";
  const int frames = argc > 2 ? std::atoi(argv[2]) : 30;
  const std::string out_dir = argc > 3 ? argv[3] : "out";
  const std::string video_path = argc > 4 ? argv[4] : "";

  std::printf("配置: %s\n", config_path.c_str());

  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  // 读第一帧：可能等建流，给 8 秒；超时说明相机没打开，直接给排查建议
  auto first = std::async(std::launch::async, [&] { camera.read(img, t); });
  if (first.wait_for(std::chrono::seconds(8)) != std::future_status::ready) {
    std::printf("\n[FAIL] 8 秒内没有拿到画面，相机没有成功打开。\n\n");
    std::printf("按顺序排查：\n");
    std::printf("  1) lsusb | grep 2bdf              确认设备在（应有 MV-CS016-10UC）\n");
    std::printf("  2) lsusb -t                       确认速率为 5000M（USB3），480M 要换口/换线\n");
    std::printf("  3) sudo ./build/camera_info ...   用 root 跑，排除权限问题\n");
    std::printf("     权限问题的永久修复见 HARDWARE.md 的 udev 规则\n");
    std::printf("  4) ls io/lib/libMvUsb3vTL.so      少了这个库就会报 'Unable to open usb!'\n");
    std::printf("     它由海康 MVS 客户端提供（/opt/MVS/lib/64/ 下），拷到 io/lib/ 即可\n");
    std::printf("  5) 关掉 MVS 客户端 / 其他占用的程序，设备被占用也会打不开\n");
    std::exit(2);
  }
  first.get();

  if (img.empty()) {
    std::printf("[FAIL] 读不到帧\n");
    return 1;
  }

  cv::VideoWriter writer;
  if (!video_path.empty()) {
    writer.open(
      video_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(img.cols, img.rows));
    if (!writer.isOpened()) std::printf("[warn] 无法写视频: %s\n", video_path.c_str());
  }

  const auto t0 = std::chrono::steady_clock::now();
  const auto first_ts = t;
  int count = 0;
  cv::Mat last = img.clone();

  for (int i = 0; i < frames; ++i) {
    camera.read(img, t);
    if (img.empty()) continue;
    last = img.clone();
    if (writer.isOpened()) writer.write(img);
    ++count;
  }
  if (writer.isOpened()) {
    writer.release();
    std::printf("录制视频 : %s (%d 帧)\n", video_path.c_str(), count);
  }

  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(t1 - t0).count();
  const double fps = count > 0 ? count / elapsed : 0.0;

  std::printf("\n==== 相机自检 ====\n");
  std::printf("分辨率   : %d x %d\n", last.cols, last.rows);
  std::printf("通道     : %d\n", last.channels());
  std::printf("抓帧数   : %d / %d\n", count, frames);
  std::printf("实测帧率 : %.1f fps\n", fps);
  std::printf("时间戳   : %.3f s 起\n",
              std::chrono::duration<double>(first_ts.time_since_epoch()).count());

  const std::string path = out_dir + "/camera_check.png";
  if (cv::imwrite(path, last)) {
    std::printf("样例图   : %s\n", path.c_str());
  }

  // 曝光检查：太暗说明曝光时间不够，太亮说明过曝
  cv::Mat gray;
  cv::cvtColor(last, gray, cv::COLOR_BGR2GRAY);
  cv::Scalar mean, stddev;
  cv::meanStdDev(gray, mean, stddev);
  std::printf("画面亮度 : 均值 %.1f  标准差 %.1f （建议均值 60~180）\n", mean[0], stddev[0]);

  return count > 0 ? 0 : 1;
}
