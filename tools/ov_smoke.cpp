// OpenVINO 自检：加载 tiny_resnet.onnx，跑一次推理，打印输入输出形状与耗时。
// 用于确认 third_party/openvino 里的运行时可用。
//
//   ./build/ov_smoke assets/tiny_resnet.onnx
#include <chrono>
#include <cstdio>
#include <string>

#include <openvino/openvino.hpp>

int main(int argc, char** argv) {
  const std::string model = argc > 1 ? argv[1] : "assets/tiny_resnet.onnx";

  ov::Core core;
  std::printf("OpenVINO: %s\n", ov::get_openvino_version().description);
  std::printf("CPU 插件: %s\n", core.get_property("CPU", "FULL_DEVICE_NAME").as<std::string>().c_str());

  auto net = core.read_model(model);

  // tiny_resnet 的输入是动态形状，编译前固定成 {1,1,32,32}（与 Classifier 的预处理一致）
  const auto& in_partial = net->input(0).get_partial_shape();
  std::printf("输入形状(原始): %s\n", in_partial.to_string().c_str());
  if (in_partial.is_dynamic()) {
    net->reshape(ov::PartialShape{1, 1, 32, 32});
    std::printf("输入形状(已 reshape): %s\n", net->input(0).get_partial_shape().to_string().c_str());
  }

  auto compiled = core.compile_model(net, "CPU");

  const auto inputs = compiled.inputs();
  const auto outputs = compiled.outputs();
  std::printf("输入: %zu 个，输出: %zu 个\n", inputs.size(), outputs.size());
  for (const auto& in : inputs) {
    std::printf("  in  %s shape=%s\n", in.get_any_name().c_str(),
                in.get_partial_shape().to_string().c_str());
  }
  for (const auto& out : outputs) {
    std::printf("  out %s shape=%s\n", out.get_any_name().c_str(),
                out.get_partial_shape().to_string().c_str());
  }

  // 用零输入跑通一次，确认推理链路
  ov::Shape shape = inputs.front().get_shape();
  ov::Tensor input_tensor(inputs.front().get_element_type(), shape);
  std::fill_n(input_tensor.data<float>(), input_tensor.get_size(), 0.0f);

  auto request = compiled.create_infer_request();
  request.set_input_tensor(input_tensor);

  const auto begin = std::chrono::steady_clock::now();
  request.infer();
  const auto end = std::chrono::steady_clock::now();

  const double ms = std::chrono::duration<double, std::milli>(end - begin).count();
  std::printf("推理成功，单次耗时 %.3f ms\n", ms);
  return 0;
}
