// =============================================================================
// yolo_detector.cpp — OpenVINO YOLO 装甲板检测器实现 (v2 — 零分配热路径)
// =============================================================================
// 修复清单:
//   [FIX-1] infer_request_ / input_blob_ 在构造函数中一次性分配
//   [FIX-2] postprocess 中禁止原地 transpose output_tensor 内存
//   [FIX-3] ROI 偏移补偿使用 clamp 后的 safe_roi 而非原始 config_.roi
//   [FIX-4] detect() 接口新增 roi_offset 输出参数，由调用方安全使用
// =============================================================================
#include "rm_vision/yolo_detector.hpp"
#include "rm_vision/armor_types.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace rm_vision
{

// =============================================================================
// 构造：加载模型 → 编译 → 一次性分配推理资源
// =============================================================================
YOLODetector::YOLODetector(const YOLOConfig & config) : config_(config)
{
  // ★★★ 所有 OpenVINO 操作包裹在 try-catch 中 ★★★
  // 任何模型加载/编译失败都会向上抛异常，由节点层 FATAL + shutdown
  try {
    auto model = core_.read_model(config_.model_path);

    // ---- 预处理流水线 ----
    ov::preprocess::PrePostProcessor ppp(model);
    auto & input = ppp.input();

    input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1,
                  static_cast<ov::Dimension::value_type>(config_.input_height),
                  static_cast<ov::Dimension::value_type>(config_.input_width), 3})
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);

    input.model().set_layout("NCHW");

    input.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale(255.0);

    model = ppp.build();

    // ★ 强制使用 "CPU" 设备 — NUC 部署环境唯一可靠后端
    compiled_model_ = core_.compile_model(
      model, "CPU",
      ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

    // ★ [FIX-1] 一次性创建 InferRequest，后续 detect() 复用
    infer_request_ = compiled_model_.create_infer_request();

    // ★ [FIX-1] 一次性分配预处理缓冲区 (640×640×3 uint8)
    input_blob_ = cv::Mat(
      config_.input_height, config_.input_width, CV_8UC3, cv::Scalar(0, 0, 0));

  } catch (const ov::Exception & e) {
    // OpenVINO 特有异常：模型文件不存在、格式错误、编译失败等
    throw std::runtime_error(
      std::string("[OpenVINO] Model load/compile FAILED: ") + e.what() +
      " | model_path=" + config_.model_path);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      std::string("[OpenVINO] Unexpected error: ") + e.what() +
      " | model_path=" + config_.model_path);
  }
}

// =============================================================================
// 核心推理入口 (v2)
// =============================================================================
// roi_offset: 如果启用了 ROI，返回 clamp 后的 safe_roi.tl()；
//             否则返回 (0,0)。调用方用此值做坐标补偿。
// =============================================================================
std::vector<Detection> YOLODetector::detect(
  const cv::Mat & bgr_img, cv::Point2f & roi_offset)
{
  roi_offset = cv::Point2f(0.0f, 0.0f);

  if (bgr_img.empty()) {
    return {};
  }

  cv::Mat cropped;
  if (config_.use_roi) {
    // ★ [FIX-3] 安全 clamp ROI 到图像边界
    cv::Rect safe_roi = config_.roi;
    if (safe_roi.width < 0)  safe_roi.width = bgr_img.cols;
    if (safe_roi.height < 0) safe_roi.height = bgr_img.rows;
    safe_roi = safe_roi & cv::Rect(0, 0, bgr_img.cols, bgr_img.rows);

    // ★ [FIX-3] 使用 clamp 后的 safe_roi 偏移，而非原始 config_.roi
    roi_offset = cv::Point2f(
      static_cast<float>(safe_roi.x),
      static_cast<float>(safe_roi.y));

    cropped = bgr_img(safe_roi);
  } else {
    cropped = bgr_img;
  }

  // ★ 预处理：复用 input_blob_，零分配
  LetterboxMeta meta = preprocess(cropped);

  // ★ 推理：复用 infer_request_，零分配
  infer_request_.infer();

  // ★ 后处理：不修改 output_tensor 内存
  return postprocess(meta);
}

// =============================================================================
// 预处理：resize → 填入预分配的 input_blob_ → 设置 input tensor
// =============================================================================
LetterboxMeta YOLODetector::preprocess(const cv::Mat & bgr_img)
{
  const int W = config_.input_width;
  const int H = config_.input_height;

  LetterboxMeta meta;
  meta.orig_w = bgr_img.cols;
  meta.orig_h = bgr_img.rows;

  double x_scale = static_cast<double>(W) / bgr_img.cols;
  double y_scale = static_cast<double>(H) / bgr_img.rows;
  meta.scale = std::min(x_scale, y_scale);

  meta.resized_w = std::clamp(
    static_cast<int>(std::round(bgr_img.cols * meta.scale)), 1, W);
  meta.resized_h = std::clamp(
    static_cast<int>(std::round(bgr_img.rows * meta.scale)), 1, H);
  meta.pad_x = (W - meta.resized_w) / 2;
  meta.pad_y = (H - meta.resized_h) / 2;

  // ★ 标准 Ultralytics/OpenVINO letterbox:
  //   1. 等比缩放
  //   2. 居中填充
  //   3. pad_value 与模型 metadata 一致为 114
  input_blob_.setTo(cv::Scalar(114, 114, 114));
  cv::resize(
    bgr_img,
    input_blob_(cv::Rect(meta.pad_x, meta.pad_y, meta.resized_w, meta.resized_h)),
    cv::Size(meta.resized_w, meta.resized_h));

  // 将 input_blob_ 的内存绑定为 OpenVINO input tensor（零拷贝）
  ov::Tensor input_tensor(
    ov::element::u8,
    {1, static_cast<unsigned long>(H), static_cast<unsigned long>(W), 3},
    input_blob_.data);

  // ★ 复用 infer_request_：直接设置 input tensor
  infer_request_.set_input_tensor(input_tensor);

  return meta;
}

// =============================================================================
// 后处理：解析 YOLO 输出 → NMS → 构造 Detection (v2)
// =============================================================================
// ★ [FIX-2] 严禁原地 transpose output_tensor 绑定的内存！
//   改为 cv::transpose(output, transposed)，transposed 是独立分配。
// =============================================================================
std::vector<Detection> YOLODetector::postprocess(
  const LetterboxMeta & meta)
{
  // 获取输出 tensor（只读！）
  auto output_tensor = infer_request_.get_output_tensor();
  auto output_shape  = output_tensor.get_shape();

  cv::Mat output(
    static_cast<int>(output_shape[1]),
    static_cast<int>(output_shape[2]),
    CV_32F, output_tensor.data());

  // ★ [FIX-2] transpose 到独立内存，不污染 output_tensor
  cv::Mat transposed;
  cv::transpose(output, transposed);

  const int num_kp_values = config_.num_keypoints * 2;

  std::vector<int>   ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> all_keypoints;

  const auto map_back_x = [&meta](float x) -> float {
    return (x - static_cast<float>(meta.pad_x)) / static_cast<float>(meta.scale);
  };
  const auto map_back_y = [&meta](float y) -> float {
    return (y - static_cast<float>(meta.pad_y)) / static_cast<float>(meta.scale);
  };

  for (int r = 0; r < transposed.rows; r++) {
    auto xywh    = transposed.row(r).colRange(0, 4);
    auto scores  = transposed.row(r).colRange(4, 4 + config_.num_classes);
    auto kp_data = transposed.row(r).colRange(
      4 + config_.num_classes, 4 + config_.num_classes + num_kp_values);

    double score;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

    if (score < config_.score_threshold) continue;

    auto cx = xywh.at<float>(0);
    auto cy = xywh.at<float>(1);
    auto bw = xywh.at<float>(2);
    auto bh = xywh.at<float>(3);

    float left_f = map_back_x(cx - 0.5f * bw);
    float top_f = map_back_y(cy - 0.5f * bh);
    float right_f = map_back_x(cx + 0.5f * bw);
    float bottom_f = map_back_y(cy + 0.5f * bh);

    boxes.emplace_back(
      static_cast<int>(std::floor(left_f)),
      static_cast<int>(std::floor(top_f)),
      static_cast<int>(std::ceil(right_f) - std::floor(left_f)),
      static_cast<int>(std::ceil(bottom_f) - std::floor(top_f)));

    std::vector<cv::Point2f> kp;
    for (int i = 0; i < config_.num_keypoints; i++) {
      kp.emplace_back(
        map_back_x(kp_data.at<float>(0, i * 2)),
        map_back_y(kp_data.at<float>(0, i * 2 + 1)));
    }

    ids.push_back(max_point.x);
    confidences.push_back(static_cast<float>(score));
    all_keypoints.push_back(std::move(kp));
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(
    boxes, confidences, config_.score_threshold, config_.nms_threshold, indices);

  // 构造输出
  std::vector<Detection> results;
  results.reserve(indices.size());

  for (int idx : indices) {
    Detection det;
    det.class_id   = ids[idx];
    det.confidence = confidences[idx];

    // 排序关键点（roi_offset 由调用方在 detect() 中返回，不在此处补偿）
    cv::Point2f kp[4];
    for (int i = 0; i < 4; i++) {
      kp[i] = all_keypoints[idx][i];
    }
    sort_keypoints(kp);

    // 丢弃映射到图像外的异常关键点，防止后续 PnP/自瞄被离谱像素污染。
    const float margin_x = std::max(8.0f, static_cast<float>(meta.orig_w) * 0.01f);
    const float margin_y = std::max(8.0f, static_cast<float>(meta.orig_h) * 0.01f);
    bool geometry_valid = true;
    for (int i = 0; i < 4; i++) {
      if (!std::isfinite(kp[i].x) || !std::isfinite(kp[i].y) ||
          kp[i].x < -margin_x || kp[i].x > static_cast<float>(meta.orig_w - 1) + margin_x ||
          kp[i].y < -margin_y || kp[i].y > static_cast<float>(meta.orig_h - 1) + margin_y)
      {
        geometry_valid = false;
        break;
      }
    }
    if (!geometry_valid) {
      continue;
    }

    cv::Rect image_rect(0, 0, meta.orig_w, meta.orig_h);
    cv::Rect clipped_box = boxes[idx] & image_rect;
    if (clipped_box.width <= 1 || clipped_box.height <= 1) {
      continue;
    }
    det.bbox = clipped_box;

    for (int i = 0; i < 4; i++) {
      kp[i].x = std::clamp(kp[i].x, 0.0f, static_cast<float>(meta.orig_w - 1));
      kp[i].y = std::clamp(kp[i].y, 0.0f, static_cast<float>(meta.orig_h - 1));
      det.apexes[i] = kp[i];
    }

    det.label = make_armor_label(det.class_id);

    if (det.confidence >= config_.min_confidence) {
      results.push_back(std::move(det));
    }
  }

  return results;
}

// =============================================================================
// 关键点排序: TL(左上) → TR(右上) → BR(右下) → BL(左下)
// =============================================================================
void YOLODetector::sort_keypoints(cv::Point2f pts[4])
{
  std::sort(pts, pts + 4, [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
  if (pts[2].x > pts[3].x) std::swap(pts[2], pts[3]);

  // 调整为 TL, TR, BR, BL
  cv::Point2f bl = pts[2];
  pts[2] = pts[3];  // BR
  pts[3] = bl;      // BL
}

}  // namespace rm_vision
