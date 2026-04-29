// =============================================================================
// yolo_detector.hpp — OpenVINO YOLO 装甲板检测器封装 (v2 — 零分配热路径)
// =============================================================================
// ★ InferRequest / input_blob_ 均在构造时一次性分配，detect() 热路径零堆分配。
// ★ 本头文件不包含任何 ROS / Eigen 头文件。
// =============================================================================
#ifndef RM_VISION__YOLO_DETECTOR_HPP
#define RM_VISION__YOLO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace rm_vision
{

// =============================================================================
// 检测输出结构 — 纯 POD，与 ROS 消息无关
// =============================================================================
struct Detection
{
  int   class_id;                        // YOLO 类别 ID
  float confidence;                      // 置信度
  cv::Rect bbox;                         // 边界框
  cv::Point2f apexes[4];                 // 四角点: TL, TR, BR, BL
  std::string label;                     // 可读标签
};

// =============================================================================
// YOLO 检测器配置
// =============================================================================
struct YOLOConfig
{
  std::string model_path;                // OpenVINO IR 或 ONNX 文件路径
  std::string device = "CPU";            // 推理设备: 强制 CPU（NUC 部署环境）

  int   input_width  = 640;              // 模型输入宽度
  int   input_height = 640;              // 模型输入高度
  int   num_classes  = 38;               // 类别数（YOLO11: 38, YOLOv8: 2）
  int   num_keypoints = 4;              // 每个目标的关键点数

  float score_threshold = 0.7f;          // 置信度阈值
  float nms_threshold   = 0.3f;          // NMS IoU 阈值
  float min_confidence  = 0.5f;          // 最小置信度（后过滤）

  bool  use_roi = false;                 // 是否使用 ROI 裁切
  cv::Rect roi;                          // ROI 区域
};

struct LetterboxMeta
{
  double scale = 1.0;                    // 原图 -> 网络输入的等比缩放
  int orig_w = 0;                        // 原图宽
  int orig_h = 0;                        // 原图高
  int resized_w = 0;                     // 缩放后宽
  int resized_h = 0;                     // 缩放后高
  int pad_x = 0;                         // 左侧 padding
  int pad_y = 0;                         // 上侧 padding
};

// =============================================================================
// YOLODetector — OpenVINO 推理 + 后处理 (v2)
// =============================================================================
// 关键设计:
//   - infer_request_  : 构造时创建一次，每次 detect() 复用
//   - input_blob_     : 构造时分配 640×640 cv::Mat 一次，每次 detect() 复用
//   - detect() 热路径中零 new/malloc 调用
// =============================================================================
class YOLODetector
{
public:
  explicit YOLODetector(const YOLOConfig & config);

  /// 核心推理接口：BGR cv::Mat → 检测结果数组（线程不安全，调用方加锁）
  /// @param bgr_img  输入 BGR 图像（ROI 裁切在内部完成）
  /// @param roi_offset  [out] ROI 裁切区域在原图中的偏移（用于坐标补偿）
  std::vector<Detection> detect(const cv::Mat & bgr_img, cv::Point2f & roi_offset);

  /// 获取配置（用于外部日志）
  const YOLOConfig & config() const { return config_; }

private:
  YOLOConfig config_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;

  // ★ 一次性分配的推理资源（构造时初始化，detect() 复用）
  ov::InferRequest infer_request_;
  cv::Mat input_blob_;                   // H×W×3 uint8 预处理缓冲区

  // 预处理：标准 centered letterbox，返回反变换元数据
  LetterboxMeta preprocess(const cv::Mat & bgr_img);

  // 后处理：解析 YOLO 输出 → NMS → 构造 Detection
  // ★ 不再原地 transpose output_tensor 内存！
  std::vector<Detection> postprocess(const LetterboxMeta & meta);

  // 关键点排序: TL, TR, BR, BL
  static void sort_keypoints(cv::Point2f pts[4]);
};

}  // namespace rm_vision

#endif  // RM_VISION__YOLO_DETECTOR_HPP
