// =============================================================================
// vision_detector_node.cpp — rm_vision_detector 节点 (v2 — 异步流水线)
// =============================================================================
// 修复清单:
//   [FIX-1] cv_bridge::toCvShare 替代 toCvCopy（真正的零拷贝）
//   [FIX-2] 线程安全单帧队列 + 独立推理线程（解除 Executor 阻塞）
//   [FIX-3] roi_offset 由 detect() 返回，在节点层做坐标补偿
//   [FIX-4] ArmorDetection 子消息 header 正确赋值
//
// 架构:
//   ROS Callback (轻量) ──push──► [Queue(max=1)] ──pop──► Inference Thread
//        │                                                         │
//   toCvShare (零拷贝)                                    detect() → publish()
//        │                                                         │
//   队列满则丢弃旧帧                                            ROI offset 补偿
// =============================================================================
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// 自定义消息
#include "rm_interfaces/msg/armor_detection.hpp"
#include "rm_interfaces/msg/armor_detections.hpp"

// 内部模块
#include "rm_vision/armor_types.hpp"
#include "rm_vision/yolo_detector.hpp"

// =============================================================================
// VisionDetectorNode — 异步推理架构
// =============================================================================
class VisionDetectorNode : public rclcpp::Node
{
public:
  VisionDetectorNode() : Node("rm_vision_detector")
  {
    // ---- 声明参数 ----
    // ★ model_path: 必须通过 launch 或命令行传入 .xml 绝对路径
    this->declare_parameter<std::string>("model_path", "");
    // ★ confidence_threshold: 统一置信度阈值（替代 min_confidence）
    this->declare_parameter<double>("confidence_threshold", 0.5);
    // ★ color_ignore: 需要忽略的颜色 (0=无, 1=Red, 2=Blue)
    this->declare_parameter<int>("color_ignore", 0);
    // 保留参数
    this->declare_parameter<int>("input_width", 640);
    this->declare_parameter<int>("input_height", 640);
    this->declare_parameter<int>("num_classes", 38);
    this->declare_parameter<double>("score_threshold", 0.7);
    this->declare_parameter<double>("nms_threshold", 0.3);
    this->declare_parameter<bool>("use_roi", false);
    this->declare_parameter<int>("roi_x", 0);
    this->declare_parameter<int>("roi_y", 0);
    this->declare_parameter<int>("roi_w", -1);
    this->declare_parameter<int>("roi_h", -1);
    this->declare_parameter<std::string>("frame_id", "camera_optical_link");
    this->declare_parameter<bool>("show_debug", false);
    this->declare_parameter<bool>("publish_debug_image", false);
    this->declare_parameter<std::string>("target_color", "blue");

    // ---- 构建检测器配置 ----
    rm_vision::YOLOConfig yolo_cfg;
    yolo_cfg.model_path      = this->get_parameter("model_path").as_string();
    yolo_cfg.device          = "CPU";   // ★ 强制 CPU，忽略用户设置
    yolo_cfg.input_width     = this->get_parameter("input_width").as_int();
    yolo_cfg.input_height    = this->get_parameter("input_height").as_int();
    yolo_cfg.num_classes     = this->get_parameter("num_classes").as_int();
    yolo_cfg.score_threshold = static_cast<float>(this->get_parameter("score_threshold").as_double());
    yolo_cfg.nms_threshold   = static_cast<float>(this->get_parameter("nms_threshold").as_double());
    yolo_cfg.min_confidence  = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
    yolo_cfg.use_roi         = this->get_parameter("use_roi").as_bool();
    yolo_cfg.roi             = cv::Rect(
      this->get_parameter("roi_x").as_int(),
      this->get_parameter("roi_y").as_int(),
      this->get_parameter("roi_w").as_int(),
      this->get_parameter("roi_h").as_int());

    frame_id_            = this->get_parameter("frame_id").as_string();
    show_debug_          = this->get_parameter("show_debug").as_bool();
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
    target_color_        = this->get_parameter("target_color").as_string();
    color_ignore_        = this->get_parameter("color_ignore").as_int();

    // ★★★ 容错拦截：model_path 为空 → 抛异常，安全终止 ★★★
    if (yolo_cfg.model_path.empty()) {
      RCLCPP_ERROR(get_logger(),
        "model_path is EMPTY! Usage:\n"
        "  ros2 run rm_vision vision_detector_node --ros-args \\\n"
        "    --params-file /path/to/params.yaml -p model_path:=/abs/path/to/yolo11.xml\n"
        "  OR: ros2 launch rm_vision vision.launch.py");
      throw std::runtime_error("model_path is empty");
    }

    RCLCPP_INFO(get_logger(), "Loading YOLO model: %s (device=CPU, classes=%d)",
      yolo_cfg.model_path.c_str(), yolo_cfg.num_classes);

    // ★★★ 容错拦截：OpenVINO 加载/编译失败 → 抛异常，安全终止 ★★★
    try {
      detector_ = std::make_unique<rm_vision::YOLODetector>(yolo_cfg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "YOLODetector init FAILED: %s\n"
        "model_path=%s\n"
        "Check model file exists and is valid OpenVINO IR.",
        e.what(), yolo_cfg.model_path.c_str());
      throw std::runtime_error(std::string("YOLODetector init failed: ") + e.what());
    }

    RCLCPP_INFO(get_logger(), "YOLO model loaded successfully on CPU.");

    // ---- 订阅：相机图像 ----
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        on_image(msg);
      });

    // ---- 发布：装甲板检测结果 ----
    armor_pub_ = this->create_publisher<rm_interfaces::msg::ArmorDetections>(
      "/detector/armors", rclcpp::SensorDataQoS());

    // ---- [FIX-VIZ] 发布：调试可视化图像（ROS-Native，不使用 imshow） ----
    if (publish_debug_image_) {
      debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/detector/image_debug", rclcpp::SensorDataQoS());
      RCLCPP_INFO(get_logger(), "  Debug image publishing: ENABLED → /detector/image_debug");
    }

    // ---- 启动异步推理线程 ----
    running_.store(true);
    inference_thread_ = std::thread(&VisionDetectorNode::inference_loop, this);

    RCLCPP_INFO(get_logger(), "rm_vision_detector node initialized (async pipeline v2).");
    RCLCPP_INFO(get_logger(), "  Subscribing: /camera/image_raw (SensorDataQoS)");
    RCLCPP_INFO(get_logger(), "  Publishing:  /detector/armors");
    RCLCPP_INFO(get_logger(), "  Target color filter: %s", target_color_.c_str());
  }

  ~VisionDetectorNode() override
  {
    // ---- 优雅关闭推理线程 ----
    running_.store(false);
    queue_cv_.notify_one();
    if (inference_thread_.joinable()) {
      inference_thread_.join();
    }
  }

private:
  // =========================================================================
  //  [FIX-2] 图像回调 — 极轻量，仅做队列推入
  //  不调用 detect()，不阻塞 Executor
  // =========================================================================
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      // 队列长度限制为 1：丢弃旧帧保证低延迟
      if (!image_queue_.empty()) {
        image_queue_.pop();
      }
      image_queue_.push(msg);
    }
    queue_cv_.notify_one();
  }

  // =========================================================================
  //  [FIX-2] 异步推理线程 — 死循环: 取图 → 推理 → 发布
  // =========================================================================
  void inference_loop()
  {
    RCLCPP_INFO(get_logger(), "Inference thread started.");

    while (running_.load()) {
      // ---- 1. 从队列取图（锁粒度最小化） ----
      sensor_msgs::msg::Image::ConstSharedPtr msg;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] {
          return !image_queue_.empty() || !running_.load();
        });

        if (!running_.load() && image_queue_.empty()) {
          break;
        }

        msg = image_queue_.front();
        image_queue_.pop();
      }

      if (!msg) continue;

      // ---- 2. [FIX-1] cv_bridge::toCvShare — 真正的零拷贝 ----
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        continue;
      }

      const cv::Mat & mat = cv_ptr->image;
      if (mat.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty image");
        continue;
      }

      // ---- 3. YOLO 推理 ----
      auto t0 = std::chrono::high_resolution_clock::now();

      cv::Point2f roi_offset;
      auto detections = detector_->detect(mat, roi_offset);

      auto t1 = std::chrono::high_resolution_clock::now();
      double infer_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

      // ---- 4. [FIX-FILTER] 颜色过滤：target_color 优先，color_ignore 兜底 ----
      rm_interfaces::msg::ArmorDetections out_msg;
      out_msg.header.stamp    = msg->header.stamp;   // ★ 透传输入图像时间戳
      out_msg.header.frame_id = frame_id_;

      // 记录每个原始检测是否被过滤（供 debug 图使用）
      std::vector<bool> filter_mask(detections.size(), false);
      size_t dropped_invalid_geometry = 0;

      for (size_t i = 0; i < detections.size(); i++) {
        const auto & det = detections[i];

        // ---- 解析装甲板属性 ----
        rm_vision::ArmorColor armor_color;
        rm_vision::ArmorName  armor_name;
        rm_vision::ArmorSize  armor_size;
        rm_vision::parse_class_id(det.class_id, armor_color, armor_name, armor_size);

        // ★ 统一过滤策略:
        //   1. target_color="all" → 不过滤（测试用）
        //   2. target_color="red"/"blue" → 白名单：只攻击目标颜色
        //   3. target_color 其他值 → 退回 color_ignore 黑名单模式
        bool filtered = false;
        if (target_color_ == "all") {
          filtered = false;
        } else if (target_color_ == "red") {
          if (armor_color != rm_vision::ArmorColor::RED)  filtered = true;
        } else if (target_color_ == "blue") {
          if (armor_color != rm_vision::ArmorColor::BLUE) filtered = true;
        } else if (color_ignore_ >= 0 && color_ignore_ <= 1) {
          auto ignore_color = static_cast<rm_vision::ArmorColor>(color_ignore_);
          if (armor_color == ignore_color) filtered = true;
        }

        filter_mask[i] = filtered;
        if (filtered) continue;

        bool geometry_valid = true;
        for (int j = 0; j < 4; j++) {
          const double px = det.apexes[j].x + roi_offset.x;
          const double py = det.apexes[j].y + roi_offset.y;
          if (!std::isfinite(px) || !std::isfinite(py) ||
              px < 0.0 || px >= static_cast<double>(mat.cols) ||
              py < 0.0 || py >= static_cast<double>(mat.rows))
          {
            geometry_valid = false;
            break;
          }
        }
        if (!geometry_valid) {
          dropped_invalid_geometry++;
          continue;
        }

        // ---- 映射到 ROS 消息 ----
        rm_interfaces::msg::ArmorDetection armor;
        armor.header.stamp    = msg->header.stamp;
        armor.header.frame_id = frame_id_;
        armor.class_id   = static_cast<uint8_t>(det.class_id);
        armor.confidence = det.confidence;
        armor.label      = det.label;

        switch (armor_color) {
          case rm_vision::ArmorColor::RED:  armor.color = armor.RED;  break;
          case rm_vision::ArmorColor::BLUE: armor.color = armor.BLUE; break;
          default:                          armor.color = armor.NONE; break;
        }

        for (int j = 0; j < 4; j++) {
          armor.apexes[j].x = static_cast<double>(det.apexes[j].x + roi_offset.x);
          armor.apexes[j].y = static_cast<double>(det.apexes[j].y + roi_offset.y);
          armor.apexes[j].z = 0.0;
        }

        armor.bbox_x = static_cast<uint32_t>(std::max(0, det.bbox.x + static_cast<int>(roi_offset.x)));
        armor.bbox_y = static_cast<uint32_t>(std::max(0, det.bbox.y + static_cast<int>(roi_offset.y)));
        armor.bbox_w = static_cast<uint32_t>(det.bbox.width);
        armor.bbox_h = static_cast<uint32_t>(det.bbox.height);

        out_msg.detections.push_back(std::move(armor));
      }

      armor_pub_->publish(out_msg);

      if (dropped_invalid_geometry > 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[Vision] dropped %zu detection(s) with out-of-image keypoints "
          "(img=%dx%d roi_offset=(%.1f, %.1f))",
          dropped_invalid_geometry, mat.cols, mat.rows, roi_offset.x, roi_offset.y);
      }

      // ---- 5. 调试可视化（显示所有原始检测，过滤的标灰×） ----
      if (publish_debug_image_ && debug_pub_) {
        draw_debug_ros_raw(mat, detections, filter_mask, roi_offset, msg->header);
      }

      // ---- 6. 周期性性能日志（~1Hz @ ~100fps） ----
      frame_count_++;
      if (frame_count_.load() % 100 == 0) {
        int raw_count = static_cast<int>(detections.size());
        RCLCPP_INFO(get_logger(),
          "[Vision] frame#=%u  infer=%.1fms  raw=%d  passed=%zu  target_color=%s",
          frame_count_.load(), infer_ms, raw_count, out_msg.detections.size(), target_color_.c_str());
      }
    }

    RCLCPP_INFO(get_logger(), "Inference thread stopped.");
  }

  // =========================================================================
  //  [FIX-DEBUG] 原始检测可视化 — 显示所有检测（过滤前），过滤的标灰×
  //  这样你可以区分"模型没检测到"还是"被颜色过滤掉了"
  // =========================================================================
  void draw_debug_ros_raw(
    const cv::Mat & img,
    const std::vector<rm_vision::Detection> & detections,
    const std::vector<bool> & filter_mask,
    const cv::Point2f & roi_offset,
    const std_msgs::msg::Header & header)
  {
    cv::Mat vis = img.clone();

    // 颜色映射
    const cv::Scalar color_red(0, 0, 255);       // RED 通过 → 红框
    const cv::Scalar color_blue(255, 0, 0);      // BLUE 通过 → 蓝框
    const cv::Scalar color_default(0, 255, 0);   // 其他 → 绿框
    const cv::Scalar color_filtered(128, 128, 128); // 被过滤 → 灰框

    for (size_t idx = 0; idx < detections.size(); idx++) {
      const auto & det = detections[idx];
      bool is_filtered = filter_mask[idx];

      // 解析装甲板颜色
      rm_vision::ArmorColor armor_color;
      rm_vision::ArmorName  armor_name;
      rm_vision::ArmorSize  armor_size;
      rm_vision::parse_class_id(det.class_id, armor_color, armor_name, armor_size);

      // 选框色：过滤前用颜色，过滤后用灰色
      cv::Scalar draw_color;
      if (is_filtered) {
        draw_color = color_filtered;
      } else if (armor_color == rm_vision::ArmorColor::RED) {
        draw_color = color_red;
      } else if (armor_color == rm_vision::ArmorColor::BLUE) {
        draw_color = color_blue;
      } else {
        draw_color = color_default;
      }

      // ROI offset 补偿
      int pts[4][2];
      for (int i = 0; i < 4; i++) {
        pts[i][0] = static_cast<int>(det.apexes[i].x + roi_offset.x);
        pts[i][1] = static_cast<int>(det.apexes[i].y + roi_offset.y);
      }

      // 画四条边
      for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        cv::line(vis, cv::Point(pts[i][0], pts[i][1]),
                     cv::Point(pts[j][0], pts[j][1]),
                     draw_color, 2);
      }

      // 画四角点
      for (int i = 0; i < 4; i++) {
        cv::circle(vis, cv::Point(pts[i][0], pts[i][1]), 3, draw_color, -1);
      }

      // 标注
      std::string info = det.label + " id=" +
        std::to_string(det.class_id) + " " +
        std::to_string(static_cast<int>(det.confidence * 100)) + "%";
      if (is_filtered) {
        info += " [FILTERED]";
      }
      cv::putText(vis, info,
        cv::Point(pts[0][0], pts[0][1] - 8),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 1);

      // 被过滤的额外画一个大 ×
      if (is_filtered) {
        int cx = (pts[0][0] + pts[2][0]) / 2;
        int cy = (pts[0][1] + pts[2][1]) / 2;
        cv::putText(vis, "X", cv::Point(cx - 8, cy + 8),
          cv::FONT_HERSHEY_SIMPLEX, 1.0, color_filtered, 2);
      }
    }

    // 左上角状态文字
    std::string status = "target=" + target_color_ +
      "  raw=" + std::to_string(detections.size()) +
      "  passed=" + std::to_string(
        std::count(filter_mask.begin(), filter_mask.end(), false));
    cv::putText(vis, status, cv::Point(10, 25),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

    // 缩小 50% 减少带宽
    cv::resize(vis, vis, {}, 0.5, 0.5);

    // cv_bridge → ROS Image 消息发布
    auto debug_msg = cv_bridge::CvImage(header, "bgr8", vis).toImageMsg();
    debug_pub_->publish(*debug_msg);
  }

  // =========================================================================
  //  成员
  // =========================================================================

  // ---- 检测器 ----
  std::unique_ptr<rm_vision::YOLODetector> detector_;

  // ---- 参数 ----
  std::string frame_id_;
  bool show_debug_ = false;
  bool publish_debug_image_ = false;    // [FIX-VIZ] ROS-Native 可视化开关
  std::string target_color_ = "blue";
  int color_ignore_ = -1;              // [FIX-FOF] -1=不过滤, 0=忽略红, 1=忽略蓝
  std::atomic<uint32_t> frame_count_{0};

  // ---- [FIX-2] 异步推理基础设施 ----
  std::thread inference_thread_;
  std::atomic<bool> running_{false};

  // 线程安全单帧队列（max size = 1，丢弃旧帧）
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> image_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // ---- ROS 2 接口 ----
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<rm_interfaces::msg::ArmorDetections>::SharedPtr armor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;  // [FIX-VIZ]
};

// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
