// =============================================================================
// self_point_filter_node.cpp — 自车点云剔除节点 (Point-LIO / 建图链专用)
// =============================================================================
// 职责:
//   输入: /livox/lidar (livox_ros_driver2/msg/CustomMsg, frame: livox_frame)
//   处理: 将点云变换到 base_link，用 exclusion boxes 剔除自车点云
//   输出: /livox/lidar/self_filtered (livox_ros_driver2/msg/CustomMsg, frame: livox_frame)
//
// 设计原则:
//   - 只做自车剔除，不做高度/范围/体素过滤（那是 obstacle_cloud_filter 的职责）
//   - 高性能: CustomMsg 透传，只过滤 points 数组
//   - 保持原始 frame_id (livox_frame) 和 per-point timestamps
//   - 所有 exclusion boxes 全部参数化，便于实车微调
// =============================================================================

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>

namespace
{

struct ExclusionBox
{
  std::string name;
  bool enabled{false};
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  Eigen::Vector3f max = Eigen::Vector3f::Zero();

  bool contains(float x, float y, float z) const
  {
    return enabled &&
           x >= min.x() && x <= max.x() &&
           y >= min.y() && y <= max.y() &&
           z >= min.z() && z <= max.z();
  }
};

Eigen::Vector3f toVector3(const std::vector<double> & values, const std::string & name)
{
  if (values.size() != 3U) {
    throw std::runtime_error(name + " must contain exactly 3 elements");
  }
  return Eigen::Vector3f(
    static_cast<float>(values[0]),
    static_cast<float>(values[1]),
    static_cast<float>(values[2]));
}

Eigen::Affine3f transformToEigen(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q = transform.transform.rotation;

  Eigen::Quaternionf rotation(
    static_cast<float>(q.w),
    static_cast<float>(q.x),
    static_cast<float>(q.y),
    static_cast<float>(q.z));
  rotation.normalize();

  Eigen::Affine3f affine = Eigen::Affine3f::Identity();
  affine.translation() = Eigen::Vector3f(
    static_cast<float>(t.x),
    static_cast<float>(t.y),
    static_cast<float>(t.z));
  affine.linear() = rotation.toRotationMatrix();
  return affine;
}

using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;

}  // namespace

class SelfPointFilterNode : public rclcpp::Node
{
public:
  SelfPointFilterNode()
  : Node("self_point_filter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/livox/lidar");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/livox/lidar/self_filtered");
    source_frame_ = this->declare_parameter<std::string>(
      "source_frame", "livox_frame");
    target_frame_ = this->declare_parameter<std::string>(
      "target_frame", "base_link");
    transform_timeout_sec_ = this->declare_parameter<double>(
      "transform_timeout_sec", 0.05);

    // ---- Exclusion Boxes (基于 50x50x55cm 车体 + 小云台 + 单雷达偏装) ----
    initExclusionBox("body_box", true,
      {-0.30, -0.30, -0.10},   // 50cm 车体，居中，前后各 25cm
      {0.30, 0.30, 0.60});      // 高度覆盖到 60cm（车体 55cm + 余量）

    initExclusionBox("gimbal_box", true,
      {-0.15, -0.15, 0.40},   // 小云台: 车体中间, 离地 55cm 附近
      {0.15, 0.15, 0.80});     // 向上延伸到 80cm 覆盖云台+枪管

    initExclusionBox("gimbal_support_box", true,
      {-0.08, -0.08, 0.25},   // 云台支撑结构
      {0.08, 0.08, 0.45});

    initExclusionBox("lidar_arm_box", true,
      {-0.05, 0.10, 0.20},    // 雷达安装臂: 偏右(y=+0.2)
      {0.10, 0.35, 0.50});

    // ---- Subscriber / Publisher ----
    pub_ = this->create_publisher<CustomMsg>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

    sub_ = this->create_subscription<CustomMsg>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SelfPointFilterNode::handleCloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Self-point filter ready: %s -> %s (filter in %s, %zu boxes)",
      input_topic_.c_str(), output_topic_.c_str(),
      target_frame_.c_str(), boxes_.size());

    for (const auto & box : boxes_) {
      if (box.enabled) {
        RCLCPP_INFO(get_logger(),
          "  Box [%s]: min=[%.3f,%.3f,%.3f] max=[%.3f,%.3f,%.3f]",
          box.name.c_str(),
          box.min.x(), box.min.y(), box.min.z(),
          box.max.x(), box.max.y(), box.max.z());
      }
    }
  }

private:
  void initExclusionBox(
    const std::string & prefix, bool default_enabled,
    std::vector<double> default_min, std::vector<double> default_max)
  {
    ExclusionBox box;
    box.name = prefix;
    box.enabled = this->declare_parameter<bool>(prefix + ".enabled", default_enabled);
    box.min = toVector3(
      this->declare_parameter<std::vector<double>>(prefix + ".min", default_min),
      prefix + ".min");
    box.max = toVector3(
      this->declare_parameter<std::vector<double>>(prefix + ".max", default_max),
      prefix + ".max");
    boxes_.push_back(box);
  }

  void handleCloud(const CustomMsg::ConstSharedPtr msg)
  {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    bool has_transform = false;

    if (source_frame_ != target_frame_) {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          target_frame_, source_frame_,
          tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_sec_));
        transform = transformToEigen(tf);
        has_transform = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF failed %s -> %s: %s, passing through",
          source_frame_.c_str(), target_frame_.c_str(), ex.what());
        pub_->publish(*msg);
        return;
      }
    } else {
      has_transform = true;
    }

    auto output = std::make_shared<CustomMsg>();
    output->header = msg->header;
    output->timebase = msg->timebase;
    output->lidar_id = msg->lidar_id;
    output->points.reserve(msg->point_num);

    std::size_t removed = 0;
    const std::size_t total = msg->point_num;
    const bool need_transform = has_transform && (source_frame_ != target_frame_);

    for (std::uint32_t i = 0; i < msg->point_num; ++i) {
      const auto & pt = msg->points[i];

      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }

      float bx = pt.x, by = pt.y, bz = pt.z;
      if (need_transform) {
        Eigen::Vector3f tf_pt = transform * Eigen::Vector3f(pt.x, pt.y, pt.z);
        bx = tf_pt.x(); by = tf_pt.y(); bz = tf_pt.z();
      }

      bool excluded = false;
      for (const auto & box : boxes_) {
        if (box.contains(bx, by, bz)) { excluded = true; break; }
      }

      if (excluded) { ++removed; continue; }

      output->points.push_back(pt);
    }

    output->point_num = static_cast<std::uint32_t>(output->points.size());

    if (++msg_count_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "Self-filter: removed %zu/%zu (%.1f%%)",
        removed, total,
        total > 0 ? 100.0 * static_cast<double>(removed) / static_cast<double>(total) : 0.0);
    }

    pub_->publish(*output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string source_frame_;
  std::string target_frame_;
  double transform_timeout_sec_{0.05};
  std::vector<ExclusionBox> boxes_;
  std::uint64_t msg_count_{0};

  rclcpp::Publisher<CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<CustomMsg>::SharedPtr sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfPointFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
