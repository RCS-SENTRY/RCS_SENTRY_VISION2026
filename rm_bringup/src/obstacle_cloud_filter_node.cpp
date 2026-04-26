#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <chrono>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct ExclusionBox
{
  std::string name;
  bool enabled{false};
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  Eigen::Vector3f max = Eigen::Vector3f::Zero();

  bool contains(const pcl::PointXYZ & point) const
  {
    return enabled &&
           point.x >= min.x() && point.x <= max.x() &&
           point.y >= min.y() && point.y <= max.y() &&
           point.z >= min.z() && point.z <= max.z();
  }
};

struct MemoryCell
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
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

std::int64_t cellKey(std::int32_t ix, std::int32_t iy)
{
  return (static_cast<std::int64_t>(ix) << 32) ^
         static_cast<std::uint32_t>(iy);
}

}  // namespace

class ObstacleCloudFilterNode : public rclcpp::Node
{
public:
  ObstacleCloudFilterNode()
  : Node("obstacle_cloud_filter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    primary_input_topic_ = this->declare_parameter<std::string>(
      "primary_input_topic", "/cloud_registered");
    secondary_input_topic_ = this->declare_parameter<std::string>(
      "secondary_input_topic", "");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/nav_obstacle_memory");
    target_frame_ = this->declare_parameter<std::string>(
      "target_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>(
      "base_frame", "base_link");
    min_height_ = static_cast<float>(this->declare_parameter<double>("min_height", 0.05));
    max_height_ = static_cast<float>(this->declare_parameter<double>("max_height", 1.50));
    min_range_ = static_cast<float>(this->declare_parameter<double>("min_range", 0.20));
    max_range_ = static_cast<float>(this->declare_parameter<double>("max_range", 5.00));
    memory_resolution_ = static_cast<float>(this->declare_parameter<double>("memory_resolution", 0.06));
    fading_timeout_sec_ = this->declare_parameter<double>("fading_timeout_sec", 1.80);
    transform_timeout_sec_ = this->declare_parameter<double>("transform_timeout_sec", 0.05);
    allow_latest_tf_fallback_ = this->declare_parameter<bool>("allow_latest_tf_fallback", true);
    debug_log_period_sec_ = this->declare_parameter<double>("debug_log_period_sec", 2.0);

    body_box_.name = "body_exclusion_box";
    body_box_.enabled = this->declare_parameter<bool>("body_box.enabled", true);
    body_box_.min = toVector3(
      this->declare_parameter<std::vector<double>>(
        "body_box.min", {-0.27, -0.27, -0.05}),
      "body_box.min");
    body_box_.max = toVector3(
      this->declare_parameter<std::vector<double>>(
        "body_box.max", {0.27, 0.27, 0.60}),
      "body_box.max");

    gimbal_box_.name = "gimbal_exclusion_box";
    gimbal_box_.enabled = this->declare_parameter<bool>("gimbal_box.enabled", true);
    gimbal_box_.min = toVector3(
      this->declare_parameter<std::vector<double>>(
        "gimbal_box.min", {-0.16, -0.16, 0.42}),
      "gimbal_box.min");
    gimbal_box_.max = toVector3(
      this->declare_parameter<std::vector<double>>(
        "gimbal_box.max", {0.16, 0.16, 0.80}),
      "gimbal_box.max");

    gimbal_support_box_.name = "gimbal_support_exclusion_box";
    gimbal_support_box_.enabled = this->declare_parameter<bool>("gimbal_support_box.enabled", true);
    gimbal_support_box_.min = toVector3(
      this->declare_parameter<std::vector<double>>(
        "gimbal_support_box.min", {-0.10, -0.10, 0.20}),
      "gimbal_support_box.min");
    gimbal_support_box_.max = toVector3(
      this->declare_parameter<std::vector<double>>(
        "gimbal_support_box.max", {0.10, 0.10, 0.48}),
      "gimbal_support_box.max");

    lidar_arm_box_.name = "lidar_arm_exclusion_box";
    lidar_arm_box_.enabled = this->declare_parameter<bool>("lidar_arm_box.enabled", true);
    lidar_arm_box_.min = toVector3(
      this->declare_parameter<std::vector<double>>(
        "lidar_arm_box.min", {-0.08, 0.10, 0.18}),
      "lidar_arm_box.min");
    lidar_arm_box_.max = toVector3(
      this->declare_parameter<std::vector<double>>(
        "lidar_arm_box.max", {0.12, 0.35, 0.52}),
      "lidar_arm_box.max");

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    debug_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/obstacle_memory_debug",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    primary_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      primary_input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ObstacleCloudFilterNode::handleCloud, this, std::placeholders::_1));

    if (!secondary_input_topic_.empty()) {
      secondary_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        secondary_input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ObstacleCloudFilterNode::handleCloud, this, std::placeholders::_1));
    }

    if (debug_log_period_sec_ > 0.0) {
      debug_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(debug_log_period_sec_)),
        std::bind(&ObstacleCloudFilterNode::logDebugState, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Obstacle memory ready: primary=%s secondary=%s output=%s memory_frame=%s",
      primary_input_topic_.c_str(),
      secondary_input_topic_.empty() ? "disabled" : secondary_input_topic_.c_str(),
      output_topic_.c_str(),
      target_frame_.c_str());
  }

private:
  void handleCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    ++received_cloud_count_;
    if (msg->header.frame_id.empty()) {
      last_drop_reason_ = "missing_frame_id";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received cloud without frame_id, dropping");
      return;
    }
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      last_drop_reason_ = "missing_stamp";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received cloud without valid stamp, dropping");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    auto filtered_cloud = filterCloud(*msg, stamp);
    if (!filtered_cloud) {
      return;
    }
    ++transformed_cloud_count_;
    if (filtered_cloud->empty()) {
      last_drop_reason_ = "filtered_empty";
    } else {
      last_drop_reason_ = "ok";
    }

    updateMemory(*filtered_cloud, stamp);
    publishMemory(stamp);
  }

  Cloud::Ptr filterCloud(const sensor_msgs::msg::PointCloud2 & msg, const rclcpp::Time & stamp)
  {
    auto input_cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(msg, *input_cloud);
    if (input_cloud->empty()) {
      ++empty_input_count_;
      last_drop_reason_ = "input_empty";
      return std::make_shared<Cloud>();
    }

    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, finite_indices);

    auto cloud_in_base = std::make_shared<Cloud>();
    if (!transformCloudExact(*input_cloud, msg.header.frame_id, base_frame_, stamp, *cloud_in_base)) {
      last_drop_reason_ = "tf_to_base_failed";
      return nullptr;
    }

    Cloud filtered_in_base;
    filtered_in_base.points.reserve(cloud_in_base->points.size());
    const float min_range_sq = min_range_ * min_range_;
    const float max_range_sq = max_range_ * max_range_;

    for (const auto & point : cloud_in_base->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }

      const float planar_range_sq = point.x * point.x + point.y * point.y;
      if (planar_range_sq < min_range_sq || planar_range_sq > max_range_sq) {
        continue;
      }
      if (point.z < min_height_ || point.z > max_height_) {
        continue;
      }
      if (body_box_.contains(point) || gimbal_box_.contains(point) ||
          gimbal_support_box_.contains(point) || lidar_arm_box_.contains(point))
      {
        continue;
      }
      filtered_in_base.points.push_back(point);
    }

    filtered_in_base.width = static_cast<std::uint32_t>(filtered_in_base.points.size());
    filtered_in_base.height = 1;
    filtered_in_base.is_dense = true;

    auto filtered_in_target = std::make_shared<Cloud>();
    if (!transformCloudExact(filtered_in_base, base_frame_, target_frame_, stamp, *filtered_in_target)) {
      last_drop_reason_ = "tf_to_memory_failed";
      return nullptr;
    }
    if (filtered_in_target->empty()) {
      ++empty_after_filter_count_;
    }
    return filtered_in_target;
  }

  void logDebugState()
  {
    std::size_t memory_cells = 0U;
    {
      std::lock_guard<std::mutex> lock(memory_mutex_);
      memory_cells = memory_.size();
    }

    std_msgs::msg::String debug_msg;
    debug_msg.data =
      "received=" + std::to_string(received_cloud_count_) +
      " transformed=" + std::to_string(transformed_cloud_count_) +
      " exact_tf=" + std::to_string(exact_tf_success_count_) +
      " latest_tf_fallback=" + std::to_string(latest_tf_fallback_count_) +
      " dropped_tf=" + std::to_string(dropped_tf_count_) +
      " empty_input=" + std::to_string(empty_input_count_) +
      " empty_after_filter=" + std::to_string(empty_after_filter_count_) +
      " cells=" + std::to_string(memory_cells) +
      " last_drop_reason=" + last_drop_reason_;
    debug_publisher_->publish(debug_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Obstacle memory status: received=%llu transformed=%llu exact_tf=%llu "
      "latest_tf_fallback=%llu dropped_tf=%llu empty_input=%llu "
      "empty_after_filter=%llu cells=%zu last_drop_reason=%s",
      static_cast<unsigned long long>(received_cloud_count_),
      static_cast<unsigned long long>(transformed_cloud_count_),
      static_cast<unsigned long long>(exact_tf_success_count_),
      static_cast<unsigned long long>(latest_tf_fallback_count_),
      static_cast<unsigned long long>(dropped_tf_count_),
      static_cast<unsigned long long>(empty_input_count_),
      static_cast<unsigned long long>(empty_after_filter_count_),
      memory_cells,
      last_drop_reason_.c_str());
  }

  bool transformCloudExact(
    const Cloud & input_cloud,
    const std::string & source_frame,
    const std::string & target_frame,
    const rclcpp::Time & stamp,
    Cloud & output_cloud)
  {
    if (source_frame == target_frame) {
      output_cloud = input_cloud;
      return true;
    }

    try {
      const auto exact_transform = tf_buffer_.lookupTransform(
        target_frame,
        source_frame,
        stamp,
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      pcl::transformPointCloud(input_cloud, output_cloud, transformToEigen(exact_transform));
      ++exact_tf_success_count_;
      return true;
    } catch (const tf2::TransformException & exact_ex) {
      if (allow_latest_tf_fallback_) {
        try {
          const auto latest_transform = tf_buffer_.lookupTransform(
            target_frame,
            source_frame,
            rclcpp::Time(0, 0, get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(transform_timeout_sec_));
          pcl::transformPointCloud(input_cloud, output_cloud, transformToEigen(latest_transform));
          ++latest_tf_fallback_count_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Exact TF failed for %s -> %s at stamp %.6f, falling back to latest TF: %s",
            source_frame.c_str(), target_frame.c_str(), stamp.seconds(), exact_ex.what());
          return true;
        } catch (const tf2::TransformException & latest_ex) {
          ++dropped_tf_count_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Failed to transform cloud %s -> %s at stamp %.6f with exact and latest TF. "
            "exact='%s' latest='%s'",
            source_frame.c_str(), target_frame.c_str(), stamp.seconds(),
            exact_ex.what(), latest_ex.what());
          return false;
        }
      }

      ++dropped_tf_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform cloud %s -> %s at stamp %.6f: %s",
        source_frame.c_str(), target_frame.c_str(), stamp.seconds(), exact_ex.what());
      return false;
    }
  }

  void updateMemory(const Cloud & cloud, const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    pruneExpiredCellsLocked(stamp);

    for (const auto & point : cloud.points) {
      const auto ix = static_cast<std::int32_t>(std::floor(point.x / memory_resolution_));
      const auto iy = static_cast<std::int32_t>(std::floor(point.y / memory_resolution_));
      const auto key = cellKey(ix, iy);

      auto & cell = memory_[key];
      cell.x = (static_cast<float>(ix) + 0.5F) * memory_resolution_;
      cell.y = (static_cast<float>(iy) + 0.5F) * memory_resolution_;
      cell.z = point.z;
      cell.last_seen = stamp;
    }
  }

  void pruneExpiredCellsLocked(const rclcpp::Time & stamp)
  {
    if (fading_timeout_sec_ <= 0.0) {
      return;
    }

    for (auto it = memory_.begin(); it != memory_.end();) {
      if ((stamp - it->second.last_seen).seconds() > fading_timeout_sec_) {
        it = memory_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void publishMemory(const rclcpp::Time & stamp)
  {
    auto output_cloud = std::make_shared<Cloud>();
    output_cloud->height = 1;
    output_cloud->is_dense = true;

    {
      std::lock_guard<std::mutex> lock(memory_mutex_);
      output_cloud->points.reserve(memory_.size());
      for (const auto & item : memory_) {
        pcl::PointXYZ point;
        point.x = item.second.x;
        point.y = item.second.y;
        point.z = item.second.z;
        output_cloud->points.push_back(point);
      }
    }

    output_cloud->width = static_cast<std::uint32_t>(output_cloud->points.size());

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*output_cloud, output_msg);
    output_msg.header.frame_id = target_frame_;
    output_msg.header.stamp = stamp;
    publisher_->publish(output_msg);
  }

  std::string primary_input_topic_;
  std::string secondary_input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string base_frame_;

  float min_height_{0.05F};
  float max_height_{1.50F};
  float min_range_{0.20F};
  float max_range_{5.50F};
  float memory_resolution_{0.06F};
  double fading_timeout_sec_{1.80};
  double transform_timeout_sec_{0.05};
  bool allow_latest_tf_fallback_{true};
  double debug_log_period_sec_{2.0};
  std::string last_drop_reason_{"startup"};
  std::uint64_t received_cloud_count_{0U};
  std::uint64_t transformed_cloud_count_{0U};
  std::uint64_t exact_tf_success_count_{0U};
  std::uint64_t latest_tf_fallback_count_{0U};
  std::uint64_t dropped_tf_count_{0U};
  std::uint64_t empty_input_count_{0U};
  std::uint64_t empty_after_filter_count_{0U};

  ExclusionBox body_box_;
  ExclusionBox gimbal_box_;
  ExclusionBox gimbal_support_box_;
  ExclusionBox lidar_arm_box_;

  std::unordered_map<std::int64_t, MemoryCell> memory_;
  std::mutex memory_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr primary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr secondary_sub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleCloudFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
