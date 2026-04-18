#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
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

struct InputState
{
  Cloud::Ptr cloud{std::make_shared<Cloud>()};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool has_data{false};
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
      "primary_input_topic", "/livox/lidar/pointcloud");
    secondary_input_topic_ = this->declare_parameter<std::string>(
      "secondary_input_topic", "");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/nav_obstacle_cloud");
    target_frame_ = this->declare_parameter<std::string>(
      "target_frame", "base_link");
    min_height_ = static_cast<float>(this->declare_parameter<double>("min_height", 0.05));
    max_height_ = static_cast<float>(this->declare_parameter<double>("max_height", 1.50));
    min_range_ = static_cast<float>(this->declare_parameter<double>("min_range", 0.20));
    max_range_ = static_cast<float>(this->declare_parameter<double>("max_range", 5.00));
    voxel_leaf_size_ = static_cast<float>(this->declare_parameter<double>("voxel_leaf_size", 0.10));
    merge_timeout_sec_ = this->declare_parameter<double>("merge_timeout_sec", 0.20);
    transform_timeout_sec_ = this->declare_parameter<double>("transform_timeout_sec", 0.05);

    body_box_.name = "body_exclusion_box";
    body_box_.enabled = this->declare_parameter<bool>("body_box.enabled", true);
    body_box_.min = toVector3(
      this->declare_parameter<std::vector<double>>(
        "body_box.min", {-0.42, -0.34, -0.20}),
      "body_box.min");
    body_box_.max = toVector3(
      this->declare_parameter<std::vector<double>>(
        "body_box.max", {0.42, 0.34, 0.45}),
      "body_box.max");

    gimbal_box_.name = "gimbal_exclusion_box";
    gimbal_box_.enabled = this->declare_parameter<bool>("gimbal_box.enabled", true);
    gimbal_box_.min = toVector3(
      this->declare_parameter<std::vector<double>>(
        "gimbal_box.min", {0.08, -0.18, 0.18}),
      "gimbal_box.min");
    gimbal_box_.max = toVector3(
      this->declare_parameter<std::vector<double>>(
        "gimbal_box.max", {0.62, 0.18, 0.90}),
      "gimbal_box.max");

    if (min_height_ >= max_height_) {
      throw std::runtime_error("min_height must be smaller than max_height");
    }
    if (min_range_ < 0.0F || min_range_ >= max_range_) {
      throw std::runtime_error("min_range must be non-negative and smaller than max_range");
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());

    primary_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      primary_input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ObstacleCloudFilterNode::handlePrimaryCloud, this, std::placeholders::_1));

    if (!secondary_input_topic_.empty()) {
      secondary_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        secondary_input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ObstacleCloudFilterNode::handleSecondaryCloud, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "Obstacle cloud filter ready: %s -> %s (frame=%s, secondary=%s)",
      primary_input_topic_.c_str(),
      output_topic_.c_str(),
      target_frame_.c_str(),
      secondary_input_topic_.empty() ? "disabled" : secondary_input_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Filter window: height=[%.2f, %.2f] range=[%.2f, %.2f] voxel=%.2f",
      min_height_, max_height_, min_range_, max_range_, voxel_leaf_size_);
  }

private:
  void handlePrimaryCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    processInput(0U, msg);
  }

  void handleSecondaryCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    processInput(1U, msg);
  }

  void processInput(
    std::size_t index,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    auto filtered_cloud = filterCloud(*msg);
    if (!filtered_cloud) {
      return;
    }

    const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
      ? this->now()
      : rclcpp::Time(msg->header.stamp);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      input_states_.at(index).cloud = filtered_cloud;
      input_states_.at(index).stamp = stamp;
      input_states_.at(index).has_data = true;
    }

    publishMergedCloud(stamp);
  }

  Cloud::Ptr filterCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    auto input_cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(msg, *input_cloud);

    if (input_cloud->empty()) {
      return std::make_shared<Cloud>();
    }

    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, finite_indices);

    auto transformed_cloud = std::make_shared<Cloud>();
    if (!transformToTargetFrame(*input_cloud, msg.header.frame_id, transformed_cloud)) {
      return nullptr;
    }

    auto filtered_cloud = std::make_shared<Cloud>();
    filtered_cloud->points.reserve(transformed_cloud->points.size());

    const float min_range_sq = min_range_ * min_range_;
    const float max_range_sq = max_range_ * max_range_;

    for (const auto & point : transformed_cloud->points) {
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

      if (body_box_.contains(point) || gimbal_box_.contains(point)) {
        continue;
      }

      filtered_cloud->points.push_back(point);
    }

    filtered_cloud->width = static_cast<std::uint32_t>(filtered_cloud->points.size());
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    return filtered_cloud;
  }

  bool transformToTargetFrame(
    const Cloud & input_cloud,
    const std::string & input_frame,
    Cloud::Ptr & transformed_cloud)
  {
    if (input_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received point cloud without frame_id, dropping message");
      return false;
    }

    if (input_frame == target_frame_) {
      *transformed_cloud = input_cloud;
      return true;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_,
        input_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      pcl::transformPointCloud(input_cloud, *transformed_cloud, transformToEigen(transform));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform cloud %s -> %s: %s",
        input_frame.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  Cloud::Ptr downsampleCloud(const Cloud::Ptr & cloud) const
  {
    if (!cloud || cloud->empty() || voxel_leaf_size_ <= std::numeric_limits<float>::epsilon()) {
      return cloud ? cloud : std::make_shared<Cloud>();
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.setInputCloud(cloud);

    auto downsampled_cloud = std::make_shared<Cloud>();
    voxel_filter.filter(*downsampled_cloud);
    return downsampled_cloud;
  }

  void publishMergedCloud(const rclcpp::Time & reference_stamp)
  {
    auto merged_cloud = std::make_shared<Cloud>();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (input_states_[0].has_data) {
        *merged_cloud += *input_states_[0].cloud;
      }

      if (!secondary_input_topic_.empty() && input_states_[1].has_data) {
        const double dt = std::abs((reference_stamp - input_states_[1].stamp).seconds());
        if (dt <= merge_timeout_sec_) {
          *merged_cloud += *input_states_[1].cloud;
        }
      }
    }

    merged_cloud->width = static_cast<std::uint32_t>(merged_cloud->points.size());

    auto output_cloud = downsampleCloud(merged_cloud);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*output_cloud, output_msg);
    output_msg.header.stamp = reference_stamp;
    output_msg.header.frame_id = target_frame_;
    publisher_->publish(output_msg);
  }

  std::string primary_input_topic_;
  std::string secondary_input_topic_;
  std::string output_topic_;
  std::string target_frame_;

  float min_height_{0.05F};
  float max_height_{1.50F};
  float min_range_{0.20F};
  float max_range_{5.00F};
  float voxel_leaf_size_{0.10F};
  double merge_timeout_sec_{0.20};
  double transform_timeout_sec_{0.05};

  ExclusionBox body_box_;
  ExclusionBox gimbal_box_;

  std::array<InputState, 2> input_states_;
  std::mutex state_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr primary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr secondary_sub_;

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
