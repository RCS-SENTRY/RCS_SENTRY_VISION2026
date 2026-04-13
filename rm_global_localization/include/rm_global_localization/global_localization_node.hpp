#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/localization_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rm_global_localization
{

struct AlignmentResult
{
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  double fitness_score = 0.0;
  double raw_error = 0.0;
  double elapsed_ms = 0.0;
  bool converged = false;
  std::size_t iterations = 0;
  std::size_t num_inliers = 0;
};

class MapManager
{
public:
  MapManager(
    const rclcpp::Logger & logger,
    const std::string & map_path,
    double voxel_size,
    int num_neighbors,
    int num_threads);

  const small_gicp::PointCloud & target_cloud() const;
  const std::shared_ptr<small_gicp::PointCloud> & target_cloud_ptr() const;
  const std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> & target_tree() const;

private:
  void load_map(const std::string & map_path);

  rclcpp::Logger logger_;
  double voxel_size_;
  int num_neighbors_;
  int num_threads_;
  std::shared_ptr<small_gicp::PointCloud> target_cloud_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;
};

class RegistrationEngine
{
public:
  RegistrationEngine(
    double scan_voxel_size,
    int num_neighbors,
    int num_threads,
    double max_correspondence_distance,
    int max_iterations,
    double translation_eps,
    double rotation_eps,
    int min_scan_points);

  AlignmentResult align(
    const std::vector<Eigen::Vector4d> & source_points_base,
    const Eigen::Isometry3d & initial_guess,
    const MapManager & map_manager) const;

private:
  double scan_voxel_size_;
  int num_neighbors_;
  int num_threads_;
  int min_scan_points_;
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration_;
};

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode();

private:
  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void on_registered_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_map_to_odom(const builtin_interfaces::msg::Time & stamp);
  void publish_localized_pose(
    const Eigen::Isometry3d & T_map_base,
    const builtin_interfaces::msg::Time & stamp);
  void publish_localization_status(
    float fitness_score,
    bool is_converged,
    const builtin_interfaces::msg::Time & stamp);
  Eigen::Isometry3d lookup_odom_to_base(const rclcpp::Time & stamp) const;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double tf_lookup_timeout_sec_;
  double fitness_threshold_;
  double relocalization_timeout_sec_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::unique_ptr<MapManager> map_manager_;
  std::unique_ptr<RegistrationEngine> registration_engine_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr localized_pose_pub_;
  rclcpp::Publisher<rm_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_status_pub_;

  Eigen::Isometry3d T_map_base_current_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
  bool has_map_odom_ = false;
  bool is_converged_ = false;
  rclcpp::Time last_accepted_alignment_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace rm_global_localization
