#include "rm_global_localization/global_localization_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>

#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace rm_global_localization
{
namespace
{

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  q.normalize();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(
    pose.position.x,
    pose.position.y,
    pose.position.z);
  return T;
}

Eigen::Isometry3d transform_to_isometry(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaterniond q(
    transform.rotation.w,
    transform.rotation.x,
    transform.rotation.y,
    transform.rotation.z);
  q.normalize();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(
    transform.translation.x,
    transform.translation.y,
    transform.translation.z);
  return T;
}

geometry_msgs::msg::TransformStamped isometry_to_transform(
  const Eigen::Isometry3d & T,
  const std::string & parent_frame,
  const std::string & child_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent_frame;
  msg.child_frame_id = child_frame;
  msg.transform.translation.x = T.translation().x();
  msg.transform.translation.y = T.translation().y();
  msg.transform.translation.z = T.translation().z();

  const Eigen::Quaterniond q(T.rotation());
  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  return msg;
}

geometry_msgs::msg::PoseStamped isometry_to_pose_stamped(
  const Eigen::Isometry3d & T,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose.position.x = T.translation().x();
  msg.pose.position.y = T.translation().y();
  msg.pose.position.z = T.translation().z();

  const Eigen::Quaterniond q(T.rotation());
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  return msg;
}

std::vector<Eigen::Vector4d> cloud_to_eigen_points(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const Eigen::Isometry3d & transform = Eigen::Isometry3d::Identity())
{
  std::vector<Eigen::Vector4d> points;
  points.reserve(cloud.size());

  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    Eigen::Vector4d p(point.x, point.y, point.z, 1.0);
    points.push_back(transform.matrix() * p);
  }

  return points;
}

}  // namespace

MapManager::MapManager(
  const rclcpp::Logger & logger,
  const std::string & map_path,
  double voxel_size,
  int num_neighbors,
  int num_threads)
: logger_(logger),
  voxel_size_(voxel_size),
  num_neighbors_(num_neighbors),
  num_threads_(num_threads)
{
  load_map(map_path);
}

const small_gicp::PointCloud & MapManager::target_cloud() const
{
  return *target_cloud_;
}

const std::shared_ptr<small_gicp::PointCloud> & MapManager::target_cloud_ptr() const
{
  return target_cloud_;
}

const std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> & MapManager::target_tree() const
{
  return target_tree_;
}

void MapManager::load_map(const std::string & map_path)
{
  pcl::PointCloud<pcl::PointXYZ> raw_map;
  if (pcl::io::loadPCDFile(map_path, raw_map) != 0) {
    throw std::runtime_error("Failed to load map PCD: " + map_path);
  }

  auto raw_points = cloud_to_eigen_points(raw_map);
  if (raw_points.empty()) {
    throw std::runtime_error("Loaded map has no valid points: " + map_path);
  }

  auto map_cloud = std::make_shared<small_gicp::PointCloud>(raw_points);
  target_cloud_ = small_gicp::voxelgrid_sampling_omp(*map_cloud, voxel_size_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
    target_cloud_, small_gicp::KdTreeBuilderOMP(num_threads_));
  small_gicp::estimate_covariances_omp(
    *target_cloud_, *target_tree_, num_neighbors_, num_threads_);

  RCLCPP_INFO(
    logger_,
    "MapManager loaded %zu raw points from %s, downsampled to %zu points (voxel=%.2f m)",
    raw_points.size(),
    map_path.c_str(),
    target_cloud_->size(),
    voxel_size_);
}

RegistrationEngine::RegistrationEngine(
  double scan_voxel_size,
  int num_neighbors,
  int num_threads,
  double max_correspondence_distance,
  int max_iterations,
  double translation_eps,
  double rotation_eps,
  int min_scan_points)
: scan_voxel_size_(scan_voxel_size),
  num_neighbors_(num_neighbors),
  num_threads_(num_threads),
  min_scan_points_(min_scan_points)
{
  registration_.reduction.num_threads = num_threads_;
  registration_.rejector.max_dist_sq =
    max_correspondence_distance * max_correspondence_distance;
  registration_.optimizer.max_iterations = max_iterations;
  registration_.criteria.translation_eps = translation_eps;
  registration_.criteria.rotation_eps = rotation_eps;
}

AlignmentResult RegistrationEngine::align(
  const std::vector<Eigen::Vector4d> & source_points_base,
  const Eigen::Isometry3d & initial_guess,
  const MapManager & map_manager) const
{
  if (static_cast<int>(source_points_base.size()) < min_scan_points_) {
    throw std::runtime_error("Too few valid source points for registration");
  }

  auto start = std::chrono::steady_clock::now();

  auto source_cloud = std::make_shared<small_gicp::PointCloud>(source_points_base);
  source_cloud = small_gicp::voxelgrid_sampling_omp(
    *source_cloud, scan_voxel_size_, num_threads_);

  if (static_cast<int>(source_cloud->size()) < min_scan_points_) {
    throw std::runtime_error("Too few source points after downsampling");
  }

  auto source_tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
    source_cloud, small_gicp::KdTreeBuilderOMP(num_threads_));
  small_gicp::estimate_covariances_omp(
    *source_cloud, *source_tree, num_neighbors_, num_threads_);

  const auto result = registration_.align(
    map_manager.target_cloud(),
    *source_cloud,
    *map_manager.target_tree(),
    initial_guess);

  auto end = std::chrono::steady_clock::now();
  AlignmentResult alignment;
  alignment.T_map_base = result.T_target_source;
  alignment.raw_error = result.error;
  alignment.fitness_score =
    (result.num_inliers > 0) ?
    (result.error / static_cast<double>(result.num_inliers)) :
    std::numeric_limits<double>::infinity();
  alignment.elapsed_ms =
    std::chrono::duration<double, std::milli>(end - start).count();
  alignment.converged = result.converged;
  alignment.iterations = result.iterations;
  alignment.num_inliers = result.num_inliers;
  return alignment;
}

LocalizationNode::LocalizationNode()
: Node("rm_global_localization"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(this)
{
  const auto map_path = this->declare_parameter<std::string>(
    "map_path", "/home/rm/Desktop/SENTRY_FULL/RMUC2026.pcd");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  const auto cloud_topic = this->declare_parameter<std::string>(
    "cloud_topic", "/cloud_registered");
  const auto initialpose_topic = this->declare_parameter<std::string>(
    "initialpose_topic", "/initialpose");
  const auto map_voxel_size = this->declare_parameter<double>("map_voxel_size", 0.25);
  const auto scan_voxel_size = this->declare_parameter<double>("scan_voxel_size", 0.15);
  const auto num_neighbors = this->declare_parameter<int>("num_neighbors", 20);
  const auto num_threads = this->declare_parameter<int>("num_threads", 4);
  const auto max_correspondence_distance =
    this->declare_parameter<double>("max_correspondence_distance", 1.5);
  const auto max_iterations = this->declare_parameter<int>("max_iterations", 20);
  const auto translation_eps = this->declare_parameter<double>("translation_eps", 1e-3);
  const auto rotation_eps = this->declare_parameter<double>("rotation_eps", 1e-3);
  const auto min_scan_points = this->declare_parameter<int>("min_scan_points", 200);
  fitness_threshold_ = this->declare_parameter<double>("fitness_threshold", 0.2);
  relocalization_timeout_sec_ =
    this->declare_parameter<double>("relocalization_timeout_sec", 1.0);
  tf_lookup_timeout_sec_ = this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);

  map_manager_ = std::make_unique<MapManager>(
    this->get_logger(), map_path, map_voxel_size, num_neighbors, num_threads);
  registration_engine_ = std::make_unique<RegistrationEngine>(
    scan_voxel_size,
    num_neighbors,
    num_threads,
    max_correspondence_distance,
    max_iterations,
    translation_eps,
    rotation_eps,
    min_scan_points);

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&LocalizationNode::on_registered_cloud, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initialpose_topic,
    rclcpp::QoS(10),
    std::bind(&LocalizationNode::on_initial_pose, this, std::placeholders::_1));

  localized_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "localized_pose", rclcpp::QoS(10));
  localization_status_pub_ =
    this->create_publisher<rm_interfaces::msg::LocalizationStatus>(
    "localization_status", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "rm_global_localization ready. waiting for /initialpose, cloud_topic=%s map=%s fitness_threshold=%.3f timeout=%.2fs",
    cloud_topic.c_str(),
    map_path.c_str(),
    fitness_threshold_,
    relocalization_timeout_sec_);
}

void LocalizationNode::on_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Ignoring /initialpose in frame '%s' (expected '%s')",
      msg->header.frame_id.c_str(),
      map_frame_.c_str());
    return;
  }

  try {
    const auto T_odom_base = lookup_odom_to_base(rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));
    const auto T_map_base = pose_to_isometry(msg->pose.pose);
    T_map_base_current_ = T_map_base;
    T_map_odom_ = T_map_base * T_odom_base.inverse();
    has_map_odom_ = true;
    is_converged_ = false;

    const auto stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
      this->now() : rclcpp::Time(msg->header.stamp);
    publish_map_to_odom(stamp);
    publish_localized_pose(T_map_base_current_, stamp);
    publish_localization_status(0.0f, false, stamp);

    RCLCPP_INFO(
      this->get_logger(),
      "Initial pose accepted. seeded map->odom from RViz pose at (%.2f, %.2f, %.2f)",
      T_map_base.translation().x(),
      T_map_base.translation().y(),
      T_map_base.translation().z());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to seed map->odom from /initialpose: %s",
      ex.what());
  }
}

void LocalizationNode::on_registered_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!has_map_odom_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for /initialpose before running global localization");
    return;
  }

  try {
    const auto stamp = rclcpp::Time(msg->header.stamp);
    const auto T_odom_base = lookup_odom_to_base(stamp);

    pcl::PointCloud<pcl::PointXYZ> cloud_odom;
    pcl::fromROSMsg(*msg, cloud_odom);
    const auto source_points_base =
      cloud_to_eigen_points(cloud_odom, T_odom_base.inverse());

    const Eigen::Isometry3d initial_guess = T_map_odom_ * T_odom_base;
    const auto alignment = registration_engine_->align(
      source_points_base, initial_guess, *map_manager_);
    publish_localized_pose(alignment.T_map_base, msg->header.stamp);

    if (!alignment.converged) {
      const bool timed_out =
        last_accepted_alignment_time_.nanoseconds() == 0 ||
        (stamp - last_accepted_alignment_time_).seconds() > relocalization_timeout_sec_;
      is_converged_ = !timed_out;
      publish_localization_status(
        static_cast<float>(alignment.fitness_score), is_converged_, msg->header.stamp);
      RCLCPP_WARN(
        this->get_logger(),
        "small_gicp did not converge. keeping previous map->odom "
        "(fitness=%.6f, raw_error=%.6f, inliers=%zu)",
        alignment.fitness_score,
        alignment.raw_error,
        alignment.num_inliers);
      return;
    }

    if (alignment.fitness_score < fitness_threshold_) {
      T_map_base_current_ = alignment.T_map_base;
      T_map_odom_ = alignment.T_map_base * T_odom_base.inverse();
      has_map_odom_ = true;
      is_converged_ = true;
      last_accepted_alignment_time_ = stamp;
      publish_map_to_odom(msg->header.stamp);
    } else {
      const bool timed_out =
        last_accepted_alignment_time_.nanoseconds() == 0 ||
        (stamp - last_accepted_alignment_time_).seconds() > relocalization_timeout_sec_;
      is_converged_ = !timed_out;
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected map->odom update due to low registration quality: "
        "fitness=%.6f threshold=%.6f raw_error=%.6f inliers=%zu",
        alignment.fitness_score,
        fitness_threshold_,
        alignment.raw_error,
        alignment.num_inliers);
    }

    publish_localization_status(
      static_cast<float>(alignment.fitness_score), is_converged_, msg->header.stamp);

    RCLCPP_INFO(
      this->get_logger(),
      "Global localization: %.2f ms, fitness=%.6f, raw_error=%.6f, inliers=%zu, iterations=%zu",
      alignment.elapsed_ms,
      alignment.fitness_score,
      alignment.raw_error,
      alignment.num_inliers,
      alignment.iterations);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "TF lookup failed in global localization: %s",
      ex.what());
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Global localization skipped current cloud: %s",
      ex.what());
  }
}

void LocalizationNode::publish_map_to_odom(const builtin_interfaces::msg::Time & stamp)
{
  tf_broadcaster_.sendTransform(
    isometry_to_transform(T_map_odom_, map_frame_, odom_frame_, stamp));
}

void LocalizationNode::publish_localized_pose(
  const Eigen::Isometry3d & T_map_base,
  const builtin_interfaces::msg::Time & stamp)
{
  localized_pose_pub_->publish(
    isometry_to_pose_stamped(T_map_base, map_frame_, stamp));
}

void LocalizationNode::publish_localization_status(
  float fitness_score,
  bool is_converged,
  const builtin_interfaces::msg::Time & stamp)
{
  rm_interfaces::msg::LocalizationStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = map_frame_;
  status.fitness_score = fitness_score;
  status.is_converged = is_converged;
  localization_status_pub_->publish(status);
}

Eigen::Isometry3d LocalizationNode::lookup_odom_to_base(const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TransformStamped tf_msg;
  const auto timeout = rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_);

  if (stamp.nanoseconds() == 0) {
    const auto latest = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    tf_msg = tf_buffer_.lookupTransform(
      odom_frame_, base_frame_, latest, timeout);
  } else {
    tf_msg = tf_buffer_.lookupTransform(
      odom_frame_, base_frame_, stamp, timeout);
  }

  return transform_to_isometry(tf_msg.transform);
}

}  // namespace rm_global_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_global_localization::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
