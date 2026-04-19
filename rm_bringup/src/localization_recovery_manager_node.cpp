#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
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

geometry_msgs::msg::TransformStamped isometryToTransform(
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

double rotationDeltaRad(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  Eigen::AngleAxisd angle_axis(a.rotation().inverse() * b.rotation());
  return std::abs(angle_axis.angle());
}

}  // namespace

class LocalizationRecoveryManagerNode : public rclcpp::Node
{
public:
  LocalizationRecoveryManagerNode()
  : Node("localization_recovery_manager"),
    tf_broadcaster_(this)
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    initialpose_topic_ =
      this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
    reset_odom_service_ =
      this->declare_parameter<std::string>("reset_odom_service", "/reset_odom");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    status_topic_ = this->declare_parameter<std::string>(
      "status_topic", "/localization_recovery_status");
    map_to_odom_publish_hz_ =
      this->declare_parameter<double>("map_to_odom_publish_hz", 20.0);
    odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 0.50);
    spin_threshold_rad_s_ =
      this->declare_parameter<double>("spin_threshold_rad_s", 5.0);
    pose_jump_distance_m_ =
      this->declare_parameter<double>("pose_jump_distance_m", 0.80);
    pose_jump_angle_rad_ =
      this->declare_parameter<double>("pose_jump_angle_rad", 1.20);
    recover_settle_sec_ =
      this->declare_parameter<double>("recover_settle_sec", 1.00);
    last_good_update_period_sec_ =
      this->declare_parameter<double>("last_good_update_period_sec", 0.20);
    max_good_linear_speed_mps_ =
      this->declare_parameter<double>("max_good_linear_speed_mps", 1.00);
    max_good_angular_speed_rad_s_ =
      this->declare_parameter<double>("max_good_angular_speed_rad_s", 1.00);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(20),
      std::bind(&LocalizationRecoveryManagerNode::onOdometry, this, std::placeholders::_1));

    initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_,
      rclcpp::QoS(10),
      std::bind(&LocalizationRecoveryManagerNode::onInitialPose, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    reset_odom_client_ = this->create_client<std_srvs::srv::Empty>(reset_odom_service_);

    if (map_to_odom_publish_hz_ > 0.0) {
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / map_to_odom_publish_hz_)),
        std::bind(&LocalizationRecoveryManagerNode::onTimer, this));
    }

    publishStatus();
    RCLCPP_INFO(
      this->get_logger(),
      "Localization recovery manager ready: odom=%s initialpose=%s reset=%s",
      odom_topic_.c_str(),
      initialpose_topic_.c_str(),
      reset_odom_service_.c_str());
  }

private:
  enum class State
  {
    WAITING_FOR_INITIALPOSE,
    NORMAL,
    LOST,
    RECOVERING
  };

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring /initialpose in frame '%s' (expected '%s')",
        msg->header.frame_id.c_str(),
        map_frame_.c_str());
      return;
    }

    pending_initial_pose_ = *msg;
    has_pending_initial_pose_ = true;
    if (has_current_odom_) {
      seedFromInitialPose(*msg, false);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Received /initialpose before odom is ready. Cached pose will be applied once odom arrives.");
    }
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
      this->now() : rclcpp::Time(msg->header.stamp);
    const auto odom_base = poseToIsometry(msg->pose.pose);
    const double linear_speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    const double angular_speed = std::abs(msg->twist.twist.angular.z);

    if (has_current_odom_) {
      const double dist_jump =
        (odom_base.translation() - current_odom_base_.translation()).norm();
      const double angle_jump = rotationDeltaRad(current_odom_base_, odom_base);
      if (state_ == State::NORMAL &&
        (angular_speed > spin_threshold_rad_s_ ||
        dist_jump > pose_jump_distance_m_ ||
        angle_jump > pose_jump_angle_rad_))
      {
        triggerRecovery(
          stamp,
          "odom anomaly: ang=" + std::to_string(angular_speed) +
          " dist_jump=" + std::to_string(dist_jump) +
          " angle_jump=" + std::to_string(angle_jump));
      }
    }

    current_odom_base_ = odom_base;
    last_odom_time_ = stamp;
    has_current_odom_ = true;

    if (has_pending_initial_pose_ && !has_map_odom_) {
      seedFromInitialPose(pending_initial_pose_, true);
    }

    if (has_map_odom_ && state_ == State::NORMAL) {
      const auto current_map_base = T_map_odom_ * current_odom_base_;
      if ((stamp - last_good_update_time_).seconds() >= last_good_update_period_sec_ &&
        linear_speed <= max_good_linear_speed_mps_ &&
        angular_speed <= max_good_angular_speed_rad_s_)
      {
        last_good_map_base_ = current_map_base;
        last_good_update_time_ = stamp;
        has_last_good_pose_ = true;
      }
    }

    if (state_ == State::RECOVERING &&
      (stamp - state_enter_time_).seconds() >= recover_settle_sec_ &&
      angular_speed <= max_good_angular_speed_rad_s_)
    {
      state_ = State::NORMAL;
      RCLCPP_INFO(this->get_logger(), "Recovery settled. Back to NORMAL.");
      publishStatus();
    }
  }

  void onTimer()
  {
    const auto now = this->now();
    if (has_current_odom_ && state_ == State::NORMAL &&
      (now - last_odom_time_).seconds() > odom_timeout_sec_)
    {
      triggerRecovery(now, "odom timeout");
    }

    if (has_map_odom_) {
      builtin_interfaces::msg::Time stamp_msg;
      stamp_msg.sec = static_cast<int32_t>(now.seconds());
      stamp_msg.nanosec =
        static_cast<uint32_t>(now.nanoseconds() - (static_cast<int64_t>(stamp_msg.sec) * 1000000000LL));
      tf_broadcaster_.sendTransform(
        isometryToTransform(T_map_odom_, map_frame_, odom_frame_, stamp_msg));
    }
    publishStatus();
  }

  void seedFromInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
    bool from_retry)
  {
    if (!has_current_odom_) {
      return;
    }

    const auto T_map_base = poseToIsometry(msg.pose.pose);
    T_map_odom_ = T_map_base * current_odom_base_.inverse();
    has_map_odom_ = true;
    has_pending_initial_pose_ = false;
    last_good_map_base_ = T_map_base;
    has_last_good_pose_ = true;
    last_good_update_time_ = this->now();
    state_ = State::NORMAL;
    publishStatus();

    RCLCPP_INFO(
      this->get_logger(),
      "%s /initialpose and seeded map->odom at (%.2f, %.2f, %.2f)",
      from_retry ? "Applied cached" : "Accepted",
      T_map_base.translation().x(),
      T_map_base.translation().y(),
      T_map_base.translation().z());
  }

  void triggerRecovery(const rclcpp::Time & stamp, const std::string & reason)
  {
    if (state_ == State::LOST || state_ == State::RECOVERING) {
      return;
    }

    state_enter_time_ = stamp;
    state_ = State::LOST;
    publishStatus();
    RCLCPP_WARN(this->get_logger(), "Localization entered LOST: %s", reason.c_str());

    if (!has_last_good_pose_) {
      RCLCPP_WARN(this->get_logger(), "No last_good_map_base available. Waiting for manual /initialpose.");
      return;
    }
    if (reset_odom_request_in_flight_) {
      return;
    }
    if (!reset_odom_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(this->get_logger(), "reset_odom service unavailable. Waiting for manual intervention.");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_odom_request_in_flight_ = true;
    reset_odom_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
        reset_odom_request_in_flight_ = false;
        try {
          future.get();
        } catch (const std::exception & ex) {
          RCLCPP_WARN(this->get_logger(), "reset_odom failed: %s", ex.what());
          state_ = State::LOST;
          publishStatus();
          return;
        }

        T_map_odom_ = last_good_map_base_;
        has_map_odom_ = true;
        state_ = State::RECOVERING;
        state_enter_time_ = this->now();
        publishStatus();
        RCLCPP_INFO(
          this->get_logger(),
          "reset_odom succeeded. Restored map->odom from last_good_map_base.");
      });
  }

  void publishStatus()
  {
    std_msgs::msg::String msg;
    switch (state_) {
      case State::WAITING_FOR_INITIALPOSE:
        msg.data = "WAITING_FOR_INITIALPOSE";
        break;
      case State::NORMAL:
        msg.data = "NORMAL";
        break;
      case State::LOST:
        msg.data = "LOST";
        break;
      case State::RECOVERING:
        msg.data = "RECOVERING";
        break;
    }
    status_pub_->publish(msg);
  }

  std::string odom_topic_;
  std::string initialpose_topic_;
  std::string reset_odom_service_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string status_topic_;

  double map_to_odom_publish_hz_{20.0};
  double odom_timeout_sec_{0.50};
  double spin_threshold_rad_s_{5.0};
  double pose_jump_distance_m_{0.80};
  double pose_jump_angle_rad_{1.20};
  double recover_settle_sec_{1.00};
  double last_good_update_period_sec_{0.20};
  double max_good_linear_speed_mps_{1.00};
  double max_good_angular_speed_rad_s_{1.00};

  State state_{State::WAITING_FOR_INITIALPOSE};

  bool has_current_odom_{false};
  bool has_map_odom_{false};
  bool has_last_good_pose_{false};
  bool has_pending_initial_pose_{false};
  bool reset_odom_request_in_flight_{false};

  Eigen::Isometry3d current_odom_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_map_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_good_map_base_{Eigen::Isometry3d::Identity()};

  geometry_msgs::msg::PoseWithCovarianceStamped pending_initial_pose_;

  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_good_update_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_odom_client_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationRecoveryManagerNode>());
  rclcpp::shutdown();
  return 0;
}
