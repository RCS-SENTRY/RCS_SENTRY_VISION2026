#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
struct RegionStats {
  double front_min = std::numeric_limits<double>::infinity();
  double back_min = std::numeric_limits<double>::infinity();
  double left_min = std::numeric_limits<double>::infinity();
  double right_min = std::numeric_limits<double>::infinity();
  int front_pts = 0;
  int back_pts = 0;
  int left_pts = 0;
  int right_pts = 0;
};

struct ScaleResult {
  double scale_vx_pos = 1.0;
  double scale_vx_neg = 1.0;
  double scale_vy_pos = 1.0;
  double scale_vy_neg = 1.0;
  bool emergency = false;
};

std::string FormatDistance(double value) {
  if (!std::isfinite(value)) {
    return "inf";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << value;
  return ss.str();
}

std::string FormatScale(double value) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << value;
  return ss.str();
}
}  // namespace

class SecondLidarSafetyLimiterNode final : public rclcpp::Node {
public:
  SecondLidarSafetyLimiterNode() : rclcpp::Node("second_lidar_safety_limiter") {
    input_cmd_vel_topic_ = declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
    output_cmd_vel_topic_ = declare_parameter<std::string>("output_cmd_vel_topic", "/cmd_vel_safe");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/second_lidar_obstacle_cloud");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/second_lidar_safety_debug");
    frame_id_ = declare_parameter<std::string>("frame_id", "gimbal_yaw");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.25);
    obstacle_timeout_sec_ = declare_parameter<double>("obstacle_timeout_sec", 0.25);
    emergency_distance_ = declare_parameter<double>("emergency_distance", 0.35);
    slow_distance_ = declare_parameter<double>("slow_distance", 0.70);
    caution_distance_ = declare_parameter<double>("caution_distance", 1.20);
    front_angle_rad_ = DegreesToRadians(declare_parameter<double>("front_angle_deg", 50.0));
    side_angle_rad_ = DegreesToRadians(declare_parameter<double>("side_angle_deg", 70.0));
    rear_angle_rad_ = DegreesToRadians(declare_parameter<double>("rear_angle_deg", 50.0));
    max_speed_scale_in_caution_ = declare_parameter<double>("max_speed_scale_in_caution", 0.6);
    max_speed_scale_in_slow_ = declare_parameter<double>("max_speed_scale_in_slow", 0.25);
    emergency_stop_ = declare_parameter<bool>("emergency_stop", true);
    min_points_for_obstacle_ = declare_parameter<int>("min_points_for_obstacle", 5);
    min_points_for_emergency_ = declare_parameter<int>("min_points_for_emergency", 3);
    enable_front_limit_ = declare_parameter<bool>("enable_front_limit", true);
    enable_back_limit_ = declare_parameter<bool>("enable_back_limit", true);
    enable_left_limit_ = declare_parameter<bool>("enable_left_limit", true);
    enable_right_limit_ = declare_parameter<bool>("enable_right_limit", true);
    pass_through_when_no_cloud_ = declare_parameter<bool>("pass_through_when_no_cloud", true);

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 1e-6) {
      publish_rate_hz_ = 20.0;
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        input_cmd_vel_topic_, 10,
        std::bind(&SecondLidarSafetyLimiterNode::OnCmdVel, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacle_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SecondLidarSafetyLimiterNode::OnCloud, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        output_cmd_vel_topic_, rclcpp::SensorDataQoS());
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&SecondLidarSafetyLimiterNode::OnTimer, this));

    latest_cmd_time_ = now();
    last_cloud_receive_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    RCLCPP_INFO(
        get_logger(),
        "Second lidar safety limiter ready: %s -> %s, obstacle=%s",
        input_cmd_vel_topic_.c_str(),
        output_cmd_vel_topic_.c_str(),
        obstacle_topic_.c_str());
  }

private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_cmd_ = *msg;
    latest_cmd_time_ = now();
    has_cmd_ = true;
  }

  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_cloud_receive_time_ = now();
    last_cloud_frame_ = msg->header.frame_id;

    RegionStats stats;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double distance = std::hypot(x, y);
      const double angle = std::atan2(y, x);

      if (enable_front_limit_ && x > 0.0 && std::abs(angle) <= front_angle_rad_) {
        stats.front_pts += 1;
        stats.front_min = std::min(stats.front_min, distance);
      }
      if (enable_back_limit_ && x < 0.0 && NearBack(angle)) {
        stats.back_pts += 1;
        stats.back_min = std::min(stats.back_min, distance);
      }
      if (enable_left_limit_ && y > 0.0 && NearSide(angle)) {
        stats.left_pts += 1;
        stats.left_min = std::min(stats.left_min, distance);
      }
      if (enable_right_limit_ && y < 0.0 && NearSide(angle)) {
        stats.right_pts += 1;
        stats.right_min = std::min(stats.right_min, distance);
      }
    }

    stats_ = stats;
  }

  void OnTimer() {
    const auto now_time = now();
    last_scales_ = ComputeScales();

    geometry_msgs::msg::Twist cmd_in;
    if (has_cmd_ && AgeSec(now_time, latest_cmd_time_) <= cmd_vel_timeout_sec_) {
      cmd_in = latest_cmd_;
    }

    const bool obstacle_ready = HasFreshObstacle(now_time);
    const bool obstacle_timeout = !obstacle_ready;

    geometry_msgs::msg::Twist cmd_out;
    if (!has_cmd_ || AgeSec(now_time, latest_cmd_time_) > cmd_vel_timeout_sec_) {
      cmd_out = geometry_msgs::msg::Twist();
    } else if (!obstacle_ready) {
      cmd_out = pass_through_when_no_cloud_ ? cmd_in : geometry_msgs::msg::Twist();
    } else {
      cmd_out = ApplyLimits(cmd_in, last_scales_);
    }

    cmd_pub_->publish(cmd_out);
    PublishDebug(cmd_in, cmd_out, obstacle_timeout);
  }

  geometry_msgs::msg::Twist ApplyLimits(
      const geometry_msgs::msg::Twist &cmd, const ScaleResult &scales) {
    geometry_msgs::msg::Twist out;
    out.angular.z = cmd.angular.z;

    if (scales.emergency && emergency_stop_) {
      out.linear.x = 0.0;
      out.linear.y = 0.0;
      return out;
    }

    if (cmd.linear.x > 0.0) {
      out.linear.x = cmd.linear.x * scales.scale_vx_pos;
    } else if (cmd.linear.x < 0.0) {
      out.linear.x = cmd.linear.x * scales.scale_vx_neg;
    } else {
      out.linear.x = 0.0;
    }

    if (cmd.linear.y > 0.0) {
      out.linear.y = cmd.linear.y * scales.scale_vy_pos;
    } else if (cmd.linear.y < 0.0) {
      out.linear.y = cmd.linear.y * scales.scale_vy_neg;
    } else {
      out.linear.y = 0.0;
    }

    return out;
  }

  ScaleResult ComputeScales() const {
    ScaleResult result;

    result = ApplyRegionScale("front", stats_.front_min, stats_.front_pts, result);
    result = ApplyRegionScale("back", stats_.back_min, stats_.back_pts, result);
    result = ApplyRegionScale("left", stats_.left_min, stats_.left_pts, result);
    result = ApplyRegionScale("right", stats_.right_min, stats_.right_pts, result);

    result.scale_vx_pos = ClampScale(result.scale_vx_pos);
    result.scale_vx_neg = ClampScale(result.scale_vx_neg);
    result.scale_vy_pos = ClampScale(result.scale_vy_pos);
    result.scale_vy_neg = ClampScale(result.scale_vy_neg);

    return result;
  }

  ScaleResult ApplyRegionScale(
      const std::string &name,
      double min_dist,
      int count,
      ScaleResult result) const {
    if (count >= min_points_for_emergency_ && min_dist <= emergency_distance_) {
      result.emergency = true;
      SetScaleForRegion(name, 0.0, result);
      return result;
    }

    if (count >= min_points_for_obstacle_) {
      if (min_dist <= slow_distance_) {
        SetScaleForRegion(name, std::min(GetScaleForRegion(name, result), max_speed_scale_in_slow_), result);
      } else if (min_dist <= caution_distance_) {
        SetScaleForRegion(name, std::min(GetScaleForRegion(name, result), max_speed_scale_in_caution_), result);
      }
    }

    return result;
  }

  static double ClampScale(double value) {
    if (value < 0.0) {
      return 0.0;
    }
    if (value > 1.0) {
      return 1.0;
    }
    return value;
  }

  static double GetScaleForRegion(const std::string &name, const ScaleResult &result) {
    if (name == "front") {
      return result.scale_vx_pos;
    }
    if (name == "back") {
      return result.scale_vx_neg;
    }
    if (name == "left") {
      return result.scale_vy_pos;
    }
    return result.scale_vy_neg;
  }

  static void SetScaleForRegion(const std::string &name, double value, ScaleResult &result) {
    if (name == "front") {
      result.scale_vx_pos = value;
    } else if (name == "back") {
      result.scale_vx_neg = value;
    } else if (name == "left") {
      result.scale_vy_pos = value;
    } else {
      result.scale_vy_neg = value;
    }
  }

  bool HasFreshObstacle(const rclcpp::Time &now_time) const {
    if (last_cloud_receive_time_.nanoseconds() == 0) {
      return false;
    }
    return AgeSec(now_time, last_cloud_receive_time_) <= obstacle_timeout_sec_;
  }

  void PublishDebug(
      const geometry_msgs::msg::Twist &cmd_in,
      const geometry_msgs::msg::Twist &cmd_out,
      bool obstacle_timeout) {
    const double cloud_age = (last_cloud_receive_time_.nanoseconds() == 0)
                                 ? std::numeric_limits<double>::infinity()
                                 : AgeSec(now(), last_cloud_receive_time_);
    const std::string cloud_age_text = std::isfinite(cloud_age)
                                           ? FormatScale(cloud_age)
                                           : "inf";

    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "second_lidar_safety: "
       << "cloud_age=" << cloud_age_text
       << " obstacle_timeout=" << (obstacle_timeout ? "true" : "false") << " "
       << "front_min=" << FormatDistance(stats_.front_min)
       << " front_pts=" << stats_.front_pts << " "
       << "back_min=" << FormatDistance(stats_.back_min)
       << " back_pts=" << stats_.back_pts << " "
       << "left_min=" << FormatDistance(stats_.left_min)
       << " left_pts=" << stats_.left_pts << " "
       << "right_min=" << FormatDistance(stats_.right_min)
       << " right_pts=" << stats_.right_pts << " "
       << "scale_vx_pos=" << FormatScale(last_scales_.scale_vx_pos) << " "
       << "scale_vx_neg=" << FormatScale(last_scales_.scale_vx_neg) << " "
       << "scale_vy_pos=" << FormatScale(last_scales_.scale_vy_pos) << " "
       << "scale_vy_neg=" << FormatScale(last_scales_.scale_vy_neg) << " "
       << std::fixed << std::setprecision(2)
       << "cmd_in=(" << cmd_in.linear.x << "," << cmd_in.linear.y << "," << cmd_in.angular.z << ") "
       << "cmd_out=(" << cmd_out.linear.x << "," << cmd_out.linear.y << "," << cmd_out.angular.z << ")";

    msg.data = ss.str();
    debug_pub_->publish(msg);
  }

  static double DegreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
  }

  bool NearBack(double angle) const {
    return std::abs(M_PI - std::abs(angle)) <= rear_angle_rad_;
  }

  bool NearSide(double angle) const {
    return std::abs(std::abs(angle) - M_PI / 2.0) <= side_angle_rad_;
  }

  static double AgeSec(const rclcpp::Time &now_time, const rclcpp::Time &stamp) {
    return std::max(0.0, (now_time - stamp).seconds());
  }

  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string obstacle_topic_;
  std::string debug_topic_;
  std::string frame_id_;

  double publish_rate_hz_ = 20.0;
  double cmd_vel_timeout_sec_ = 0.25;
  double obstacle_timeout_sec_ = 0.25;
  double emergency_distance_ = 0.35;
  double slow_distance_ = 0.70;
  double caution_distance_ = 1.20;
  double front_angle_rad_ = DegreesToRadians(50.0);
  double side_angle_rad_ = DegreesToRadians(70.0);
  double rear_angle_rad_ = DegreesToRadians(50.0);
  double max_speed_scale_in_caution_ = 0.6;
  double max_speed_scale_in_slow_ = 0.25;
  bool emergency_stop_ = true;
  int min_points_for_obstacle_ = 5;
  int min_points_for_emergency_ = 3;
  bool enable_front_limit_ = true;
  bool enable_back_limit_ = true;
  bool enable_left_limit_ = true;
  bool enable_right_limit_ = true;
  bool pass_through_when_no_cloud_ = true;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time latest_cmd_time_;
  bool has_cmd_ = false;

  rclcpp::Time last_cloud_receive_time_;
  std::string last_cloud_frame_;
  RegionStats stats_;
  ScaleResult last_scales_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SecondLidarSafetyLimiterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
