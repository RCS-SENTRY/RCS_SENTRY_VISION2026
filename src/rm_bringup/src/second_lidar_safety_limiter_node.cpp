#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
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

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

double degToRad(double degrees)
{
  return degrees * kPi / 180.0;
}

struct DirectionStats
{
  double min_distance{kInf};
  int points{0};
  bool enabled{true};
};
}  // namespace

class SecondLidarSafetyLimiter : public rclcpp::Node
{
public:
  SecondLidarSafetyLimiter()
  : Node("second_lidar_safety_limiter")
  {
    input_cmd_vel_topic_ = declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
    output_cmd_vel_topic_ = declare_parameter<std::string>("output_cmd_vel_topic", "/cmd_vel_safe");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/second_lidar_obstacle_cloud");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/second_lidar_safety_debug");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.25);
    obstacle_timeout_sec_ = declare_parameter<double>("obstacle_timeout_sec", 0.25);

    emergency_distance_ = declare_parameter<double>("emergency_distance", 0.35);
    slow_distance_ = declare_parameter<double>("slow_distance", 0.50);
    caution_distance_ = declare_parameter<double>("caution_distance", 0.85);

    front_angle_rad_ = degToRad(declare_parameter<double>("front_angle_deg", 50.0));
    side_angle_rad_ = degToRad(declare_parameter<double>("side_angle_deg", 70.0));
    rear_angle_rad_ = degToRad(declare_parameter<double>("rear_angle_deg", 50.0));

    max_speed_scale_in_caution_ =
      declare_parameter<double>("max_speed_scale_in_caution", 0.85);
    max_speed_scale_in_slow_ = declare_parameter<double>("max_speed_scale_in_slow", 0.5);

    emergency_stop_ = declare_parameter<bool>("emergency_stop", true);
    min_points_for_obstacle_ = declare_parameter<int>("min_points_for_obstacle", 8);
    min_points_for_emergency_ = declare_parameter<int>("min_points_for_emergency", 4);

    front_.enabled = declare_parameter<bool>("enable_front_limit", true);
    back_.enabled = declare_parameter<bool>("enable_back_limit", true);
    left_.enabled = declare_parameter<bool>("enable_left_limit", true);
    right_.enabled = declare_parameter<bool>("enable_right_limit", true);

    pass_through_when_no_cloud_ = declare_parameter<bool>("pass_through_when_no_cloud", true);

    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    min_points_for_obstacle_ = std::max(1, min_points_for_obstacle_);
    min_points_for_emergency_ = std::max(1, min_points_for_emergency_);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_vel_topic_, 10,
      std::bind(&SecondLidarSafetyLimiter::onCmdVel, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SecondLidarSafetyLimiter::onCloud, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);

    latest_cmd_time_ = now();
    last_cloud_receive_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&SecondLidarSafetyLimiter::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "Second lidar safety limiter ready: %s + %s -> %s",
      input_cmd_vel_topic_.c_str(), obstacle_topic_.c_str(), output_cmd_vel_topic_.c_str());
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    latest_cmd_time_ = now();
    has_cmd_ = true;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    resetStats();
    last_cloud_receive_time_ = now();
    has_cloud_ = true;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        const double x = static_cast<double>(*iter_x);
        const double y = static_cast<double>(*iter_y);
        if (!std::isfinite(x) || !std::isfinite(y)) {
          continue;
        }
        const double distance = std::hypot(x, y);
        if (distance <= 1e-6) {
          continue;
        }
        const double angle = std::atan2(y, x);
        classifyPoint(angle, distance);
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to parse obstacle cloud: %s", ex.what());
    }
  }

  void onTimer()
  {
    const auto current_time = now();
    const double cmd_age = has_cmd_ ? (current_time - latest_cmd_time_).seconds() : kInf;
    const double cloud_age = has_cloud_ ? (current_time - last_cloud_receive_time_).seconds() : kInf;
    const bool cmd_timeout = !has_cmd_ || cmd_age > cmd_vel_timeout_sec_;
    const bool obstacle_timeout = !has_cloud_ || cloud_age > obstacle_timeout_sec_;

    geometry_msgs::msg::Twist output;
    if (cmd_timeout) {
      publish(output, cloud_age, obstacle_timeout);
      return;
    }

    output = latest_cmd_;
    if (obstacle_timeout) {
      if (!pass_through_when_no_cloud_) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
      }
      publish(output, cloud_age, true);
      return;
    }

    applyLimits(output);
    publish(output, cloud_age, false);
  }

  void resetStats()
  {
    front_.min_distance = kInf;
    front_.points = 0;
    back_.min_distance = kInf;
    back_.points = 0;
    left_.min_distance = kInf;
    left_.points = 0;
    right_.min_distance = kInf;
    right_.points = 0;
  }

  void updateStats(DirectionStats & stats, double distance)
  {
    if (!stats.enabled) {
      return;
    }
    stats.points += 1;
    stats.min_distance = std::min(stats.min_distance, distance);
  }

  void classifyPoint(double angle, double distance)
  {
    if (std::cos(angle) > 0.0 && std::abs(angle) <= front_angle_rad_) {
      updateStats(front_, distance);
    }
    if (std::cos(angle) < 0.0 && std::abs(kPi - std::abs(angle)) <= rear_angle_rad_) {
      updateStats(back_, distance);
    }
    if (std::sin(angle) > 0.0 && std::abs(angle - kPi / 2.0) <= side_angle_rad_) {
      updateStats(left_, distance);
    }
    if (std::sin(angle) < 0.0 && std::abs(angle + kPi / 2.0) <= side_angle_rad_) {
      updateStats(right_, distance);
    }
  }

  bool isObstacle(const DirectionStats & stats) const
  {
    return stats.enabled && stats.points >= min_points_for_obstacle_;
  }

  bool isEmergency(const DirectionStats & stats) const
  {
    return stats.enabled && stats.points >= min_points_for_emergency_ &&
           stats.min_distance <= emergency_distance_;
  }

  double scaleFor(const DirectionStats & stats) const
  {
    if (!isObstacle(stats)) {
      return 1.0;
    }
    if (stats.min_distance <= emergency_distance_) {
      return 0.0;
    }
    if (stats.min_distance <= slow_distance_) {
      return max_speed_scale_in_slow_;
    }
    if (stats.min_distance <= caution_distance_) {
      return max_speed_scale_in_caution_;
    }
    return 1.0;
  }

  void applyLimits(geometry_msgs::msg::Twist & cmd) const
  {
    const bool front_emergency = cmd.linear.x > 0.0 && isEmergency(front_);
    const bool back_emergency = cmd.linear.x < 0.0 && isEmergency(back_);
    const bool left_emergency = cmd.linear.y > 0.0 && isEmergency(left_);
    const bool right_emergency = cmd.linear.y < 0.0 && isEmergency(right_);
    if (emergency_stop_ &&
      (front_emergency || back_emergency || left_emergency || right_emergency))
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      return;
    }

    if (cmd.linear.x > 0.0) {
      cmd.linear.x *= scaleFor(front_);
    } else if (cmd.linear.x < 0.0) {
      cmd.linear.x *= scaleFor(back_);
    }

    if (cmd.linear.y > 0.0) {
      cmd.linear.y *= scaleFor(left_);
    } else if (cmd.linear.y < 0.0) {
      cmd.linear.y *= scaleFor(right_);
    }
  }

  std::string statsToString(const DirectionStats & stats) const
  {
    if (!std::isfinite(stats.min_distance)) {
      return "inf";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << stats.min_distance;
    return stream.str();
  }

  void publish(
    const geometry_msgs::msg::Twist & output, double cloud_age, bool obstacle_timeout)
  {
    cmd_pub_->publish(output);

    std_msgs::msg::String debug;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "cloud_age=" << cloud_age
           << " obstacle_timeout=" << (obstacle_timeout ? "true" : "false")
           << " front_min=" << statsToString(front_) << " front_pts=" << front_.points
           << " back_min=" << statsToString(back_) << " back_pts=" << back_.points
           << " left_min=" << statsToString(left_) << " left_pts=" << left_.points
           << " right_min=" << statsToString(right_) << " right_pts=" << right_.points
           << " cmd_in=(" << latest_cmd_.linear.x << "," << latest_cmd_.linear.y << ","
           << latest_cmd_.angular.z << ")"
           << " cmd_out=(" << output.linear.x << "," << output.linear.y << ","
           << output.angular.z << ")";
    debug.data = stream.str();
    debug_pub_->publish(debug);
  }

  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string obstacle_topic_;
  std::string debug_topic_;

  double publish_rate_hz_{20.0};
  double cmd_vel_timeout_sec_{0.25};
  double obstacle_timeout_sec_{0.25};
  double emergency_distance_{0.35};
  double slow_distance_{0.50};
  double caution_distance_{0.85};
  double front_angle_rad_{degToRad(50.0)};
  double side_angle_rad_{degToRad(70.0)};
  double rear_angle_rad_{degToRad(50.0)};
  double max_speed_scale_in_caution_{0.85};
  double max_speed_scale_in_slow_{0.5};
  bool emergency_stop_{true};
  int min_points_for_obstacle_{8};
  int min_points_for_emergency_{4};
  bool pass_through_when_no_cloud_{true};

  DirectionStats front_;
  DirectionStats back_;
  DirectionStats left_;
  DirectionStats right_;

  bool has_cmd_{false};
  bool has_cloud_{false};
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time latest_cmd_time_;
  rclcpp::Time last_cloud_receive_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SecondLidarSafetyLimiter>());
  rclcpp::shutdown();
  return 0;
}
