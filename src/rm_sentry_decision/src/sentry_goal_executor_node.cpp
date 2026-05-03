#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rm_interfaces/msg/sentry_intent.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
struct GoalPose
{
    std::string name{};
    std::uint8_t id{0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}
}  // namespace

class SentryGoalExecutorNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    SentryGoalExecutorNode() : Node("sentry_goal_executor")
    {
        intent_topic_ = declare_parameter<std::string>("intent_topic", "/sentry/intent");
        debug_topic_ = declare_parameter<std::string>("debug_topic", "/sentry/nav_goal_debug");
        goals_file_ = declare_parameter<std::string>("goals_file", "");
        goal_frame_id_ = declare_parameter<std::string>("goal_frame_id", "map");
        action_name_ = declare_parameter<std::string>("navigate_action_name", "navigate_to_pose");
        min_goal_interval_sec_ = declare_parameter<double>("min_goal_interval_sec", 2.0);

        LoadGoals(goals_file_);

        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
        intent_sub_ = create_subscription<rm_interfaces::msg::SentryIntent>(
            intent_topic_, rclcpp::SensorDataQoS(),
            [this](rm_interfaces::msg::SentryIntent::SharedPtr msg) {
                OnIntent(*msg);
            });
        debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, rclcpp::QoS(10));

        RCLCPP_INFO(
            get_logger(),
            "sentry_goal_executor: subscribing %s, sending Nav2 action %s, debug=%s",
            intent_topic_.c_str(), action_name_.c_str(), debug_topic_.c_str());
    }

private:
    void LoadGoals(const std::string& goals_file)
    {
        if (goals_file.empty())
        {
            RCLCPP_WARN(get_logger(), "goals_file is empty; goal executor will only debug misses");
            return;
        }

        const YAML::Node root = YAML::LoadFile(goals_file);
        const YAML::Node goals = root["goals"];
        if (!goals)
        {
            RCLCPP_WARN(get_logger(), "No 'goals' map in %s", goals_file.c_str());
            return;
        }

        for (const auto& item : goals)
        {
            const std::string name = item.first.as<std::string>();
            const YAML::Node node = item.second;
            GoalPose pose;
            pose.name = name;
            pose.id = static_cast<std::uint8_t>(node["id"].as<int>());
            pose.x = node["x"].as<double>();
            pose.y = node["y"].as<double>();
            pose.yaw = node["yaw"].as<double>();
            goals_by_id_[pose.id] = pose;
        }

        RCLCPP_INFO(
            get_logger(), "Loaded %zu sentry goals from %s",
            goals_by_id_.size(), goals_file.c_str());
    }

    void OnIntent(const rm_interfaces::msg::SentryIntent& intent)
    {
        if (intent.goal_id == 0)
        {
            PublishDebug(intent.goal_id, "NONE", 0.0, 0.0, 0.0, false, "goal_id=0");
            return;
        }

        const auto goal_it = goals_by_id_.find(intent.goal_id);
        if (goal_it == goals_by_id_.end())
        {
            PublishDebug(
                intent.goal_id, "UNKNOWN", 0.0, 0.0, 0.0, false,
                "goal_id not present in sentry_goals.yaml");
            return;
        }

        const auto now_time = now();
        if (last_goal_id_ == intent.goal_id)
        {
            PublishDebug(
                intent.goal_id, goal_it->second.name, goal_it->second.x, goal_it->second.y,
                goal_it->second.yaw, false, "same goal_id, not re-sending");
            return;
        }

        if (last_send_time_.nanoseconds() != 0 &&
            (now_time - last_send_time_).seconds() < min_goal_interval_sec_)
        {
            PublishDebug(
                intent.goal_id, goal_it->second.name, goal_it->second.x, goal_it->second.y,
                goal_it->second.yaw, false, "min_goal_interval_sec active");
            return;
        }

        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Nav2 NavigateToPose action server '%s' is not available",
                action_name_.c_str());
            PublishDebug(
                intent.goal_id, goal_it->second.name, goal_it->second.x, goal_it->second.y,
                goal_it->second.yaw, false, "Nav2 action server unavailable");
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose.header.stamp = now_time;
        goal.pose.header.frame_id = goal_frame_id_;
        goal.pose.pose.position.x = goal_it->second.x;
        goal.pose.pose.position.y = goal_it->second.y;
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation = QuaternionFromYaw(goal_it->second.yaw);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        action_client_->async_send_goal(goal, options);
        last_goal_id_ = intent.goal_id;
        last_send_time_ = now_time;
        PublishDebug(
            intent.goal_id, goal_it->second.name, goal_it->second.x, goal_it->second.y,
            goal_it->second.yaw, true, intent.reason);
    }

    void PublishDebug(std::uint8_t goal_id, const std::string& goal_name, double x, double y,
                      double yaw, bool sent_new_goal, const std::string& reason)
    {
        std_msgs::msg::String debug;
        std::ostringstream oss;
        oss << "goal_id=" << static_cast<int>(goal_id)
            << " goal_name=" << goal_name
            << " x=" << x
            << " y=" << y
            << " yaw=" << yaw
            << " sent_new_goal=" << (sent_new_goal ? "true" : "false")
            << " reason=" << reason;
        debug.data = oss.str();
        debug_pub_->publish(debug);
    }

    std::string intent_topic_{};
    std::string debug_topic_{};
    std::string goals_file_{};
    std::string goal_frame_id_{"map"};
    std::string action_name_{"navigate_to_pose"};
    double min_goal_interval_sec_{2.0};
    std::unordered_map<std::uint8_t, GoalPose> goals_by_id_{};
    std::uint8_t last_goal_id_{0};
    rclcpp::Time last_send_time_{0, 0, get_clock()->get_clock_type()};

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_{};
    rclcpp::Subscription<rm_interfaces::msg::SentryIntent>::SharedPtr intent_sub_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<SentryGoalExecutorNode>());
    }
    catch (const std::exception& ex)
    {
        std::cerr << "sentry_goal_executor failed: " << ex.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
