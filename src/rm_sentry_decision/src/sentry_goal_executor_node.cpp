#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rm_interfaces/msg/sentry_intent.hpp>
#include <rm_interfaces/msg/sentry_nav_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace
{
using action_msgs::msg::GoalStatus;

struct GoalPose
{
    std::string name{};
    std::uint8_t id{0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double radius{0.0};
    std::vector<GoalPose> candidates{};
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

const char* BoolString(bool value)
{
    return value ? "true" : "false";
}

bool IsNoTimeoutGoal(std::uint8_t goal_id)
{
    return goal_id == 19;  // BASE_HOME: death-return hard task must not be timed out away.
}

GoalPose PoseFromYaml(const std::string& name, std::uint8_t id, double default_radius,
                      const YAML::Node& node)
{
    GoalPose pose;
    pose.name = name;
    pose.id = id;
    if (node.IsSequence())
    {
        pose.x = node.size() > 0 ? node[0].as<double>() : 0.0;
        pose.y = node.size() > 1 ? node[1].as<double>() : 0.0;
        pose.yaw = node.size() > 2 ? node[2].as<double>() : 0.0;
    }
    else
    {
        pose.x = node["x"].as<double>();
        pose.y = node["y"].as<double>();
        pose.yaw = node["yaw"].as<double>();
        pose.radius = node["radius"] ? node["radius"].as<double>() : default_radius;
    }
    pose.radius = pose.radius > 1e-6 ? pose.radius : default_radius;
    return pose;
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
        nav_status_topic_ =
            declare_parameter<std::string>("nav_status_topic", "/sentry/nav_status");
        goals_file_ = declare_parameter<std::string>("goals_file", "");
        goal_frame_id_ = declare_parameter<std::string>("goal_frame_id", "map");
        action_name_ = declare_parameter<std::string>("navigate_action_name", "navigate_to_pose");
        min_goal_interval_sec_ = declare_parameter<double>("min_goal_interval_sec", 2.0);
        enable_tactical_reach_ = declare_parameter<bool>("enable_tactical_reach", true);
        tactical_reach_radius_ = declare_parameter<double>("tactical_reach_radius", 0.45);
        tactical_reach_hold_sec_ = declare_parameter<double>("tactical_reach_hold_sec", 0.30);
        cancel_nav2_on_tactical_reach_ =
            declare_parameter<bool>("cancel_nav2_on_tactical_reach", true);
        goal_active_timeout_sec_ = declare_parameter<double>("goal_active_timeout_sec", 10.0);
        cancel_nav2_on_active_timeout_ =
            declare_parameter<bool>("cancel_nav2_on_active_timeout", true);
        robot_base_frame_ =
            declare_parameter<std::string>("robot_base_frame", "gimbal_yaw_fake");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);

        LoadGoals(goals_file_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

        intent_sub_ = create_subscription<rm_interfaces::msg::SentryIntent>(
            intent_topic_, rclcpp::SensorDataQoS(),
            [this](rm_interfaces::msg::SentryIntent::SharedPtr msg) {
                OnIntent(*msg);
            });
        debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, rclcpp::QoS(10));
        nav_status_pub_ = create_publisher<rm_interfaces::msg::SentryNavStatus>(
            nav_status_topic_, rclcpp::QoS(10));
        status_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { UpdateTacticalReachAndPublish(); });

        RCLCPP_INFO(
            get_logger(),
            "sentry_goal_executor: intent=%s, nav_status=%s, action=%s, tf=%s->%s",
            intent_topic_.c_str(), nav_status_topic_.c_str(), action_name_.c_str(),
            map_frame_.c_str(), robot_base_frame_.c_str());
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
            const auto id = static_cast<std::uint8_t>(node["id"].as<int>());
            pose = PoseFromYaml(name, id, tactical_reach_radius_, node);
            if (node["candidates"] && node["candidates"].IsSequence())
            {
                for (const auto& candidate_node : node["candidates"])
                {
                    auto candidate =
                        PoseFromYaml(name, id, pose.radius, candidate_node);
                    pose.candidates.push_back(candidate);
                }
            }
            if (pose.candidates.empty())
            {
                pose.candidates.push_back(pose);
            }
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
            const bool had_goal = active_goal_id_ != 0 || last_goal_id_ != 0;
            const bool was_active = active_;
            const bool was_reached = reached_;
            if (active_ && current_goal_handle_)
            {
                nav2_status_ = GoalStatus::STATUS_CANCELING;
                action_client_->async_cancel_goal(current_goal_handle_);
            }
            active_ = false;
            reached_ = had_goal && was_reached;
            failed_ = false;
            canceled_by_tactical_reach_ = had_goal;
            tactical_cancel_requested_ = false;
            active_timeout_cancel_requested_ = false;
            if (!had_goal || (was_active && !was_reached))
            {
                active_goal_ = GoalPose{};
                active_goal_id_ = 0;
                last_goal_id_ = 0;
            }
            reach_enter_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            const std::string stop_reason =
                !had_goal
                    ? "goal_id=0; no Nav2 goal sent"
                    : (was_reached
                           ? "goal_id=0; tactical stop after reached, keep dwell context"
                           : "goal_id=0; tactical stop before reached, clear goal for resume");
            PublishStatus(stop_reason);
            PublishDebug(false, stop_reason);
            return;
        }

        auto effective_goal_id = intent.goal_id;
        auto goal_it = goals_by_id_.find(effective_goal_id);
        std::string fallback_reason;
        if (goal_it == goals_by_id_.end() && intent.goal_id == 19)
        {
            for (const auto fallback_id : {static_cast<std::uint8_t>(5),
                                           static_cast<std::uint8_t>(6)})
            {
                const auto fallback_it = goals_by_id_.find(fallback_id);
                if (fallback_it != goals_by_id_.end())
                {
                    effective_goal_id = fallback_id;
                    goal_it = fallback_it;
                    fallback_reason =
                        "BASE_HOME missing in sentry_goals.yaml; fallback to supply goal";
                    break;
                }
            }
        }
        if (goal_it == goals_by_id_.end())
        {
            failed_ = true;
            active_ = false;
            active_goal_id_ = effective_goal_id;
            active_goal_ = GoalPose{"UNKNOWN", effective_goal_id, 0.0, 0.0, 0.0};
            nav2_status_ = GoalStatus::STATUS_UNKNOWN;
            PublishStatus("goal_id not present in sentry_goals.yaml");
            PublishDebug(false, "goal_id not present in sentry_goals.yaml");
            return;
        }

        const auto now_time = now();
        if (last_goal_id_ == effective_goal_id && !(failed_ && IsNoTimeoutGoal(effective_goal_id)))
        {
            PublishStatus(active_ ? "same goal_id still active" : "same goal_id already handled");
            PublishDebug(false, active_ ? "same goal_id still active" : "same goal_id already handled");
            return;
        }

        if (last_send_time_.nanoseconds() != 0 &&
            (now_time - last_send_time_).seconds() < min_goal_interval_sec_)
        {
            PublishStatus("min_goal_interval_sec active; not sending new goal");
            PublishDebug(false, "min_goal_interval_sec active; not sending new goal");
            return;
        }

        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Nav2 NavigateToPose action server '%s' is not available",
                action_name_.c_str());
            active_goal_ = goal_it->second;
            active_goal_id_ = effective_goal_id;
            active_ = false;
            reached_ = false;
            failed_ = true;
            canceled_by_tactical_reach_ = false;
            nav2_status_ = GoalStatus::STATUS_UNKNOWN;
            PublishStatus("Nav2 action server unavailable");
            PublishDebug(false, "Nav2 action server unavailable");
            return;
        }

        if (active_ && current_goal_handle_)
        {
            action_client_->async_cancel_goal(current_goal_handle_);
        }

        GoalPose selected_goal = SelectGoalCandidate(goal_it->second);

        NavigateToPose::Goal goal;
        goal.pose.header.stamp = now_time;
        goal.pose.header.frame_id = goal_frame_id_;
        goal.pose.pose.position.x = selected_goal.x;
        goal.pose.pose.position.y = selected_goal.y;
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation = QuaternionFromYaw(selected_goal.yaw);

        active_goal_ = selected_goal;
        active_goal_.name = goal_it->second.name;
        active_goal_.id = goal_it->second.id;
        active_goal_.radius = goal_it->second.radius;
        active_goal_id_ = effective_goal_id;
        active_ = true;
        reached_ = false;
        failed_ = false;
        canceled_by_tactical_reach_ = false;
        tactical_cancel_requested_ = false;
        active_timeout_cancel_requested_ = false;
        nav2_status_ = GoalStatus::STATUS_ACCEPTED;
        reach_enter_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        goal_start_time_ = now_time;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback =
            [this](GoalHandleNavigateToPose::SharedPtr handle) {
                if (!handle)
                {
                    active_ = false;
                    failed_ = true;
                    nav2_status_ = GoalStatus::STATUS_UNKNOWN;
                    PublishStatus("Nav2 rejected NavigateToPose goal");
                    PublishDebug(false, "Nav2 rejected NavigateToPose goal");
                    return;
                }
                current_goal_handle_ = handle;
                nav2_status_ = GoalStatus::STATUS_EXECUTING;
                PublishStatus("Nav2 accepted NavigateToPose goal");
                PublishDebug(true, "Nav2 accepted NavigateToPose goal");
            };
        options.feedback_callback =
            [this](GoalHandleNavigateToPose::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback>) {
                nav2_status_ = GoalStatus::STATUS_EXECUTING;
            };
        options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult& result) {
                OnActionResult(result);
            };

        action_client_->async_send_goal(goal, options);
        last_goal_id_ = effective_goal_id;
        last_send_time_ = now_time;
        const auto send_reason =
            fallback_reason.empty()
                ? (intent.reason.empty() ? std::string("sent new NavigateToPose goal")
                                         : intent.reason)
                : (fallback_reason + "; " +
                   (intent.reason.empty() ? std::string("sent new NavigateToPose goal")
                                          : intent.reason));
        PublishStatus(send_reason);
        PublishDebug(true, send_reason);
    }

    void OnActionResult(const GoalHandleNavigateToPose::WrappedResult& result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                active_ = false;
                reached_ = true;
                failed_ = false;
                nav2_status_ = GoalStatus::STATUS_SUCCEEDED;
                PublishStatus("Nav2 result SUCCEEDED");
                PublishDebug(false, "Nav2 result SUCCEEDED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                active_ = false;
                nav2_status_ = GoalStatus::STATUS_CANCELED;
                if (tactical_cancel_requested_ || canceled_by_tactical_reach_)
                {
                    reached_ = true;
                    failed_ = false;
                    canceled_by_tactical_reach_ = true;
                    PublishStatus("Nav2 result CANCELED after tactical reach");
                    PublishDebug(false, "Nav2 result CANCELED after tactical reach");
                }
                else
                {
                    reached_ = false;
                    failed_ = true;
                    PublishStatus("Nav2 result CANCELED externally");
                    PublishDebug(false, "Nav2 result CANCELED externally");
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
            default:
                active_ = false;
                reached_ = false;
                failed_ = true;
                nav2_status_ = GoalStatus::STATUS_ABORTED;
                PublishStatus("Nav2 result ABORTED");
                PublishDebug(false, "Nav2 result ABORTED");
                break;
        }
    }

    GoalPose SelectGoalCandidate(const GoalPose& goal)
    {
        if (goal.candidates.empty())
        {
            return goal;
        }

        std::string reason;
        const bool pose_ok = UpdateRobotPose(reason);
        auto best = goal.candidates.front();
        double best_score = std::numeric_limits<double>::infinity();
        for (const auto& candidate : goal.candidates)
        {
            const double score = pose_ok
                ? std::hypot(robot_x_ - candidate.x, robot_y_ - candidate.y)
                : std::hypot(goal.x - candidate.x, goal.y - candidate.y);
            if (score < best_score)
            {
                best = candidate;
                best_score = score;
            }
        }
        return best;
    }

    bool UpdateRobotPose(std::string& reason)
    {
        try
        {
            const auto transform = tf_buffer_->lookupTransform(
                map_frame_, robot_base_frame_, tf2::TimePointZero,
                tf2::durationFromSec(tf_timeout_sec_));
            robot_x_ = transform.transform.translation.x;
            robot_y_ = transform.transform.translation.y;
            robot_yaw_ = static_cast<float>(tf2::getYaw(transform.transform.rotation));
            has_robot_pose_ = true;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            reason = std::string("tf unavailable: ") + ex.what();
            has_robot_pose_ = false;
            return false;
        }
    }

    void UpdateTacticalReachAndPublish()
    {
        std::string reason = "timer";
        if (active_goal_id_ != 0)
        {
            if (UpdateRobotPose(reason))
            {
                distance_to_goal_ = static_cast<float>(std::hypot(
                    robot_x_ - active_goal_.x, robot_y_ - active_goal_.y));
                if (active_ && enable_tactical_reach_)
                {
                    const auto now_time = now();
                    const bool goal_timeout =
                        goal_active_timeout_sec_ > 0.0 &&
                        !IsNoTimeoutGoal(active_goal_id_) &&
                        goal_start_time_.nanoseconds() != 0 &&
                        (now_time - goal_start_time_).seconds() >= goal_active_timeout_sec_;
                    const double reach_radius =
                        active_goal_.radius > 1e-6 ? active_goal_.radius : tactical_reach_radius_;
                    if (distance_to_goal_ <= reach_radius)
                    {
                        if (reach_enter_time_.nanoseconds() == 0)
                        {
                            reach_enter_time_ = now_time;
                            reason = "inside goal/candidate reach radius";
                        }
                        else if ((now_time - reach_enter_time_).seconds() >=
                                 tactical_reach_hold_sec_)
                        {
                            active_ = false;
                            reached_ = true;
                            failed_ = false;
                            canceled_by_tactical_reach_ = true;
                            reason = "tactical reach hold satisfied";
                            if (cancel_nav2_on_tactical_reach_ && current_goal_handle_ &&
                                !tactical_cancel_requested_)
                            {
                                tactical_cancel_requested_ = true;
                                nav2_status_ = GoalStatus::STATUS_CANCELING;
                                action_client_->async_cancel_goal(current_goal_handle_);
                                reason += "; canceling Nav2 goal";
                            }
                        }
                        else
                        {
                            reason = "holding inside goal/candidate reach radius";
                        }
                    }
                    else if (goal_timeout)
                    {
                        active_ = false;
                        reached_ = true;
                        failed_ = true;
                        canceled_by_tactical_reach_ = true;
                        nav2_status_ = GoalStatus::STATUS_CANCELING;
                        reason =
                            "goal active timeout; treating current pose as tactical hold, "
                            "then switching fallback after dwell";
                        if (cancel_nav2_on_active_timeout_ && current_goal_handle_ &&
                            !active_timeout_cancel_requested_)
                        {
                            active_timeout_cancel_requested_ = true;
                            action_client_->async_cancel_goal(current_goal_handle_);
                            reason += "; canceling Nav2 goal";
                        }
                        PublishDebug(false, reason);
                    }
                    else
                    {
                        reach_enter_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
                        reason = "outside tactical reach radius";
                    }
                }
            }
            else if (active_)
            {
                reach_enter_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            }
        }
        else
        {
            distance_to_goal_ = 0.0f;
            reason = "no active goal";
        }

        PublishStatus(reason);
    }

    void PublishStatus(const std::string& reason)
    {
        rm_interfaces::msg::SentryNavStatus status;
        status.header.stamp = now();
        status.header.frame_id = map_frame_;
        status.goal_id = active_goal_id_;
        status.goal_name = active_goal_.name;
        status.active = active_;
        status.reached = reached_;
        status.failed = failed_;
        status.canceled_by_tactical_reach = canceled_by_tactical_reach_;
        status.target_x = static_cast<float>(active_goal_.x);
        status.target_y = static_cast<float>(active_goal_.y);
        status.target_yaw = static_cast<float>(active_goal_.yaw);
        status.robot_x = static_cast<float>(robot_x_);
        status.robot_y = static_cast<float>(robot_y_);
        status.robot_yaw = robot_yaw_;
        status.distance_to_goal = distance_to_goal_;
        status.reach_radius = static_cast<float>(
            active_goal_.radius > 1e-6 ? active_goal_.radius : tactical_reach_radius_);
        status.nav2_status = nav2_status_;
        status.reason = reason;
        nav_status_pub_->publish(status);
        last_status_reason_ = reason;
    }

    void PublishDebug(bool sent_new_goal, const std::string& reason)
    {
        std_msgs::msg::String debug;
        std::ostringstream oss;
        oss << "goal_id=" << static_cast<int>(active_goal_id_)
            << " goal_name=" << active_goal_.name
            << " active=" << BoolString(active_)
            << " reached=" << BoolString(reached_)
            << " failed=" << BoolString(failed_)
            << " canceled_by_tactical_reach=" << BoolString(canceled_by_tactical_reach_)
            << " x=" << active_goal_.x
            << " y=" << active_goal_.y
            << " yaw=" << active_goal_.yaw
            << " robot_x=" << robot_x_
            << " robot_y=" << robot_y_
            << " robot_yaw=" << robot_yaw_
            << " distance_to_goal=" << distance_to_goal_
            << " nav2_status=" << static_cast<int>(nav2_status_)
            << " sent_new_goal=" << BoolString(sent_new_goal)
            << " reason=" << reason;
        debug.data = oss.str();
        debug_pub_->publish(debug);
    }

    std::string intent_topic_{};
    std::string debug_topic_{};
    std::string nav_status_topic_{};
    std::string goals_file_{};
    std::string goal_frame_id_{"map"};
    std::string action_name_{"navigate_to_pose"};
    std::string robot_base_frame_{"gimbal_yaw_fake"};
    std::string map_frame_{"map"};
    double min_goal_interval_sec_{2.0};
    bool enable_tactical_reach_{true};
    double tactical_reach_radius_{0.45};
    double tactical_reach_hold_sec_{0.30};
    bool cancel_nav2_on_tactical_reach_{true};
    double goal_active_timeout_sec_{10.0};
    bool cancel_nav2_on_active_timeout_{true};
    double tf_timeout_sec_{0.05};

    std::unordered_map<std::uint8_t, GoalPose> goals_by_id_{};
    GoalPose active_goal_{};
    std::uint8_t active_goal_id_{0};
    std::uint8_t last_goal_id_{0};
    bool active_{false};
    bool reached_{false};
    bool failed_{false};
    bool canceled_by_tactical_reach_{false};
    bool tactical_cancel_requested_{false};
    bool active_timeout_cancel_requested_{false};
    bool has_robot_pose_{false};
    double robot_x_{0.0};
    double robot_y_{0.0};
    float robot_yaw_{0.0f};
    float distance_to_goal_{0.0f};
    std::uint8_t nav2_status_{GoalStatus::STATUS_UNKNOWN};
    std::string last_status_reason_{};
    rclcpp::Time last_send_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time reach_enter_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time goal_start_time_{0, 0, get_clock()->get_clock_type()};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_{};
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_{};
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_{};
    rclcpp::Subscription<rm_interfaces::msg::SentryIntent>::SharedPtr intent_sub_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_{};
    rclcpp::Publisher<rm_interfaces::msg::SentryNavStatus>::SharedPtr nav_status_pub_{};
    rclcpp::TimerBase::SharedPtr status_timer_{};
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
