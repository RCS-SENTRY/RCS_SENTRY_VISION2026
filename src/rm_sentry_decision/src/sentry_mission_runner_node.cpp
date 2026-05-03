#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/autoaim_target_status.hpp>
#include <rm_interfaces/msg/gimbal_status.hpp>
#include <rm_interfaces/msg/sentry_intent.hpp>
#include <rm_interfaces/msg/sentry_nav_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
template <typename T>
void ReadOptional(const YAML::Node& node, const char* key, T& value)
{
    if (node && node[key])
    {
        value = node[key].as<T>();
    }
}

struct MissionStep
{
    std::string name{};
    std::uint8_t goal_id{0};
    std::uint8_t tactical_state{0};
    std::uint8_t posture_intent{0};
    std::uint8_t fire_policy{0};
    std::uint8_t spin_mode{0};
    std::uint8_t supercap_mode{0};
    bool wait_reached{false};
    double timeout_sec{0.0};
    double duration_sec{0.0};
    std::string fallback_step{};
    std::string reason{};
};
}  // namespace

class SentryMissionRunnerNode : public rclcpp::Node
{
public:
    SentryMissionRunnerNode() : Node("sentry_mission_runner")
    {
        intent_topic_ = declare_parameter<std::string>("intent_topic", "/sentry/intent");
        nav_status_topic_ =
            declare_parameter<std::string>("nav_status_topic", "/sentry/nav_status");
        target_status_topic_ =
            declare_parameter<std::string>("target_status_topic", "/autoaim/target_status");
        gimbal_status_topic_ =
            declare_parameter<std::string>("gimbal_status_topic", "/gimbal_status");
        debug_topic_ =
            declare_parameter<std::string>("debug_topic", "/sentry/mission_debug");
        mission_file_ = declare_parameter<std::string>("mission_file", "");
        loop_ = declare_parameter<bool>("loop", false);
        timeout_policy_ = declare_parameter<std::string>("timeout_policy", "continue");
        const double publish_rate_hz =
            std::max(1.0, declare_parameter<double>("publish_rate_hz", 10.0));

        LoadMission(mission_file_);

        intent_pub_ = create_publisher<rm_interfaces::msg::SentryIntent>(
            intent_topic_, rclcpp::SensorDataQoS());
        debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, rclcpp::QoS(10));
        nav_status_sub_ = create_subscription<rm_interfaces::msg::SentryNavStatus>(
            nav_status_topic_, rclcpp::SensorDataQoS(),
            [this](rm_interfaces::msg::SentryNavStatus::SharedPtr msg) {
                latest_nav_status_ = *msg;
                has_nav_status_ = true;
            });
        target_status_sub_ = create_subscription<rm_interfaces::msg::AutoaimTargetStatus>(
            target_status_topic_, rclcpp::SensorDataQoS(),
            [this](rm_interfaces::msg::AutoaimTargetStatus::SharedPtr msg) {
                latest_target_status_ = *msg;
                has_target_status_ = true;
            });
        gimbal_status_sub_ = create_subscription<rm_interfaces::msg::GimbalStatus>(
            gimbal_status_topic_, rclcpp::SensorDataQoS(),
            [this](rm_interfaces::msg::GimbalStatus::SharedPtr msg) {
                latest_gimbal_status_ = *msg;
                has_gimbal_status_ = true;
            });

        step_start_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz)),
            [this]() { Tick(); });

        RCLCPP_INFO(
            get_logger(),
            "sentry_mission_runner: mission=%s steps=%zu loop=%s intent=%s",
            mission_file_.c_str(), steps_.size(), loop_ ? "true" : "false",
            intent_topic_.c_str());
    }

private:
    void LoadMission(const std::string& mission_file)
    {
        if (mission_file.empty())
        {
            RCLCPP_WARN(get_logger(), "mission_file is empty; mission runner will stay idle");
            return;
        }

        const YAML::Node root = YAML::LoadFile(mission_file);
        const YAML::Node mission = root["mission"];
        if (!mission || !mission.IsSequence())
        {
            RCLCPP_WARN(get_logger(), "No mission sequence in %s", mission_file.c_str());
            return;
        }

        for (const auto& node : mission)
        {
            MissionStep step;
            step.name = node["name"] ? node["name"].as<std::string>() : "";
            step.goal_id = static_cast<std::uint8_t>(node["goal_id"].as<int>());
            step.tactical_state = static_cast<std::uint8_t>(node["tactical_state"].as<int>());
            step.posture_intent = static_cast<std::uint8_t>(node["posture_intent"].as<int>());
            step.fire_policy = static_cast<std::uint8_t>(node["fire_policy"].as<int>());
            step.spin_mode = static_cast<std::uint8_t>(node["spin_mode"].as<int>());
            ReadOptional(node, "supercap_mode", step.supercap_mode);
            ReadOptional(node, "wait_reached", step.wait_reached);
            ReadOptional(node, "timeout_sec", step.timeout_sec);
            ReadOptional(node, "duration_sec", step.duration_sec);
            ReadOptional(node, "fallback_step", step.fallback_step);
            ReadOptional(node, "reason", step.reason);
            if (step.name.empty())
            {
                step.name = "step_" + std::to_string(steps_.size());
            }
            step_index_by_name_[step.name] = steps_.size();
            steps_.push_back(step);
        }
    }

    void Tick()
    {
        if (steps_.empty() || mission_finished_)
        {
            PublishDebug("idle");
            return;
        }

        const auto& step = steps_[current_step_index_];
        PublishIntent(step);

        const double elapsed = (now() - step_start_time_).seconds();
        if (step.wait_reached && has_nav_status_ &&
            latest_nav_status_.goal_id == step.goal_id && latest_nav_status_.reached)
        {
            Advance("nav_status reached");
            return;
        }

        if (step.duration_sec > 0.0 && elapsed >= step.duration_sec)
        {
            Advance("duration elapsed");
            return;
        }

        if (step.timeout_sec > 0.0 && elapsed >= step.timeout_sec)
        {
            if (!step.fallback_step.empty())
            {
                const auto it = step_index_by_name_.find(step.fallback_step);
                if (it != step_index_by_name_.end())
                {
                    current_step_index_ = it->second;
                    step_start_time_ = now();
                    PublishDebug("timeout fallback to " + step.fallback_step);
                    return;
                }
            }
            if (timeout_policy_ == "hold")
            {
                step_start_time_ = now();
                PublishDebug("timeout hold current step");
                return;
            }
            Advance("timeout continue");
            return;
        }

        PublishDebug("running");
    }

    void PublishIntent(const MissionStep& step)
    {
        rm_interfaces::msg::SentryIntent intent;
        intent.header.stamp = now();
        intent.header.frame_id = "sentry_mission_runner";
        intent.protocol_version = 1;
        intent.goal_id = step.goal_id;
        intent.tactical_state = step.tactical_state;
        intent.posture_intent = step.posture_intent;
        intent.fire_policy = step.fire_policy;
        intent.spin_mode = step.spin_mode;
        intent.supercap_mode = step.supercap_mode;
        intent.rule_action_type = 0;
        intent.ammo_exchange_target_total = 0;
        intent.request_revive_confirm = 0;
        intent.request_instant_revive = 0;
        intent.request_remote_ammo_once = 0;
        intent.request_remote_hp_once = 0;
        intent.request_posture_referee = 0;
        intent.request_activate_energy = 0;
        intent.reason = step.reason.empty() ? ("mission step: " + step.name) : step.reason;
        intent_pub_->publish(intent);
    }

    void Advance(const std::string& reason)
    {
        if (current_step_index_ + 1 < steps_.size())
        {
            ++current_step_index_;
        }
        else if (loop_)
        {
            current_step_index_ = 0;
        }
        else
        {
            mission_finished_ = true;
        }
        step_start_time_ = now();
        PublishDebug(reason);
    }

    void PublishDebug(const std::string& reason)
    {
        std_msgs::msg::String debug;
        std::ostringstream oss;
        oss << "step_index=" << current_step_index_
            << " total_steps=" << steps_.size()
            << " finished=" << (mission_finished_ ? "true" : "false");
        if (!steps_.empty() && current_step_index_ < steps_.size())
        {
            const auto& step = steps_[current_step_index_];
            oss << " step=" << step.name
                << " goal_id=" << static_cast<int>(step.goal_id)
                << " wait_reached=" << (step.wait_reached ? "true" : "false")
                << " elapsed=" << (now() - step_start_time_).seconds();
        }
        oss << " nav_status_seen=" << (has_nav_status_ ? "true" : "false");
        if (has_nav_status_)
        {
            oss << " nav_goal_id=" << static_cast<int>(latest_nav_status_.goal_id)
                << " nav_reached=" << (latest_nav_status_.reached ? "true" : "false");
        }
        oss << " autoaim_seen=" << (has_target_status_ ? "true" : "false");
        if (has_target_status_)
        {
            oss << " autoaim_target=" << (latest_target_status_.has_target ? "true" : "false");
        }
        oss << " gimbal_seen=" << (has_gimbal_status_ ? "true" : "false")
            << " reason=" << reason;
        debug.data = oss.str();
        debug_pub_->publish(debug);
    }

    std::string intent_topic_{};
    std::string nav_status_topic_{};
    std::string target_status_topic_{};
    std::string gimbal_status_topic_{};
    std::string debug_topic_{};
    std::string mission_file_{};
    bool loop_{false};
    std::string timeout_policy_{"continue"};
    std::vector<MissionStep> steps_{};
    std::unordered_map<std::string, std::size_t> step_index_by_name_{};
    std::size_t current_step_index_{0};
    bool mission_finished_{false};
    rclcpp::Time step_start_time_{0, 0, get_clock()->get_clock_type()};

    rm_interfaces::msg::SentryNavStatus latest_nav_status_{};
    rm_interfaces::msg::AutoaimTargetStatus latest_target_status_{};
    rm_interfaces::msg::GimbalStatus latest_gimbal_status_{};
    bool has_nav_status_{false};
    bool has_target_status_{false};
    bool has_gimbal_status_{false};

    rclcpp::Publisher<rm_interfaces::msg::SentryIntent>::SharedPtr intent_pub_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_{};
    rclcpp::Subscription<rm_interfaces::msg::SentryNavStatus>::SharedPtr nav_status_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::AutoaimTargetStatus>::SharedPtr target_status_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_sub_{};
    rclcpp::TimerBase::SharedPtr timer_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<SentryMissionRunnerNode>());
    }
    catch (const std::exception& ex)
    {
        std::cerr << "sentry_mission_runner failed: " << ex.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
