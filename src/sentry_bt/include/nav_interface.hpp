#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/nav_cmd.hpp"

#include "robot_context.hpp"

class NavInterface
{
public:
    struct GoalCommand
    {
        // 导航层最终需要关心的是“去哪”和“以什么机动姿态过去”。
        std::string goal_id{"SAFE_HOLD"};
        TacticalState tactical_state{TacticalState::SEARCH};
        Posture posture{Posture::MOVE};
        SpinMode spin_mode{SpinMode::OFF};
        SupercapMode supercap_mode{SupercapMode::OFF};
    };

    explicit NavInterface(rclcpp::Node& node);

    // Fake/debug only: 将行为树 goal 粗略映射成 NavCmd，便于无导航栈台架联调。
    // 比赛正式链路禁止使用该输出；正式运行必须只消费 /sentry/intent 的 goal_id，
    // 速度完全由导航栈、速度控制层和双雷达避障链决定。
    void PublishCommand(RobotContext& ctx);

    const GoalCommand& last_command() const;
    const rm_interfaces::msg::NavCmd& last_nav_msg() const;
    const std::string& output_topic() const;

private:
    rclcpp::Publisher<rm_interfaces::msg::NavCmd>::SharedPtr publisher_{};
    GoalCommand last_command_{};
    rm_interfaces::msg::NavCmd last_nav_msg_{};
    std::string output_topic_{};
    bool enable_debug_nav_cmd_{false};
    double linear_speed_{0.75};
    double strafe_speed_{0.55};
    double retreat_speed_{0.70};
    double slow_speed_{0.35};
    double scan_wz_{0.45};
};
