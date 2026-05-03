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

    // 将行为树的抽象 goal 收敛成底盘速度指令。
    // 当前版本采用可配置的启发式速度表，保证没有完整定位/路径规划时也能形成闭环；
    // 后续若接入正式导航模块，可保持 BT 输出不变，只替换这里的 goal 执行器。
    void PublishCommand(RobotContext& ctx);

    const GoalCommand& last_command() const;
    const rm_interfaces::msg::NavCmd& last_nav_msg() const;
    const std::string& output_topic() const;

private:
    rclcpp::Publisher<rm_interfaces::msg::NavCmd>::SharedPtr publisher_{};
    GoalCommand last_command_{};
    rm_interfaces::msg::NavCmd last_nav_msg_{};
    std::string output_topic_{};
    double linear_speed_{0.75};
    double strafe_speed_{0.55};
    double retreat_speed_{0.70};
    double slow_speed_{0.35};
    double scan_wz_{0.45};
};
