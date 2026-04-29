#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/gimbal_cmd.hpp"

#include "robot_context.hpp"

class ShooterInterface
{
public:
    explicit ShooterInterface(rclcpp::Node& node);

    // 执行层输出的统一占位接口。
    // BT executor 会先算出这些值，后续真实项目可以在这里对接底盘、
    // 射击机构、姿态控制器或其他中间件调用。
    void PublishCommand(RobotContext& ctx);

    const rm_interfaces::msg::GimbalCmd& last_command() const;
    const std::string& output_topic() const;

private:
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr publisher_{};
    rm_interfaces::msg::GimbalCmd last_command_{};
    std::string output_topic_{};
};
