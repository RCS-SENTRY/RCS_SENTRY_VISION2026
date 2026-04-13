#pragma once

#include <string>

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

    NavInterface();

    // 行为树最终导航结果的发布占位接口。
    // 当前实现只会缓存最近一次命令，并把摘要回写到 RobotContext，
    // 方便日志与调试；后续可在这里接 ROS topic、action 或自定义导航模块。
    void PublishCommand(RobotContext& ctx);

    const GoalCommand& last_command() const;

private:
    GoalCommand last_command_{};
};
