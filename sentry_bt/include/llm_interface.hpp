#pragma once

#include "robot_context.hpp"

class LLMInterface
{
public:
    LLMInterface();

    // 刷新战术层使用的建议信号。
    // 当前实现仍是启发式占位，目的主要是把“外部智能建议”这条通路预留出来，
    // 以后可在此对接真实规划器、模型推理服务或其他高层策略模块。
    void Update(RobotContext& ctx);
};
