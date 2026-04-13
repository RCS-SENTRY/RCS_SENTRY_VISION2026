#pragma once

#include <memory>

#include "bt_compat.hpp"

#include "robot_context.hpp"

void RegisterExecutorNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx);
