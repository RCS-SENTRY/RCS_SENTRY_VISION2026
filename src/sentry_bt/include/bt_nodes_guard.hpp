#pragma once

#include <memory>

#include "bt_compat.hpp"

#include "robot_context.hpp"

void RegisterGuardNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx);
