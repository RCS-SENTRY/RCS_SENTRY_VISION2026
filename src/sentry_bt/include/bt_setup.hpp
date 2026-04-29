#pragma once

#include <filesystem>
#include <memory>

#include "bt_compat.hpp"

#include "robot_context.hpp"

void RegisterAllNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx);

void ExportTreeModelXML(const BT::BehaviorTreeFactory& factory,
                        const std::filesystem::path& template_path,
                        const std::filesystem::path& output_path,
                        bool include_builtin_models = false);
