#pragma once

#include <memory>
#include <string>
#include <utility>

#include "bt_compat.hpp"

#include "robot_context.hpp"

// 行为树节点的公共基类，统一封装对 RobotContext 的访问方式。
// 把这部分集中到一个头文件里，后续扩展节点时就不用在每个 cpp 中
// 重复写同样的上下文注入和样板代码。
class ContextSyncActionNode : public BT::SyncActionNode
{
public:
    ContextSyncActionNode(const std::string& name, const BT::NodeConfiguration& config,
                          std::shared_ptr<RobotContext> ctx)
      : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

protected:
    std::shared_ptr<RobotContext> ctx_;
};

class ContextConditionNode : public BT::ConditionNode
{
public:
    ContextConditionNode(const std::string& name, const BT::NodeConfiguration& config,
                         std::shared_ptr<RobotContext> ctx)
      : BT::ConditionNode(name, config), ctx_(std::move(ctx))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

protected:
    std::shared_ptr<RobotContext> ctx_;
};

class ContextStatefulActionNode : public BT::StatefulActionNode
{
public:
    ContextStatefulActionNode(const std::string& name, const BT::NodeConfiguration& config,
                              std::shared_ptr<RobotContext> ctx)
      : BT::StatefulActionNode(name, config), ctx_(std::move(ctx))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

protected:
    std::shared_ptr<RobotContext> ctx_;
};

inline float SafeRatio(int value, int max_value)
{
    return static_cast<float>(value) / static_cast<float>(max_value > 0 ? max_value : 1);
}

inline void AddGoalCandidate(RobotContext& ctx, std::string goal_id, float score,
                             std::string rationale = {})
{
    // 统一的候选目标写入口，便于后续在这里集中补充调试统计、
    // 去重、限长或更多打分维度。
    ctx.goal_candidates.push_back(
        GoalCandidate{std::move(goal_id), score, std::move(rationale)});
}

template <typename T>
inline void RegisterContextNode(BT::BehaviorTreeFactory& factory, const std::string& id,
                                const std::shared_ptr<RobotContext>& ctx)
{
    BT::NodeBuilder builder = [ctx](const std::string& name,
                                    const BT::NodeConfiguration& config) {
        return std::make_unique<T>(name, config, ctx);
    };
    factory.registerBuilder(BT::CreateManifest<T>(id), builder);
}
