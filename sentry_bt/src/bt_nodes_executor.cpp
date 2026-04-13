#include "bt_nodes_executor.hpp"

#include <cstdint>
#include <sstream>

#include "bt_node_common.hpp"
#include "sentry_decision_protocol.h"

// executor 节点文件负责“战术意图向最终控制输出的收口”。
// 可以把它理解为最后一道保险层：
// tactical 给出 preferred_*，executor 结合 guard / 规则状态后，
// 再生成真正下发的 desired_*。

namespace
{
std::uint8_t PostureToProtocol(Posture posture)
{
    switch (posture)
    {
        case Posture::ATTACK:
            return SENTRY_POSTURE_ATTACK;
        case Posture::DEFENSE:
            return SENTRY_POSTURE_DEFENSE;
        case Posture::MOVE:
            return SENTRY_POSTURE_MOVE;
    }
    return SENTRY_POSTURE_MOVE;
}

// 对火力策略做上限裁剪。
// 例如 preferred 是 AGGRESSIVE，但当前资源紧张，
// executor 可以把它压成 CONSERVATIVE。
FirePolicy ClampFirePolicy(FirePolicy preferred, FirePolicy upper_bound)
{
    return (static_cast<int>(preferred) < static_cast<int>(upper_bound)) ? preferred
                                                                         : upper_bound;
}

// 当 tactical 层没有明确给出目标点时，按战术态选择一个兜底目标。
// 这个函数的意义主要是防止“某个 subtree 漏设置目标”导致系统无目标可发。
std::string DefaultGoalForState(TacticalState state)
{
    switch (state)
    {
        case TacticalState::RETREAT:
            return "SAFE_RETREAT_A";
        case TacticalState::RESUPPLY:
            return "SUPPLY_LEFT";
        case TacticalState::HOLD:
            return "FORTRESS_HOLD";
        case TacticalState::ENGAGE:
            return "COMBAT_HOLD_A";
        case TacticalState::SEARCH:
            return "SEARCH_AREA_A";
        case TacticalState::REPOSITION:
            return "HIGHGROUND_CENTER";
    }
    return "SAFE_HOLD";
}

// 姿态执行收口节点。
// tactical 可能希望切姿态，但如果当前还在冷却，就只能沿用 current_posture。
class ApplyPostureDecisionNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead)
        {
            // 死亡时不应继续切姿态，保持当前值即可。
            ctx_->desired_posture = ctx_->current_posture;
            return BT::NodeStatus::SUCCESS;
        }

        // 冷却期未结束时，不允许采纳新的 preferred_posture。
        ctx_->desired_posture = ctx_->posture_cooldown_guard_active ? ctx_->current_posture
                                                                    : ctx_->preferred_posture;
        if (ctx_->desired_posture != ctx_->current_posture && ctx_->posture_cooldown_ok &&
            (ctx_->now_ms - ctx_->last_posture_command_ms) >= 5000U)
        {
            ctx_->posture_cmd_referee = PostureToProtocol(ctx_->desired_posture);
            ctx_->last_posture_command_ms = ctx_->now_ms;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

// 火力执行收口节点。
// 这是执行层里最像“安全阀”的节点：
// 先看 preferred_fire_policy，再根据热量、弹药、规则动作进一步收紧。
class ApplyFireDecisionNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead)
        {
            ctx_->desired_fire_policy = FirePolicy::HOLD_FIRE;
            return BT::NodeStatus::SUCCESS;
        }

        FirePolicy policy = ctx_->preferred_fire_policy;
        if (ctx_->heat_guard_active || ctx_->ammo_17 <= 0 ||
            ctx_->rule_action_type == RuleActionType::EXCHANGE_AMMO_AT_POINT ||
            ctx_->rule_action_type == RuleActionType::CLAIM_PERIODIC_AMMO)
        {
            // 这些情况都意味着“当前不适合开火”。
            policy = FirePolicy::HOLD_FIRE;
        }
        else
        {
            // 没有强制停火时，再按照资源压力对火力等级做上限裁剪。
            if (ctx_->ammo_guard_active)
            {
                policy = ClampFirePolicy(policy, FirePolicy::CONSERVATIVE);
            }
            if (ctx_->power_guard_active)
            {
                policy = ClampFirePolicy(policy, FirePolicy::CONSERVATIVE);
            }
        }

        ctx_->desired_fire_policy = policy;
        return BT::NodeStatus::SUCCESS;
    }
};

// 小陀螺执行收口节点。
// 如果已经死亡，或者功率/超级电容受限，就直接关掉旋转。
class ApplySpinDecisionNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead || ctx_->power_guard_active || ctx_->supercap_guard_active)
        {
            ctx_->desired_spin_mode = SpinMode::OFF;
            return BT::NodeStatus::SUCCESS;
        }

        ctx_->desired_spin_mode = ctx_->preferred_spin_mode;
        return BT::NodeStatus::SUCCESS;
    }
};

// 超级电容执行收口节点。
// tactical 可以偏向 BURST，但 executor 仍会基于 guard 状态做最后压缩。
class ApplySupercapDecisionNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead || ctx_->supercap_guard_active)
        {
            ctx_->desired_supercap_mode = SupercapMode::OFF;
            return BT::NodeStatus::SUCCESS;
        }

        ctx_->desired_supercap_mode =
            (ctx_->power_guard_active && ctx_->preferred_supercap_mode == SupercapMode::BURST)
                ? SupercapMode::KEEP
                : ctx_->preferred_supercap_mode;
        return BT::NodeStatus::SUCCESS;
    }
};

// 目标点发布前整理节点。
// 这个节点完成两件事：
// 1. 决定最终下发哪个 goal；
// 2. 生成 executor_summary，方便你在日志里快速读懂本轮输出。
class PublishGoalToNavigatorNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead)
        {
            // 死亡时不再下发真实目标点，统一停留在等待复活状态。
            ctx_->desired_goal = "WAIT_REVIVE";
        }
        else if (!ctx_->preferred_goal.empty())
        {
            // 正常情况下优先采用战术层已经挑选好的目标点。
            ctx_->desired_goal = ctx_->preferred_goal;
        }
        else
        {
            // 如果战术层没有明确给出目标，则按战术状态兜底选一个默认点。
            ctx_->desired_goal = DefaultGoalForState(ctx_->tactical_state);
        }

        std::ostringstream oss;
        oss << "goal=" << ctx_->desired_goal
            << ", posture=" << PostureToString(ctx_->desired_posture)
            << ", fire=" << FirePolicyToString(ctx_->desired_fire_policy)
            << ", spin=" << SpinModeToString(ctx_->desired_spin_mode)
            << ", supercap=" << SupercapModeToString(ctx_->desired_supercap_mode)
            << ", ammo_target_total=" << ctx_->ammo_exchange_target_total
            << ", revive_cmd=" << static_cast<int>(ctx_->revive_cmd)
            << ", remote_ammo_inc=" << static_cast<int>(ctx_->remote_ammo_req_inc)
            << ", remote_hp_inc=" << static_cast<int>(ctx_->remote_hp_req_inc)
            << ", posture_cmd_referee=" << static_cast<int>(ctx_->posture_cmd_referee)
            << ", activate_energy=" << static_cast<int>(ctx_->activate_energy_confirm)
            << ", claim_periodic_ammo=" << static_cast<int>(ctx_->claim_periodic_ammo);
        ctx_->executor_summary = oss.str();

        return BT::NodeStatus::SUCCESS;
    }
};
}  // 匿名命名空间

void RegisterExecutorNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx)
{
    // executor 节点注册表。
    RegisterContextNode<ApplyPostureDecisionNode>(factory, "ApplyPostureDecision", ctx);
    RegisterContextNode<ApplyFireDecisionNode>(factory, "ApplyFireDecision", ctx);
    RegisterContextNode<ApplySpinDecisionNode>(factory, "ApplySpinDecision", ctx);
    RegisterContextNode<ApplySupercapDecisionNode>(factory, "ApplySupercapDecision", ctx);
    RegisterContextNode<PublishGoalToNavigatorNode>(factory, "PublishGoalToNavigator", ctx);
}
