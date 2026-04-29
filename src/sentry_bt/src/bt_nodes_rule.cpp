#include "bt_nodes_rule.hpp"

#include <algorithm>

#include "bt_node_common.hpp"
#include "sentry_decision_protocol.h"

// rule 节点文件处理“规则侧优先动作”和“复活流程”。
// 这一层在主树里的优先级高于常规 tactical subtree，
// 因为很多动作不是“想不想做”，而是“当前规则要求先处理”。

namespace
{
constexpr std::uint64_t kRemoteRequestCooldownMs = 6000U;
constexpr std::uint64_t kPulseCooldownMs = 1000U;

int ComputeRemoteHpCost(const RobotContext& ctx)
{
    const int elapsed = std::clamp(420 - ctx.stage_remain_time, 0, 420);
    const int rounded_block = (elapsed + 59) / 60;
    return 50 + (rounded_block * 20);
}

// 规则动作优先级判定函数。
// 顺序本身就代表优先级：越靠前，越先抢占执行权。
// 因此后续如果你要改优先级，不一定要改很多节点，
// 很可能只需要重排这里的判断顺序。
RuleActionType EvaluateRuleAction(const RobotContext& ctx, std::string& reason)
{
    if (!ctx.match_started)
    {
        reason = "比赛尚未进入正式对抗阶段，不触发规则动作。";
        return RuleActionType::NONE;
    }
    if (ctx.can_activate_energy_mechanism)
    {
        reason = "规则命令要求激活能量机关。";
        return RuleActionType::ACTIVATE_ENERGY;
    }
    if (ctx.is_disengaged && ctx.hp < (ctx.hp_max / 3) && ctx.gold >= ComputeRemoteHpCost(ctx))
    {
        reason = "当前处于脱战且血量偏低，满足远程补血条件。";
        return RuleActionType::REMOTE_HP;
    }
    if (ctx.can_claim_periodic_ammo)
    {
        reason = "当前可领取周期补弹奖励。";
        return RuleActionType::CLAIM_PERIODIC_AMMO;
    }
    if (ctx.posture_switch_requested)
    {
        reason = "外部命令请求切换姿态。";
        return RuleActionType::SWITCH_POSTURE;
    }
    if (ctx.ammo_low && ctx.team_17mm_exchange_remain >= 10)
    {
        reason = "当前弹药不足，可预先抬高非远程补弹累计目标值。";
        return RuleActionType::EXCHANGE_AMMO_AT_POINT;
    }
    if (ctx.is_disengaged && ctx.ammo_low && ctx.gold >= 150)
    {
        reason = "当前处于脱战且弹药不足，满足远程补弹条件。";
        return RuleActionType::REMOTE_AMMO;
    }

    reason = "当前没有待执行的显式规则动作。";
    return RuleActionType::NONE;
}

// 复活确认条件判断节点。
// 这里只判断“能不能确认”，不负责真正发命令。
class CanConfirmReviveNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        return ctx_->can_confirm_revive ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// 发送确认复活动作。
// 当前版本只是写日志/说明字段，真实项目里应在这里接入裁判系统接口。
class ConfirmReviveNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->revive_cmd = SENTRY_REVIVE_CMD_CONFIRM_FREE;
        ctx_->last_rule_command = "ConfirmRevive：向裁判系统发送确认复活请求。";
        ctx_->rule_reason = "当前已满足确认复活条件。";
        return BT::NodeStatus::SUCCESS;
    }
};

// 判断是否允许购买立即复活。
// 条件拆在这里，是为了让 XML 中的 revive_fallback 结构保持清晰。
class ShouldBuyImmediateReviveNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        const bool can_buy = ctx_->can_buy_immediate_revive &&
                             ctx_->gold >= ctx_->immediate_revive_cost &&
                             ctx_->immediate_revive_cost > 0;
        return can_buy ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// 立即复活动作节点。
// 后续如果要对接资源扣减、确认回执等逻辑，可以从这里继续扩展。
class ConfirmImmediateReviveNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->revive_cmd = SENTRY_REVIVE_CMD_CONFIRM_IMMEDIATE;
        ctx_->last_rule_command = "ConfirmImmediateRevive：消耗资源进行快速复活。";
        ctx_->rule_reason = "当前允许立即复活，且资源充足。";
        return BT::NodeStatus::SUCCESS;
    }
};

// 等待复活节点。
// 这里使用 StatefulActionNode，是因为“等待复活”天然是一个持续态过程，
// 它不是一帧内就能完成的同步动作。
class WaitReviveStatefulNode : public ContextStatefulActionNode
{
public:
    using ContextStatefulActionNode::ContextStatefulActionNode;

    BT::NodeStatus onStart() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->revive_cmd = SENTRY_REVIVE_CMD_WAIT;
        ctx_->last_rule_command = "WaitRevive：保持等待，直到复活条件发生变化。";

        // onStart 在节点第一次进入 RUNNING 时调用。
        // 如果这一刻其实已经不死了，也可以直接返回 SUCCESS。
        return ctx_->is_dead ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->revive_cmd = SENTRY_REVIVE_CMD_WAIT;

        // 只要还处于死亡态，就保持 RUNNING。
        return ctx_->is_dead ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
    }
};

// 规则动作评估节点。
// 这个节点负责把 EvaluateRuleAction 的结果同步到 blackboard 输出口，
// 这样 XML 中的 Switch6 才能根据字符串进入对应子分支。
class EvaluateRuleActionNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("rule_action_type")};
    }

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        ctx_->rule_action_type = EvaluateRuleAction(*ctx_, ctx_->rule_reason);
        ctx_->need_rule_action = ctx_->rule_action_type != RuleActionType::NONE;
        setOutput("rule_action_type", std::string(RuleActionTypeToString(ctx_->rule_action_type)));
        return ctx_->need_rule_action ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// 以下动作节点都属于“规则动作执行占位”。
// 当前版本的重点是把框架跑通，并把每个动作应该做什么说明清楚；
// 真正对接裁判系统命令、服务调用或反馈回执时，就在这些节点里补。
class ExchangeAmmoAtPointNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        if (ctx_->team_17mm_exchange_remain < 10)
        {
            return BT::NodeStatus::FAILURE;
        }

        if (ctx_->ammo_exchange_target_total <=
            static_cast<std::uint16_t>(ctx_->exchanged_projectile_allowance))
        {
            int delta = (ctx_->ammo_17 < 40) ? 100 : 50;
            delta = std::min(delta, ctx_->team_17mm_exchange_remain);
            delta = (delta / 10) * 10;
            if (delta <= 0)
            {
                return BT::NodeStatus::FAILURE;
            }

            const int next_total = ctx_->exchanged_projectile_allowance + delta;
            ctx_->ammo_exchange_target_total =
                static_cast<std::uint16_t>(std::clamp(next_total, 0, 1000));
        }

        ctx_->last_rule_command =
            "ExchangeAmmoAtPoint：提高非远程补弹累计目标值，等待底层在合法点位执行。";
        return BT::NodeStatus::SUCCESS;
    }
};

class RemoteExchangeAmmoNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        if (!ctx_->is_disengaged || ctx_->gold < 150)
        {
            return BT::NodeStatus::FAILURE;
        }
        if ((ctx_->now_ms - ctx_->last_remote_ammo_request_ms) < kRemoteRequestCooldownMs)
        {
            return BT::NodeStatus::FAILURE;
        }

        ctx_->remote_ammo_req_inc = 1;
        ctx_->last_remote_ammo_request_ms = ctx_->now_ms;
        ctx_->last_rule_command = "RemoteExchangeAmmo：触发一次远程补弹请求。";
        return BT::NodeStatus::SUCCESS;
    }
};

class RemoteExchangeHPNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        const int remote_hp_cost = ComputeRemoteHpCost(*ctx_);
        if (!ctx_->is_disengaged || ctx_->gold < remote_hp_cost)
        {
            return BT::NodeStatus::FAILURE;
        }
        if ((ctx_->now_ms - ctx_->last_remote_hp_request_ms) < kRemoteRequestCooldownMs)
        {
            return BT::NodeStatus::FAILURE;
        }

        ctx_->remote_hp_req_inc = 1;
        ctx_->last_remote_hp_request_ms = ctx_->now_ms;
        ctx_->last_rule_command = "RemoteExchangeHP：触发一次远程补血请求。";
        return BT::NodeStatus::SUCCESS;
    }
};

class ActivateEnergyMechanismNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        if (!ctx_->can_activate_energy_mechanism)
        {
            return BT::NodeStatus::FAILURE;
        }
        if ((ctx_->now_ms - ctx_->last_energy_activate_ms) < kPulseCooldownMs)
        {
            return BT::NodeStatus::FAILURE;
        }

        ctx_->activate_energy_confirm = 1;
        ctx_->last_energy_activate_ms = ctx_->now_ms;
        ctx_->last_rule_command = "ActivateEnergyMechanism：发送一次能量机关激活确认。";
        return BT::NodeStatus::SUCCESS;
    }
};

class ClaimPeriodicAmmoNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        if (!ctx_->can_claim_periodic_ammo)
        {
            return BT::NodeStatus::FAILURE;
        }
        if ((ctx_->now_ms - ctx_->last_periodic_ammo_claim_ms) < kPulseCooldownMs)
        {
            return BT::NodeStatus::FAILURE;
        }

        ctx_->claim_periodic_ammo = 1;
        ctx_->last_periodic_ammo_claim_ms = ctx_->now_ms;
        ctx_->last_rule_command = "ClaimPeriodicAmmo：发送一次周期补弹领取脉冲。";
        return BT::NodeStatus::SUCCESS;
    }
};

class HandlePostureSwitchRequestNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        if (!ctx_->posture_switch_requested)
        {
            return BT::NodeStatus::FAILURE;
        }

        ctx_->preferred_posture =
            (ctx_->current_posture == Posture::DEFENSE) ? Posture::ATTACK : Posture::DEFENSE;
        ctx_->last_rule_command = "HandlePostureSwitchRequest：在防守姿态与攻击姿态之间切换。";
        return BT::NodeStatus::SUCCESS;
    }
};

class NoRuleActionNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        // 这是一个防御性占位节点。
        // 如果 Switch6 落到了“无规则动作”这一支，让它返回 FAILURE，
        // 便于上层逻辑继续走常规 tactical 分支。
        return BT::NodeStatus::FAILURE;
    }
};
}  // 匿名命名空间

void RegisterRuleNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx)
{
    // rule 节点注册表。
    RegisterContextNode<CanConfirmReviveNode>(factory, "CanConfirmRevive", ctx);
    RegisterContextNode<ConfirmReviveNode>(factory, "ConfirmRevive", ctx);
    RegisterContextNode<ShouldBuyImmediateReviveNode>(factory, "ShouldBuyImmediateRevive", ctx);
    RegisterContextNode<ConfirmImmediateReviveNode>(factory, "ConfirmImmediateRevive", ctx);
    RegisterContextNode<WaitReviveStatefulNode>(factory, "WaitRevive", ctx);
    RegisterContextNode<EvaluateRuleActionNode>(factory, "EvaluateRuleAction", ctx);
    RegisterContextNode<ExchangeAmmoAtPointNode>(factory, "ExchangeAmmoAtPoint", ctx);
    RegisterContextNode<RemoteExchangeAmmoNode>(factory, "RemoteExchangeAmmo", ctx);
    RegisterContextNode<RemoteExchangeHPNode>(factory, "RemoteExchangeHP", ctx);
    RegisterContextNode<ActivateEnergyMechanismNode>(factory, "ActivateEnergyMechanism", ctx);
    RegisterContextNode<ClaimPeriodicAmmoNode>(factory, "ClaimPeriodicAmmo", ctx);
    RegisterContextNode<HandlePostureSwitchRequestNode>(factory, "HandlePostureSwitchRequest", ctx);
    RegisterContextNode<NoRuleActionNode>(factory, "NoRuleAction", ctx);
}
