#include "bt_nodes_executor.hpp"

#include <algorithm>
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
    return PostureToProtocolValue(posture);
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

float BasePostureScore(const RobotContext& ctx, Posture posture)
{
    const float hp_ratio = SafeRatio(ctx.hp, ctx.hp_max);
    const float heat_ratio = SafeRatio(ctx.heat, ctx.heat_limit);

    switch (ctx.tactical_state)
    {
        case TacticalState::RETREAT:
            return (posture == Posture::MOVE) ? 1.00f
                   : (posture == Posture::DEFENSE) ? 0.35f
                                                  : 0.10f;
        case TacticalState::RESUPPLY:
            return (posture == Posture::MOVE) ? 1.00f
                   : (posture == Posture::DEFENSE) ? (ctx.enemy_in_view ? 0.55f : 0.40f)
                                                  : 0.08f;
        case TacticalState::HOLD:
            return (posture == Posture::DEFENSE) ? 1.00f
                   : (posture == Posture::MOVE) ? 0.72f
                                                : 0.38f;
        case TacticalState::ENGAGE:
            if (posture == Posture::ATTACK)
            {
                return 1.00f;
            }
            if (posture == Posture::MOVE)
            {
                return (ctx.enemy_distance_m < 4.0f || heat_ratio > 0.80f) ? 0.88f : 0.72f;
            }
            return (hp_ratio < 0.45f || heat_ratio > 0.78f) ? 0.82f : 0.56f;
        case TacticalState::SEARCH:
            return (posture == Posture::MOVE) ? 1.00f
                   : (posture == Posture::ATTACK) ? 0.24f
                                                  : 0.18f;
        case TacticalState::REPOSITION:
            return (posture == Posture::MOVE) ? 1.00f
                   : (posture == Posture::ATTACK) ? (ctx.enemy_in_view ? 0.42f : 0.24f)
                                                  : 0.26f;
    }
    return (posture == Posture::MOVE) ? 1.00f : 0.20f;
}

float PostureDebuffPenalty(const RobotContext& ctx, Posture posture)
{
    if (IsPostureDebuffed(ctx, posture))
    {
        return 0.55f;
    }

    if (ctx.posture_debuff_rotate_margin_ms == 0)
    {
        return 0.0f;
    }

    return (RemainingBeforePostureDebuff(ctx, posture) <= ctx.posture_debuff_rotate_margin_ms)
               ? 0.18f
               : 0.0f;
}

bool HasExplicitPostureOverride(const RobotContext& ctx)
{
    return ctx.rule_action_type == RuleActionType::SWITCH_POSTURE || ctx.posture_switch_requested;
}

bool IsMotionTacticalState(TacticalState state)
{
    return state == TacticalState::REPOSITION || state == TacticalState::SEARCH ||
           state == TacticalState::RESUPPLY || state == TacticalState::RETREAT;
}

Posture ExplicitPostureTarget(const RobotContext& ctx)
{
    return (ctx.current_posture == Posture::DEFENSE) ? Posture::ATTACK : Posture::DEFENSE;
}

Posture SelectPostureWithDebuffAwareness(const RobotContext& ctx, std::string& reason)
{
    struct Candidate
    {
        Posture posture{Posture::MOVE};
        float score{-1.0f};
    };

    Candidate best{};
    const Posture candidates[] = {Posture::ATTACK, Posture::DEFENSE, Posture::MOVE};
    for (const auto posture : candidates)
    {
        float score = BasePostureScore(ctx, posture) - PostureDebuffPenalty(ctx, posture);
        if (posture == ctx.preferred_posture)
        {
            score += 0.12f;
        }
        if (posture == ctx.current_posture)
        {
            score += 0.05f;
        }
        if (score > best.score)
        {
            best = Candidate{posture, score};
        }
    }

    if (best.posture == ctx.preferred_posture)
    {
        if (IsPostureDebuffed(ctx, ctx.preferred_posture))
        {
            reason = std::string("首选姿态 ") + PostureToString(ctx.preferred_posture) +
                     " 已衰减，但当前战术收益仍高于备选姿态，继续保持。";
        }
        else if (ctx.posture_debuff_rotate_margin_ms > 0 &&
                 RemainingBeforePostureDebuff(ctx, ctx.preferred_posture) <=
                     ctx.posture_debuff_rotate_margin_ms)
        {
            reason = std::string("首选姿态 ") + PostureToString(ctx.preferred_posture) +
                     " 已接近衰减阈值，但当前战术仍以该姿态收益最高。";
        }
        else
        {
            reason = std::string("首选姿态 ") + PostureToString(ctx.preferred_posture) +
                     " 尚未触发明显衰减风险，按战术偏好执行。";
        }
    }
    else if (IsPostureDebuffed(ctx, ctx.preferred_posture))
    {
        reason = std::string("首选姿态 ") + PostureToString(ctx.preferred_posture) +
                 " 已累计超过衰减阈值，改用 " + PostureToString(best.posture) +
                 " 以保留未衰减姿态收益。";
    }
    else
    {
        reason = std::string("首选姿态 ") + PostureToString(ctx.preferred_posture) +
                 " 接近衰减阈值，预留轮换并暂改用 " + PostureToString(best.posture) + "。";
    }

    return best.posture;
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

        if (ctx_->is_dead || !ctx_->referee_link_fresh || !ctx_->match_started)
        {
            // 死亡时不应继续切姿态，保持当前值即可。
            ctx_->desired_posture = ctx_->current_posture;
            ctx_->posture_reason = ctx_->is_dead
                                       ? "死亡状态下冻结姿态输出，保持当前确认姿态。"
                                       : (!ctx_->referee_link_fresh
                                              ? "输入链路超时，冻结姿态输出并等待状态恢复。"
                                              : "比赛未开始，冻结姿态输出并保持待机。");
            return BT::NodeStatus::SUCCESS;
        }

        if (ctx_->need_emergency_safety && ctx_->pending_posture_target != Posture::MOVE)
        {
            ctx_->desired_posture = Posture::MOVE;
            ctx_->posture_reason =
                "紧急安全分支已接管，忽略非移动姿态的待确认目标，先按 MOVE 输出。";
            return BT::NodeStatus::SUCCESS;
        }

        if (ctx_->posture_switch_pending)
        {
            ctx_->desired_posture = ctx_->pending_posture_target;
            ctx_->posture_reason = std::string("已有姿态切换待反馈确认，继续等待目标姿态 ") +
                                   PostureToString(ctx_->pending_posture_target) + " 生效。";
            return BT::NodeStatus::SUCCESS;
        }

        Posture selected_posture = ctx_->preferred_posture;
        if (HasExplicitPostureOverride(*ctx_))
        {
            selected_posture = ExplicitPostureTarget(*ctx_);
            ctx_->posture_reason = std::string("当前存在显式姿态切换请求，直接请求切换到 ") +
                                   PostureToString(selected_posture) + "。";
        }
        else if (ctx_->nav_goal_reached && ctx_->tactical_state == TacticalState::HOLD)
        {
            selected_posture = ctx_->hold_reached_posture;
            ctx_->posture_reason = std::string("导航目标已到达，HOLD 态按参数切到 ") +
                                   PostureToString(selected_posture) + "。";
        }
        else if (ctx_->nav_goal_reached && ctx_->tactical_state == TacticalState::ENGAGE)
        {
            selected_posture = Posture::ATTACK;
            ctx_->posture_reason = "导航目标已到达且处于 ENGAGE 态，切到 ATTACK 姿态。";
        }
        else if (!ctx_->nav_goal_reached && IsMotionTacticalState(ctx_->tactical_state))
        {
            selected_posture = Posture::MOVE;
            ctx_->posture_reason =
                "导航目标尚未到达且当前为机动类战术态，保持 MOVE 姿态。";
        }
        else
        {
            selected_posture = SelectPostureWithDebuffAwareness(*ctx_, ctx_->posture_reason);
        }

        // 冷却期未结束时，不允许采纳新的 preferred_posture。
        ctx_->desired_posture =
            ctx_->posture_cooldown_guard_active ? ctx_->current_posture : selected_posture;
        if (ctx_->posture_cooldown_guard_active && selected_posture != ctx_->current_posture)
        {
            ctx_->posture_reason +=
                " 但姿态切换仍在冷却期，暂时维持当前姿态等待冷却结束。";
        }
        if (ctx_->desired_posture != ctx_->current_posture && ctx_->posture_cooldown_ok)
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

        if (ctx_->is_dead || !ctx_->referee_link_fresh || !ctx_->match_started)
        {
            ctx_->desired_fire_policy = FirePolicy::HOLD_FIRE;
            return BT::NodeStatus::SUCCESS;
        }

        FirePolicy policy = ctx_->preferred_fire_policy;
        if (ctx_->nav_goal_reached && ctx_->tactical_state == TacticalState::HOLD)
        {
            policy = FirePolicy::NORMAL;
        }
        else if (ctx_->nav_goal_reached && ctx_->tactical_state == TacticalState::ENGAGE)
        {
            policy = ClampFirePolicy(
                std::max(policy, FirePolicy::NORMAL,
                         [](FirePolicy lhs, FirePolicy rhs) {
                             return static_cast<int>(lhs) < static_cast<int>(rhs);
                         }),
                FirePolicy::AGGRESSIVE);
        }
        else if (!ctx_->nav_goal_reached && IsMotionTacticalState(ctx_->tactical_state))
        {
            policy = ctx_->enemy_in_view ? FirePolicy::CONSERVATIVE : FirePolicy::HOLD_FIRE;
        }
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

        if (ctx_->is_dead || !ctx_->referee_link_fresh || !ctx_->match_started ||
            ctx_->power_guard_active || ctx_->supercap_guard_active)
        {
            ctx_->desired_spin_mode = SpinMode::OFF;
            ctx_->spin_reason = ctx_->is_dead
                                    ? "执行层关闭小陀螺：机器人已死亡。"
                                    : (!ctx_->referee_link_fresh
                                           ? "执行层关闭小陀螺：输入链路超时。"
                                           : (!ctx_->match_started
                                                  ? "执行层关闭小陀螺：比赛未开始。"
                                                  : (ctx_->power_guard_active
                                                         ? "执行层关闭小陀螺：底盘功率守卫触发。"
                                                         : "执行层关闭小陀螺：超级电容守卫触发。")));
            return BT::NodeStatus::SUCCESS;
        }

        ctx_->desired_spin_mode = ctx_->preferred_spin_mode;
        if (ctx_->spin_reason.empty())
        {
            ctx_->spin_reason = ctx_->desired_spin_mode == SpinMode::ON
                                    ? "执行层采纳战术偏好，启用小陀螺。"
                                    : "执行层采纳战术偏好，关闭小陀螺。";
        }
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

        if (ctx_->is_dead || !ctx_->referee_link_fresh || !ctx_->match_started ||
            ctx_->supercap_guard_active)
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

        if (!ctx_->referee_link_fresh)
        {
            ctx_->desired_goal = "SAFE_HOLD";
            if (ctx_->goal_reason.empty())
            {
                ctx_->goal_reason = ctx_->input_health_reason;
            }
        }
        else if (!ctx_->match_started)
        {
            ctx_->desired_goal = "SAFE_HOLD";
            if (ctx_->goal_reason.empty())
            {
                ctx_->goal_reason = "比赛未开始，目标点冻结在安全待机。";
            }
        }
        else if (ctx_->is_dead)
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
            << ", posture_reason=" << ctx_->posture_reason
            << ", fire=" << FirePolicyToString(ctx_->desired_fire_policy)
            << ", spin=" << SpinModeToString(ctx_->desired_spin_mode)
            << ", spin_reason=" << ctx_->spin_reason
            << ", supercap=" << SupercapModeToString(ctx_->desired_supercap_mode)
            << ", referee_fresh=" << (ctx_->referee_link_fresh ? "true" : "false")
            << ", status_age_ms=" << ctx_->referee_status_age_ms
            << ", sim_fresh=" << (ctx_->sim_input_fresh ? "true" : "false")
            << ", sim_age_ms=" << ctx_->sim_input_age_ms
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
