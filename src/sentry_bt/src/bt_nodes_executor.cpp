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
            return "SEARCH_AREA_A";
        case TacticalState::SEARCH:
            return "SEARCH_AREA_A";
        case TacticalState::REPOSITION:
            return "MID_CROSS";
    }
    return "SAFE_HOLD";
}

std::string GoalNameFromProtocolId(std::uint8_t goal_id)
{
    switch (goal_id)
    {
        case SENTRY_GOAL_ID_SAFE_HOLD:
            return "SAFE_HOLD";
        case SENTRY_GOAL_ID_WAIT_REVIVE:
            return "WAIT_REVIVE";
        case SENTRY_GOAL_ID_SAFE_RETREAT_A:
            return "SAFE_RETREAT_A";
        case SENTRY_GOAL_ID_SAFE_RETREAT_B:
            return "SAFE_RETREAT_B";
        case SENTRY_GOAL_ID_SUPPLY_LEFT:
            return "SUPPLY_LEFT";
        case SENTRY_GOAL_ID_SUPPLY_RIGHT:
            return "SUPPLY_RIGHT";
        case SENTRY_GOAL_ID_FORTRESS_HOLD:
            return "FORTRESS_HOLD";
        case SENTRY_GOAL_ID_OUTPOST_HOLD:
            return "OUTPOST_HOLD";
        case SENTRY_GOAL_ID_SEARCH_AREA_A:
            return "SEARCH_AREA_A";
        case SENTRY_GOAL_ID_SEARCH_AREA_B:
            return "SEARCH_AREA_B";
        case SENTRY_GOAL_ID_MID_CROSS:
            return "MID_CROSS";
        case SENTRY_GOAL_ID_BASE_HOME:
            return "BASE_HOME";
        case SENTRY_GOAL_ID_BASE_HOLD:
            return "BASE_HOLD";
        default:
            return {};
    }
}

std::uint64_t ElapsedMs(std::uint64_t now_ms, std::uint64_t since_ms)
{
    return (since_ms == 0 || now_ms < since_ms) ? 0 : (now_ms - since_ms);
}

bool ElapsedEnough(std::uint64_t now_ms, std::uint64_t since_ms, std::uint64_t required_ms)
{
    if (required_ms == 0 || since_ms == 0 || now_ms < since_ms)
    {
        return true;
    }
    return (now_ms - since_ms) >= required_ms;
}

bool DwellCanHoldGoal(const RobotContext& ctx)
{
    if (!ctx.dwell_active || ctx.dwell_complete || ctx.dwell_required_ms == 0)
    {
        return false;
    }
    if (!ctx.match_started || !ctx.referee_link_fresh || ctx.is_dead || ctx.need_emergency_safety ||
        ctx.need_supply)
    {
        return false;
    }
    return ctx.tactical_state == TacticalState::HOLD ||
           ctx.tactical_state == TacticalState::SEARCH ||
           ctx.tactical_state == TacticalState::REPOSITION;
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

bool StableSince(const RobotContext& ctx, std::uint64_t since_ms, std::uint64_t required_ms)
{
    return since_ms != 0 && ctx.now_ms >= since_ms && (ctx.now_ms - since_ms) >= required_ms;
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

InternalMotionState InternalMotionFromPosture(Posture posture)
{
    switch (posture)
    {
        case Posture::ATTACK:
            return InternalMotionState::ATTACK;
        case Posture::DEFENSE:
            return InternalMotionState::DEFENSE;
        case Posture::MOVE:
            return InternalMotionState::NAV;
    }
    return InternalMotionState::NAV;
}

InternalMotionState InternalMotionFromDecision(const RobotContext& ctx, Posture posture)
{
    if (posture == Posture::ATTACK)
    {
        return InternalMotionState::ATTACK;
    }
    if (posture == Posture::DEFENSE)
    {
        return InternalMotionState::DEFENSE;
    }

    switch (ctx.tactical_state)
    {
        case TacticalState::RETREAT:
            return InternalMotionState::RETREAT;
        case TacticalState::RESUPPLY:
            return InternalMotionState::RESUPPLY;
        case TacticalState::ENGAGE:
            return InternalMotionState::ATTACK;
        case TacticalState::HOLD:
            return InternalMotionState::DEFENSE;
        case TacticalState::SEARCH:
        case TacticalState::REPOSITION:
            return InternalMotionState::NAV;
    }
    return InternalMotionState::NAV;
}

bool InternalMotionUsesChassisNavigation(InternalMotionState state)
{
    return state == InternalMotionState::NAV ||
           state == InternalMotionState::RESUPPLY ||
           state == InternalMotionState::RETREAT;
}

bool InternalMotionUsesCombatOutput(InternalMotionState state)
{
    return state == InternalMotionState::ATTACK ||
           state == InternalMotionState::DEFENSE;
}

void ApplyInternalMotionLatch(RobotContext& ctx, InternalMotionState proposed, bool allow_preempt)
{
    if (!ctx.internal_motion_initialized)
    {
        ctx.internal_motion_initialized = true;
        ctx.internal_motion_latched = proposed;
        ctx.internal_motion_last_change_ms = ctx.now_ms;
        ctx.desired_posture = InternalMotionToRefereePosture(proposed);
        return;
    }

    if (proposed != ctx.internal_motion_latched)
    {
        const auto held_ms = ElapsedMs(ctx.now_ms, ctx.internal_motion_last_change_ms);
        if (allow_preempt || held_ms >= ctx.internal_motion_min_hold_ms)
        {
            ctx.internal_motion_latched = proposed;
            ctx.internal_motion_last_change_ms = ctx.now_ms;
        }
        else
        {
            ctx.posture_reason +=
                " 内部运动态锁存中，继续保持 " +
                std::string(InternalMotionStateToString(ctx.internal_motion_latched)) +
                "，remaining_ms=" +
                std::to_string(ctx.internal_motion_min_hold_ms - held_ms) + "。";
        }
    }

    ctx.desired_posture = InternalMotionToRefereePosture(ctx.internal_motion_latched);
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

Posture CandidateWithDebuffAwareness(const RobotContext& ctx, Posture raw_candidate,
                                     std::string& reason)
{
    if (raw_candidate == Posture::MOVE || IsPostureDebuffed(ctx, raw_candidate))
    {
        return raw_candidate;
    }
    if (ctx.posture_debuff_rotate_margin_ms > 0 &&
        RemainingBeforePostureDebuff(ctx, raw_candidate) <= ctx.posture_debuff_rotate_margin_ms)
    {
        std::string debuff_reason;
        const auto debuff_candidate = SelectPostureWithDebuffAwareness(ctx, debuff_reason);
        reason += " " + debuff_reason;
        return debuff_candidate;
    }
    return raw_candidate;
}

std::uint64_t ConfirmMsForPosture(const RobotContext& ctx, Posture posture)
{
    switch (posture)
    {
        case Posture::ATTACK:
            return std::max(ctx.posture_candidate_confirm_ms, ctx.attack_enter_confirm_ms);
        case Posture::DEFENSE:
            return std::max(ctx.posture_candidate_confirm_ms, ctx.defense_enter_confirm_ms);
        case Posture::MOVE:
            return std::max(ctx.posture_candidate_confirm_ms, ctx.move_enter_confirm_ms);
    }
    return ctx.posture_candidate_confirm_ms;
}

Posture SelectPostureCandidate(const RobotContext& ctx, std::string& reason)
{
    const bool referee_unavailable = !ctx.referee_link_fresh;
    const bool hard_move = ctx.is_dead || !ctx.match_started || referee_unavailable ||
                           ctx.need_supply || ctx.tactical_state == TacticalState::RESUPPLY ||
                           ctx.tactical_state == TacticalState::RETREAT;
    if (hard_move)
    {
        reason = "硬安全/补给/撤退候选 MOVE，姿态只输出裁判系统姿态，不控制 nav/fire/spin。";
        return Posture::MOVE;
    }

    const bool fire_ready_stable =
        StableSince(ctx, ctx.autoaim_fire_ready_since_ms, ctx.attack_enter_confirm_ms);
    const bool tracking_stable =
        StableSince(ctx, ctx.autoaim_tracking_since_ms, ctx.attack_enter_confirm_ms);
    if (fire_ready_stable || tracking_stable)
    {
        reason = "自瞄 tracking/fire_ready 已稳定，ATTACK 作为姿态候选；goal/fire/spin 不被抢占。";
        return CandidateWithDebuffAwareness(ctx, Posture::ATTACK, reason);
    }

    const bool nav_timeout_temp_defense =
        ctx.nav_goal_active && !ctx.nav_goal_reached && ctx.nav_goal_active_since_ms != 0 &&
        ctx.nav_goal_timeout_as_temp_defense_ms > 0 &&
        ElapsedMs(ctx.now_ms, ctx.nav_goal_active_since_ms) >=
            ctx.nav_goal_timeout_as_temp_defense_ms;
    if ((ctx.dwell_active && !ctx.dwell_complete) ||
        (ctx.nav_goal_reached && ctx.tactical_state == TacticalState::SEARCH) ||
        nav_timeout_temp_defense)
    {
        reason = nav_timeout_temp_defense
                     ? "导航目标 12s 未到点，当前点按临时扫描/防守点处理，DEFENSE 作为姿态候选。"
                     : "到点 dwell/SEARCH reached 稳定防守，DEFENSE 作为姿态候选。";
        return CandidateWithDebuffAwareness(ctx, Posture::DEFENSE, reason);
    }

    if (ctx.nav_goal_active && !ctx.nav_goal_reached)
    {
        reason = "导航 active 且未到点，MOVE 作为姿态候选；只影响裁判姿态收益，不输出速度。";
        return Posture::MOVE;
    }

    reason = "默认保持当前锁存姿态，避免 1/2/3 无意义跳变。";
    return ctx.desired_posture;
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

        if (!ctx_->internal_motion_initialized)
        {
            ctx_->internal_motion_initialized = true;
            ctx_->internal_motion_latched = InternalMotionFromPosture(ctx_->desired_posture);
            ctx_->internal_motion_last_change_ms = ctx_->now_ms;
        }

        std::string candidate_reason;
        Posture selected_posture = SelectPostureCandidate(*ctx_, candidate_reason);
        if (HasExplicitPostureOverride(*ctx_) && ctx_->posture_cooldown_ok &&
            !ctx_->posture_switch_pending)
        {
            selected_posture = ExplicitPostureTarget(*ctx_);
            candidate_reason = std::string("显式姿态切换请求，目标候选=") +
                               PostureToString(selected_posture) + "。";
        }

        if (selected_posture != ctx_->posture_candidate)
        {
            ctx_->posture_candidate = selected_posture;
            ctx_->posture_candidate_since_ms = ctx_->now_ms;
        }
        else if (ctx_->posture_candidate_since_ms == 0)
        {
            ctx_->posture_candidate_since_ms = ctx_->now_ms;
        }

        ctx_->posture_reason = candidate_reason;
        if (ctx_->posture_switch_pending)
        {
            ctx_->desired_posture = ctx_->pending_posture_target;
            ctx_->internal_motion_latched = InternalMotionFromPosture(ctx_->desired_posture);
            ctx_->posture_reason += std::string(" 已有姿态切换待反馈确认，继续等待 ") +
                                    PostureToString(ctx_->pending_posture_target) + "。";
            return BT::NodeStatus::SUCCESS;
        }

        const auto candidate_confirm_ms = ConfirmMsForPosture(*ctx_, ctx_->posture_candidate);
        const bool candidate_confirmed =
            ElapsedEnough(ctx_->now_ms, ctx_->posture_candidate_since_ms, candidate_confirm_ms);
        const bool min_hold_ok =
            ElapsedEnough(ctx_->now_ms, ctx_->last_posture_command_ms, ctx_->posture_min_hold_ms);
        const bool can_switch =
            ctx_->posture_candidate != ctx_->desired_posture &&
            ctx_->posture_candidate != ctx_->current_posture &&
            ctx_->posture_cooldown_ok && !ctx_->posture_cooldown_guard_active &&
            candidate_confirmed && min_hold_ok;

        if (can_switch)
        {
            ctx_->desired_posture = ctx_->posture_candidate;
            ctx_->internal_motion_latched = InternalMotionFromPosture(ctx_->desired_posture);
            ctx_->internal_motion_last_change_ms = ctx_->now_ms;
            ctx_->posture_cmd_referee = PostureToProtocol(ctx_->posture_candidate);
            ctx_->last_posture_command_ms = ctx_->now_ms;
            ctx_->posture_reason += std::string(" 候选稳定且冷却/最小保持满足，发送裁判姿态 ") +
                                    PostureToString(ctx_->desired_posture) + "。";
        }
        else
        {
            ctx_->internal_motion_latched = InternalMotionFromPosture(ctx_->desired_posture);
            ctx_->posture_reason += std::string(" 当前锁存输出=") +
                                    PostureToString(ctx_->desired_posture) +
                                    ", candidate=" +
                                    PostureToString(ctx_->posture_candidate) +
                                    ", confirmed=" + (candidate_confirmed ? "true" : "false") +
                                    ", cooldown_ok=" +
                                    (ctx_->posture_cooldown_ok ? "true" : "false") +
                                    ", min_hold_ok=" + (min_hold_ok ? "true" : "false") + "。";
        }
        return BT::NodeStatus::SUCCESS;
    }
};

// 自瞄开关执行收口节点。
// fire_policy 下发给下位机作为自瞄开关：1=OFF，2=ON。
class ApplyFireDecisionNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead || !ctx_->referee_link_fresh || !ctx_->match_started)
        {
            ctx_->desired_fire_policy = FirePolicy::CONSERVATIVE;
            ctx_->fire_filtered_policy = FirePolicy::CONSERVATIVE;
            ctx_->fire_policy_enable_since_ms = 0;
            ctx_->fire_policy_disable_since_ms = ctx_->now_ms;
            return BT::NodeStatus::SUCCESS;
        }

        const bool hard_disable =
            ctx_->need_emergency_safety ||
            ctx_->need_supply ||
            ctx_->tactical_state == TacticalState::RESUPPLY ||
            ctx_->rule_action_type == RuleActionType::EXCHANGE_AMMO_AT_POINT ||
            ctx_->rule_action_type == RuleActionType::CLAIM_PERIODIC_AMMO;
        if (hard_disable)
        {
            ctx_->desired_fire_policy = FirePolicy::CONSERVATIVE;
            ctx_->fire_filtered_policy = FirePolicy::CONSERVATIVE;
            ctx_->fire_policy_enable_since_ms = 0;
            ctx_->fire_policy_disable_since_ms = ctx_->now_ms;
            ctx_->fire_policy_last_change_ms = ctx_->now_ms;
            return BT::NodeStatus::SUCCESS;
        }

        const bool request_on =
            ctx_->tactical_state == TacticalState::SEARCH ||
            ctx_->tactical_state == TacticalState::HOLD ||
            ctx_->tactical_state == TacticalState::ENGAGE ||
            ctx_->tactical_state == TacticalState::RETREAT ||
            ctx_->tactical_state == TacticalState::REPOSITION;
        if (request_on)
        {
            ctx_->desired_fire_policy = FirePolicy::NORMAL;
            if (ctx_->fire_filtered_policy != FirePolicy::NORMAL)
            {
                ctx_->fire_policy_last_change_ms = ctx_->now_ms;
            }
            ctx_->fire_filtered_policy = FirePolicy::NORMAL;
            ctx_->fire_policy_enable_since_ms = ctx_->now_ms;
            ctx_->fire_policy_disable_since_ms = 0;
            return BT::NodeStatus::SUCCESS;
        }

        ctx_->desired_fire_policy = FirePolicy::NORMAL;
        ctx_->fire_filtered_policy = FirePolicy::NORMAL;
        ctx_->fire_policy_enable_since_ms = ctx_->now_ms;
        ctx_->fire_policy_disable_since_ms = 0;
        return BT::NodeStatus::SUCCESS;
    }
};

// 小陀螺执行收口节点。
// 小陀螺只由“目标保持窗”或“受击保持窗”驱动，窗口内保持旋转。
class ApplySpinDecisionNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead || !ctx_->referee_link_fresh || !ctx_->match_started ||
            ctx_->need_emergency_safety || ctx_->need_supply ||
            ctx_->tactical_state == TacticalState::RESUPPLY)
        {
            const bool was_on = ctx_->spin_filtered_mode == SpinMode::ON ||
                                ctx_->desired_spin_mode == SpinMode::ON;
            ctx_->desired_spin_mode = SpinMode::OFF;
            ctx_->spin_filtered_mode = SpinMode::OFF;
            ctx_->spin_preference_on_since_ms = 0;
            ctx_->spin_preference_off_since_ms = 0;
            ctx_->spin_target_last_seen_ms = 0;
            ctx_->spin_under_attack_last_seen_ms = 0;
            if (ctx_->spin_last_change_ms == 0 || was_on)
            {
                ctx_->spin_last_change_ms = ctx_->now_ms;
            }
            if (ctx_->is_dead)
            {
                ctx_->spin_reason = "执行层关闭小陀螺：机器人已死亡。";
            }
            else if (!ctx_->referee_link_fresh)
            {
                ctx_->spin_reason = "执行层关闭小陀螺：输入链路超时。";
            }
            else if (!ctx_->match_started)
            {
                ctx_->spin_reason = "执行层关闭小陀螺：比赛未开始。";
            }
            else if (ctx_->need_supply)
            {
                ctx_->spin_reason = "执行层关闭小陀螺：低血/低弹补给闭环优先。";
            }
            else
            {
                ctx_->spin_reason = "执行层关闭小陀螺：紧急安全分支接管。";
            }
            return BT::NodeStatus::SUCCESS;
        }

        if (ctx_->under_attack)
        {
            ctx_->spin_under_attack_last_seen_ms = ctx_->now_ms;
        }

        const bool under_attack_recent =
            ctx_->spin_under_attack_last_seen_ms != 0 &&
            ElapsedMs(ctx_->now_ms, ctx_->spin_under_attack_last_seen_ms) <=
                ctx_->spin_under_attack_hold_ms;
        const bool request_on = true;

        if (request_on)
        {
            if (ctx_->spin_preference_on_since_ms == 0)
            {
                ctx_->spin_preference_on_since_ms = ctx_->now_ms;
            }
            ctx_->spin_preference_off_since_ms = 0;
        }
        else
        {
            if (ctx_->spin_preference_off_since_ms == 0)
            {
                ctx_->spin_preference_off_since_ms = ctx_->now_ms;
            }
            ctx_->spin_preference_on_since_ms = 0;
        }

        SpinMode filtered = request_on ? SpinMode::ON : SpinMode::OFF;
        if (ctx_->spin_hysteresis_enabled && ctx_->spin_filtered_mode == SpinMode::ON &&
            !request_on)
        {
            const bool can_turn_off =
                ElapsedEnough(ctx_->now_ms, ctx_->spin_preference_off_since_ms,
                              ctx_->spin_off_confirm_ms) &&
                ElapsedEnough(ctx_->now_ms, ctx_->spin_last_change_ms, ctx_->spin_min_on_ms);
            if (!can_turn_off)
            {
                filtered = SpinMode::ON;
            }
        }
        else if (ctx_->spin_hysteresis_enabled && ctx_->spin_filtered_mode == SpinMode::OFF &&
                 request_on)
        {
            const bool can_turn_on =
                ElapsedEnough(ctx_->now_ms, ctx_->spin_preference_on_since_ms,
                              ctx_->spin_on_confirm_ms) &&
                ElapsedEnough(ctx_->now_ms, ctx_->spin_last_change_ms, ctx_->spin_min_off_ms);
            filtered = can_turn_on ? SpinMode::ON : SpinMode::OFF;
        }

        if (filtered != ctx_->spin_filtered_mode)
        {
            ctx_->spin_filtered_mode = filtered;
            ctx_->spin_last_change_ms = ctx_->now_ms;
        }
        ctx_->desired_spin_mode = ctx_->spin_filtered_mode;

        std::ostringstream reason;
        reason << "执行层小陀螺热修策略：raw_pref="
               << SpinModeToString(ctx_->preferred_spin_mode)
               << ", under_attack_recent=" << (under_attack_recent ? "true" : "false")
               << ", under_attack_hold_ms=" << ctx_->spin_under_attack_hold_ms
               << ", on_confirm_ms=" << ctx_->spin_on_confirm_ms
               << ", off_confirm_ms=" << ctx_->spin_off_confirm_ms
               << ", min_on_ms=" << ctx_->spin_min_on_ms
               << ", output=" << SpinModeToString(ctx_->desired_spin_mode) << "。";
        if (under_attack_recent && ctx_->preferred_spin_mode == SpinMode::OFF)
        {
            reason << " 受击保持窗仍有效，禁止因未见目标关闭小陀螺。";
        }
        reason << " spin 与 posture/internal_motion/nav_goal_failed/避障重规划解耦。";
        ctx_->spin_reason = reason.str();
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
            ctx_->tactical_state == TacticalState::RESUPPLY || ctx_->need_supply)
        {
            ctx_->desired_supercap_mode = SupercapMode::OFF;
            return BT::NodeStatus::SUCCESS;
        }

        ctx_->desired_supercap_mode = SupercapMode::KEEP;
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
            if (ctx_->dead_return_home_enabled && ctx_->dead_chassis_can_move)
            {
                ctx_->desired_goal = ctx_->dead_return_goal.empty() ? "BASE_HOME"
                                                                    : ctx_->dead_return_goal;
                ctx_->desired_posture = Posture::MOVE;
                ctx_->desired_fire_policy = FirePolicy::CONSERVATIVE;
                ctx_->desired_spin_mode = SpinMode::OFF;
                ctx_->dead_return_home_active = true;
                if (ctx_->dead_return_start_ms == 0)
                {
                    ctx_->dead_return_start_ms = ctx_->now_ms;
                }
                if (ctx_->goal_reason.empty())
                {
                    ctx_->goal_reason = ctx_->dead_return_reason.empty()
                                            ? "死亡回基地硬任务：导航到 BASE_HOME 等待回血。"
                                            : ctx_->dead_return_reason;
                }
            }
            else
            {
                // 只有在底盘不可动或裁判链路不可靠时，WAIT_REVIVE 才作为纯状态输出。
                ctx_->desired_goal = "WAIT_REVIVE";
            }
        }
        else if (ctx_->dead_return_home_active || ctx_->dead_waiting_full_hp)
        {
            ctx_->desired_goal = ctx_->dead_return_goal.empty() ? "BASE_HOME"
                                                                : ctx_->dead_return_goal;
            ctx_->desired_posture = Posture::MOVE;
            ctx_->desired_fire_policy = FirePolicy::CONSERVATIVE;
            ctx_->desired_spin_mode = SpinMode::OFF;
            if (ctx_->goal_reason.empty())
            {
                ctx_->goal_reason =
                    "死亡回基地任务已进入等待回血阶段，满血阈值前不允许恢复普通决策。";
            }
        }
        else if (DwellCanHoldGoal(*ctx_))
        {
            const auto dwell_goal = GoalNameFromProtocolId(ctx_->dwell_goal_id);
            if (!dwell_goal.empty())
            {
                ctx_->desired_goal = dwell_goal;
                std::ostringstream dwell_reason;
                dwell_reason << "到点 dwell 中，剩余 " << ctx_->dwell_remaining_ms
                             << " ms，暂不切换巡航目标。";
                ctx_->goal_reason = dwell_reason.str();
            }
            else if (!ctx_->preferred_goal.empty())
            {
                ctx_->desired_goal = ctx_->preferred_goal;
            }
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
        const float hp_ratio = SafeRatio(ctx_->hp, ctx_->hp_max);
        oss << "goal=" << ctx_->desired_goal
            << ", posture=" << PostureToString(ctx_->desired_posture)
            << ", internal_motion=" << InternalMotionStateToString(ctx_->internal_motion_latched)
            << ", posture_reason=" << ctx_->posture_reason
            << ", fire=" << FirePolicyToString(ctx_->desired_fire_policy)
            << ", fire_at_patrol_hold="
            << ((ctx_->dwell_active && !ctx_->dwell_complete &&
                 ctx_->dwell_required_ms > 0 &&
                 (ctx_->tactical_state == TacticalState::HOLD ||
                  ctx_->tactical_state == TacticalState::SEARCH ||
                  ctx_->tactical_state == TacticalState::REPOSITION))
                    ? "true"
                    : "false")
            << ", spin=" << SpinModeToString(ctx_->desired_spin_mode)
            << ", spin_reason=" << ctx_->spin_reason
            << ", spin_filtered=" << SpinModeToString(ctx_->spin_filtered_mode)
            << ", spin_target_recent="
            << ((ctx_->spin_target_last_seen_ms != 0 &&
                 ElapsedMs(ctx_->now_ms, ctx_->spin_target_last_seen_ms) <=
                     ctx_->spin_target_hold_ms)
                    ? "true"
                    : "false")
            << ", spin_target_last_seen_ms=" << ctx_->spin_target_last_seen_ms
            << ", under_attack=" << (ctx_->under_attack ? "true" : "false")
            << ", armor_id=" << static_cast<int>(ctx_->armor_id)
            << ", hp_deduction_reason=" << static_cast<int>(ctx_->hp_deduction_reason)
            << ", spin_under_attack_recent="
            << ((ctx_->spin_under_attack_last_seen_ms != 0 &&
                 ElapsedMs(ctx_->now_ms, ctx_->spin_under_attack_last_seen_ms) <=
                     ctx_->spin_under_attack_hold_ms)
                    ? "true"
                    : "false")
            << ", spin_under_attack_last_seen_ms=" << ctx_->spin_under_attack_last_seen_ms
            << ", spin_on_since_ms=" << ctx_->spin_preference_on_since_ms
            << ", spin_off_since_ms=" << ctx_->spin_preference_off_since_ms
            << ", spin_last_change_ms=" << ctx_->spin_last_change_ms
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
        oss << ", is_dead=" << (ctx_->is_dead ? "true" : "false")
            << ", dead_return_home_active="
            << (ctx_->dead_return_home_active ? "true" : "false")
            << ", dead_return_goal=" << ctx_->dead_return_goal
            << ", dead_home_rfid_confirmed="
            << (ctx_->dead_home_rfid_confirmed ? "true" : "false")
            << ", dead_waiting_full_hp=" << (ctx_->dead_waiting_full_hp ? "true" : "false")
            << ", hp=" << ctx_->hp
            << ", hp_max=" << ctx_->hp_max
            << ", hp_ratio=" << hp_ratio
            << ", on_base=" << (ctx_->on_base ? "true" : "false")
            << ", on_supply=" << (ctx_->on_supply ? "true" : "false")
            << ", on_fortress=" << (ctx_->on_fortress ? "true" : "false")
            << ", on_outpost=" << (ctx_->on_outpost ? "true" : "false")
            << ", on_highground=" << (ctx_->on_highground ? "true" : "false")
            << ", rfid_status=" << ctx_->rfid_status
            << ", rfid_status_2=" << static_cast<int>(ctx_->rfid_status_2)
            << ", recovery_buff=" << static_cast<int>(ctx_->recovery_buff)
            << ", at_valid_recovery_rfid="
            << (ctx_->at_valid_recovery_rfid ? "true" : "false")
            << ", recovery_confirmed_by=" << ctx_->recovery_confirmed_by
            << ", need_supply=" << (ctx_->need_supply ? "true" : "false")
            << ", hp_recovery_active=" << (ctx_->hp_recovery_active ? "true" : "false")
            << ", ammo_recovery_active=" << (ctx_->ammo_recovery_active ? "true" : "false")
            << ", resupply_goal_current=" << ctx_->resupply_goal_current
            << ", resupply_rfid_confirmed="
            << (ctx_->resupply_rfid_confirmed ? "true" : "false")
            << ", resupply_waiting_recovery="
            << (ctx_->resupply_waiting_recovery ? "true" : "false")
            << ", resupply_reason=" << ctx_->resupply_reason
            << ", dwell_active=" << (ctx_->dwell_active ? "true" : "false")
            << ", dwell_complete=" << (ctx_->dwell_complete ? "true" : "false")
            << ", dwell_required_ms=" << ctx_->dwell_required_ms
            << ", dwell_remaining_ms=" << ctx_->dwell_remaining_ms
            << ", dead_return_reason=" << ctx_->dead_return_reason;
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
