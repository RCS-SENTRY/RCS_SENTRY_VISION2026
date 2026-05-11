#include "bt_nodes_basic.hpp"

#include <algorithm>
#include <sstream>

#include "bt_node_common.hpp"
#include "sentry_decision_protocol.h"

// basic 节点文件存放“最基础、最通用”的行为树节点。
// 这些节点通常不承载复杂战术逻辑，而是负责：
// 1. 每个 tick 的上下文整理；
// 2. 最基础的状态判断；
// 3. 提供少量通用动作节点，方便在 XML 中直接复用。

namespace
{
bool UpdateHysteresis(bool previous, bool enter, bool exit)
{
    return previous ? !exit : enter;
}

std::uint64_t ElapsedSince(std::uint64_t now_ms, std::uint64_t then_ms)
{
    return (then_ms == 0 || now_ms <= then_ms) ? 0 : (now_ms - then_ms);
}

std::uint64_t DwellRequiredForState(const RobotContext& ctx)
{
    switch (ctx.tactical_state)
    {
        case TacticalState::HOLD:
            return ctx.goal_dwell_hold_ms;
        case TacticalState::SEARCH:
        case TacticalState::REPOSITION:
            return ctx.goal_dwell_search_ms;
        case TacticalState::RESUPPLY:
            return ctx.goal_dwell_resupply_ms;
        case TacticalState::ENGAGE:
            return ctx.goal_dwell_engage_ms;
        case TacticalState::RETREAT:
            return 0;
    }
    return ctx.goal_dwell_default_ms;
}

// UpdateBlackboardNode 是整棵主树的起始节点。
// 它的核心作用不是“从外部取数”，而是把接口层已经写入的输入快照
// 重新整理成适合本轮决策使用的中间状态。
//
// 你后续如果要扩展新的守卫标志、解释字段或临时缓存，
// 通常都应该先想一想是否要在这里重置。
class UpdateBlackboardNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        // 这是每次 BT tick 的入口节点。
        // 它本身不直接访问外部 IO，而是负责整理接口层已经写入的最新快照。
        ++ctx_->bt_tick_index;

        // 这些字段属于“本轮决策中间态”，必须在每个 tick 开头清空，
        // 否则会残留上一轮的结果，导致行为树看起来像“记错状态”。
        ctx_->goal_candidates.clear();
        ctx_->rule_action_type = RuleActionType::NONE;
        ctx_->posture_cooldown_guard_active = false;
        ctx_->rule_cmd_guard_active = false;

        // 先以上一轮执行结果作为默认基线。
        // 这样当前激活的 subtree 只需要覆写自己关心的部分即可。
        ctx_->preferred_goal = ctx_->desired_goal;
        ctx_->preferred_posture = ctx_->desired_posture;
        ctx_->preferred_fire_policy = ctx_->desired_fire_policy;
        ctx_->preferred_spin_mode = ctx_->desired_spin_mode;
        ctx_->preferred_supercap_mode = ctx_->desired_supercap_mode;

        // 便捷状态来自原始输入的快速归纳。
        // 这些值不是底层传感器直接给出的，而是为了后面的节点判断更方便。
        ctx_->ammo_low = UpdateHysteresis(ctx_->ammo_low,
                                          ctx_->ammo_17 < ctx_->ammo_resupply_enter_count,
                                          ctx_->ammo_17 > ctx_->ammo_resupply_exit_count);
        const float hp_ratio = SafeRatio(ctx_->hp, ctx_->hp_max);
        ctx_->at_valid_recovery_rfid = ctx_->on_supply || ctx_->on_base;
        ctx_->hp_recovery_active = UpdateHysteresis(
            ctx_->hp_recovery_active, hp_ratio < ctx_->hp_resupply_enter_ratio,
            hp_ratio > ctx_->hp_resupply_exit_ratio);
        ctx_->ammo_recovery_active = UpdateHysteresis(
            ctx_->ammo_recovery_active, ctx_->ammo_17 < ctx_->ammo_resupply_enter_count,
            ctx_->ammo_17 > ctx_->ammo_resupply_exit_count);
        if (ctx_->is_dead)
        {
            ctx_->hp_recovery_active = false;
            ctx_->ammo_recovery_active = false;
        }
        ctx_->resupply_active = ctx_->hp_recovery_active || ctx_->ammo_recovery_active;
        ctx_->need_supply = ctx_->resupply_active;
        ctx_->need_emergency_safety = !ctx_->referee_link_fresh;
        ctx_->need_rule_action = false;

        if (!ctx_->resupply_active)
        {
            ctx_->resupply_enter_ms = 0;
            ctx_->resupply_reached_ms = 0;
            ctx_->resupply_rfid_confirm_ms = 0;
            ctx_->resupply_last_candidate_switch_ms = 0;
            ctx_->resupply_goal_start_ms = 0;
            ctx_->resupply_candidate_index = 0;
            ctx_->resupply_goal_current = ctx_->resupply_candidates.empty()
                                              ? std::string("SUPPLY_LEFT")
                                              : ctx_->resupply_candidates.front();
            ctx_->resupply_rfid_confirmed = false;
            ctx_->resupply_waiting_recovery = false;
            ctx_->resupply_reason.clear();
        }
        else if (ctx_->resupply_enter_ms == 0)
        {
            ctx_->resupply_enter_ms = ctx_->now_ms;
            ctx_->resupply_goal_start_ms = ctx_->now_ms;
            ctx_->resupply_goal_current = ctx_->resupply_candidates.empty()
                                              ? std::string("SUPPLY_LEFT")
                                              : ctx_->resupply_candidates.front();
            ctx_->resupply_reason = "低血/低弹进入补给闭环。";
        }

        ctx_->dead_chassis_can_move = ctx_->match_started && ctx_->referee_link_fresh;
        if (ctx_->dead_return_home_active && hp_ratio >= ctx_->dead_full_hp_exit_ratio &&
            ctx_->hp > 0)
        {
            ctx_->dead_return_home_active = false;
            ctx_->dead_home_rfid_confirmed = false;
            ctx_->dead_waiting_full_hp = false;
            ctx_->dead_return_start_ms = 0;
            ctx_->dead_home_rfid_confirm_ms = 0;
            ctx_->dead_return_reason = "血量已恢复到退出阈值，死亡回家硬任务结束。";
        }
        else if (ctx_->is_dead && ctx_->dead_return_home_enabled && ctx_->dead_chassis_can_move)
        {
            if (!ctx_->dead_return_home_active)
            {
                ctx_->dead_return_start_ms = ctx_->now_ms;
                ctx_->dead_home_rfid_confirm_ms = 0;
            }
            ctx_->dead_return_home_active = true;
            ctx_->dead_return_reason =
                "机器人死亡但裁判链路新鲜，按热修规则硬导航回基地/补给区等待回血。";

            const bool at_dead_recovery = ctx_->on_base || ctx_->on_supply;
            if (at_dead_recovery)
            {
                if (ctx_->dead_home_rfid_confirm_ms == 0)
                {
                    ctx_->dead_home_rfid_confirm_ms = ctx_->now_ms;
                }
                const bool hold_ok =
                    ElapsedSince(ctx_->now_ms, ctx_->dead_home_rfid_confirm_ms) >=
                    ctx_->dead_return_rfid_confirm_hold_ms;
                ctx_->dead_home_rfid_confirmed = hold_ok;
                ctx_->dead_waiting_full_hp = hold_ok;
                ctx_->dead_return_reason += ctx_->on_base
                                                ? " RFID=on_base，等待回血。"
                                                : " RFID=on_supply，作为恢复点等待回血。";
            }
            else
            {
                ctx_->dead_home_rfid_confirm_ms = 0;
                ctx_->dead_home_rfid_confirmed = false;
                ctx_->dead_waiting_full_hp = false;
            }
        }
        else if (!ctx_->is_dead && !ctx_->dead_waiting_full_hp)
        {
            ctx_->dead_return_home_active = false;
            ctx_->dead_home_rfid_confirmed = false;
            ctx_->dead_return_start_ms = 0;
            ctx_->dead_home_rfid_confirm_ms = 0;
        }

        const bool nav_goal_changed = ctx_->current_goal_id != ctx_->last_seen_nav_goal_id;
        const bool reached_edge = ctx_->nav_goal_reached && !ctx_->last_nav_goal_reached;
        if (nav_goal_changed)
        {
            ctx_->dwell_active = false;
            ctx_->dwell_complete = false;
            ctx_->dwell_start_ms = 0;
            ctx_->dwell_goal_id = 0;
            ctx_->dwell_remaining_ms = 0;
            ctx_->dwell_reason.clear();
        }
        if (ctx_->nav_goal_reached && ctx_->current_goal_id != 0 &&
            (reached_edge || nav_goal_changed))
        {
            ctx_->dwell_goal_id = ctx_->current_goal_id;
            ctx_->dwell_start_ms = ctx_->now_ms;
            ctx_->dwell_active = true;
            ctx_->dwell_complete = false;
            ctx_->dwell_reason = "导航到点，开始 sentry_bt dwell。";
        }
        if (ctx_->dwell_active)
        {
            ctx_->dwell_required_ms = DwellRequiredForState(*ctx_);
            const auto elapsed = ElapsedSince(ctx_->now_ms, ctx_->dwell_start_ms);
            ctx_->dwell_complete = elapsed >= ctx_->dwell_required_ms;
            ctx_->dwell_remaining_ms =
                ctx_->dwell_complete ? 0 : (ctx_->dwell_required_ms - elapsed);
        }
        ctx_->last_seen_nav_goal_id = ctx_->current_goal_id;
        ctx_->last_nav_goal_reached = ctx_->nav_goal_reached;

        // 清理本轮要重新生成的解释性文本。
        ctx_->tactical_reason.clear();
        ctx_->rule_reason.clear();
        ctx_->goal_reason.clear();
        ctx_->posture_reason.clear();
        ctx_->spin_reason.clear();
        ctx_->executor_summary.clear();
        ctx_->last_rule_command.clear();
        ctx_->revive_cmd = SENTRY_REVIVE_CMD_NONE;
        ctx_->remote_ammo_req_inc = 0;
        ctx_->remote_hp_req_inc = 0;
        ctx_->posture_cmd_referee = 0;
        ctx_->activate_energy_confirm = 0;
        ctx_->claim_periodic_ammo = 0;

        return BT::NodeStatus::SUCCESS;
    }
};

// IsDeadNode 是一个纯条件节点。
// 在 XML 里它主要用于把“复活分支”单独拉出来。
// 这里保持极简即可，不要在这个节点里顺手改其他状态，
// 否则会让“判断”和“副作用”耦合，后续很难调试。
class IsDeadNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        return ctx_->is_dead ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// NavigateToGoalNode 是一个通用的“直接指定目标点”节点。
// 当前主树里没有大规模依赖它，但保留这个节点有两个价值：
// 1. 以后如果某个分支要直接跳特定目标点，不必再写新节点；
// 2. 调试时可以在 XML 里临时插入它，验证导航链路是否正常。
class NavigateToGoalNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("goal")};
    }

    BT::NodeStatus tick() override
    {
        const auto goal = getInput<std::string>("goal");
        if (!goal)
        {
            return BT::NodeStatus::FAILURE;
        }

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->desired_goal = goal.value();
        ctx_->goal_reason = "目标点由 NavigateToGoal 节点直接覆盖指定。";
        return BT::NodeStatus::SUCCESS;
    }
};
}  // 匿名命名空间

void RegisterBasicNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx)
{
    // 这里维护“XML 节点名 -> C++ 类型”的注册关系。
    // 如果你新增了 basic 类节点，但忘了在这里注册，
    // XML 能写出来，但运行时 createTreeFromFile 会失败。
    RegisterContextNode<UpdateBlackboardNode>(factory, "UpdateBlackboard", ctx);
    RegisterContextNode<IsDeadNode>(factory, "IsDead", ctx);
    RegisterContextNode<NavigateToGoalNode>(factory, "NavigateToGoal", ctx);
}
