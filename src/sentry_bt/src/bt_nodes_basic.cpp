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

std::uint8_t GoalNameToProtocolId(const std::string& goal_id)
{
    if (goal_id == "CURRENT_HOLD") return 0;
    if (goal_id == "SAFE_HOLD") return 1;
    if (goal_id == "WAIT_REVIVE") return 2;
    if (goal_id == "SAFE_RETREAT_A") return 3;
    if (goal_id == "SAFE_RETREAT_B") return 4;
    if (goal_id == "SUPPLY_LEFT") return 5;
    if (goal_id == "SUPPLY_RIGHT") return 6;
    if (goal_id == "FORTRESS_HOLD") return 7;
    if (goal_id == "OUTPOST_HOLD") return 8;
    if (goal_id == "SEARCH_AREA_A") return 14;
    if (goal_id == "SEARCH_AREA_B") return 15;
    if (goal_id == "MID_CROSS") return 18;
    if (goal_id == "BASE_HOME") return 19;
    if (goal_id == "BASE_HOLD") return 20;
    return 0;
}

std::string CurrentPatrolGoal(const RobotContext& ctx)
{
    if (ctx.patrol_goals.empty())
    {
        return "SEARCH_AREA_A";
    }
    const auto index = static_cast<std::size_t>(
        std::clamp(ctx.patrol_goal_index, 0,
                   static_cast<int>(ctx.patrol_goals.size()) - 1));
    return ctx.patrol_goals[index];
}

void AdvancePatrolGoal(RobotContext& ctx, const std::string& reason)
{
    if (ctx.patrol_goals.empty())
    {
        ctx.patrol_goal_index = 0;
        ctx.patrol_reason = reason + " 巡航序列为空，fallback=SEARCH_AREA_A。";
        return;
    }
    ctx.patrol_goal_index =
        (ctx.patrol_goal_index + 1) % static_cast<int>(ctx.patrol_goals.size());
    ctx.patrol_reason =
        reason + " 下一个巡航防守点=" + CurrentPatrolGoal(ctx) + "。";
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
        const bool damage_status_valid = ctx_->match_started && ctx_->referee_link_fresh &&
                                         ctx_->hp > 0;
        const bool projectile_damage_reason =
            damage_status_valid && ctx_->armor_id != 0 && ctx_->hp_deduction_reason == 0;
        const bool collision_or_offline_damage =
            damage_status_valid &&
            (ctx_->hp_deduction_reason == 1 || ctx_->hp_deduction_reason == 5);
        bool hp_drop_hit_event = false;
        if (damage_status_valid)
        {
            if (ctx_->spin_hp_observation_initialized)
            {
                const int hp_drop = ctx_->spin_last_observed_hp - ctx_->hp;
                hp_drop_hit_event = hp_drop > ctx_->spin_hp_drop_threshold;
                if (hp_drop > 0 && collision_or_offline_damage)
                {
                    ctx_->safety_interrupt_last_seen_ms = ctx_->now_ms;
                    ctx_->safety_interrupt_reason =
                        ctx_->hp_deduction_reason == 5
                            ? "碰撞扣血：记录避障风险，保持默认目标规划。"
                            : "模块离线扣血：记录传感器风险，保持默认目标规划。";
                }
            }
            ctx_->spin_last_observed_hp = ctx_->hp;
            ctx_->spin_hp_observation_initialized = true;
        }
        else if (!ctx_->referee_link_fresh || ctx_->hp <= 0)
        {
            ctx_->spin_hp_observation_initialized = false;
            ctx_->spin_last_observed_hp = std::max(0, ctx_->hp);
        }
        ctx_->under_attack =
            (projectile_damage_reason && hp_drop_hit_event) ||
            (ctx_->spin_hp_drop_triggers_under_attack && hp_drop_hit_event &&
             !collision_or_offline_damage);
        if (ctx_->under_attack)
        {
            ctx_->spin_under_attack_last_seen_ms = ctx_->now_ms;
        }

        ctx_->ammo_low = UpdateHysteresis(ctx_->ammo_low,
                                          ctx_->ammo_17 < ctx_->ammo_resupply_enter_count,
                                          ctx_->ammo_17 > ctx_->ammo_resupply_exit_count);
        const float hp_ratio = SafeRatio(ctx_->hp, ctx_->hp_max);
        const bool recovery_rfid_present = ctx_->on_supply || ctx_->on_base;
        const bool recovery_buff_ok =
            !ctx_->require_recovery_buff_for_confirm || ctx_->recovery_buff > 0;
        ctx_->at_valid_recovery_rfid = recovery_rfid_present && recovery_buff_ok;
        if (!recovery_rfid_present || !recovery_buff_ok)
        {
            ctx_->recovery_confirmed_by = "none";
        }
        else if (ctx_->on_base && ctx_->require_recovery_buff_for_confirm)
        {
            ctx_->recovery_confirmed_by = "on_base+recovery_buff";
        }
        else if (ctx_->on_supply && ctx_->require_recovery_buff_for_confirm)
        {
            ctx_->recovery_confirmed_by = "on_supply+recovery_buff";
        }
        else if (ctx_->on_base)
        {
            ctx_->recovery_confirmed_by = "on_base";
        }
        else
        {
            ctx_->recovery_confirmed_by = "on_supply";
        }
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
        if (!ctx_->is_dead && hp_ratio < ctx_->hp_resupply_exit_ratio)
        {
            ctx_->hp_recovery_active = true;
            ctx_->resupply_active = true;
        }
        ctx_->need_supply = ctx_->resupply_active;
        ctx_->need_emergency_safety = !ctx_->referee_link_fresh;
        ctx_->need_rule_action = false;

        if (ctx_->nav_goal_active && ctx_->current_goal_id != 0)
        {
            if (ctx_->nav_goal_active_since_ms == 0 ||
                ctx_->nav_goal_active_tracked_id != ctx_->current_goal_id ||
                ctx_->nav_goal_reached)
            {
                ctx_->nav_goal_active_since_ms = ctx_->now_ms;
                ctx_->nav_goal_active_tracked_id = ctx_->current_goal_id;
            }
        }
        else
        {
            ctx_->nav_goal_active_since_ms = 0;
            ctx_->nav_goal_active_tracked_id = 0;
        }

        if (ctx_->autoaim_tracking)
        {
            if (ctx_->autoaim_tracking_since_ms == 0)
            {
                ctx_->autoaim_tracking_since_ms = ctx_->now_ms;
            }
        }
        else
        {
            ctx_->autoaim_tracking_since_ms = 0;
        }
        if (ctx_->autoaim_fire_ready)
        {
            if (ctx_->autoaim_fire_ready_since_ms == 0)
            {
                ctx_->autoaim_fire_ready_since_ms = ctx_->now_ms;
            }
        }
        else
        {
            ctx_->autoaim_fire_ready_since_ms = 0;
        }
        const bool first_lidar_problem =
            !ctx_->main_lidar_seen ||
            (ctx_->main_lidar_last_seen_ms != 0 &&
             ElapsedSince(ctx_->now_ms, ctx_->main_lidar_last_seen_ms) >
                 ctx_->main_lidar_timeout_ms);
        const bool second_lidar_debug_fresh =
            ctx_->safety_debug_seen && ctx_->safety_debug_last_seen_ms != 0 &&
            ElapsedSince(ctx_->now_ms, ctx_->safety_debug_last_seen_ms) <=
                ctx_->safety_debug_timeout_ms;
        const bool second_lidar_problem =
            ctx_->safety_debug_seen &&
            (!second_lidar_debug_fresh || ctx_->safety_emergency_active ||
             ctx_->safety_obstacle_timeout);
        const bool recent_damage_interrupt =
            ctx_->safety_interrupt_last_seen_ms != 0 &&
            ElapsedSince(ctx_->now_ms, ctx_->safety_interrupt_last_seen_ms) <=
                ctx_->safety_collision_interrupt_ms;
        if (first_lidar_problem)
        {
            ctx_->safety_interrupt_reason =
                ctx_->main_lidar_seen
                    ? "第一雷达点云超时，保持默认目标规划并等待导航避障恢复。"
                    : "尚未收到第一雷达点云，保持默认目标规划并等待导航避障恢复。";
        }
        else if (second_lidar_problem)
        {
            ctx_->safety_interrupt_reason =
                ctx_->safety_emergency_active
                    ? "第二雷达 safety emergency，保持默认目标规划并交给导航避障。"
                    : "第二雷达 safety 点云超时或避障未通过，保持默认目标规划并交给导航避障。";
        }
        ctx_->safety_interrupt_active =
            first_lidar_problem || second_lidar_problem || recent_damage_interrupt ||
            ctx_->nav_goal_failed;
        if (ctx_->nav_goal_failed)
        {
            ctx_->safety_interrupt_reason =
                "当前导航目标失败/避障未通过，交给候选点切换逻辑。";
        }

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

            if (ctx_->at_valid_recovery_rfid)
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
                ctx_->dead_return_reason +=
                    " RFID 确认来源=" + ctx_->recovery_confirmed_by + "，等待回血。";
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
            ctx_->dwell_reason =
                ctx_->nav_goal_failed
                    ? "导航超时等效进入临时战术点，开始 sentry_bt dwell，结束后切备用点。"
                    : "导航到点，开始 sentry_bt dwell。";
        }
        if (ctx_->dwell_active)
        {
            ctx_->dwell_required_ms = DwellRequiredForState(*ctx_);
            const auto elapsed = ElapsedSince(ctx_->now_ms, ctx_->dwell_start_ms);
            ctx_->dwell_complete = elapsed >= ctx_->dwell_required_ms;
            ctx_->dwell_remaining_ms =
                ctx_->dwell_complete ? 0 : (ctx_->dwell_required_ms - elapsed);
            if (ctx_->dwell_complete && ctx_->dwell_goal_id != 0 &&
                !ctx_->is_dead && !ctx_->need_supply && !ctx_->need_emergency_safety)
            {
                const auto patrol_goal = CurrentPatrolGoal(*ctx_);
                if (GoalNameToProtocolId(patrol_goal) == ctx_->dwell_goal_id &&
                    ctx_->patrol_last_advanced_goal_id != ctx_->dwell_goal_id)
                {
                    ctx_->patrol_last_advanced_goal_id = ctx_->dwell_goal_id;
                    AdvancePatrolGoal(
                        *ctx_, ctx_->nav_goal_failed
                                   ? "当前巡航点导航超时等效防守结束"
                                   : "当前巡航点防守 dwell 结束");
                }
            }
        }
        // 普通导航失败/避障重规划不再推进巡航点，也不关闭自瞄/小陀螺。
        // 到点超时由姿态层当作临时防守候选处理，补给候选切换仍在 resupply 子树内完成。
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
