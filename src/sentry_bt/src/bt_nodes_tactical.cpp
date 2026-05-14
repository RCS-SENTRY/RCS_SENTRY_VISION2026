#include "bt_nodes_tactical.hpp"

#include <algorithm>

#include "bt_node_common.hpp"

// tactical 节点文件负责“给出战术意图”。
// 这一层通常不直接下发最终控制命令，而是完成三件事：
// 1. 评估当前应该处于哪种 tactical_state；
// 2. 生成候选目标点并打分；
// 3. 给出 posture / fire / spin / supercap 等“理想偏好”。
//
// 最终是否能完全按这些偏好执行，要交给 executor 层二次收口。

namespace
{
// AdviceBonus 用于把外部建议源（当前是 LLMAdvice 占位）折算为一个轻量加分项。
// 这里故意只给很小的加分，避免软建议直接压过硬逻辑。
float AdviceBonus(const RobotContext& ctx, const std::string& goal_id)
{
    (void)ctx;
    (void)goal_id;
    return 0.0f;
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

std::uint64_t ElapsedSince(std::uint64_t now_ms, std::uint64_t then_ms)
{
    return (then_ms == 0 || now_ms <= then_ms) ? 0 : (now_ms - then_ms);
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

bool TargetWithinEngageDistance(const RobotContext& ctx)
{
    const bool target_seen = ctx.autoaim_fire_ready || ctx.autoaim_tracking ||
                             ctx.autoaim_has_target ||
                             (ctx.enemy_in_view && ctx.enemy_confidence >= 0.45f);
    const float target_distance =
        ctx.autoaim_target_distance > 0.0f ? ctx.autoaim_target_distance : ctx.enemy_distance_m;
    return target_seen && target_distance > 0.0f &&
           target_distance <= ctx.engage_target_max_distance_m;
}

bool CanPreemptTacticalState(TacticalState state)
{
    return state == TacticalState::RETREAT || state == TacticalState::RESUPPLY ||
           state == TacticalState::SEARCH;
}

void SwitchToNextResupplyCandidate(RobotContext& ctx, const std::string& reason)
{
    if (ctx.resupply_candidates.empty())
    {
        ctx.resupply_goal_current = "SUPPLY_LEFT";
        ctx.resupply_reason = reason + " 候选列表为空，fallback=SUPPLY_LEFT。";
        return;
    }

    ctx.resupply_candidate_index =
        (ctx.resupply_candidate_index + 1) %
        static_cast<int>(ctx.resupply_candidates.size());
    ctx.resupply_goal_current =
        ctx.resupply_candidates[static_cast<std::size_t>(ctx.resupply_candidate_index)];
    ctx.resupply_goal_start_ms = ctx.now_ms;
    ctx.resupply_last_candidate_switch_ms = ctx.now_ms;
    ctx.resupply_rfid_confirm_ms = 0;
    ctx.resupply_rfid_confirmed = false;
    ctx.resupply_waiting_recovery = false;
    ctx.resupply_reason = reason + " 切换到候选补给点 " + ctx.resupply_goal_current + "。";
}

SpinMode DecideSituationalSpinPreference(const RobotContext& ctx, std::string& reason)
{
    if (!ctx.match_started)
    {
        reason = "比赛未开始，不启用小陀螺。";
        return SpinMode::OFF;
    }
    if (!ctx.referee_link_fresh)
    {
        reason = "输入链路超时，小陀螺偏好保持关闭，等待 executor 进入安全输出。";
        return SpinMode::OFF;
    }
    if (ctx.is_dead)
    {
        reason = "机器人已死亡，不启用小陀螺。";
        return SpinMode::OFF;
    }
    if (ctx.need_supply || ctx.tactical_state == TacticalState::RESUPPLY)
    {
        reason = "补给闭环中关闭小陀螺，便于稳定进入 RFID 区。";
        return SpinMode::OFF;
    }

    reason = "巡航/驻留/接敌/撤退/避障阶段保持小陀螺开启，普通导航状态不抢占 spin。";
    return SpinMode::ON;
}

void ApplySituationalSpinPreference(RobotContext& ctx)
{
    ctx.preferred_spin_mode = DecideSituationalSpinPreference(ctx, ctx.spin_reason);
}

// 战术态评估节点。
// 它是整套 tactical 层里最关键的分发入口：
// 先决定当前属于哪种 tactical_state，
// 后面的 Switch6 才知道该进入哪个 subtree。
//
// 如果你后面要大改整体战术风格，优先看这个节点。
class EvaluateTacticalStateNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("tactical_state")};
    }

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        const float hp_ratio = SafeRatio(ctx_->hp, ctx_->hp_max);
        const auto previous_state = ctx_->tactical_state;
        if (ctx_->tactical_state_enter_ms == 0)
        {
            ctx_->tactical_state_enter_ms = ctx_->now_ms;
        }

        if (!ctx_->match_started)
        {
            ctx_->tactical_state = TacticalState::SEARCH;
            ctx_->tactical_reason = "比赛未开始，保持 SEARCH/安全待机；fire/spin 由 executor 关闭。";
        }
        else if (!ctx_->referee_link_fresh)
        {
            ctx_->tactical_state = TacticalState::RETREAT;
            ctx_->tactical_reason = ctx_->input_health_reason;
        }
        else if (ctx_->is_dead)
        {
            ctx_->tactical_state = TacticalState::RETREAT;
            ctx_->tactical_reason =
                ctx_->dead_return_home_enabled && ctx_->dead_chassis_can_move
                    ? "死亡但底盘默认可动，进入回基地补满血硬任务；复活确认分支并行保留。"
                    : "当前已判定为死亡状态，应立即让复活分支接管。";
        }
        else if (ctx_->need_emergency_safety)
        {
            ctx_->tactical_state = TacticalState::RETREAT;
            ctx_->tactical_reason = "真正紧急安全状态触发，进入 RETREAT。";
        }
        else if (ctx_->need_supply)
        {
            ctx_->tactical_state = TacticalState::RESUPPLY;
            ctx_->tactical_reason =
                "低血/低弹已进入补给闭环，优先级高于普通搜索和交战。";
        }
        else if (hp_ratio < ctx_->hp_resupply_exit_ratio)
        {
            ctx_->tactical_state = TacticalState::RESUPPLY;
            ctx_->tactical_reason =
                "血量未恢复到健康阈值，优先回补给点直到血量补上。";
        }
        else
        {
            ctx_->tactical_state = TacticalState::SEARCH;
            ctx_->tactical_reason =
                "常规状态保持 SEARCH 三点巡航；目标出现只作为并行自瞄/姿态候选输入，不抢导航。";
        }

        const auto proposed_state = ctx_->tactical_state;
        const auto proposed_reason = ctx_->tactical_reason;
        if (proposed_state != previous_state)
        {
            const auto held_ms = ElapsedSince(ctx_->now_ms, ctx_->tactical_state_enter_ms);
            const bool can_switch =
                held_ms >= ctx_->tactical_state_min_hold_ms ||
                CanPreemptTacticalState(proposed_state);
            if (can_switch)
            {
                ctx_->tactical_state_enter_ms = ctx_->now_ms;
            }
            else
            {
                ctx_->tactical_state = previous_state;
                ctx_->tactical_reason =
                    proposed_reason + " 但战术态最小保持窗未结束，继续保持 " +
                    TacticalStateToString(previous_state) + "，remaining_ms=" +
                    std::to_string(ctx_->tactical_state_min_hold_ms - held_ms) + "。";
            }
        }

        setOutput("tactical_state", std::string(TacticalStateToString(ctx_->tactical_state)));
        return BT::NodeStatus::SUCCESS;
    }
};

// 清空候选目标列表。
// 每个 tactical subtree 在评估自己的目标点前，都应该先执行一次这个节点，
// 避免上一个 subtree 的候选残留到本轮。
class ResetGoalCandidatesNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->goal_candidates.clear();
        ctx_->goal_reason.clear();
        return BT::NodeStatus::SUCCESS;
    }
};

// 在当前候选集中选择分数最高的目标点。
// 注意：这个节点只做“挑最好”，不负责“怎么算分”。
// 真正的打分逻辑分散在各个 Evaluate*Goals 节点中。
class ChooseBestGoalNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        if (ctx_->goal_candidates.empty())
        {
            ctx_->goal_reason = "当前激活的战术子树没有产出任何候选目标点。";
            return BT::NodeStatus::FAILURE;
        }

        auto best = ctx_->goal_candidates.end();
        bool avoided_failed_goal = false;
        for (auto it = ctx_->goal_candidates.begin(); it != ctx_->goal_candidates.end(); ++it)
        {
            const bool is_current_failed_goal =
                ctx_->nav_goal_failed && ctx_->current_goal_id != 0 &&
                GoalNameToProtocolId(it->id) == ctx_->current_goal_id;
            if (is_current_failed_goal && ctx_->goal_candidates.size() > 1)
            {
                avoided_failed_goal = true;
                continue;
            }
            if (best == ctx_->goal_candidates.end() || it->score > best->score)
            {
                best = it;
            }
        }

        if (best == ctx_->goal_candidates.end())
        {
            best = std::max_element(
                ctx_->goal_candidates.begin(), ctx_->goal_candidates.end(),
                [](const GoalCandidate& lhs, const GoalCandidate& rhs) {
                    return lhs.score < rhs.score;
                });
        }

        ctx_->preferred_goal = best->id;
        ctx_->goal_reason = best->rationale.empty()
                                ? "在当前候选列表中选择得分最高的目标点。"
                                : best->rationale;
        if (avoided_failed_goal)
        {
            ctx_->goal_reason += " 当前目标被导航执行器标记为卡住/失败，本轮切换到备用候选点。";
        }
        return BT::NodeStatus::SUCCESS;
    }
};

// 下面四个 SetPreferred* 节点是“参数型节点”。
// 它们最大的意义在于：很多简单策略可以直接在 XML 中通过写参数完成，
// 不必为了一个常量赋值动作专门新增 C++ 节点。
class SetPreferredPostureNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("posture")};
    }

    BT::NodeStatus tick() override
    {
        const auto posture_value = getInput<std::string>("posture");
        if (!posture_value)
        {
            return BT::NodeStatus::FAILURE;
        }

        Posture posture = Posture::MOVE;
        if (!ParsePosture(posture_value.value(), posture))
        {
            return BT::NodeStatus::FAILURE;
        }

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->preferred_posture = posture;
        return BT::NodeStatus::SUCCESS;
    }
};

class SetPreferredFirePolicyNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("fire_policy")};
    }

    BT::NodeStatus tick() override
    {
        const auto policy_value = getInput<std::string>("fire_policy");
        if (!policy_value)
        {
            return BT::NodeStatus::FAILURE;
        }

        FirePolicy policy = FirePolicy::NORMAL;
        if (!ParseFirePolicy(policy_value.value(), policy))
        {
            return BT::NodeStatus::FAILURE;
        }

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->preferred_fire_policy = policy;
        return BT::NodeStatus::SUCCESS;
    }
};

class SetPreferredSpinModeNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("spin_mode")};
    }

    BT::NodeStatus tick() override
    {
        const auto spin_value = getInput<std::string>("spin_mode");
        if (!spin_value)
        {
            return BT::NodeStatus::FAILURE;
        }

        SpinMode mode = SpinMode::OFF;
        if (!ParseSpinMode(spin_value.value(), mode))
        {
            return BT::NodeStatus::FAILURE;
        }

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->preferred_spin_mode = mode;
        ctx_->spin_reason = std::string("战术子树显式设置小陀螺为 ") +
                            SpinModeToString(mode) + "。";
        return BT::NodeStatus::SUCCESS;
    }
};

class SetPreferredSupercapModeNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("supercap_mode")};
    }

    BT::NodeStatus tick() override
    {
        const auto mode_value = getInput<std::string>("supercap_mode");
        if (!mode_value)
        {
            return BT::NodeStatus::FAILURE;
        }

        SupercapMode mode = SupercapMode::OFF;
        if (!ParseSupercapMode(mode_value.value(), mode))
        {
            return BT::NodeStatus::FAILURE;
        }

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->preferred_supercap_mode = mode;
        return BT::NodeStatus::SUCCESS;
    }
};

class SetCombatPosturePreferenceNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        ctx_->preferred_posture = Posture::ATTACK;
        return BT::NodeStatus::SUCCESS;
    }
};

// 交战时的自瞄开关更新。
// fire_policy 在下位机协议里作为自瞄开关：1=OFF，2=ON。
class UpdateCombatFirePolicyNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        ctx_->preferred_fire_policy = FirePolicy::NORMAL;

        return BT::NodeStatus::SUCCESS;
    }
};

// 统一的小陀螺偏好更新。
// 小陀螺不再只依赖“近距离交战”一个条件，而是综合战术态、敌情和底盘资源。
class SetSituationalSpinPreferenceNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ApplySituationalSpinPreference(*ctx_);
        return BT::NodeStatus::SUCCESS;
    }
};

// 兼容旧 XML 名称：交战子树如果还使用这个节点，也走同一套统一策略。
class SetCombatSpinPreferenceNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ApplySituationalSpinPreference(*ctx_);
        return BT::NodeStatus::SUCCESS;
    }
};

// 交战时的超级电容使用偏好。
// 这里的思路是：距离较远且电量较高时更值得爆发，否则以 KEEP 为主。
class SetCombatSupercapPreferenceNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->preferred_supercap_mode = SupercapMode::KEEP;
        return BT::NodeStatus::SUCCESS;
    }
};

// 下面这些 Evaluate*Goals 节点共同遵循同一种模式：
// 1. 根据当前 tactical_state 生成若干候选目标点；
// 2. 给每个目标点附带一个分数；
// 3. 记录一段中文理由，方便后续理解“为什么这个点被选中”。
//
// 你后续最常做的改动，大概率就是在这些节点里调分数、增删候选点、
// 或引入更复杂的环境特征。
class EvaluateRetreatGoalsNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        AddGoalCandidate(*ctx_, "SAFE_RETREAT_A", 1.00f,
                         "撤退子树选择了预定义候选中最安全的回退点。");
        AddGoalCandidate(*ctx_, "SAFE_RETREAT_B", ctx_->enemy_in_view ? 0.78f : 0.90f,
                         "如果主撤退路线受压，备用撤退点仍然可用。");
        return BT::NodeStatus::SUCCESS;
    }
};

class EvaluateResupplyGoalsNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        if (!ctx_->need_supply)
        {
            ctx_->resupply_rfid_confirmed = false;
            ctx_->resupply_waiting_recovery = false;
            ctx_->resupply_reason = "资源已恢复到退出阈值，退出补给闭环。";
            return BT::NodeStatus::SUCCESS;
        }

        if (ctx_->resupply_goal_current.empty())
        {
            ctx_->resupply_goal_current = ctx_->resupply_candidates.empty()
                                              ? std::string("SUPPLY_LEFT")
                                              : ctx_->resupply_candidates.front();
        }

        if (ctx_->at_valid_recovery_rfid)
        {
            if (ctx_->on_base)
            {
                ctx_->resupply_goal_current = "BASE_HOME";
            }
            if (ctx_->resupply_rfid_confirm_ms == 0)
            {
                ctx_->resupply_rfid_confirm_ms = ctx_->now_ms;
            }
            const bool hold_ok =
                ElapsedSince(ctx_->now_ms, ctx_->resupply_rfid_confirm_ms) >=
                ctx_->resupply_rfid_confirm_hold_ms;
            ctx_->resupply_rfid_confirmed = hold_ok;
            ctx_->resupply_waiting_recovery = hold_ok;
            ctx_->resupply_reason =
                "RFID 确认恢复点，来源=" + ctx_->recovery_confirmed_by +
                "，停留等待血量/弹量恢复；本次热修不因 RFID 已确认后的等待超时切候选。";
            AddGoalCandidate(*ctx_, ctx_->resupply_goal_current, 2.0f,
                             ctx_->resupply_reason);
            return BT::NodeStatus::SUCCESS;
        }

        ctx_->resupply_rfid_confirm_ms = 0;
        ctx_->resupply_rfid_confirmed = false;
        ctx_->resupply_waiting_recovery = false;

        const bool current_candidate_failed =
            ctx_->nav_goal_failed && ctx_->current_goal_id != 0 &&
            GoalNameToProtocolId(ctx_->resupply_goal_current) == ctx_->current_goal_id;
        const bool current_candidate_reached_without_rfid =
            ctx_->nav_goal_reached && ctx_->current_goal_id != 0 &&
            GoalNameToProtocolId(ctx_->resupply_goal_current) == ctx_->current_goal_id;
        const bool candidate_timeout =
            ctx_->resupply_goal_timeout_ms > 0 &&
            ElapsedSince(ctx_->now_ms, ctx_->resupply_goal_start_ms) >=
                ctx_->resupply_goal_timeout_ms;
        const bool switch_cooldown_ok =
            ElapsedSince(ctx_->now_ms, ctx_->resupply_last_candidate_switch_ms) >=
            ctx_->resupply_candidate_switch_cooldown_ms;

        if ((current_candidate_failed || current_candidate_reached_without_rfid ||
             candidate_timeout) &&
            switch_cooldown_ok)
        {
            if (current_candidate_failed)
            {
                SwitchToNextResupplyCandidate(*ctx_, "当前补给候选导航失败");
            }
            else if (current_candidate_reached_without_rfid)
            {
                SwitchToNextResupplyCandidate(
                    *ctx_, "已到达补给候选点但 RFID 未确认恢复区");
            }
            else
            {
                SwitchToNextResupplyCandidate(*ctx_, "补给候选点导航超时");
            }
        }
        else
        {
            ctx_->resupply_reason =
                "需要补给但 RFID 未确认，导航到候选恢复点 " +
                ctx_->resupply_goal_current + "。";
        }

        AddGoalCandidate(*ctx_, ctx_->resupply_goal_current, 2.0f, ctx_->resupply_reason);
        return BT::NodeStatus::SUCCESS;
    }
};

class EvaluateHoldGoalsNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        const auto goal = CurrentPatrolGoal(*ctx_);
        AddGoalCandidate(*ctx_, goal, 1.00f + AdviceBonus(*ctx_, goal),
                         "HOLD 态也沿用固定巡航防守序列：" + goal + "。");
        return BT::NodeStatus::SUCCESS;
    }
};

class EvaluateEngageGoalsNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        const auto goal = CurrentPatrolGoal(*ctx_);
        AddGoalCandidate(
            *ctx_, goal, 1.00f,
            "交战态不抢导航，不发布 CURRENT_HOLD；继续保持当前巡航/导航目标并并行自瞄。");

        return BT::NodeStatus::SUCCESS;
    }
};

class EvaluateSearchGoalsNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        const auto goal = CurrentPatrolGoal(*ctx_);
        AddGoalCandidate(*ctx_, goal, 1.00f + AdviceBonus(*ctx_, goal),
                         "按固定顺序巡航防守，当前目标=" + goal + "。");
        return BT::NodeStatus::SUCCESS;
    }
};

class EvaluateRepositionGoalsNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        const auto goal = CurrentPatrolGoal(*ctx_);
        AddGoalCandidate(*ctx_, goal, 1.00f + AdviceBonus(*ctx_, goal),
                         "REPOSITION 态沿用固定巡航防守序列，当前目标=" + goal + "。");
        return BT::NodeStatus::SUCCESS;
    }
};
}  // 匿名命名空间

void RegisterTacticalNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx)
{
    // tactical 节点注册表。
    // 如果 XML 中某个战术节点创建失败，优先检查这里是否漏注册、节点名是否一致。
    RegisterContextNode<EvaluateTacticalStateNode>(factory, "EvaluateTacticalState", ctx);
    RegisterContextNode<ResetGoalCandidatesNode>(factory, "ResetGoalCandidates", ctx);
    RegisterContextNode<ChooseBestGoalNode>(factory, "ChooseBestGoal", ctx);
    RegisterContextNode<SetPreferredPostureNode>(factory, "SetPreferredPosture", ctx);
    RegisterContextNode<SetPreferredFirePolicyNode>(factory, "SetPreferredFirePolicy", ctx);
    RegisterContextNode<SetPreferredSpinModeNode>(factory, "SetPreferredSpinMode", ctx);
    RegisterContextNode<SetPreferredSupercapModeNode>(factory, "SetPreferredSupercapMode", ctx);
    RegisterContextNode<SetCombatPosturePreferenceNode>(factory, "SetCombatPosturePreference", ctx);
    RegisterContextNode<UpdateCombatFirePolicyNode>(factory, "UpdateCombatFirePolicy", ctx);
    RegisterContextNode<SetSituationalSpinPreferenceNode>(factory, "SetSituationalSpinPreference", ctx);
    RegisterContextNode<SetCombatSpinPreferenceNode>(factory, "SetCombatSpinPreference", ctx);
    RegisterContextNode<SetCombatSupercapPreferenceNode>(factory, "SetCombatSupercapPreference", ctx);
    RegisterContextNode<EvaluateRetreatGoalsNode>(factory, "EvaluateRetreatGoals", ctx);
    RegisterContextNode<EvaluateResupplyGoalsNode>(factory, "EvaluateResupplyGoals", ctx);
    RegisterContextNode<EvaluateHoldGoalsNode>(factory, "EvaluateHoldGoals", ctx);
    RegisterContextNode<EvaluateEngageGoalsNode>(factory, "EvaluateEngageGoals", ctx);
    RegisterContextNode<EvaluateSearchGoalsNode>(factory, "EvaluateSearchGoals", ctx);
    RegisterContextNode<EvaluateRepositionGoalsNode>(factory, "EvaluateRepositionGoals", ctx);
}
