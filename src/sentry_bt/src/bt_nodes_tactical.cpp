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
    if (!ctx.llm_advice.valid || ctx.llm_advice.goal_id != goal_id)
    {
        return 0.0f;
    }
    return 0.10f * ctx.llm_advice.confidence;
}

std::uint8_t GoalNameToProtocolId(const std::string& goal_id)
{
    if (goal_id == "SAFE_HOLD") return 1;
    if (goal_id == "WAIT_REVIVE") return 2;
    if (goal_id == "SAFE_RETREAT_A") return 3;
    if (goal_id == "SAFE_RETREAT_B") return 4;
    if (goal_id == "SUPPLY_LEFT") return 5;
    if (goal_id == "SUPPLY_RIGHT") return 6;
    if (goal_id == "FORTRESS_HOLD") return 7;
    if (goal_id == "OUTPOST_HOLD") return 8;
    if (goal_id == "COMBAT_KITE_A") return 9;
    if (goal_id == "COMBAT_HOLD_A") return 10;
    if (goal_id == "MID_PRESSURE") return 11;
    if (goal_id == "HIGHGROUND_PEEK") return 12;
    if (goal_id == "COMBAT_PUSH_A") return 13;
    if (goal_id == "SEARCH_AREA_A") return 14;
    if (goal_id == "SEARCH_AREA_B") return 15;
    if (goal_id == "HIGHGROUND_SCAN") return 16;
    if (goal_id == "HIGHGROUND_CENTER") return 17;
    if (goal_id == "MID_CROSS") return 18;
    if (goal_id == "BASE_HOME") return 19;
    if (goal_id == "BASE_HOLD") return 20;
    return 0;
}

std::uint64_t ElapsedSince(std::uint64_t now_ms, std::uint64_t then_ms)
{
    return (then_ms == 0 || now_ms <= then_ms) ? 0 : (now_ms - then_ms);
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
    const float hp_ratio = SafeRatio(ctx.hp, ctx.hp_max);
    const bool enemy_pressure = ctx.enemy_in_view && ctx.enemy_confidence >= 0.45f;
    const bool strong_enemy_pressure = ctx.enemy_in_view && ctx.enemy_confidence >= 0.70f;

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
    if (ctx.power_guard_active)
    {
        reason = "底盘功率已进入守卫区，小陀螺先关闭避免继续冲功率。";
        return SpinMode::OFF;
    }
    if (ctx.supercap_guard_active || ctx.supercap_soc < 0.14f)
    {
        reason = "超级电容余量过低，小陀螺先关闭。";
        return SpinMode::OFF;
    }

    if (ctx.llm_advice.valid && ctx.llm_advice.spin_preference == SpinMode::ON &&
        ctx.llm_advice.confidence >= 0.75f)
    {
        reason = "外部建议明确启用小陀螺，且资源守卫未触发，采纳该偏好。";
        return SpinMode::ON;
    }

    switch (ctx.tactical_state)
    {
        case TacticalState::RETREAT:
            if (enemy_pressure || hp_ratio < 0.35f)
            {
                reason = "撤退或低血受压时优先开小陀螺，提升脱离过程的抗命中能力。";
                return SpinMode::ON;
            }
            reason = "撤退态但当前没有明显敌情压力，暂不开小陀螺以保留机动余量。";
            return SpinMode::OFF;
        case TacticalState::RESUPPLY:
            if (enemy_pressure || !ctx.is_disengaged)
            {
                reason = "补给路上仍有敌情或尚未脱战，开小陀螺保护转移。";
                return SpinMode::ON;
            }
            reason = "已脱战补给且敌情不明显，关闭小陀螺节省底盘功率。";
            return SpinMode::OFF;
        case TacticalState::HOLD:
            reason = "固守高价值区域时默认开小陀螺，降低静态防守被集火命中的概率。";
            return SpinMode::ON;
        case TacticalState::ENGAGE:
            if (enemy_pressure)
            {
                reason = strong_enemy_pressure
                             ? "交战目标置信度高，开小陀螺进行对抗。"
                             : "已有可疑敌情，开小陀螺提高遭遇战容错。";
                return SpinMode::ON;
            }
            reason = "交战态未形成可靠敌情，暂不开小陀螺。";
            return SpinMode::OFF;
        case TacticalState::SEARCH:
            if (ctx.supercap_soc >= 0.18f)
            {
                reason = "搜索巡逻默认开小陀螺，兼顾扫视和防偷袭。";
                return SpinMode::ON;
            }
            reason = "搜索态电容余量偏低，暂不开小陀螺。";
            return SpinMode::OFF;
        case TacticalState::REPOSITION:
            if (enemy_pressure || ctx.supercap_soc >= 0.22f)
            {
                reason = "转位过程中开小陀螺，减少跨区移动时的暴露风险。";
                return SpinMode::ON;
            }
            reason = "转位态但电容余量偏低且无敌情，暂不开小陀螺。";
            return SpinMode::OFF;
    }

    reason = "未匹配到战术态，默认不开小陀螺。";
    return SpinMode::OFF;
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
        const float heat_ratio = SafeRatio(ctx_->heat, ctx_->heat_limit);

        // 优先级顺序尽量贴合 XML 中主树的语义设计：
        // 1. 生存压力与硬约束；
        // 2. 补给压力；
        // 3. 稳定据点防守；
        // 4. 明确的交战机会；
        // 5. 外部建议带来的软引导。
        if (!ctx_->match_started)
        {
            ctx_->tactical_state = TacticalState::SEARCH;
            ctx_->tactical_reason = "比赛尚未进入七分钟对抗阶段，仅保持安全待机。";
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
        else if (ctx_->need_emergency_safety || heat_ratio > 0.90f)
        {
            ctx_->tactical_state = TacticalState::RETREAT;
            ctx_->tactical_reason = "热量或紧急守卫触发，需立即撤退。";
        }
        else if (ctx_->need_supply)
        {
            ctx_->tactical_state = TacticalState::RESUPPLY;
            ctx_->tactical_reason =
                "低血/低弹已进入补给闭环，优先级高于普通搜索和交战。";
        }
        else if ((ctx_->on_base || ctx_->on_fortress || ctx_->on_outpost) &&
                 !ctx_->enemy_in_view)
        {
            ctx_->tactical_state = TacticalState::HOLD;
            ctx_->tactical_reason = "当前已占据高价值据点，且没有必须切换目标的压力。";
        }
        else if (ctx_->enemy_in_view && ctx_->enemy_confidence > 0.70f)
        {
            ctx_->tactical_state = TacticalState::ENGAGE;
            ctx_->tactical_reason = "敌方目标置信度足够高，可以明确进入交战态。";
        }
        else if (ctx_->llm_advice.valid && ctx_->llm_advice.confidence >= 0.70f)
        {
            // 外部建议只在“没有更硬的理由”时才参与分流，
            // 否则会让整体策略显得不稳定。
            ctx_->tactical_state = ctx_->llm_advice.tactical_state;
            ctx_->tactical_reason = "外部建议对战术进行了软引导：" + ctx_->llm_advice.reason;
        }
        else if (!ctx_->on_highground && ctx_->supercap_soc > 0.30f)
        {
            ctx_->tactical_state = TacticalState::REPOSITION;
            ctx_->tactical_reason = "机动资源充足，可转向更优地形与视野点。";
        }
        else
        {
            ctx_->tactical_state = TacticalState::SEARCH;
            ctx_->tactical_reason = "没有更紧急的目标或约束，继续执行搜索侦察。";
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

        // 交战姿态目前优先采信外部建议；如果没有建议，则默认采用 ATTACK。
        // 后续如果要加入“掩体战 / 推进战 / 拉扯战”等细分风格，
        // 可以从这里继续扩展。
        ctx_->preferred_posture = ctx_->llm_advice.valid ? ctx_->llm_advice.posture_preference
                                                         : Posture::ATTACK;
        return BT::NodeStatus::SUCCESS;
    }
};

// 交战时的火力策略更新。
// 这里综合考虑了弹药、热量和目标置信度，是交战态里最值得继续打磨的节点之一。
class UpdateCombatFirePolicyNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        const float heat_ratio = SafeRatio(ctx_->heat, ctx_->heat_limit);
        if (ctx_->ammo_17 <= 0)
        {
            // 没弹药时直接强制停火。
            ctx_->preferred_fire_policy = FirePolicy::HOLD_FIRE;
        }
        else if (ctx_->llm_advice.valid && ctx_->llm_advice.fire_policy == FirePolicy::AGGRESSIVE &&
                 heat_ratio < 0.65f)
        {
            // 外部建议只在热量尚可时，才有机会把火力推高。
            ctx_->preferred_fire_policy = FirePolicy::AGGRESSIVE;
        }
        else if (heat_ratio < 0.55f && ctx_->enemy_confidence > 0.85f)
        {
            ctx_->preferred_fire_policy = FirePolicy::AGGRESSIVE;
        }
        else if (heat_ratio < 0.75f)
        {
            ctx_->preferred_fire_policy = FirePolicy::NORMAL;
        }
        else
        {
            ctx_->preferred_fire_policy = FirePolicy::CONSERVATIVE;
        }

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
        ctx_->preferred_supercap_mode =
            (ctx_->supercap_soc > 0.35f && ctx_->enemy_distance_m > 5.0f)
                ? SupercapMode::BURST
                : SupercapMode::KEEP;
        if (ctx_->llm_advice.valid && ctx_->llm_advice.supercap_preference == SupercapMode::BURST &&
            ctx_->supercap_soc > 0.45f)
        {
            ctx_->preferred_supercap_mode = SupercapMode::BURST;
        }
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
            const bool wait_timeout =
                ctx_->resupply_wait_recovery_timeout_ms > 0 &&
                ElapsedSince(ctx_->now_ms, ctx_->resupply_rfid_confirm_ms) >=
                    ctx_->resupply_wait_recovery_timeout_ms;
            if (hold_ok && wait_timeout)
            {
                SwitchToNextResupplyCandidate(
                    *ctx_, "RFID 已确认但血量/弹量等待恢复超时");
                AddGoalCandidate(*ctx_, ctx_->resupply_goal_current, 2.0f,
                                 ctx_->resupply_reason);
                return BT::NodeStatus::SUCCESS;
            }
            ctx_->resupply_rfid_confirmed = hold_ok;
            ctx_->resupply_waiting_recovery = hold_ok;
            ctx_->resupply_reason =
                ctx_->on_base
                    ? "RFID 确认在基地恢复点，停留等待血量/弹量恢复。"
                    : "RFID 确认在补给区，停留等待血量/弹量恢复。";
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
        AddGoalCandidate(*ctx_, "BASE_HOLD",
                         (ctx_->on_base ? 1.08f : 0.55f) + AdviceBonus(*ctx_, "BASE_HOLD"),
                         "小场地或基地 RFID 可用时，基地增益点可作为保守固守点。");
        AddGoalCandidate(*ctx_, "FORTRESS_HOLD",
                         (ctx_->on_fortress ? 1.10f : 0.95f) +
                             AdviceBonus(*ctx_, "FORTRESS_HOLD"),
                         "若当前已在堡垒区域，优先固守该位置。");
        AddGoalCandidate(*ctx_, "OUTPOST_HOLD",
                         (ctx_->on_outpost ? 1.05f : 0.80f) + AdviceBonus(*ctx_, "OUTPOST_HOLD"),
                         "当堡垒不是首要目标时，可转为固守前哨点。");
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

        // 这里按敌我距离做了一个很粗的分段。
        // 真正实战里，你可以继续引入更多特征，例如：
        // 敌方类型、掩体情况、己方血量、炮口朝向、地形可逃逸性等。
        if (ctx_->enemy_distance_m < 3.0f)
        {
            AddGoalCandidate(*ctx_, "COMBAT_KITE_A", 1.00f + AdviceBonus(*ctx_, "COMBAT_KITE_A"),
                             "近距离交战时，更适合边打边拉扯以提升生存率。");
            AddGoalCandidate(*ctx_, "COMBAT_HOLD_A",
                             0.82f + AdviceBonus(*ctx_, "COMBAT_HOLD_A"),
                             "若机动空间不足，则退而采用据点卡角方案。");
        }
        else if (ctx_->enemy_distance_m < 8.0f)
        {
            AddGoalCandidate(*ctx_, "COMBAT_HOLD_A",
                             1.00f + AdviceBonus(*ctx_, "COMBAT_HOLD_A"),
                             "中距离交战时，优先选择稳定火力通道。");
            AddGoalCandidate(*ctx_, "MID_PRESSURE", 0.88f + AdviceBonus(*ctx_, "MID_PRESSURE"),
                             "当敌方已被压制时，中路施压路线也具备可行性。");
        }
        else
        {
            AddGoalCandidate(*ctx_, "HIGHGROUND_PEEK",
                             1.00f + AdviceBonus(*ctx_, "HIGHGROUND_PEEK"),
                             "远距离接触时，更适合借助高点掩体进行探头输出。");
            AddGoalCandidate(*ctx_, "COMBAT_PUSH_A", 0.86f + AdviceBonus(*ctx_, "COMBAT_PUSH_A"),
                             "若掩体条件不足，可将推进路线作为后续选择。");
        }

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
        AddGoalCandidate(*ctx_, "SEARCH_AREA_A", 1.00f + AdviceBonus(*ctx_, "SEARCH_AREA_A"),
                         "主搜索路线用于维持更广的地图覆盖。");
        AddGoalCandidate(*ctx_, "SEARCH_AREA_B", 0.84f + AdviceBonus(*ctx_, "SEARCH_AREA_B"),
                         "次搜索路线用于避免重复扫图过于单一。");
        AddGoalCandidate(*ctx_, "HIGHGROUND_SCAN",
                         (ctx_->on_highground ? 1.05f : 0.78f) +
                             AdviceBonus(*ctx_, "HIGHGROUND_SCAN"),
                         "一旦已占据高点，就优先利用高点进行扫描观察。");
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
        AddGoalCandidate(*ctx_, "HIGHGROUND_CENTER",
                         1.00f + AdviceBonus(*ctx_, "HIGHGROUND_CENTER"),
                         "转位子树优先争取更好的中心视野和射界几何。");
        AddGoalCandidate(*ctx_, "MID_CROSS", 0.86f + AdviceBonus(*ctx_, "MID_CROSS"),
                         "若中心区域竞争激烈，则跨区转移作为备选。");
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
