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
        if (ctx_->is_dead)
        {
            ctx_->tactical_state = TacticalState::RETREAT;
            ctx_->tactical_reason = "当前已判定为死亡状态，应立即让复活分支接管。";
        }
        else if (ctx_->need_emergency_safety || hp_ratio < 0.20f || heat_ratio > 0.90f)
        {
            ctx_->tactical_state = TacticalState::RETREAT;
            ctx_->tactical_reason = "血量、热量或紧急守卫触发，需立即撤退。";
        }
        else if (ctx_->need_supply)
        {
            ctx_->tactical_state = TacticalState::RESUPPLY;
            ctx_->tactical_reason = "弹药或血量已低于补给阈值，转入补给态。";
        }
        else if ((ctx_->on_fortress || ctx_->on_outpost) && !ctx_->enemy_in_view)
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

        const auto best = std::max_element(
            ctx_->goal_candidates.begin(), ctx_->goal_candidates.end(),
            [](const GoalCandidate& lhs, const GoalCandidate& rhs) {
                return lhs.score < rhs.score;
            });

        ctx_->preferred_goal = best->id;
        ctx_->goal_reason = best->rationale.empty()
                                ? "在当前候选列表中选择得分最高的目标点。"
                                : best->rationale;
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

// 交战时的旋转策略更新。
// 当前逻辑偏简单：近距离更倾向开小陀螺，或直接采用外部建议。
class SetCombatSpinPreferenceNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->preferred_spin_mode = ctx_->llm_advice.valid ? ctx_->llm_advice.spin_preference
                                                           : ((ctx_->enemy_distance_m < 3.5f)
                                                                  ? SpinMode::ON
                                                                  : SpinMode::OFF);
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
        AddGoalCandidate(*ctx_, "SUPPLY_LEFT", 0.95f + AdviceBonus(*ctx_, "SUPPLY_LEFT"),
                         "资源偏低时，优先考虑左侧补给路线。");
        AddGoalCandidate(*ctx_, "SUPPLY_RIGHT", 0.82f + AdviceBonus(*ctx_, "SUPPLY_RIGHT"),
                         "右侧补给路线作为主路线受阻时的备选方案。");
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
    RegisterContextNode<SetCombatSpinPreferenceNode>(factory, "SetCombatSpinPreference", ctx);
    RegisterContextNode<SetCombatSupercapPreferenceNode>(factory, "SetCombatSupercapPreference", ctx);
    RegisterContextNode<EvaluateRetreatGoalsNode>(factory, "EvaluateRetreatGoals", ctx);
    RegisterContextNode<EvaluateResupplyGoalsNode>(factory, "EvaluateResupplyGoals", ctx);
    RegisterContextNode<EvaluateHoldGoalsNode>(factory, "EvaluateHoldGoals", ctx);
    RegisterContextNode<EvaluateEngageGoalsNode>(factory, "EvaluateEngageGoals", ctx);
    RegisterContextNode<EvaluateSearchGoalsNode>(factory, "EvaluateSearchGoals", ctx);
    RegisterContextNode<EvaluateRepositionGoalsNode>(factory, "EvaluateRepositionGoals", ctx);
}
