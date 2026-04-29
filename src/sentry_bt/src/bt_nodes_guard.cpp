#include "bt_nodes_guard.hpp"

#include "bt_node_common.hpp"

// guard 节点文件负责“硬约束检测”。
// 它们的职责不是直接决定完整战术，而是先把那些不能无视的限制条件
// 转换成布尔标志，供后续战术层和执行层统一消费。
//
// 理解这一层时可以记住一个原则：
// tactical 负责“想做什么”，executor 负责“最终能做什么”，
// guard 则负责“哪些事情已经不安全了”。

namespace
{
// 热量守卫。
// 一旦枪口热量接近上限，就把 heat_guard_active 拉高。
// 这里阈值写成 0.92，是一个启发式占位值，后续很可能要结合实际控热策略改。
class HeatGuardNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->heat_guard_active = SafeRatio(ctx_->heat, ctx_->heat_limit) > 0.92f;
        return ctx_->heat_guard_active ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
};

// 功率守卫。
// 底盘功率逼近上限时，这里只负责给出“危险”标志，
// 真正如何降级动作，会在 executor 层统一处理。
class PowerGuardNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->power_guard_active =
            ctx_->chassis_power_now > (0.95f * ctx_->chassis_power_limit);
        return ctx_->power_guard_active ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
};

// 弹药守卫。
// 这里把“低弹药”同时映射成 ammo_low 和 ammo_guard_active。
// ammo_low 更像一个通用状态，ammo_guard_active 更偏向“受限标志”。
class AmmoGuardNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->ammo_low = (ctx_->ammo_17 < 80);
        ctx_->ammo_guard_active = ctx_->ammo_low;
        return ctx_->ammo_guard_active ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
};

// 超级电容守卫。
// 当超级电容电量太低时，意味着高机动和爆发动作都需要收敛。
class SupercapGuardNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->supercap_guard_active = ctx_->supercap_soc < 0.10f;
        return ctx_->supercap_guard_active ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
};

// 姿态切换冷却守卫。
// 如果当前姿态系统还处于冷却期，就不允许 executor 立刻采纳新的姿态请求。
class PostureCooldownGuardNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);
        ctx_->posture_cooldown_guard_active = !ctx_->posture_cooldown_ok;
        return ctx_->posture_cooldown_guard_active ? BT::NodeStatus::FAILURE
                                                   : BT::NodeStatus::SUCCESS;
    }
};

// 规则命令守卫。
// 这个节点的逻辑比较特殊：
// 1. 它负责判断“当前是否存在需要优先处理的规则动作”；
// 2. 它会把结果写入 need_rule_action / rule_cmd_guard_active；
// 3. 在主树里它被 ForceSuccess 包裹，所以这里返回 FAILURE 只是为了表达
//    “规则侧有事情待处理”，不会真正打断 HardConstraintGuard 子树。
//
// 也就是说，这里 FAILURE 的语义不是“系统崩了”，而是“规则动作待插队”。
class RuleCmdGuardNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        const bool can_exchange_at_point =
            ctx_->ammo_low && (ctx_->on_supply || ctx_->on_outpost || ctx_->on_fortress);
        const bool can_remote_ammo = ctx_->ammo_low && ctx_->gold >= 50;
        const bool can_remote_hp = ctx_->hp < (ctx_->hp_max / 3) && ctx_->gold >= 80;

        ctx_->rule_cmd_guard_active = can_exchange_at_point || can_remote_ammo ||
                                      can_remote_hp || ctx_->can_activate_energy_mechanism ||
                                      ctx_->can_claim_periodic_ammo ||
                                      ctx_->posture_switch_requested;
        ctx_->need_rule_action = ctx_->rule_cmd_guard_active;

        return ctx_->rule_cmd_guard_active ? BT::NodeStatus::FAILURE
                                           : BT::NodeStatus::SUCCESS;
    }
};

// 紧急安全判定节点。
// 它和前面的单项 guard 不同：前面只是给出单维度受限标志，
// 这里才把多个危险信号综合成“是否进入 emergency_branch”。
//
// 这里专门对 is_dead 做了特判：
// 死亡后不应被 emergency 分支截走，而应让 revive 分支优先接管。
class NeedEmergencySafetyNode : public ContextConditionNode
{
public:
    using ContextConditionNode::ContextConditionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        if (ctx_->is_dead)
        {
            ctx_->need_emergency_safety = false;
            return BT::NodeStatus::FAILURE;
        }

        const float hp_ratio = SafeRatio(ctx_->hp, ctx_->hp_max);
        const bool need_safety = ctx_->heat_guard_active || ctx_->power_guard_active ||
                                 ctx_->supercap_guard_active ||
                                 ((hp_ratio < 0.20f) && ctx_->enemy_in_view);

        ctx_->need_emergency_safety = need_safety;
        if (need_safety)
        {
            ctx_->tactical_reason = "进入紧急安全分支，优先级高于常规战术决策。";
        }
        return need_safety ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// 紧急安全动作节点。
// 一旦进入 emergency_branch，就用这个节点把偏好直接压成保守策略。
// 它不关心长期收益，只关心“先脱离危险再说”。
class SetSafePoliciesNode : public ContextSyncActionNode
{
public:
    using ContextSyncActionNode::ContextSyncActionNode;

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(ctx_->mtx);

        // 这里的策略故意偏保守。
        // 一旦硬约束触发到危险程度，就优先保命，并主动放弃可选动作。
        ctx_->preferred_posture = Posture::MOVE;
        ctx_->preferred_fire_policy = FirePolicy::HOLD_FIRE;
        ctx_->preferred_spin_mode = SpinMode::OFF;
        ctx_->preferred_supercap_mode = SupercapMode::OFF;
        ctx_->preferred_goal = "SAFE_RETREAT_A";
        ctx_->goal_reason = "硬约束触发后，选择预设的安全撤退点作为回退目标。";
        return BT::NodeStatus::SUCCESS;
    }
};
}  // 匿名命名空间

void RegisterGuardNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<RobotContext> ctx)
{
    // guard 节点注册表。
    // 这部分名称要和 XML 中的节点 ID 严格一致。
    RegisterContextNode<HeatGuardNode>(factory, "HeatGuard", ctx);
    RegisterContextNode<PowerGuardNode>(factory, "PowerGuard", ctx);
    RegisterContextNode<AmmoGuardNode>(factory, "AmmoGuard", ctx);
    RegisterContextNode<SupercapGuardNode>(factory, "SupercapGuard", ctx);
    RegisterContextNode<PostureCooldownGuardNode>(factory, "PostureCooldownGuard", ctx);
    RegisterContextNode<RuleCmdGuardNode>(factory, "RuleCmdGuard", ctx);
    RegisterContextNode<NeedEmergencySafetyNode>(factory, "NeedEmergencySafety", ctx);
    RegisterContextNode<SetSafePoliciesNode>(factory, "SetSafePolicies", ctx);
}
