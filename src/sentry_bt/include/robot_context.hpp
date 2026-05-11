#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

enum class TacticalState
{
    RETREAT,
    RESUPPLY,
    HOLD,
    ENGAGE,
    SEARCH,
    REPOSITION
};

enum class Posture
{
    MOVE,
    ATTACK,
    DEFENSE
};

enum class FirePolicy
{
    HOLD_FIRE,
    CONSERVATIVE,
    NORMAL,
    AGGRESSIVE
};

enum class SpinMode
{
    OFF,
    ON
};

enum class SupercapMode
{
    OFF,
    KEEP,
    BURST
};

enum class RuleActionType
{
    NONE,
    EXCHANGE_AMMO_AT_POINT,
    REMOTE_AMMO,
    REMOTE_HP,
    ACTIVATE_ENERGY,
    CLAIM_PERIODIC_AMMO,
    SWITCH_POSTURE
};

struct GoalCandidate
{
    std::string id{};
    float score{0.0f};
    std::string rationale{};
};

struct LLMAdvice
{
    bool valid{false};
    TacticalState tactical_state{TacticalState::SEARCH};
    std::string goal_id{};
    Posture posture_preference{Posture::MOVE};
    FirePolicy fire_policy{FirePolicy::NORMAL};
    SpinMode spin_preference{SpinMode::OFF};
    SupercapMode supercap_preference{SupercapMode::OFF};
    std::string reason{};
    float confidence{0.0f};
    std::uint64_t expire_time_ms{0};
};

struct RobotContext
{
    std::mutex mtx;

    // 循环计数主要用于演示和日志输出。
    // 后续接入真实系统后，也可以用它把外部输入快照与 BT 决策结果对应起来。
    std::uint64_t frame_index{0};
    std::uint64_t bt_tick_index{0};

    // 原始输入层。
    // 这些字段应当在每次 BT tick 之前，由接口层先行刷新。
    bool is_dead{false};
    bool match_started{false};
    bool can_confirm_revive{false};
    bool can_buy_immediate_revive{false};
    bool is_disengaged{true};
    int immediate_revive_cost{80};
    int game_progress{0};
    int stage_remain_time{420};
    int hp{400};
    int hp_max{400};
    int heat{0};
    int heat_limit{260};
    int cooling{30};
    int ammo_17{200};
    int gold{0};
    int exchanged_projectile_allowance{0};
    int remote_exchange_projectile_count{0};
    int remote_exchange_hp_count{0};
    int team_17mm_exchange_remain{0};
    float chassis_power_now{0.0f};
    float chassis_power_limit{100.0f};
    float supercap_soc{0.5f};
    bool enemy_in_view{false};
    float enemy_confidence{0.0f};
    float enemy_distance_m{0.0f};
    bool on_supply{false};
    bool on_base{false};
    bool on_fortress{false};
    bool on_outpost{false};
    bool on_highground{false};
    bool at_valid_recovery_rfid{false};
    std::uint32_t rfid_status{0};
    std::uint8_t recovery_buff{0};
    Posture reported_posture{Posture::MOVE};
    Posture current_posture{Posture::MOVE};
    Posture pending_posture_target{Posture::MOVE};
    bool posture_switch_pending{false};
    bool posture_cooldown_ok{true};
    std::uint64_t posture_cooldown_remaining_ms{0};
    bool referee_link_fresh{false};
    bool sim_input_fresh{false};
    bool health_data_degraded{false};
    std::uint64_t referee_status_age_ms{0};
    std::uint64_t sim_input_age_ms{0};
    bool nav_goal_active{false};
    bool nav_goal_reached{false};
    bool nav_goal_failed{false};
    std::uint8_t current_goal_id{0};
    std::uint64_t nav_status_age_ms{0};
    bool autoaim_has_target{false};
    bool autoaim_tracking{false};
    bool autoaim_fire_ready{false};
    float autoaim_target_distance{0.0f};
    std::uint64_t autoaim_status_age_ms{0};
    Posture hold_reached_posture{Posture::DEFENSE};

    // 由原始输入推导出的便捷状态。
    // UpdateBlackboard 和各类 Guard 节点都可以在每个 tick 内重算这些值。
    bool ammo_low{false};
    bool need_supply{false};
    bool hp_recovery_active{false};
    bool ammo_recovery_active{false};
    bool resupply_active{false};
    bool need_emergency_safety{false};
    bool need_rule_action{false};
    bool can_activate_energy_mechanism{false};
    bool can_claim_periodic_ammo{false};
    bool posture_switch_requested{false};

    // Guard 层输出。
    // 这些字段表示本 tick 检测到的硬约束，后续会被战术层和执行层继续消费。
    bool heat_guard_active{false};
    bool power_guard_active{false};
    bool ammo_guard_active{false};
    bool supercap_guard_active{false};
    bool posture_cooldown_guard_active{false};
    bool rule_cmd_guard_active{false};

    // BT 中间结果。
    // preferred_* 表示战术层给出的“理想意图”，
    // desired_* 表示执行层在考虑约束后的“最终下发值”。
    RuleActionType rule_action_type{RuleActionType::NONE};
    TacticalState tactical_state{TacticalState::SEARCH};
    std::vector<GoalCandidate> goal_candidates{};
    std::string preferred_goal{"SAFE_HOLD"};
    Posture preferred_posture{Posture::MOVE};
    FirePolicy preferred_fire_policy{FirePolicy::NORMAL};
    SpinMode preferred_spin_mode{SpinMode::OFF};
    SupercapMode preferred_supercap_mode{SupercapMode::OFF};
    std::string desired_goal{"SAFE_HOLD"};
    Posture desired_posture{Posture::MOVE};
    FirePolicy desired_fire_policy{FirePolicy::NORMAL};
    SpinMode desired_spin_mode{SpinMode::OFF};
    SupercapMode desired_supercap_mode{SupercapMode::OFF};

    // 旁路建议信号。
    // 战术层可以把它作为软参考，但优先级始终低于硬约束和显式规则动作。
    LLMAdvice llm_advice{};

    // 可读性较强的解释字段。
    // 主要用于日志、联调，以及后续接 GUI / 遥测面板时快速说明“为什么这样决策”。
    std::string tactical_reason{};
    std::string rule_reason{};
    std::string goal_reason{};
    std::string spin_reason{};
    std::string executor_summary{};
    std::string last_nav_command{};
    std::string last_shooter_command{};
    std::string last_rule_command{};
    std::string input_health_reason{};

    // 协议输出层。
    // 这些字段会被直接映射到 /gimbal_cmd 中同名字段。
    std::uint8_t protocol_version{1};
    std::uint16_t ammo_exchange_target_total{0};
    std::uint8_t revive_cmd{0};
    std::uint8_t remote_ammo_req_inc{0};
    std::uint8_t remote_hp_req_inc{0};
    std::uint8_t posture_cmd_referee{0};
    std::uint8_t activate_energy_confirm{0};
    std::uint8_t claim_periodic_ammo{0};

    // 本地时序记忆。
    // 规则里有不少“单调递增”“每次只能 +1”“5 秒冷却”之类的约束，
    // 这些约束需要在决策层自己维持，不能靠 BT 每帧无状态地产生。
    std::uint64_t now_ms{0};
    std::uint64_t last_remote_ammo_request_ms{0};
    std::uint64_t last_remote_hp_request_ms{0};
    std::uint64_t last_posture_command_ms{0};
    std::uint64_t last_energy_activate_ms{0};
    std::uint64_t last_periodic_ammo_claim_ms{0};

    // 死亡后回基地补满血硬任务。WAIT_REVIVE 只表达复活状态，不再作为导航点使用。
    bool dead_return_home_enabled{true};
    bool dead_return_no_timeout{true};
    bool dead_chassis_can_move{false};
    bool dead_return_home_active{false};
    bool dead_home_rfid_confirmed{false};
    bool dead_waiting_full_hp{false};
    std::uint64_t dead_return_start_ms{0};
    std::uint64_t dead_home_rfid_confirm_ms{0};
    std::string dead_return_goal{"BASE_HOME"};
    std::string dead_return_reason{};
    std::uint64_t dead_return_rfid_confirm_hold_ms{300};
    float dead_full_hp_exit_ratio{0.98f};

    // 普通低血/低弹补给闭环。
    std::uint64_t resupply_enter_ms{0};
    std::uint64_t resupply_reached_ms{0};
    std::uint64_t resupply_rfid_confirm_ms{0};
    std::uint64_t resupply_last_candidate_switch_ms{0};
    std::uint64_t resupply_goal_start_ms{0};
    std::string resupply_goal_current{"SUPPLY_LEFT"};
    int resupply_candidate_index{0};
    bool resupply_rfid_confirmed{false};
    bool resupply_waiting_recovery{false};
    std::string resupply_reason{};
    float hp_resupply_enter_ratio{0.35f};
    float hp_resupply_exit_ratio{0.60f};
    int ammo_resupply_enter_count{80};
    int ammo_resupply_exit_count{120};
    std::uint64_t resupply_rfid_confirm_hold_ms{300};
    std::uint64_t resupply_goal_timeout_ms{12000};
    std::uint64_t resupply_wait_recovery_timeout_ms{12000};
    std::uint64_t resupply_candidate_switch_cooldown_ms{1500};
    std::vector<std::string> resupply_candidates{"SUPPLY_LEFT", "SUPPLY_RIGHT", "BASE_HOME"};

    // sentry_bt 全自动巡航/到点 dwell。补给、交战、撤退和死亡硬任务不受 dwell 约束。
    std::uint8_t dwell_goal_id{0};
    std::uint8_t last_seen_nav_goal_id{0};
    bool last_nav_goal_reached{false};
    std::uint64_t dwell_start_ms{0};
    bool dwell_active{false};
    bool dwell_complete{false};
    std::uint64_t dwell_required_ms{0};
    std::uint64_t dwell_remaining_ms{0};
    std::string dwell_reason{};
    std::uint64_t goal_dwell_default_ms{1500};
    std::uint64_t goal_dwell_search_ms{2500};
    std::uint64_t goal_dwell_hold_ms{3000};
    std::uint64_t goal_dwell_resupply_ms{0};
    std::uint64_t goal_dwell_engage_ms{0};

    // 姿态累计与衰减信息。
    // 规则要求“单姿态累计超过 3 分钟后收益下降”，
    // 所以上位机需要自己跨 tick 维护累计时长。
    std::array<std::uint64_t, 3> posture_accumulated_ms{{0, 0, 0}};
    std::array<bool, 3> posture_debuffed{{false, false, false}};
    std::uint64_t posture_debuff_threshold_ms{180000};
    std::uint64_t posture_debuff_rotate_margin_ms{15000};
    std::string posture_reason{};
};

inline const char* TacticalStateToString(TacticalState state)
{
    switch (state)
    {
        case TacticalState::RETREAT:
            return "RETREAT";
        case TacticalState::RESUPPLY:
            return "RESUPPLY";
        case TacticalState::HOLD:
            return "HOLD";
        case TacticalState::ENGAGE:
            return "ENGAGE";
        case TacticalState::SEARCH:
            return "SEARCH";
        case TacticalState::REPOSITION:
            return "REPOSITION";
    }
    return "SEARCH";
}

inline const char* PostureToString(Posture posture)
{
    switch (posture)
    {
        case Posture::MOVE:
            return "MOVE";
        case Posture::ATTACK:
            return "ATTACK";
        case Posture::DEFENSE:
            return "DEFENSE";
    }
    return "MOVE";
}

inline std::uint8_t PostureToProtocolValue(Posture posture)
{
    switch (posture)
    {
        case Posture::ATTACK:
            return 1;
        case Posture::DEFENSE:
            return 2;
        case Posture::MOVE:
            return 3;
    }
    return 3;
}

inline std::size_t PostureToIndex(Posture posture)
{
    switch (posture)
    {
        case Posture::ATTACK:
            return 0;
        case Posture::DEFENSE:
            return 1;
        case Posture::MOVE:
            return 2;
    }
    return 2;
}

inline std::uint64_t GetAccumulatedPostureMs(const RobotContext& ctx, Posture posture)
{
    return ctx.posture_accumulated_ms[PostureToIndex(posture)];
}

inline bool IsPostureDebuffed(const RobotContext& ctx, Posture posture)
{
    return ctx.posture_debuffed[PostureToIndex(posture)];
}

inline std::uint64_t RemainingBeforePostureDebuff(const RobotContext& ctx, Posture posture)
{
    const auto used_ms = GetAccumulatedPostureMs(ctx, posture);
    return (used_ms >= ctx.posture_debuff_threshold_ms) ? 0
                                                        : (ctx.posture_debuff_threshold_ms -
                                                           used_ms);
}

inline bool ParsePostureProtocolValue(std::uint8_t value, Posture& posture)
{
    switch (value)
    {
        case 1:
            posture = Posture::ATTACK;
            return true;
        case 2:
            posture = Posture::DEFENSE;
            return true;
        case 3:
            posture = Posture::MOVE;
            return true;
        default:
            posture = Posture::MOVE;
            return false;
    }
}

inline const char* FirePolicyToString(FirePolicy policy)
{
    switch (policy)
    {
        case FirePolicy::HOLD_FIRE:
            return "HOLD_FIRE";
        case FirePolicy::CONSERVATIVE:
            return "CONSERVATIVE";
        case FirePolicy::NORMAL:
            return "NORMAL";
        case FirePolicy::AGGRESSIVE:
            return "AGGRESSIVE";
    }
    return "NORMAL";
}

inline const char* SpinModeToString(SpinMode mode)
{
    switch (mode)
    {
        case SpinMode::OFF:
            return "OFF";
        case SpinMode::ON:
            return "ON";
    }
    return "OFF";
}

inline const char* SupercapModeToString(SupercapMode mode)
{
    switch (mode)
    {
        case SupercapMode::OFF:
            return "OFF";
        case SupercapMode::KEEP:
            return "KEEP";
        case SupercapMode::BURST:
            return "BURST";
    }
    return "OFF";
}

inline const char* RuleActionTypeToString(RuleActionType type)
{
    switch (type)
    {
        case RuleActionType::NONE:
            return "NONE";
        case RuleActionType::EXCHANGE_AMMO_AT_POINT:
            return "EXCHANGE_AMMO_AT_POINT";
        case RuleActionType::REMOTE_AMMO:
            return "REMOTE_AMMO";
        case RuleActionType::REMOTE_HP:
            return "REMOTE_HP";
        case RuleActionType::ACTIVATE_ENERGY:
            return "ACTIVATE_ENERGY";
        case RuleActionType::CLAIM_PERIODIC_AMMO:
            return "CLAIM_PERIODIC_AMMO";
        case RuleActionType::SWITCH_POSTURE:
            return "SWITCH_POSTURE";
    }
    return "NONE";
}

inline std::uint8_t TacticalStateToProtocolValue(TacticalState state)
{
    switch (state)
    {
        case TacticalState::HOLD:
            return 1;
        case TacticalState::ENGAGE:
            return 2;
        case TacticalState::RETREAT:
            return 3;
        case TacticalState::RESUPPLY:
            return 4;
        case TacticalState::SEARCH:
            return 5;
        case TacticalState::REPOSITION:
            return 6;
    }
    return 0;
}

inline std::uint8_t FirePolicyToProtocolValue(FirePolicy policy)
{
    switch (policy)
    {
        case FirePolicy::HOLD_FIRE:
            return 0;
        case FirePolicy::CONSERVATIVE:
            return 1;
        case FirePolicy::NORMAL:
            return 2;
        case FirePolicy::AGGRESSIVE:
            return 3;
    }
    return 0;
}

inline std::uint8_t SpinModeToProtocolValue(SpinMode mode)
{
    return mode == SpinMode::ON ? 1 : 0;
}

inline std::uint8_t SupercapModeToProtocolValue(SupercapMode mode)
{
    switch (mode)
    {
        case SupercapMode::OFF:
            return 0;
        case SupercapMode::KEEP:
            return 1;
        case SupercapMode::BURST:
            return 2;
    }
    return 0;
}

inline std::uint8_t RuleActionTypeToProtocolValue(RuleActionType type)
{
    switch (type)
    {
        case RuleActionType::NONE:
            return 0;
        case RuleActionType::EXCHANGE_AMMO_AT_POINT:
            return 1;
        case RuleActionType::REMOTE_AMMO:
            return 2;
        case RuleActionType::REMOTE_HP:
            return 3;
        case RuleActionType::ACTIVATE_ENERGY:
            return 7;
        case RuleActionType::CLAIM_PERIODIC_AMMO:
            // Not part of RM2026 official sentry 0x0120 command in phase 1.
            return 0;
        case RuleActionType::SWITCH_POSTURE:
            return 6;
    }
    return 0;
}

inline bool ParsePosture(std::string_view value, Posture& posture)
{
    if (value == "MOVE")
    {
        posture = Posture::MOVE;
        return true;
    }
    if (value == "ATTACK")
    {
        posture = Posture::ATTACK;
        return true;
    }
    if (value == "DEFENSE")
    {
        posture = Posture::DEFENSE;
        return true;
    }
    return false;
}

inline bool ParseFirePolicy(std::string_view value, FirePolicy& policy)
{
    if (value == "HOLD_FIRE")
    {
        policy = FirePolicy::HOLD_FIRE;
        return true;
    }
    if (value == "CONSERVATIVE")
    {
        policy = FirePolicy::CONSERVATIVE;
        return true;
    }
    if (value == "NORMAL")
    {
        policy = FirePolicy::NORMAL;
        return true;
    }
    if (value == "AGGRESSIVE")
    {
        policy = FirePolicy::AGGRESSIVE;
        return true;
    }
    return false;
}

inline bool ParseSpinMode(std::string_view value, SpinMode& mode)
{
    if (value == "OFF")
    {
        mode = SpinMode::OFF;
        return true;
    }
    if (value == "ON")
    {
        mode = SpinMode::ON;
        return true;
    }
    return false;
}

inline bool ParseSupercapMode(std::string_view value, SupercapMode& mode)
{
    if (value == "OFF")
    {
        mode = SupercapMode::OFF;
        return true;
    }
    if (value == "KEEP")
    {
        mode = SupercapMode::KEEP;
        return true;
    }
    if (value == "BURST")
    {
        mode = SupercapMode::BURST;
        return true;
    }
    return false;
}
