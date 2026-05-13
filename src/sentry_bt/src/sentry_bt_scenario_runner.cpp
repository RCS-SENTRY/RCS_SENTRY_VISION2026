#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "bt_compat.hpp"
#include "bt_setup.hpp"
#include "referee_interface.hpp"
#include "rm_interfaces/msg/gimbal_status.hpp"
#include "robot_context.hpp"
#include "sentry_decision_protocol.h"

namespace
{
template <typename T>
void ReadScalar(const YAML::Node& node, const char* key, T& value)
{
    if (node && node[key])
    {
        value = node[key].as<T>();
    }
}

void NormalizePostureDebuffs(RobotContext& ctx)
{
    for (std::size_t index = 0; index < ctx.posture_debuffed.size(); ++index)
    {
        ctx.posture_debuffed[index] =
            ctx.posture_accumulated_ms[index] >= ctx.posture_debuff_threshold_ms;
    }
}

bool ParseInternalMotionState(const std::string& value, InternalMotionState& state)
{
    if (value == "NAV")
    {
        state = InternalMotionState::NAV;
        return true;
    }
    if (value == "RESUPPLY")
    {
        state = InternalMotionState::RESUPPLY;
        return true;
    }
    if (value == "RETREAT")
    {
        state = InternalMotionState::RETREAT;
        return true;
    }
    if (value == "ATTACK")
    {
        state = InternalMotionState::ATTACK;
        return true;
    }
    if (value == "DEFENSE")
    {
        state = InternalMotionState::DEFENSE;
        return true;
    }
    return false;
}

void SetDefaultContext(RobotContext& ctx)
{
    ctx.frame_index = 1;
    ctx.bt_tick_index = 0;
    ctx.is_dead = false;
    ctx.match_started = true;
    ctx.can_confirm_revive = false;
    ctx.can_buy_immediate_revive = false;
    ctx.is_disengaged = true;
    ctx.immediate_revive_cost = 80;
    ctx.game_progress = 4;
    ctx.stage_remain_time = 420;
    ctx.hp = 400;
    ctx.hp_max = 400;
    ctx.heat = 0;
    ctx.heat_limit = 260;
    ctx.cooling = 30;
    ctx.ammo_17 = 300;
    ctx.gold = 400;
    ctx.exchanged_projectile_allowance = 0;
    ctx.remote_exchange_projectile_count = 0;
    ctx.remote_exchange_hp_count = 0;
    ctx.team_17mm_exchange_remain = 1000;
    ctx.chassis_power_now = 20.0f;
    ctx.chassis_power_limit = 100.0f;
    ctx.supercap_soc = 0.50f;
    ctx.enemy_in_view = false;
    ctx.enemy_confidence = 0.0f;
    ctx.enemy_distance_m = 12.0f;
    ctx.on_supply = false;
    ctx.on_base = false;
    ctx.on_fortress = false;
    ctx.on_outpost = false;
    ctx.on_highground = false;
    ctx.reported_posture = Posture::MOVE;
    ctx.current_posture = Posture::MOVE;
    ctx.pending_posture_target = Posture::MOVE;
    ctx.posture_switch_pending = false;
    ctx.posture_cooldown_ok = true;
    ctx.posture_cooldown_remaining_ms = 0;
    ctx.referee_link_fresh = true;
    ctx.sim_input_fresh = true;
    ctx.health_data_degraded = false;
    ctx.referee_status_age_ms = 0;
    ctx.sim_input_age_ms = 0;
    ctx.can_activate_energy_mechanism = false;
    ctx.can_claim_periodic_ammo = false;
    ctx.posture_switch_requested = false;
    ctx.desired_goal = "SAFE_HOLD";
    ctx.desired_posture = Posture::MOVE;
    ctx.desired_fire_policy = FirePolicy::HOLD_FIRE;
    ctx.desired_spin_mode = SpinMode::OFF;
    ctx.desired_supercap_mode = SupercapMode::OFF;
    ctx.preferred_goal = "SAFE_HOLD";
    ctx.preferred_posture = Posture::MOVE;
    ctx.preferred_fire_policy = FirePolicy::HOLD_FIRE;
    ctx.preferred_spin_mode = SpinMode::OFF;
    ctx.preferred_supercap_mode = SupercapMode::OFF;
    ctx.now_ms = 10000;
    ctx.last_remote_ammo_request_ms = 0;
    ctx.last_remote_hp_request_ms = 0;
    ctx.last_posture_command_ms = 0;
    ctx.last_energy_activate_ms = 0;
    ctx.last_periodic_ammo_claim_ms = 0;
    ctx.posture_accumulated_ms = {{0, 0, 0}};
    ctx.posture_debuffed = {{false, false, false}};
    ctx.posture_debuff_threshold_ms = 180000;
    ctx.posture_debuff_rotate_margin_ms = 15000;
}

void ApplyContextPatch(const YAML::Node& node, RobotContext& ctx)
{
    if (!node)
    {
        return;
    }

    ReadScalar(node, "frame_index", ctx.frame_index);
    ReadScalar(node, "is_dead", ctx.is_dead);
    ReadScalar(node, "match_started", ctx.match_started);
    ReadScalar(node, "can_confirm_revive", ctx.can_confirm_revive);
    ReadScalar(node, "can_buy_immediate_revive", ctx.can_buy_immediate_revive);
    ReadScalar(node, "is_disengaged", ctx.is_disengaged);
    ReadScalar(node, "immediate_revive_cost", ctx.immediate_revive_cost);
    ReadScalar(node, "game_progress", ctx.game_progress);
    ReadScalar(node, "stage_remain_time", ctx.stage_remain_time);
    ReadScalar(node, "hp", ctx.hp);
    ReadScalar(node, "hp_max", ctx.hp_max);
    ReadScalar(node, "heat", ctx.heat);
    ReadScalar(node, "heat_limit", ctx.heat_limit);
    ReadScalar(node, "cooling", ctx.cooling);
    ReadScalar(node, "ammo_17", ctx.ammo_17);
    ReadScalar(node, "gold", ctx.gold);
    ReadScalar(node, "exchanged_projectile_allowance", ctx.exchanged_projectile_allowance);
    ReadScalar(node, "remote_exchange_projectile_count", ctx.remote_exchange_projectile_count);
    ReadScalar(node, "remote_exchange_hp_count", ctx.remote_exchange_hp_count);
    ReadScalar(node, "team_17mm_exchange_remain", ctx.team_17mm_exchange_remain);
    ReadScalar(node, "chassis_power_now", ctx.chassis_power_now);
    ReadScalar(node, "chassis_power_limit", ctx.chassis_power_limit);
    ReadScalar(node, "supercap_soc", ctx.supercap_soc);
    ReadScalar(node, "enemy_in_view", ctx.enemy_in_view);
    ReadScalar(node, "enemy_confidence", ctx.enemy_confidence);
    ReadScalar(node, "enemy_distance_m", ctx.enemy_distance_m);
    ReadScalar(node, "under_attack", ctx.under_attack);
    ReadScalar(node, "armor_id", ctx.armor_id);
    ReadScalar(node, "hp_deduction_reason", ctx.hp_deduction_reason);
    ReadScalar(node, "on_supply", ctx.on_supply);
    ReadScalar(node, "on_base", ctx.on_base);
    ReadScalar(node, "on_fortress", ctx.on_fortress);
    ReadScalar(node, "on_outpost", ctx.on_outpost);
    ReadScalar(node, "on_highground", ctx.on_highground);
    ReadScalar(node, "posture_switch_pending", ctx.posture_switch_pending);
    ReadScalar(node, "posture_cooldown_ok", ctx.posture_cooldown_ok);
    ReadScalar(node, "posture_cooldown_remaining_ms", ctx.posture_cooldown_remaining_ms);
    ReadScalar(node, "referee_link_fresh", ctx.referee_link_fresh);
    ReadScalar(node, "sim_input_fresh", ctx.sim_input_fresh);
    ReadScalar(node, "health_data_degraded", ctx.health_data_degraded);
    ReadScalar(node, "referee_status_age_ms", ctx.referee_status_age_ms);
    ReadScalar(node, "sim_input_age_ms", ctx.sim_input_age_ms);
    ReadScalar(node, "nav_goal_active", ctx.nav_goal_active);
    ReadScalar(node, "nav_goal_reached", ctx.nav_goal_reached);
    ReadScalar(node, "nav_goal_failed", ctx.nav_goal_failed);
    ReadScalar(node, "current_goal_id", ctx.current_goal_id);
    ReadScalar(node, "nav_status_age_ms", ctx.nav_status_age_ms);
    ReadScalar(node, "can_activate_energy_mechanism", ctx.can_activate_energy_mechanism);
    ReadScalar(node, "can_claim_periodic_ammo", ctx.can_claim_periodic_ammo);
    ReadScalar(node, "posture_switch_requested", ctx.posture_switch_requested);
    ReadScalar(node, "now_ms", ctx.now_ms);
    ReadScalar(node, "last_remote_ammo_request_ms", ctx.last_remote_ammo_request_ms);
    ReadScalar(node, "last_remote_hp_request_ms", ctx.last_remote_hp_request_ms);
    ReadScalar(node, "last_posture_command_ms", ctx.last_posture_command_ms);
    ReadScalar(node, "last_energy_activate_ms", ctx.last_energy_activate_ms);
    ReadScalar(node, "last_periodic_ammo_claim_ms", ctx.last_periodic_ammo_claim_ms);
    ReadScalar(node, "spin_hysteresis_enabled", ctx.spin_hysteresis_enabled);
    ReadScalar(node, "spin_last_change_ms", ctx.spin_last_change_ms);
    ReadScalar(node, "spin_preference_on_since_ms", ctx.spin_preference_on_since_ms);
    ReadScalar(node, "spin_preference_off_since_ms", ctx.spin_preference_off_since_ms);
    ReadScalar(node, "spin_target_last_seen_ms", ctx.spin_target_last_seen_ms);
    ReadScalar(node, "spin_under_attack_last_seen_ms", ctx.spin_under_attack_last_seen_ms);
    ReadScalar(node, "spin_hp_observation_initialized", ctx.spin_hp_observation_initialized);
    ReadScalar(node, "spin_last_observed_hp", ctx.spin_last_observed_hp);
    ReadScalar(node, "spin_on_confirm_ms", ctx.spin_on_confirm_ms);
    ReadScalar(node, "spin_off_confirm_ms", ctx.spin_off_confirm_ms);
    ReadScalar(node, "spin_min_on_ms", ctx.spin_min_on_ms);
    ReadScalar(node, "spin_min_off_ms", ctx.spin_min_off_ms);
    ReadScalar(node, "spin_target_hold_ms", ctx.spin_target_hold_ms);
    ReadScalar(node, "spin_under_attack_hold_ms", ctx.spin_under_attack_hold_ms);
    ReadScalar(node, "spin_hp_drop_threshold", ctx.spin_hp_drop_threshold);
    ReadScalar(node, "dwell_goal_id", ctx.dwell_goal_id);
    ReadScalar(node, "dwell_start_ms", ctx.dwell_start_ms);
    ReadScalar(node, "dwell_active", ctx.dwell_active);
    ReadScalar(node, "dwell_complete", ctx.dwell_complete);
    ReadScalar(node, "dwell_required_ms", ctx.dwell_required_ms);
    ReadScalar(node, "dwell_remaining_ms", ctx.dwell_remaining_ms);
    ReadScalar(node, "tactical_state_enter_ms", ctx.tactical_state_enter_ms);
    ReadScalar(node, "tactical_state_min_hold_ms", ctx.tactical_state_min_hold_ms);
    ReadScalar(node, "internal_motion_initialized", ctx.internal_motion_initialized);
    ReadScalar(node, "internal_motion_last_change_ms", ctx.internal_motion_last_change_ms);
    ReadScalar(node, "internal_motion_min_hold_ms", ctx.internal_motion_min_hold_ms);
    ReadScalar(node, "posture_debuff_threshold_ms", ctx.posture_debuff_threshold_ms);
    ReadScalar(node, "posture_debuff_rotate_margin_ms", ctx.posture_debuff_rotate_margin_ms);

    Posture posture = Posture::MOVE;
    if (node["reported_posture"] &&
        ParsePosture(node["reported_posture"].as<std::string>(), posture))
    {
        ctx.reported_posture = posture;
    }
    if (node["current_posture"] &&
        ParsePosture(node["current_posture"].as<std::string>(), posture))
    {
        ctx.current_posture = posture;
    }
    if (node["pending_posture_target"] &&
        ParsePosture(node["pending_posture_target"].as<std::string>(), posture))
    {
        ctx.pending_posture_target = posture;
    }
    if (node["desired_posture"] &&
        ParsePosture(node["desired_posture"].as<std::string>(), posture))
    {
        ctx.desired_posture = posture;
    }
    if (node["preferred_posture"] &&
        ParsePosture(node["preferred_posture"].as<std::string>(), posture))
    {
        ctx.preferred_posture = posture;
    }
    InternalMotionState internal_motion = InternalMotionState::NAV;
    if (node["internal_motion_latched"] &&
        ParseInternalMotionState(node["internal_motion_latched"].as<std::string>(),
                                 internal_motion))
    {
        ctx.internal_motion_latched = internal_motion;
    }

    SpinMode spin_mode = SpinMode::OFF;
    if (node["desired_spin_mode"] &&
        ParseSpinMode(node["desired_spin_mode"].as<std::string>(), spin_mode))
    {
        ctx.desired_spin_mode = spin_mode;
    }
    if (node["preferred_spin_mode"] &&
        ParseSpinMode(node["preferred_spin_mode"].as<std::string>(), spin_mode))
    {
        ctx.preferred_spin_mode = spin_mode;
    }
    if (node["spin_filtered_mode"] &&
        ParseSpinMode(node["spin_filtered_mode"].as<std::string>(), spin_mode))
    {
        ctx.spin_filtered_mode = spin_mode;
    }

    if (node["posture_accumulated_ms"])
    {
        const auto accum = node["posture_accumulated_ms"];
        if (accum["attack"]) ctx.posture_accumulated_ms[PostureToIndex(Posture::ATTACK)] =
            accum["attack"].as<std::uint64_t>();
        if (accum["defense"]) ctx.posture_accumulated_ms[PostureToIndex(Posture::DEFENSE)] =
            accum["defense"].as<std::uint64_t>();
        if (accum["move"]) ctx.posture_accumulated_ms[PostureToIndex(Posture::MOVE)] =
            accum["move"].as<std::uint64_t>();
    }

    if (!node["is_dead"])
    {
        ctx.is_dead = ctx.hp <= 0;
    }
    NormalizePostureDebuffs(ctx);
}

rm_interfaces::msg::GimbalStatus MakeDefaultStatus()
{
    rm_interfaces::msg::GimbalStatus status;
    status.game_progress = 4;
    status.stage_remain_time = 420;
    status.robot_id = 7;
    status.robot_level = 1;
    status.current_hp = 400;
    status.maximum_hp = 400;
    status.ally_7_robot_hp = 400;
    status.ally_outpost_hp = 1500;
    status.ally_base_hp = 5000;
    status.shooter_barrel_cooling_value = 30;
    status.shooter_barrel_heat_limit = 260;
    status.chassis_power_limit = 100;
    status.projectile_allowance_17mm = 300;
    status.remaining_gold_coin = 400;
    status.team_17mm_exchange_remain = 1000;
    status.is_disengaged = 1;
    status.instant_revival_cost = 80;
    status.sentry_posture = 3;
    status.chassis_power = 20.0f;
    status.remain_energy = 50.0f;
    return status;
}

void ApplyStatusPatch(const YAML::Node& node, rm_interfaces::msg::GimbalStatus& status)
{
    if (!node)
    {
        return;
    }

    if (node["game_progress"]) status.game_progress = node["game_progress"].as<int>();
    if (node["stage_remain_time"])
    {
        status.stage_remain_time = node["stage_remain_time"].as<int>();
    }
    if (node["current_hp"])
    {
        status.current_hp = static_cast<std::uint16_t>(std::max(0, node["current_hp"].as<int>()));
    }
    if (node["maximum_hp"])
    {
        status.maximum_hp = static_cast<std::uint16_t>(std::max(0, node["maximum_hp"].as<int>()));
    }
    if (node["ally_7_robot_hp"])
    {
        status.ally_7_robot_hp =
            static_cast<std::uint16_t>(std::max(0, node["ally_7_robot_hp"].as<int>()));
    }
    if (node["ally_outpost_hp"])
    {
        status.ally_outpost_hp =
            static_cast<std::uint16_t>(std::max(0, node["ally_outpost_hp"].as<int>()));
    }
    if (node["ally_base_hp"])
    {
        status.ally_base_hp =
            static_cast<std::uint16_t>(std::max(0, node["ally_base_hp"].as<int>()));
    }
    if (node["can_confirm_revival"])
    {
        status.can_confirm_revival =
            static_cast<std::uint8_t>(node["can_confirm_revival"].as<int>());
    }
    if (node["can_buy_instant_revival"])
    {
        status.can_buy_instant_revival =
            static_cast<std::uint8_t>(node["can_buy_instant_revival"].as<int>());
    }
    if (node["instant_revival_cost"])
    {
        status.instant_revival_cost =
            static_cast<std::uint16_t>(std::max(0, node["instant_revival_cost"].as<int>()));
    }
    if (node["is_disengaged"])
    {
        status.is_disengaged = static_cast<std::uint8_t>(node["is_disengaged"].as<int>());
    }
    if (node["shooter_barrel_cooling_value"])
    {
        status.shooter_barrel_cooling_value = static_cast<std::uint16_t>(
            std::max(0, node["shooter_barrel_cooling_value"].as<int>()));
    }
    if (node["shooter_barrel_heat_limit"])
    {
        status.shooter_barrel_heat_limit = static_cast<std::uint16_t>(
            std::max(0, node["shooter_barrel_heat_limit"].as<int>()));
    }
    if (node["shooter_17mm_1_barrel_heat"])
    {
        status.shooter_17mm_1_barrel_heat = static_cast<std::uint16_t>(
            std::max(0, node["shooter_17mm_1_barrel_heat"].as<int>()));
    }
    if (node["chassis_power_limit"])
    {
        status.chassis_power_limit =
            static_cast<std::uint16_t>(std::max(0, node["chassis_power_limit"].as<int>()));
    }
    if (node["chassis_power"]) status.chassis_power = node["chassis_power"].as<float>();
    if (node["remain_energy"]) status.remain_energy = node["remain_energy"].as<float>();
    if (node["projectile_allowance_17mm"])
    {
        status.projectile_allowance_17mm = static_cast<std::uint16_t>(
            std::max(0, node["projectile_allowance_17mm"].as<int>()));
    }
    if (node["remaining_gold_coin"])
    {
        status.remaining_gold_coin =
            static_cast<std::uint16_t>(std::max(0, node["remaining_gold_coin"].as<int>()));
    }
    if (node["team_17mm_exchange_remain"])
    {
        status.team_17mm_exchange_remain = static_cast<std::uint16_t>(
            std::max(0, node["team_17mm_exchange_remain"].as<int>()));
    }
    if (node["exchanged_projectile_allowance"])
    {
        status.exchanged_projectile_allowance = static_cast<std::uint16_t>(
            std::max(0, node["exchanged_projectile_allowance"].as<int>()));
    }
    if (node["rfid_status"]) status.rfid_status = node["rfid_status"].as<std::uint32_t>();
    if (node["rfid_status_2"])
    {
        status.rfid_status_2 = static_cast<std::uint8_t>(node["rfid_status_2"].as<int>());
    }
    if (node["sentry_posture"])
    {
        status.sentry_posture = static_cast<std::uint8_t>(node["sentry_posture"].as<int>());
    }
    if (node["can_activate_energy_mechanism"])
    {
        status.can_activate_energy_mechanism =
            static_cast<std::uint8_t>(node["can_activate_energy_mechanism"].as<int>());
    }
    if (node["armor_id"])
    {
        status.armor_id = static_cast<std::uint8_t>(node["armor_id"].as<int>());
    }
    if (node["hp_deduction_reason"])
    {
        status.hp_deduction_reason =
            static_cast<std::uint8_t>(node["hp_deduction_reason"].as<int>());
    }
}

void SyncStatusPatchToContext(const YAML::Node& status_node, RobotContext& ctx)
{
    auto status = MakeDefaultStatus();
    ApplyStatusPatch(status_node, status);

    RefereeInterface referee;
    referee.ConfigureInputFreshness(300, 800);
    referee.ConfigureSimInput(false, 500);
    referee.UpdateFromStatus(status);
    referee.SyncToContext(ctx);
}

std::string DescribeOutput(const RobotContext& ctx)
{
    std::ostringstream oss;
    oss << "tactical=" << TacticalStateToString(ctx.tactical_state)
        << " rule=" << RuleActionTypeToString(ctx.rule_action_type)
        << " goal=" << ctx.desired_goal
        << " posture=" << PostureToString(ctx.desired_posture)
        << " fire=" << FirePolicyToString(ctx.desired_fire_policy)
        << " spin=" << SpinModeToString(ctx.desired_spin_mode)
        << " supercap=" << SupercapModeToString(ctx.desired_supercap_mode)
        << " revive=" << static_cast<int>(ctx.revive_cmd)
        << " remote_ammo=" << static_cast<int>(ctx.remote_ammo_req_inc)
        << " remote_hp=" << static_cast<int>(ctx.remote_hp_req_inc)
        << " posture_cmd=" << static_cast<int>(ctx.posture_cmd_referee)
        << " energy=" << static_cast<int>(ctx.activate_energy_confirm)
        << " claim=" << static_cast<int>(ctx.claim_periodic_ammo)
        << " ammo_target=" << ctx.ammo_exchange_target_total;
    if (ctx.health_data_degraded)
    {
        oss << " health_degraded=true";
    }
    return oss.str();
}

void AddFailure(std::vector<std::string>& failures, const std::string& key,
                const std::string& expected, const std::string& actual)
{
    std::ostringstream oss;
    oss << key << ": expected " << expected << ", actual " << actual;
    failures.push_back(oss.str());
}

void CheckString(const YAML::Node& expected, const char* key, const std::string& actual,
                 std::vector<std::string>& failures)
{
    if (expected[key])
    {
        const auto want = expected[key].as<std::string>();
        if (actual != want)
        {
            AddFailure(failures, key, want, actual);
        }
    }
}

void CheckInt(const YAML::Node& expected, const char* key, int actual,
              std::vector<std::string>& failures)
{
    if (expected[key])
    {
        const int want = expected[key].as<int>();
        if (actual != want)
        {
            AddFailure(failures, key, std::to_string(want), std::to_string(actual));
        }
    }
}

void CheckBool(const YAML::Node& expected, const char* key, bool actual,
               std::vector<std::string>& failures)
{
    if (expected[key])
    {
        const bool want = expected[key].as<bool>();
        if (actual != want)
        {
            AddFailure(failures, key, want ? "true" : "false", actual ? "true" : "false");
        }
    }
}

std::vector<std::string> CheckExpected(const RobotContext& ctx, const YAML::Node& expected)
{
    std::vector<std::string> failures;
    CheckString(expected, "tactical_state", TacticalStateToString(ctx.tactical_state), failures);
    CheckString(expected, "rule_action_type", RuleActionTypeToString(ctx.rule_action_type),
                failures);
    CheckString(expected, "desired_goal", ctx.desired_goal, failures);
    CheckString(expected, "desired_posture", PostureToString(ctx.desired_posture), failures);
    CheckString(expected, "fire_policy", FirePolicyToString(ctx.desired_fire_policy), failures);
    CheckString(expected, "spin_mode", SpinModeToString(ctx.desired_spin_mode), failures);
    CheckString(expected, "supercap_mode", SupercapModeToString(ctx.desired_supercap_mode),
                failures);
    CheckInt(expected, "revive_cmd", ctx.revive_cmd, failures);
    CheckInt(expected, "remote_ammo_req_inc", ctx.remote_ammo_req_inc, failures);
    CheckInt(expected, "remote_hp_req_inc", ctx.remote_hp_req_inc, failures);
    CheckInt(expected, "posture_cmd_referee", ctx.posture_cmd_referee, failures);
    CheckInt(expected, "activate_energy_confirm", ctx.activate_energy_confirm, failures);
    CheckInt(expected, "claim_periodic_ammo", ctx.claim_periodic_ammo, failures);
    CheckInt(expected, "ammo_exchange_target_total", ctx.ammo_exchange_target_total, failures);
    CheckInt(expected, "hp", ctx.hp, failures);
    CheckInt(expected, "hp_max", ctx.hp_max, failures);
    CheckBool(expected, "is_dead", ctx.is_dead, failures);
    CheckBool(expected, "heat_guard_active", ctx.heat_guard_active, failures);
    CheckBool(expected, "power_guard_active", ctx.power_guard_active, failures);
    CheckBool(expected, "ammo_guard_active", ctx.ammo_guard_active, failures);
    CheckBool(expected, "supercap_guard_active", ctx.supercap_guard_active, failures);
    CheckBool(expected, "need_emergency_safety", ctx.need_emergency_safety, failures);
    CheckBool(expected, "on_base", ctx.on_base, failures);
    CheckBool(expected, "under_attack", ctx.under_attack, failures);
    CheckBool(expected, "health_data_degraded", ctx.health_data_degraded, failures);
    return failures;
}

bool RunCase(const YAML::Node& defaults, const YAML::Node& case_node,
             const std::filesystem::path& tree_path)
{
    const std::string name =
        case_node["name"] ? case_node["name"].as<std::string>() : "unnamed_case";
    auto ctx = std::make_shared<RobotContext>();
    SetDefaultContext(*ctx);
    ApplyContextPatch(defaults, *ctx);
    if (case_node["status"])
    {
        SyncStatusPatchToContext(case_node["status"], *ctx);
    }
    ApplyContextPatch(case_node["context"], *ctx);

    BT::BehaviorTreeFactory factory;
    RegisterAllNodes(factory, ctx);
    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", ctx);
    auto tree = factory.createTreeFromFile(tree_path.string(), blackboard);

    const int ticks = case_node["ticks"] ? std::max(1, case_node["ticks"].as<int>()) : 1;
    for (int i = 0; i < ticks; ++i)
    {
        TickRootOnceCompat(tree);
    }

    const auto failures = CheckExpected(*ctx, case_node["expected"]);
    if (failures.empty())
    {
        std::cout << "[PASS] " << name << " -> " << DescribeOutput(*ctx) << '\n';
        return true;
    }

    std::cout << "[FAIL] " << name << " -> " << DescribeOutput(*ctx) << '\n';
    for (const auto& failure : failures)
    {
        std::cout << "  - " << failure << '\n';
    }
    std::cout << "  reasons: tactical=" << ctx->tactical_reason
              << "; rule=" << ctx->rule_reason
              << "; goal=" << ctx->goal_reason
              << "; posture=" << ctx->posture_reason
              << "; spin=" << ctx->spin_reason << '\n';
    return false;
}
}  // namespace

int main(int argc, char** argv)
{
    const std::filesystem::path scenario_path =
        (argc > 1) ? std::filesystem::path(argv[1])
                   : (std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "scenarios" /
                      "rule_tuning_matrix.yaml");
    const auto tree_path =
        std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "tree" / "sentry_main.template.xml";

    try
    {
        const YAML::Node root = YAML::LoadFile(scenario_path.string());
        if (!root["cases"] || !root["cases"].IsSequence())
        {
            std::cerr << "scenario file must contain a cases sequence: " << scenario_path
                      << '\n';
            return 2;
        }

        int passed = 0;
        int failed = 0;
        for (const auto& case_node : root["cases"])
        {
            if (RunCase(root["defaults"], case_node, tree_path))
            {
                ++passed;
            }
            else
            {
                ++failed;
            }
        }

        std::cout << "summary: passed=" << passed << " failed=" << failed << '\n';
        return failed == 0 ? 0 : 1;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "sentry_bt_scenario_runner failed: " << ex.what() << '\n';
        return 2;
    }
}
