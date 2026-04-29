#include "shooter_interface.hpp"

#include <algorithm>
#include <sstream>

#include "sentry_decision_protocol.h"

namespace
{
std::uint8_t GoalIdToProtocol(const std::string& goal)
{
    if (goal == "SAFE_HOLD") return SENTRY_GOAL_ID_SAFE_HOLD;
    if (goal == "WAIT_REVIVE") return SENTRY_GOAL_ID_WAIT_REVIVE;
    if (goal == "SAFE_RETREAT_A") return SENTRY_GOAL_ID_SAFE_RETREAT_A;
    if (goal == "SAFE_RETREAT_B") return SENTRY_GOAL_ID_SAFE_RETREAT_B;
    if (goal == "SUPPLY_LEFT") return SENTRY_GOAL_ID_SUPPLY_LEFT;
    if (goal == "SUPPLY_RIGHT") return SENTRY_GOAL_ID_SUPPLY_RIGHT;
    if (goal == "FORTRESS_HOLD") return SENTRY_GOAL_ID_FORTRESS_HOLD;
    if (goal == "OUTPOST_HOLD") return SENTRY_GOAL_ID_OUTPOST_HOLD;
    if (goal == "COMBAT_KITE_A") return SENTRY_GOAL_ID_COMBAT_KITE_A;
    if (goal == "COMBAT_HOLD_A") return SENTRY_GOAL_ID_COMBAT_HOLD_A;
    if (goal == "MID_PRESSURE") return SENTRY_GOAL_ID_MID_PRESSURE;
    if (goal == "HIGHGROUND_PEEK") return SENTRY_GOAL_ID_HIGHGROUND_PEEK;
    if (goal == "COMBAT_PUSH_A") return SENTRY_GOAL_ID_COMBAT_PUSH_A;
    if (goal == "SEARCH_AREA_A") return SENTRY_GOAL_ID_SEARCH_AREA_A;
    if (goal == "SEARCH_AREA_B") return SENTRY_GOAL_ID_SEARCH_AREA_B;
    if (goal == "HIGHGROUND_SCAN") return SENTRY_GOAL_ID_HIGHGROUND_SCAN;
    if (goal == "HIGHGROUND_CENTER") return SENTRY_GOAL_ID_HIGHGROUND_CENTER;
    if (goal == "MID_CROSS") return SENTRY_GOAL_ID_MID_CROSS;
    return SENTRY_GOAL_ID_INVALID;
}

std::uint8_t TacticalStateToProtocol(TacticalState state)
{
    switch (state)
    {
        case TacticalState::RETREAT:
            return SENTRY_TACTICAL_STATE_RETREAT;
        case TacticalState::RESUPPLY:
            return SENTRY_TACTICAL_STATE_RESUPPLY;
        case TacticalState::HOLD:
            return SENTRY_TACTICAL_STATE_HOLD;
        case TacticalState::ENGAGE:
            return SENTRY_TACTICAL_STATE_ENGAGE;
        case TacticalState::SEARCH:
            return SENTRY_TACTICAL_STATE_SEARCH;
        case TacticalState::REPOSITION:
            return SENTRY_TACTICAL_STATE_REPOSITION;
    }
    return SENTRY_TACTICAL_STATE_SEARCH;
}

std::uint8_t PostureToProtocol(Posture posture)
{
    switch (posture)
    {
        case Posture::ATTACK:
            return SENTRY_POSTURE_ATTACK;
        case Posture::DEFENSE:
            return SENTRY_POSTURE_DEFENSE;
        case Posture::MOVE:
            return SENTRY_POSTURE_MOVE;
    }
    return SENTRY_POSTURE_MOVE;
}

std::int8_t PostureToLegacyStateSwitch(Posture posture)
{
    switch (posture)
    {
        case Posture::MOVE:
            return 1;
        case Posture::ATTACK:
            return 2;
        case Posture::DEFENSE:
            return 3;
    }
    return 1;
}

std::uint8_t FirePolicyToProtocol(FirePolicy policy)
{
    switch (policy)
    {
        case FirePolicy::HOLD_FIRE:
            return SENTRY_FIRE_POLICY_HOLD_FIRE;
        case FirePolicy::CONSERVATIVE:
            return SENTRY_FIRE_POLICY_CONSERVATIVE;
        case FirePolicy::NORMAL:
            return SENTRY_FIRE_POLICY_NORMAL;
        case FirePolicy::AGGRESSIVE:
            return SENTRY_FIRE_POLICY_AGGRESSIVE;
    }
    return SENTRY_FIRE_POLICY_HOLD_FIRE;
}

std::uint8_t SpinModeToProtocol(SpinMode mode)
{
    return mode == SpinMode::ON ? SENTRY_SPIN_MODE_ON : SENTRY_SPIN_MODE_OFF;
}

std::uint8_t SupercapModeToProtocol(SupercapMode mode)
{
    switch (mode)
    {
        case SupercapMode::OFF:
            return SENTRY_SUPERCAP_MODE_OFF;
        case SupercapMode::KEEP:
            return SENTRY_SUPERCAP_MODE_KEEP;
        case SupercapMode::BURST:
            return SENTRY_SUPERCAP_MODE_BURST;
    }
    return SENTRY_SUPERCAP_MODE_OFF;
}

std::uint8_t RuleActionTypeToProtocol(RuleActionType type)
{
    switch (type)
    {
        case RuleActionType::NONE:
            return SENTRY_RULE_ACTION_NONE;
        case RuleActionType::EXCHANGE_AMMO_AT_POINT:
            return SENTRY_RULE_ACTION_EXCHANGE_AMMO_AT_POINT;
        case RuleActionType::REMOTE_AMMO:
            return SENTRY_RULE_ACTION_REMOTE_AMMO;
        case RuleActionType::REMOTE_HP:
            return SENTRY_RULE_ACTION_REMOTE_HP;
        case RuleActionType::ACTIVATE_ENERGY:
            return SENTRY_RULE_ACTION_ACTIVATE_ENERGY;
        case RuleActionType::CLAIM_PERIODIC_AMMO:
            return SENTRY_RULE_ACTION_CLAIM_PERIODIC_AMMO;
        case RuleActionType::SWITCH_POSTURE:
            return SENTRY_RULE_ACTION_SWITCH_POSTURE;
    }
    return SENTRY_RULE_ACTION_NONE;
}
}  // 匿名命名空间

ShooterInterface::ShooterInterface(rclcpp::Node& node)
{
    output_topic_ = node.declare_parameter<std::string>(
        "gimbal_cmd_topic", "/decision/gimbal_cmd");
    publisher_ = node.create_publisher<rm_interfaces::msg::GimbalCmd>(
        output_topic_, rclcpp::SensorDataQoS());

    if (output_topic_ == "/gimbal_cmd")
    {
        RCLCPP_WARN(
            node.get_logger(),
            "sentry_bt is publishing directly to /gimbal_cmd. This will conflict with rm_autoaim "
            "unless you have a dedicated command fusion design.");
    }
}

void ShooterInterface::PublishCommand(RobotContext& ctx)
{
    rm_interfaces::msg::GimbalCmd message;
    std::string summary;

    std::lock_guard<std::mutex> lock(ctx.mtx);

    message.target_yaw = 0.0;
    message.target_pitch = 0.0;
    message.yaw_vel = 0.0;
    message.pitch_vel = 0.0;
    message.state_switch = PostureToLegacyStateSwitch(ctx.desired_posture);
    message.fire_control = ctx.desired_fire_policy == FirePolicy::HOLD_FIRE ? 0 : 1;
    message.mode = message.fire_control != 0 ? 1 : 0;

    message.protocol_version = ctx.protocol_version;
    message.goal_id = GoalIdToProtocol(ctx.desired_goal);
    message.tactical_state = TacticalStateToProtocol(ctx.tactical_state);
    message.posture = PostureToProtocol(ctx.desired_posture);
    message.fire_policy = FirePolicyToProtocol(ctx.desired_fire_policy);
    message.spin_mode = SpinModeToProtocol(ctx.desired_spin_mode);
    message.supercap_mode = SupercapModeToProtocol(ctx.desired_supercap_mode);
    message.rule_action_type = RuleActionTypeToProtocol(ctx.rule_action_type);

    message.ammo_exchange_target_total =
        static_cast<std::uint16_t>(std::min<int>(ctx.ammo_exchange_target_total, 65535));
    message.revive_cmd = ctx.revive_cmd;
    message.remote_ammo_req_inc = ctx.remote_ammo_req_inc;
    message.remote_hp_req_inc = ctx.remote_hp_req_inc;
    message.posture_cmd_referee = ctx.posture_cmd_referee;
    message.activate_energy_confirm = ctx.activate_energy_confirm;
    message.claim_periodic_ammo = ctx.claim_periodic_ammo;

    last_command_ = message;

    std::ostringstream oss;
    oss << "goal=" << ctx.desired_goal
        << " posture=" << PostureToString(ctx.desired_posture)
        << " fire=" << FirePolicyToString(ctx.desired_fire_policy)
        << " spin=" << SpinModeToString(ctx.desired_spin_mode)
        << " supercap=" << SupercapModeToString(ctx.desired_supercap_mode)
        << " rule=" << RuleActionTypeToString(ctx.rule_action_type)
        << " ammo_target_total=" << ctx.ammo_exchange_target_total
        << " revive_cmd=" << static_cast<int>(ctx.revive_cmd)
        << " posture_cmd_referee=" << static_cast<int>(ctx.posture_cmd_referee)
        << " activate_energy=" << static_cast<int>(ctx.activate_energy_confirm)
        << " claim_periodic_ammo=" << static_cast<int>(ctx.claim_periodic_ammo);
    summary = oss.str();
    ctx.last_shooter_command = summary;

    publisher_->publish(last_command_);
}

const rm_interfaces::msg::GimbalCmd& ShooterInterface::last_command() const
{
    return last_command_;
}

const std::string& ShooterInterface::output_topic() const
{
    return output_topic_;
}
