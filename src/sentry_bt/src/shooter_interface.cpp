#include "shooter_interface.hpp"

#include <algorithm>
#include <sstream>

#include "sentry_decision_protocol.h"

namespace
{
std::uint8_t GoalIdToProtocol(const std::string& goal)
{
    if (goal == "CURRENT_HOLD") return SENTRY_GOAL_ID_INVALID;
    if (goal == "SAFE_HOLD") return SENTRY_GOAL_ID_SAFE_HOLD;
    if (goal == "WAIT_REVIVE") return SENTRY_GOAL_ID_WAIT_REVIVE;
    if (goal == "SAFE_RETREAT_A") return SENTRY_GOAL_ID_SAFE_RETREAT_A;
    if (goal == "SAFE_RETREAT_B") return SENTRY_GOAL_ID_SAFE_RETREAT_B;
    if (goal == "SUPPLY_LEFT") return SENTRY_GOAL_ID_SUPPLY_LEFT;
    if (goal == "SUPPLY_RIGHT") return SENTRY_GOAL_ID_SUPPLY_RIGHT;
    if (goal == "FORTRESS_HOLD") return SENTRY_GOAL_ID_FORTRESS_HOLD;
    if (goal == "OUTPOST_HOLD") return SENTRY_GOAL_ID_OUTPOST_HOLD;
    if (goal == "SEARCH_AREA_A") return SENTRY_GOAL_ID_SEARCH_AREA_A;
    if (goal == "SEARCH_AREA_B") return SENTRY_GOAL_ID_SEARCH_AREA_B;
    if (goal == "MID_CROSS") return SENTRY_GOAL_ID_MID_CROSS;
    if (goal == "BASE_HOLD") return SENTRY_GOAL_ID_SAFE_HOLD;
    return SENTRY_GOAL_ID_INVALID;
}

std::uint8_t TacticalStateToProtocol(TacticalState state)
{
    return TacticalStateToProtocolValue(state);
}

std::uint8_t PostureToProtocol(Posture posture)
{
    return PostureToProtocolValue(posture);
}

std::int8_t PostureToStateSwitch(Posture posture)
{
    return static_cast<std::int8_t>(PostureToProtocolValue(posture));
}

std::uint8_t FirePolicyToProtocol(FirePolicy policy)
{
    return FirePolicyToProtocolValue(policy);
}

std::uint8_t SpinModeToProtocol(SpinMode mode)
{
    return SpinModeToProtocolValue(mode);
}

std::uint8_t SupercapModeToProtocol(SupercapMode mode)
{
    return SupercapModeToProtocolValue(mode);
}

std::uint8_t RuleActionTypeToProtocol(RuleActionType type)
{
    return RuleActionTypeToProtocolValue(type);
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
            "Deprecated ShooterInterface was asked to publish directly to /gimbal_cmd. "
            "Do not use this path on the robot; use /sentry/intent + command_mux.");
    }
}

void ShooterInterface::PublishCommand(RobotContext& ctx)
{
    rm_interfaces::msg::GimbalCmd message;
    std::string summary;

    std::lock_guard<std::mutex> lock(ctx.mtx);

    const bool fire_allowed =
        ctx.match_started && ctx.referee_link_fresh && !ctx.is_dead && ctx.enemy_in_view &&
        ctx.enemy_confidence >= 0.55f && ctx.desired_fire_policy != FirePolicy::HOLD_FIRE;

    message.target_yaw = 0.0;
    message.target_pitch = 0.0;
    message.yaw_vel = 0.0;
    message.pitch_vel = 0.0;
    message.state_switch = PostureToStateSwitch(ctx.desired_posture);
    message.fire_control = fire_allowed ? 1 : 0;
    message.mode = (ctx.enemy_in_view && ctx.referee_link_fresh && !ctx.is_dead) ? 1 : 0;

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
        << " claim_periodic_ammo=" << static_cast<int>(ctx.claim_periodic_ammo)
        << " fire_gate=" << (fire_allowed ? "open" : "closed");
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
