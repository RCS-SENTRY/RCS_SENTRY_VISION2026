#include "nav_interface.hpp"

#include <algorithm>
#include <sstream>

namespace
{
struct VelocityProfile
{
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    bool reached{false};
};

double NavigationAngularZ(const NavInterface::GoalCommand& command, double nominal_wz)
{
    return command.spin_mode == SpinMode::ON ? 0.0 : nominal_wz;
}

VelocityProfile GoalVelocity(const RobotContext& ctx, const NavInterface::GoalCommand& command,
                             double linear_speed, double strafe_speed, double retreat_speed,
                             double slow_speed, double scan_wz)
{
    // Fake/debug only. 比赛正式链路禁止使用 GoalVelocity():
    // sentry_bt 只发布 goal_id/intent，不拥有 vx/vy/wz。
    // 正式速度必须由导航栈、速度控制层和双雷达避障链生成。
    const std::string& goal = command.goal_id;
    if (!ctx.match_started || ctx.is_dead || !ctx.referee_link_fresh ||
        goal == "CURRENT_HOLD" || goal == "SAFE_HOLD" || goal == "WAIT_REVIVE")
    {
        return VelocityProfile{0.0, 0.0, 0.0, true};
    }

    if (goal == "SAFE_RETREAT_A")
    {
        return VelocityProfile{-retreat_speed, strafe_speed * 0.45,
                               NavigationAngularZ(command, 0.0), false};
    }
    if (goal == "SAFE_RETREAT_B")
    {
        return VelocityProfile{-retreat_speed, -strafe_speed * 0.45,
                               NavigationAngularZ(command, 0.0), false};
    }
    if (goal == "SUPPLY_LEFT")
    {
        return VelocityProfile{-slow_speed, strafe_speed,
                               NavigationAngularZ(command, 0.0), false};
    }
    if (goal == "SUPPLY_RIGHT")
    {
        return VelocityProfile{-slow_speed, -strafe_speed,
                               NavigationAngularZ(command, 0.0), false};
    }
    if (goal == "MID_CROSS")
    {
        return VelocityProfile{linear_speed, 0.0, NavigationAngularZ(command, 0.0), false};
    }
    if (goal == "SEARCH_AREA_A")
    {
        return VelocityProfile{slow_speed, strafe_speed * 0.35,
                               NavigationAngularZ(command, scan_wz), false};
    }
    if (goal == "SEARCH_AREA_B")
    {
        return VelocityProfile{slow_speed, -strafe_speed * 0.35,
                               NavigationAngularZ(command, -scan_wz), false};
    }
    if (goal == "BASE_HOLD" || goal == "FORTRESS_HOLD" || goal == "OUTPOST_HOLD")
    {
        return VelocityProfile{0.0, 0.0, NavigationAngularZ(command, 0.0), true};
    }

    return VelocityProfile{0.0, 0.0, NavigationAngularZ(command, scan_wz * 0.5), false};
}
}  // namespace

NavInterface::NavInterface(rclcpp::Node& node)
{
    enable_debug_nav_cmd_ =
        node.declare_parameter<bool>("enable_debug_nav_cmd", false);
    output_topic_ = node.declare_parameter<std::string>("nav_cmd_topic", "/decision/nav_cmd");
    linear_speed_ = std::max(0.0, node.declare_parameter<double>("nav_linear_speed", 0.75));
    strafe_speed_ = std::max(0.0, node.declare_parameter<double>("nav_strafe_speed", 0.55));
    retreat_speed_ = std::max(0.0, node.declare_parameter<double>("nav_retreat_speed", 0.70));
    slow_speed_ = std::max(0.0, node.declare_parameter<double>("nav_slow_speed", 0.35));
    scan_wz_ = std::max(0.0, node.declare_parameter<double>("nav_scan_wz", 0.45));
    if (enable_debug_nav_cmd_)
    {
        publisher_ = node.create_publisher<rm_interfaces::msg::NavCmd>(
            output_topic_, rclcpp::SensorDataQoS());
    }
}

void NavInterface::PublishCommand(RobotContext& ctx)
{
    std::lock_guard<std::mutex> lock(ctx.mtx);

    last_command_.goal_id = ctx.desired_goal;
    last_command_.tactical_state = ctx.tactical_state;
    last_command_.posture = ctx.desired_posture;
    last_command_.spin_mode = ctx.desired_spin_mode;
    last_command_.supercap_mode = ctx.desired_supercap_mode;

    if (!enable_debug_nav_cmd_)
    {
        last_nav_msg_ = rm_interfaces::msg::NavCmd{};
        ctx.last_nav_command =
            "disabled: NavInterface debug velocity output is off; formal chain is intent-only.";
        return;
    }

    const auto velocity = GoalVelocity(ctx, last_command_, linear_speed_, strafe_speed_,
                                       retreat_speed_, slow_speed_, scan_wz_);
    last_nav_msg_.linear_x = velocity.vx;
    last_nav_msg_.linear_y = velocity.vy;
    last_nav_msg_.angular_z = velocity.wz;
    last_nav_msg_.is_reached = velocity.reached ? 1 : 0;

    std::ostringstream oss;
    oss << "goal=" << last_command_.goal_id
        << " tactical=" << TacticalStateToString(last_command_.tactical_state)
        << " posture=" << PostureToString(last_command_.posture)
        << " spin=" << SpinModeToString(last_command_.spin_mode)
        << " supercap=" << SupercapModeToString(last_command_.supercap_mode)
        << " nav(vx=" << last_nav_msg_.linear_x
        << ", vy=" << last_nav_msg_.linear_y
        << ", wz=" << last_nav_msg_.angular_z
        << ", reached=" << static_cast<int>(last_nav_msg_.is_reached) << ")";
    ctx.last_nav_command = oss.str();

    if (publisher_)
    {
        publisher_->publish(last_nav_msg_);
    }
}

const NavInterface::GoalCommand& NavInterface::last_command() const
{
    return last_command_;
}

const rm_interfaces::msg::NavCmd& NavInterface::last_nav_msg() const
{
    return last_nav_msg_;
}

const std::string& NavInterface::output_topic() const
{
    return output_topic_;
}
