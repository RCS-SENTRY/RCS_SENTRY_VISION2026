#include "nav_interface.hpp"

#include <sstream>

NavInterface::NavInterface() = default;

void NavInterface::PublishCommand(RobotContext& ctx)
{
    std::lock_guard<std::mutex> lock(ctx.mtx);

    last_command_.goal_id = ctx.desired_goal;
    last_command_.tactical_state = ctx.tactical_state;
    last_command_.posture = ctx.desired_posture;
    last_command_.spin_mode = ctx.desired_spin_mode;
    last_command_.supercap_mode = ctx.desired_supercap_mode;

    std::ostringstream oss;
    oss << "goal=" << last_command_.goal_id
        << " tactical=" << TacticalStateToString(last_command_.tactical_state)
        << " posture=" << PostureToString(last_command_.posture)
        << " spin=" << SpinModeToString(last_command_.spin_mode)
        << " supercap=" << SupercapModeToString(last_command_.supercap_mode);
    ctx.last_nav_command = oss.str();
}

const NavInterface::GoalCommand& NavInterface::last_command() const
{
    return last_command_;
}
