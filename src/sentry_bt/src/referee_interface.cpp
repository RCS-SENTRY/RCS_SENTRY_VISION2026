#include "referee_interface.hpp"

#include <algorithm>
#include <chrono>

namespace
{
std::uint64_t SteadyNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

Posture ParseReportedPosture(std::uint8_t posture_value)
{
    switch (posture_value)
    {
        case 1:
            return Posture::ATTACK;
        case 2:
            return Posture::DEFENSE;
        case 3:
            return Posture::MOVE;
        default:
            return Posture::MOVE;
    }
}
}  // 匿名命名空间

RefereeInterface::RefereeInterface() = default;

void RefereeInterface::UpdateFromStatus(const rm_interfaces::msg::GimbalStatus& status)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Snapshot snapshot;
    snapshot.now_ms = SteadyNowMs();
    snapshot.frame_index = has_snapshot_ ? (latest_snapshot_.frame_index + 1U) : 1U;
    snapshot.game_progress = status.game_progress;
    snapshot.match_started = status.game_progress == 4;
    snapshot.stage_remain_time = std::max(0, status.stage_remain_time);
    snapshot.hp = static_cast<int>(status.current_hp);
    snapshot.hp_max = std::max(1, static_cast<int>(status.maximum_hp));
    snapshot.is_dead = snapshot.hp <= 0;
    snapshot.can_confirm_revive = status.can_confirm_revival != 0;
    snapshot.can_buy_immediate_revive = status.can_buy_instant_revival != 0;
    snapshot.is_disengaged = status.is_disengaged != 0;
    snapshot.immediate_revive_cost = static_cast<int>(status.instant_revival_cost);
    snapshot.heat = static_cast<int>(status.shooter_17mm_1_barrel_heat);
    snapshot.heat_limit = std::max(1, static_cast<int>(status.shooter_barrel_heat_limit));
    snapshot.cooling = static_cast<int>(status.shooter_barrel_cooling_value);
    snapshot.ammo_17 = static_cast<int>(status.projectile_allowance_17mm);
    snapshot.gold = static_cast<int>(status.remaining_gold_coin);
    snapshot.exchanged_projectile_allowance =
        static_cast<int>(status.exchanged_projectile_allowance);
    snapshot.remote_exchange_projectile_count =
        static_cast<int>(status.remote_exchange_projectile_count);
    snapshot.remote_exchange_hp_count = static_cast<int>(status.remote_exchange_hp_count);
    snapshot.team_17mm_exchange_remain = static_cast<int>(status.team_17mm_exchange_remain);
    snapshot.chassis_power_now = status.chassis_power;
    snapshot.chassis_power_limit = std::max(1.0f, static_cast<float>(status.chassis_power_limit));
    snapshot.supercap_soc = std::clamp(status.remain_energy / 100.0f, 0.0f, 1.0f);

    snapshot.current_posture = ParseReportedPosture(status.sentry_posture);
    if (!has_snapshot_)
    {
        last_observed_posture_ = snapshot.current_posture;
        last_posture_change_ms_ = snapshot.now_ms >= 5000U ? (snapshot.now_ms - 5000U) : 0U;
    }
    else if (snapshot.current_posture != last_observed_posture_)
    {
        last_observed_posture_ = snapshot.current_posture;
        last_posture_change_ms_ = snapshot.now_ms;
    }

    snapshot.posture_cooldown_ok = (snapshot.now_ms - last_posture_change_ms_) >= 5000U;
    snapshot.can_activate_energy_mechanism = status.can_activate_energy_mechanism != 0;

    // 当前上行消息里没有单独的“视觉敌方目标”字段，
    // 这里先用是否脱战作为“是否仍处于交战上下文”的代理量。
    snapshot.enemy_in_view = snapshot.match_started && !snapshot.is_disengaged;
    snapshot.enemy_confidence =
        snapshot.enemy_in_view ? ((status.bullet_speed > 12.0f) ? 0.85f : 0.72f) : 0.0f;
    snapshot.enemy_distance_m = snapshot.enemy_in_view ? 6.0f : 12.0f;

    // 当前 /gimbal_status 尚未显式发布“在补给区/堡垒/前哨/高地”的占位信息，
    // 这里先保持 false，等底层状态补进来后再直接接线。
    snapshot.on_supply = false;
    snapshot.on_fortress = false;
    snapshot.on_outpost = false;
    snapshot.on_highground = false;

    // 周期补弹当前也没有直接同步位，先关闭自动触发，
    // 避免 BT 在缺少场地占领条件时误发脉冲。
    snapshot.can_claim_periodic_ammo = false;
    snapshot.posture_switch_requested = false;

    latest_snapshot_ = snapshot;
    has_snapshot_ = true;
}

void RefereeInterface::SyncToContext(RobotContext& ctx) const
{
    std::lock_guard<std::mutex> snapshot_lock(mutex_);
    std::lock_guard<std::mutex> lock(ctx.mtx);

    ctx.frame_index = latest_snapshot_.frame_index;
    ctx.is_dead = latest_snapshot_.is_dead;
    ctx.match_started = latest_snapshot_.match_started;
    ctx.can_confirm_revive = latest_snapshot_.can_confirm_revive;
    ctx.can_buy_immediate_revive = latest_snapshot_.can_buy_immediate_revive;
    ctx.is_disengaged = latest_snapshot_.is_disengaged;
    ctx.immediate_revive_cost = latest_snapshot_.immediate_revive_cost;
    ctx.game_progress = latest_snapshot_.game_progress;
    ctx.stage_remain_time = latest_snapshot_.stage_remain_time;
    ctx.hp = latest_snapshot_.hp;
    ctx.hp_max = latest_snapshot_.hp_max;
    ctx.heat = latest_snapshot_.heat;
    ctx.heat_limit = latest_snapshot_.heat_limit;
    ctx.cooling = latest_snapshot_.cooling;
    ctx.ammo_17 = latest_snapshot_.ammo_17;
    ctx.gold = latest_snapshot_.gold;
    ctx.exchanged_projectile_allowance = latest_snapshot_.exchanged_projectile_allowance;
    ctx.remote_exchange_projectile_count = latest_snapshot_.remote_exchange_projectile_count;
    ctx.remote_exchange_hp_count = latest_snapshot_.remote_exchange_hp_count;
    ctx.team_17mm_exchange_remain = latest_snapshot_.team_17mm_exchange_remain;
    ctx.chassis_power_now = latest_snapshot_.chassis_power_now;
    ctx.chassis_power_limit = latest_snapshot_.chassis_power_limit;
    ctx.supercap_soc = latest_snapshot_.supercap_soc;
    ctx.enemy_in_view = latest_snapshot_.enemy_in_view;
    ctx.enemy_confidence = latest_snapshot_.enemy_confidence;
    ctx.enemy_distance_m = latest_snapshot_.enemy_distance_m;
    ctx.on_supply = latest_snapshot_.on_supply;
    ctx.on_fortress = latest_snapshot_.on_fortress;
    ctx.on_outpost = latest_snapshot_.on_outpost;
    ctx.on_highground = latest_snapshot_.on_highground;
    ctx.current_posture = latest_snapshot_.current_posture;
    ctx.posture_cooldown_ok = latest_snapshot_.posture_cooldown_ok;
    ctx.can_activate_energy_mechanism = latest_snapshot_.can_activate_energy_mechanism;
    ctx.can_claim_periodic_ammo = latest_snapshot_.can_claim_periodic_ammo;
    ctx.posture_switch_requested = latest_snapshot_.posture_switch_requested;
    ctx.now_ms = latest_snapshot_.now_ms;
}

bool RefereeInterface::HasSnapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return has_snapshot_;
}

const RefereeInterface::Snapshot& RefereeInterface::latest_snapshot() const
{
    return latest_snapshot_;
}
