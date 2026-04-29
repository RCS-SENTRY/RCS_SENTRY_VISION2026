#pragma once

#include <cstdint>
#include <mutex>

#include "rm_interfaces/msg/gimbal_status.hpp"

#include "robot_context.hpp"

class RefereeInterface
{
public:
    struct Snapshot
    {
        // 这里的快照结构用于承接外部输入。
        // 当前版本由演示数据填充，后续可以直接映射真实裁判系统、
        // 感知、定位、底盘状态等输入源。
        std::uint64_t frame_index{0};
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
        bool on_fortress{false};
        bool on_outpost{false};
        bool on_highground{false};
        Posture current_posture{Posture::MOVE};
        bool posture_cooldown_ok{true};
        bool can_activate_energy_mechanism{false};
        bool can_claim_periodic_ammo{false};
        bool posture_switch_requested{false};
        std::uint64_t now_ms{0};
    };

    RefereeInterface();

    void UpdateFromStatus(const rm_interfaces::msg::GimbalStatus& status);

    // 将最新快照写入 RobotContext。
    // 显式保留这一层拷贝，有助于把“IO 采集”和“BT 决策”两部分职责边界划清。
    void SyncToContext(RobotContext& ctx) const;

    bool HasSnapshot() const;
    const Snapshot& latest_snapshot() const;

private:
    mutable std::mutex mutex_{};
    Snapshot latest_snapshot_{};
    bool has_snapshot_{false};
    Posture last_observed_posture_{Posture::MOVE};
    std::uint64_t last_posture_change_ms_{0};
};
