#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "rm_interfaces/msg/gimbal_status.hpp"
#include "rm_interfaces/msg/sentry_sim_input.hpp"

#include "robot_context.hpp"

class RefereeInterface
{
public:
    RefereeInterface();

    void ConfigurePostureTiming(std::uint64_t posture_switch_cooldown_ms,
                                std::uint64_t posture_feedback_stable_ms);
    void ConfigurePostureDebuff(std::uint64_t posture_debuff_threshold_ms,
                                std::uint64_t posture_debuff_rotate_margin_ms);
    void ConfigureInputFreshness(std::uint64_t status_timeout_ms,
                                 std::uint64_t enemy_memory_ms);
    void ConfigureSimInput(bool enable_sim_input, std::uint64_t sim_input_timeout_ms);

    void UpdateFromStatus(const rm_interfaces::msg::GimbalStatus& status);
    void UpdateFromSimInput(const rm_interfaces::msg::SentrySimInput& sim_input);
    void ObserveDecisionOutput(RobotContext& ctx);

    // 将最新输入与本地时序状态机合并后写入 RobotContext。
    void SyncToContext(RobotContext& ctx);

    bool HasSnapshot() const;

private:
    mutable std::mutex mutex_{};
    rm_interfaces::msg::GimbalStatus latest_status_{};
    rm_interfaces::msg::SentrySimInput latest_sim_input_{};
    bool has_status_{false};
    bool has_sim_input_{false};
    bool enable_sim_input_{false};
    std::uint64_t frame_index_{0};
    std::uint64_t status_timeout_ms_{300};
    std::uint64_t posture_switch_cooldown_ms_{5000};
    std::uint64_t posture_feedback_stable_ms_{200};
    std::uint64_t posture_debuff_threshold_ms_{180000};
    std::uint64_t posture_debuff_rotate_margin_ms_{15000};
    std::uint64_t sim_input_timeout_ms_{500};
    std::uint64_t enemy_memory_ms_{800};
    std::uint64_t last_status_ms_{0};
    std::uint64_t last_sim_input_ms_{0};
    bool enemy_seen_latched_{false};
    float enemy_confidence_filtered_{0.0f};
    float enemy_distance_filtered_m_{12.0f};
    std::uint64_t last_enemy_observation_ms_{0};
    bool last_energy_available_raw_{false};
    bool energy_activation_pending_{false};
    bool last_periodic_ammo_raw_{false};
    bool periodic_ammo_claim_pending_{false};
    bool last_posture_switch_raw_{false};
    bool posture_switch_request_pending_{false};
    Posture last_reported_posture_{Posture::MOVE};
    std::uint64_t last_reported_posture_change_ms_{0};
    bool has_reported_posture_{false};
    Posture stable_posture_{Posture::MOVE};
    bool stable_posture_initialized_{false};
    bool posture_switch_pending_{false};
    Posture pending_posture_target_{Posture::MOVE};
    bool has_last_valid_hp_{false};
    int last_valid_hp_{400};
    int last_valid_hp_max_{400};
    std::array<std::uint64_t, 3> posture_accumulated_ms_{{0, 0, 0}};
    std::uint64_t last_posture_accounting_ms_{0};
    bool posture_accounting_initialized_{false};
};
