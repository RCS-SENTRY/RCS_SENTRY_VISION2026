#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <cmath>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "rm_interfaces/msg/gimbal_status.hpp"
#include "rm_interfaces/msg/sentry_intent.hpp"
#include "rm_interfaces/msg/sentry_sim_input.hpp"

namespace
{
std::uint64_t SteadyNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

int ComputeRemoteHpCost(int stage_remain_time)
{
    const int elapsed = std::clamp(420 - stage_remain_time, 0, 420);
    const int rounded_block = (elapsed + 59) / 60;
    return 50 + (rounded_block * 20);
}

template <typename T>
void ReadOptional(const YAML::Node& node, const char* key, std::optional<T>& output)
{
    if (node && node[key])
    {
        output = node[key].as<T>();
    }
}

struct StatusPatch
{
    std::optional<int> game_progress;
    std::optional<int> stage_remain_time;
    std::optional<float> bullet_speed;
    std::optional<std::uint32_t> event_data;
    std::optional<int> current_hp;
    std::optional<int> maximum_hp;
    std::optional<int> shooter_barrel_cooling_value;
    std::optional<int> shooter_barrel_heat_limit;
    std::optional<int> chassis_power_limit;
    std::optional<int> exchanged_projectile_allowance;
    std::optional<int> remote_exchange_projectile_count;
    std::optional<int> remote_exchange_hp_count;
    std::optional<int> can_confirm_revival;
    std::optional<int> can_buy_instant_revival;
    std::optional<int> instant_revival_cost;
    std::optional<int> is_disengaged;
    std::optional<int> team_17mm_exchange_remain;
    std::optional<int> sentry_posture;
    std::optional<int> can_activate_energy_mechanism;
    std::optional<int> projectile_allowance_17mm;
    std::optional<int> remaining_gold_coin;
    std::optional<int> projectile_allowance_fortress;
    std::optional<float> chassis_power;
    std::optional<float> remain_energy;
    std::optional<int> shooter_17mm_1_barrel_heat;
    std::optional<std::uint32_t> rfid_status;
    std::optional<int> rfid_status_2;
    std::optional<int> ally_outpost_hp;
    std::optional<int> ally_base_hp;
};

struct SimPatch
{
    std::optional<bool> enemy_in_view;
    std::optional<float> enemy_confidence;
    std::optional<float> enemy_distance_m;
    std::optional<bool> on_supply;
    std::optional<bool> on_fortress;
    std::optional<bool> on_outpost;
    std::optional<bool> on_highground;
    std::optional<bool> can_claim_periodic_ammo;
    std::optional<bool> posture_switch_requested;
    std::optional<bool> has_hp_override;
    std::optional<int> hp_override;
    std::optional<bool> has_ammo_17_override;
    std::optional<int> ammo_17_override;
    std::optional<bool> has_heat_override;
    std::optional<int> heat_override;
    std::optional<bool> has_gold_override;
    std::optional<int> gold_override;
    std::optional<bool> has_stage_remain_time_override;
    std::optional<int> stage_remain_time_override;
    std::optional<bool> has_is_disengaged_override;
    std::optional<bool> is_disengaged_override;
    std::optional<bool> has_sentry_posture_override;
    std::optional<int> sentry_posture_override;
    std::optional<bool> has_can_activate_energy_mechanism_override;
    std::optional<bool> can_activate_energy_mechanism_override;
};

struct Phase
{
    std::uint64_t duration_ms{0};
    StatusPatch status_patch{};
    SimPatch sim_patch{};
    std::optional<bool> publish_status{};
    std::optional<bool> publish_sim_input{};
};

StatusPatch ParseStatusPatch(const YAML::Node& node)
{
    StatusPatch patch;
    if (!node)
    {
        return patch;
    }

    ReadOptional(node, "game_progress", patch.game_progress);
    ReadOptional(node, "stage_remain_time", patch.stage_remain_time);
    ReadOptional(node, "bullet_speed", patch.bullet_speed);
    ReadOptional(node, "event_data", patch.event_data);
    ReadOptional(node, "current_hp", patch.current_hp);
    ReadOptional(node, "maximum_hp", patch.maximum_hp);
    ReadOptional(node, "shooter_barrel_cooling_value", patch.shooter_barrel_cooling_value);
    ReadOptional(node, "shooter_barrel_heat_limit", patch.shooter_barrel_heat_limit);
    ReadOptional(node, "chassis_power_limit", patch.chassis_power_limit);
    ReadOptional(node, "exchanged_projectile_allowance", patch.exchanged_projectile_allowance);
    ReadOptional(
        node, "remote_exchange_projectile_count", patch.remote_exchange_projectile_count);
    ReadOptional(node, "remote_exchange_hp_count", patch.remote_exchange_hp_count);
    ReadOptional(node, "can_confirm_revival", patch.can_confirm_revival);
    ReadOptional(node, "can_buy_instant_revival", patch.can_buy_instant_revival);
    ReadOptional(node, "instant_revival_cost", patch.instant_revival_cost);
    ReadOptional(node, "is_disengaged", patch.is_disengaged);
    ReadOptional(node, "team_17mm_exchange_remain", patch.team_17mm_exchange_remain);
    ReadOptional(node, "sentry_posture", patch.sentry_posture);
    ReadOptional(node, "can_activate_energy_mechanism", patch.can_activate_energy_mechanism);
    ReadOptional(node, "projectile_allowance_17mm", patch.projectile_allowance_17mm);
    ReadOptional(node, "remaining_gold_coin", patch.remaining_gold_coin);
    ReadOptional(node, "projectile_allowance_fortress", patch.projectile_allowance_fortress);
    ReadOptional(node, "chassis_power", patch.chassis_power);
    ReadOptional(node, "remain_energy", patch.remain_energy);
    ReadOptional(node, "shooter_17mm_1_barrel_heat", patch.shooter_17mm_1_barrel_heat);
    ReadOptional(node, "rfid_status", patch.rfid_status);
    ReadOptional(node, "rfid_status_2", patch.rfid_status_2);
    ReadOptional(node, "ally_outpost_hp", patch.ally_outpost_hp);
    ReadOptional(node, "ally_base_hp", patch.ally_base_hp);
    return patch;
}

SimPatch ParseSimPatch(const YAML::Node& node)
{
    SimPatch patch;
    if (!node)
    {
        return patch;
    }

    ReadOptional(node, "enemy_in_view", patch.enemy_in_view);
    ReadOptional(node, "enemy_confidence", patch.enemy_confidence);
    ReadOptional(node, "enemy_distance_m", patch.enemy_distance_m);
    ReadOptional(node, "on_supply", patch.on_supply);
    ReadOptional(node, "on_fortress", patch.on_fortress);
    ReadOptional(node, "on_outpost", patch.on_outpost);
    ReadOptional(node, "on_highground", patch.on_highground);
    ReadOptional(node, "can_claim_periodic_ammo", patch.can_claim_periodic_ammo);
    ReadOptional(node, "posture_switch_requested", patch.posture_switch_requested);
    ReadOptional(node, "has_hp_override", patch.has_hp_override);
    ReadOptional(node, "hp_override", patch.hp_override);
    ReadOptional(node, "has_ammo_17_override", patch.has_ammo_17_override);
    ReadOptional(node, "ammo_17_override", patch.ammo_17_override);
    ReadOptional(node, "has_heat_override", patch.has_heat_override);
    ReadOptional(node, "heat_override", patch.heat_override);
    ReadOptional(node, "has_gold_override", patch.has_gold_override);
    ReadOptional(node, "gold_override", patch.gold_override);
    ReadOptional(node, "has_stage_remain_time_override", patch.has_stage_remain_time_override);
    ReadOptional(node, "stage_remain_time_override", patch.stage_remain_time_override);
    ReadOptional(node, "has_is_disengaged_override", patch.has_is_disengaged_override);
    ReadOptional(node, "is_disengaged_override", patch.is_disengaged_override);
    ReadOptional(node, "has_sentry_posture_override", patch.has_sentry_posture_override);
    ReadOptional(node, "sentry_posture_override", patch.sentry_posture_override);
    ReadOptional(
        node, "has_can_activate_energy_mechanism_override",
        patch.has_can_activate_energy_mechanism_override);
    ReadOptional(
        node, "can_activate_energy_mechanism_override",
        patch.can_activate_energy_mechanism_override);
    return patch;
}

void ApplyPatch(const StatusPatch& patch, rm_interfaces::msg::GimbalStatus& status)
{
    if (patch.game_progress.has_value()) status.game_progress = *patch.game_progress;
    if (patch.stage_remain_time.has_value()) status.stage_remain_time = *patch.stage_remain_time;
    if (patch.bullet_speed.has_value()) status.bullet_speed = *patch.bullet_speed;
    if (patch.event_data.has_value()) status.event_data = *patch.event_data;
    if (patch.current_hp.has_value()) status.current_hp = static_cast<std::uint16_t>(*patch.current_hp);
    if (patch.maximum_hp.has_value()) status.maximum_hp = static_cast<std::uint16_t>(*patch.maximum_hp);
    if (patch.shooter_barrel_cooling_value.has_value())
    {
        status.shooter_barrel_cooling_value =
            static_cast<std::uint16_t>(*patch.shooter_barrel_cooling_value);
    }
    if (patch.shooter_barrel_heat_limit.has_value())
    {
        status.shooter_barrel_heat_limit =
            static_cast<std::uint16_t>(*patch.shooter_barrel_heat_limit);
    }
    if (patch.chassis_power_limit.has_value())
    {
        status.chassis_power_limit = static_cast<std::uint16_t>(*patch.chassis_power_limit);
    }
    if (patch.exchanged_projectile_allowance.has_value())
    {
        status.exchanged_projectile_allowance =
            static_cast<std::uint16_t>(*patch.exchanged_projectile_allowance);
    }
    if (patch.remote_exchange_projectile_count.has_value())
    {
        status.remote_exchange_projectile_count =
            static_cast<std::uint8_t>(*patch.remote_exchange_projectile_count);
    }
    if (patch.remote_exchange_hp_count.has_value())
    {
        status.remote_exchange_hp_count =
            static_cast<std::uint8_t>(*patch.remote_exchange_hp_count);
    }
    if (patch.can_confirm_revival.has_value())
    {
        status.can_confirm_revival = static_cast<std::uint8_t>(*patch.can_confirm_revival);
    }
    if (patch.can_buy_instant_revival.has_value())
    {
        status.can_buy_instant_revival =
            static_cast<std::uint8_t>(*patch.can_buy_instant_revival);
    }
    if (patch.instant_revival_cost.has_value())
    {
        status.instant_revival_cost = static_cast<std::uint16_t>(*patch.instant_revival_cost);
    }
    if (patch.is_disengaged.has_value())
    {
        status.is_disengaged = static_cast<std::uint8_t>(*patch.is_disengaged);
    }
    if (patch.team_17mm_exchange_remain.has_value())
    {
        status.team_17mm_exchange_remain =
            static_cast<std::uint16_t>(*patch.team_17mm_exchange_remain);
    }
    if (patch.sentry_posture.has_value())
    {
        status.sentry_posture = static_cast<std::uint8_t>(*patch.sentry_posture);
    }
    if (patch.can_activate_energy_mechanism.has_value())
    {
        status.can_activate_energy_mechanism =
            static_cast<std::uint8_t>(*patch.can_activate_energy_mechanism);
    }
    if (patch.projectile_allowance_17mm.has_value())
    {
        status.projectile_allowance_17mm =
            static_cast<std::uint16_t>(*patch.projectile_allowance_17mm);
    }
    if (patch.remaining_gold_coin.has_value())
    {
        status.remaining_gold_coin = static_cast<std::uint16_t>(*patch.remaining_gold_coin);
    }
    if (patch.projectile_allowance_fortress.has_value())
    {
        status.projectile_allowance_fortress =
            static_cast<std::uint16_t>(*patch.projectile_allowance_fortress);
    }
    if (patch.chassis_power.has_value()) status.chassis_power = *patch.chassis_power;
    if (patch.remain_energy.has_value()) status.remain_energy = *patch.remain_energy;
    if (patch.shooter_17mm_1_barrel_heat.has_value())
    {
        status.shooter_17mm_1_barrel_heat =
            static_cast<std::uint16_t>(*patch.shooter_17mm_1_barrel_heat);
    }
    if (patch.rfid_status.has_value()) status.rfid_status = *patch.rfid_status;
    if (patch.rfid_status_2.has_value())
    {
        status.rfid_status_2 = static_cast<std::uint8_t>(*patch.rfid_status_2);
    }
    if (patch.ally_outpost_hp.has_value())
    {
        status.ally_outpost_hp = static_cast<std::uint16_t>(*patch.ally_outpost_hp);
    }
    if (patch.ally_base_hp.has_value())
    {
        status.ally_base_hp = static_cast<std::uint16_t>(*patch.ally_base_hp);
    }
}

void ApplyPatch(const SimPatch& patch, rm_interfaces::msg::SentrySimInput& sim_input)
{
    if (patch.enemy_in_view.has_value()) sim_input.enemy_in_view = *patch.enemy_in_view;
    if (patch.enemy_confidence.has_value()) sim_input.enemy_confidence = *patch.enemy_confidence;
    if (patch.enemy_distance_m.has_value()) sim_input.enemy_distance_m = *patch.enemy_distance_m;
    if (patch.on_supply.has_value()) sim_input.on_supply = *patch.on_supply;
    if (patch.on_fortress.has_value()) sim_input.on_fortress = *patch.on_fortress;
    if (patch.on_outpost.has_value()) sim_input.on_outpost = *patch.on_outpost;
    if (patch.on_highground.has_value()) sim_input.on_highground = *patch.on_highground;
    if (patch.can_claim_periodic_ammo.has_value())
    {
        sim_input.can_claim_periodic_ammo = *patch.can_claim_periodic_ammo;
    }
    if (patch.posture_switch_requested.has_value())
    {
        sim_input.posture_switch_requested = *patch.posture_switch_requested;
    }
    if (patch.has_hp_override.has_value()) sim_input.has_hp_override = *patch.has_hp_override;
    if (patch.hp_override.has_value())
    {
        sim_input.hp_override = *patch.hp_override;
        sim_input.has_hp_override = true;
    }
    if (patch.has_ammo_17_override.has_value())
    {
        sim_input.has_ammo_17_override = *patch.has_ammo_17_override;
    }
    if (patch.ammo_17_override.has_value())
    {
        sim_input.ammo_17_override = *patch.ammo_17_override;
        sim_input.has_ammo_17_override = true;
    }
    if (patch.has_heat_override.has_value()) sim_input.has_heat_override = *patch.has_heat_override;
    if (patch.heat_override.has_value())
    {
        sim_input.heat_override = *patch.heat_override;
        sim_input.has_heat_override = true;
    }
    if (patch.has_gold_override.has_value()) sim_input.has_gold_override = *patch.has_gold_override;
    if (patch.gold_override.has_value())
    {
        sim_input.gold_override = *patch.gold_override;
        sim_input.has_gold_override = true;
    }
    if (patch.has_stage_remain_time_override.has_value())
    {
        sim_input.has_stage_remain_time_override = *patch.has_stage_remain_time_override;
    }
    if (patch.stage_remain_time_override.has_value())
    {
        sim_input.stage_remain_time_override = *patch.stage_remain_time_override;
        sim_input.has_stage_remain_time_override = true;
    }
    if (patch.has_is_disengaged_override.has_value())
    {
        sim_input.has_is_disengaged_override = *patch.has_is_disengaged_override;
    }
    if (patch.is_disengaged_override.has_value())
    {
        sim_input.is_disengaged_override = *patch.is_disengaged_override;
        sim_input.has_is_disengaged_override = true;
    }
    if (patch.has_sentry_posture_override.has_value())
    {
        sim_input.has_sentry_posture_override = *patch.has_sentry_posture_override;
    }
    if (patch.sentry_posture_override.has_value())
    {
        sim_input.sentry_posture_override =
            static_cast<std::uint8_t>(*patch.sentry_posture_override);
        sim_input.has_sentry_posture_override = true;
    }
    if (patch.has_can_activate_energy_mechanism_override.has_value())
    {
        sim_input.has_can_activate_energy_mechanism_override =
            *patch.has_can_activate_energy_mechanism_override;
    }
    if (patch.can_activate_energy_mechanism_override.has_value())
    {
        sim_input.can_activate_energy_mechanism_override =
            *patch.can_activate_energy_mechanism_override;
        sim_input.has_can_activate_energy_mechanism_override = true;
    }
}
}  // namespace

class SentryBtSimNode : public rclcpp::Node
{
public:
    SentryBtSimNode()
      : Node("sentry_bt_sim")
    {
        const double tick_hz = std::max(1.0, this->declare_parameter<double>("tick_hz", 20.0));
        intent_topic_ =
            this->declare_parameter<std::string>("intent_topic", "/sentry/intent");
        status_topic_ =
            this->declare_parameter<std::string>("status_topic", "/gimbal_status");
        sim_input_topic_ =
            this->declare_parameter<std::string>("sim_input_topic", "/sentry_bt/sim_input");
        override_topic_ =
            this->declare_parameter<std::string>("override_topic", "/sentry_bt/sim_override");
        scenario_path_ = this->declare_parameter<std::string>(
            "scenario_path",
            (std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "scenarios" / "posture_stress.yaml")
                .string());
        sim_posture_apply_delay_ms_ = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("sim_posture_apply_delay_ms", 150)));
        disengage_delay_ms_ = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("disengage_delay_ms", 6000)));
        fire_rate_hz_ = std::max(1.0, this->declare_parameter<double>("fire_rate_hz", 8.0));
        fire_heat_per_shot_ =
            std::max(1.0, this->declare_parameter<double>("fire_heat_per_shot", 10.0));

        status_ = MakeDefaultStatus();
        sim_input_ = MakeDefaultSimInput();
        heat_state_ = static_cast<double>(status_.shooter_17mm_1_barrel_heat);
        last_tick_ms_ = SteadyNowMs();
        last_combat_activity_ms_ = last_tick_ms_;

        LoadScenario(scenario_path_);

        intent_sub_ = this->create_subscription<rm_interfaces::msg::SentryIntent>(
            intent_topic_, rclcpp::SensorDataQoS(),
            [this](const rm_interfaces::msg::SentryIntent::SharedPtr msg) {
                this->OnIntent(*msg);
            });
        override_sub_ = this->create_subscription<rm_interfaces::msg::SentrySimInput>(
            override_topic_, rclcpp::SensorDataQoS(),
            [this](const rm_interfaces::msg::SentrySimInput::SharedPtr msg) {
                this->OnOverride(*msg);
            });

        status_pub_ = this->create_publisher<rm_interfaces::msg::GimbalStatus>(
            status_topic_, rclcpp::SensorDataQoS());
        sim_input_pub_ = this->create_publisher<rm_interfaces::msg::SentrySimInput>(
            sim_input_topic_, rclcpp::SensorDataQoS());

        const auto tick_period =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / tick_hz));
        timer_ = this->create_wall_timer(tick_period, [this]() { this->Tick(); });

        RCLCPP_INFO(
            get_logger(),
            "sentry_bt_sim initialized: intent=%s status=%s sim_input=%s scenario=%s",
            intent_topic_.c_str(), status_topic_.c_str(), sim_input_topic_.c_str(),
            scenario_path_.c_str());
    }

private:
    static rm_interfaces::msg::GimbalStatus MakeDefaultStatus()
    {
        rm_interfaces::msg::GimbalStatus status;
        status.header.frame_id = "sentry_bt_sim";
        status.mode = 0;
        status.game_status_nav = 1;
        status.game_progress = 4;
        status.stage_remain_time = 420;
        status.robot_id = 7;
        status.robot_level = 1;
        status.current_hp = 400;
        status.maximum_hp = 400;
        status.shooter_barrel_cooling_value = 30;
        status.shooter_barrel_heat_limit = 260;
        status.chassis_power_limit = 100;
        status.exchanged_projectile_allowance = 0;
        status.remote_exchange_projectile_count = 0;
        status.remote_exchange_hp_count = 0;
        status.can_confirm_revival = 0;
        status.can_buy_instant_revival = 0;
        status.instant_revival_cost = 80;
        status.is_disengaged = 1;
        status.team_17mm_exchange_remain = 1000;
        status.sentry_posture = 3;
        status.can_activate_energy_mechanism = 0;
        status.projectile_allowance_17mm = 300;
        status.projectile_allowance_42mm = 0;
        status.remaining_gold_coin = 500;
        status.projectile_allowance_fortress = 0;
        status.chassis_power = 20.0f;
        status.remain_energy = 50.0f;
        status.buffer_energy = 50;
        status.shooter_17mm_1_barrel_heat = 0;
        status.ally_7_robot_hp = status.current_hp;
        status.ally_outpost_hp = 1500;
        status.ally_base_hp = 5000;
        return status;
    }

    static rm_interfaces::msg::SentrySimInput MakeDefaultSimInput()
    {
        rm_interfaces::msg::SentrySimInput sim_input;
        sim_input.header.frame_id = "sentry_bt_sim";
        sim_input.enemy_in_view = false;
        sim_input.enemy_confidence = 0.0f;
        sim_input.enemy_distance_m = 12.0f;
        sim_input.on_supply = false;
        sim_input.on_fortress = false;
        sim_input.on_outpost = false;
        sim_input.on_highground = false;
        sim_input.can_claim_periodic_ammo = false;
        sim_input.posture_switch_requested = false;
        sim_input.has_hp_override = false;
        sim_input.has_ammo_17_override = false;
        sim_input.has_heat_override = false;
        sim_input.has_gold_override = false;
        sim_input.has_stage_remain_time_override = false;
        sim_input.has_is_disengaged_override = false;
        sim_input.has_sentry_posture_override = false;
        sim_input.has_can_activate_energy_mechanism_override = false;
        return sim_input;
    }

    void LoadScenario(const std::string& scenario_path)
    {
        const YAML::Node root = YAML::LoadFile(scenario_path);
        ApplyPatch(ParseStatusPatch(root["initial_status"]), status_);
        ApplyPatch(ParseSimPatch(root["initial_sim_input"]), sim_input_);
        ApplyPublishedOverrides();
        heat_state_ = static_cast<double>(status_.shooter_17mm_1_barrel_heat);

        phases_.clear();
        if (root["phases"])
        {
            for (const auto& phase_node : root["phases"])
            {
                Phase phase;
                phase.duration_ms =
                    phase_node["duration_ms"] ? phase_node["duration_ms"].as<std::uint64_t>() : 0;
                phase.status_patch = ParseStatusPatch(phase_node["status"]);
                phase.sim_patch = ParseSimPatch(phase_node["sim_input"]);
                ReadOptional(phase_node, "publish_status", phase.publish_status);
                ReadOptional(phase_node, "publish_sim_input", phase.publish_sim_input);
                phases_.push_back(phase);
            }
        }

        current_phase_index_ = 0;
        phase_start_ms_ = last_tick_ms_;
        if (!phases_.empty())
        {
            ApplyPhase(phases_.front());
        }
    }

    void ApplyPhase(const Phase& phase)
    {
        const auto previous_hp = status_.current_hp;
        ApplyPatch(phase.status_patch, status_);
        ApplyPatch(phase.sim_patch, sim_input_);
        if (phase.publish_status.has_value())
        {
            publish_status_ = *phase.publish_status;
        }
        if (phase.publish_sim_input.has_value())
        {
            publish_sim_input_ = *phase.publish_sim_input;
        }
        ApplyPublishedOverrides();
        HandleHpTransition(previous_hp, status_.current_hp);
        heat_state_ = static_cast<double>(status_.shooter_17mm_1_barrel_heat);
    }

    void AdvanceScenario(std::uint64_t now_ms)
    {
        if (current_phase_index_ >= phases_.size())
        {
            return;
        }

        while (current_phase_index_ < phases_.size() &&
               (now_ms - phase_start_ms_) >= phases_[current_phase_index_].duration_ms)
        {
            phase_start_ms_ += phases_[current_phase_index_].duration_ms;
            ++current_phase_index_;
            if (current_phase_index_ < phases_.size())
            {
                ApplyPhase(phases_[current_phase_index_]);
            }
        }
    }

    void HandleHpTransition(std::uint16_t previous_hp, std::uint16_t current_hp)
    {
        if (current_hp < previous_hp)
        {
            last_combat_activity_ms_ = last_tick_ms_;
            status_.is_disengaged = 0;
        }

        if (previous_hp > 0 && current_hp == 0)
        {
            status_.can_confirm_revival = 1;
            status_.can_buy_instant_revival = 1;
            revive_pending_ = false;
            remote_hp_pending_ = false;
            remote_hp_pending_heal_ = 0;
        }
        else if (current_hp > 0)
        {
            status_.can_confirm_revival = 0;
            status_.can_buy_instant_revival = 0;
        }
    }

    void ApplyPublishedOverrides()
    {
        if (sim_input_.has_hp_override)
        {
            status_.current_hp = static_cast<std::uint16_t>(
                std::clamp(sim_input_.hp_override, 0, static_cast<int>(status_.maximum_hp)));
        }
        if (sim_input_.has_ammo_17_override)
        {
            status_.projectile_allowance_17mm = static_cast<std::uint16_t>(
                std::max(0, sim_input_.ammo_17_override));
        }
        if (sim_input_.has_heat_override)
        {
            status_.shooter_17mm_1_barrel_heat = static_cast<std::uint16_t>(
                std::max(0, sim_input_.heat_override));
            heat_state_ = static_cast<double>(status_.shooter_17mm_1_barrel_heat);
        }
        if (sim_input_.has_gold_override)
        {
            status_.remaining_gold_coin = static_cast<std::uint16_t>(
                std::max(0, sim_input_.gold_override));
        }
        if (sim_input_.has_stage_remain_time_override)
        {
            status_.stage_remain_time =
                std::max(0, sim_input_.stage_remain_time_override);
        }
        if (sim_input_.has_is_disengaged_override)
        {
            status_.is_disengaged = sim_input_.is_disengaged_override ? 1 : 0;
        }
        if (sim_input_.has_sentry_posture_override)
        {
            status_.sentry_posture = sim_input_.sentry_posture_override;
            posture_transition_pending_ = false;
        }
        if (sim_input_.has_can_activate_energy_mechanism_override)
        {
            status_.can_activate_energy_mechanism =
                sim_input_.can_activate_energy_mechanism_override ? 1 : 0;
        }
    }

    void ApplyAmmoTarget(std::uint16_t ammo_exchange_target_total)
    {
        const bool on_base_rfid = (status_.rfid_status & 0x1U) != 0U;
        if (!(sim_input_.on_supply || sim_input_.on_outpost || sim_input_.on_fortress ||
              on_base_rfid))
        {
            return;
        }

        if (ammo_exchange_target_total <= status_.exchanged_projectile_allowance)
        {
            return;
        }

        const int delta = std::min<int>(
            ammo_exchange_target_total - status_.exchanged_projectile_allowance,
            status_.team_17mm_exchange_remain);
        const int affordable_delta =
            (std::min(delta, static_cast<int>(status_.remaining_gold_coin)) / 10) * 10;
        if (affordable_delta <= 0)
        {
            return;
        }

        status_.exchanged_projectile_allowance =
            static_cast<std::uint16_t>(status_.exchanged_projectile_allowance + affordable_delta);
        status_.projectile_allowance_17mm =
            static_cast<std::uint16_t>(status_.projectile_allowance_17mm + affordable_delta);
        status_.team_17mm_exchange_remain =
            static_cast<std::uint16_t>(status_.team_17mm_exchange_remain - affordable_delta);
        status_.remaining_gold_coin =
            static_cast<std::uint16_t>(status_.remaining_gold_coin - affordable_delta);
    }

    void ApplyRemoteAmmoRequest(std::uint8_t count, std::uint64_t now_ms)
    {
        if (count == 0 || status_.is_disengaged == 0 || remote_ammo_pending_)
        {
            return;
        }

        for (std::uint8_t i = 0; i < count; ++i)
        {
            if (status_.remaining_gold_coin < 150 || status_.team_17mm_exchange_remain < 100)
            {
                break;
            }
            status_.remaining_gold_coin =
                static_cast<std::uint16_t>(status_.remaining_gold_coin - 150);
            status_.team_17mm_exchange_remain =
                static_cast<std::uint16_t>(status_.team_17mm_exchange_remain - 100);
            status_.remote_exchange_projectile_count =
                static_cast<std::uint8_t>(status_.remote_exchange_projectile_count + 1);
            remote_ammo_pending_rounds_ =
                static_cast<std::uint16_t>(remote_ammo_pending_rounds_ + 100);
            remote_ammo_pending_ = true;
            remote_ammo_apply_at_ms_ = now_ms + 6000U;
        }
    }

    void ApplyRemoteHpRequest(std::uint8_t count, std::uint64_t now_ms)
    {
        if (count == 0 || status_.is_disengaged == 0 || remote_hp_pending_)
        {
            return;
        }

        for (std::uint8_t i = 0; i < count; ++i)
        {
            const int cost = ComputeRemoteHpCost(status_.stage_remain_time);
            if (status_.remaining_gold_coin < cost)
            {
                break;
            }
            status_.remaining_gold_coin =
                static_cast<std::uint16_t>(status_.remaining_gold_coin - cost);
            status_.remote_exchange_hp_count =
                static_cast<std::uint8_t>(status_.remote_exchange_hp_count + 1);
            remote_hp_pending_heal_ =
                std::max(1, static_cast<int>(std::lround(status_.maximum_hp * 0.60)));
            remote_hp_pending_ = true;
            remote_hp_apply_at_ms_ = now_ms + 6000U;
        }
    }

    void ApplyReviveCommand(std::uint8_t revive_cmd, std::uint64_t now_ms)
    {
        if (status_.current_hp != 0)
        {
            return;
        }

        if (revive_cmd == 1 && status_.can_confirm_revival != 0)
        {
            revive_pending_ = true;
            revive_apply_at_ms_ = now_ms + 1500U;
        }
        else if (revive_cmd == 2 && status_.can_buy_instant_revival != 0 &&
                 status_.remaining_gold_coin >= status_.instant_revival_cost)
        {
            status_.remaining_gold_coin = static_cast<std::uint16_t>(
                status_.remaining_gold_coin - status_.instant_revival_cost);
            revive_pending_ = true;
            revive_apply_at_ms_ = now_ms + 500U;
        }
    }

    void OnIntent(const rm_interfaces::msg::SentryIntent& intent)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_intent_ = intent;
        has_last_intent_ = true;

        const std::uint64_t now_ms = SteadyNowMs();
        ApplyAmmoTarget(intent.ammo_exchange_target_total);
        ApplyRemoteAmmoRequest(intent.request_remote_ammo_once, now_ms);
        ApplyRemoteHpRequest(intent.request_remote_hp_once, now_ms);
        if (intent.request_instant_revive != 0)
        {
            ApplyReviveCommand(2, now_ms);
        }
        else if (intent.request_revive_confirm != 0)
        {
            ApplyReviveCommand(1, now_ms);
        }

        if (intent.request_posture_referee != 0)
        {
            posture_transition_pending_ = true;
            pending_posture_protocol_ = intent.request_posture_referee;
            posture_apply_at_ms_ = now_ms + sim_posture_apply_delay_ms_;
        }
        if (intent.request_activate_energy != 0 && status_.can_activate_energy_mechanism != 0)
        {
            status_.can_activate_energy_mechanism = 0;
        }
    }

    void OnOverride(const rm_interfaces::msg::SentrySimInput& override_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto previous_hp = status_.current_hp;
        sim_input_ = override_msg;
        ApplyPublishedOverrides();
        HandleHpTransition(previous_hp, status_.current_hp);
        heat_state_ = static_cast<double>(status_.shooter_17mm_1_barrel_heat);
    }

    void Tick()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::uint64_t now_ms = SteadyNowMs();
        const double dt_s =
            static_cast<double>(now_ms - last_tick_ms_) / 1000.0;
        last_tick_ms_ = now_ms;

        AdvanceScenario(now_ms);

        if (status_.game_progress == 4 && status_.stage_remain_time > 0)
        {
            stage_time_accumulator_s_ += dt_s;
            while (stage_time_accumulator_s_ >= 1.0 && status_.stage_remain_time > 0)
            {
                stage_time_accumulator_s_ -= 1.0;
                --status_.stage_remain_time;
            }
        }

        if (posture_transition_pending_ && now_ms >= posture_apply_at_ms_)
        {
            status_.sentry_posture = pending_posture_protocol_;
            posture_transition_pending_ = false;
        }

        if (revive_pending_ && now_ms >= revive_apply_at_ms_)
        {
            status_.current_hp = status_.maximum_hp;
            status_.is_disengaged = 1;
            status_.can_confirm_revival = 0;
            status_.can_buy_instant_revival = 0;
            revive_pending_ = false;
            last_combat_activity_ms_ = now_ms;
        }

        if (remote_ammo_pending_ && now_ms >= remote_ammo_apply_at_ms_)
        {
            status_.projectile_allowance_17mm =
                static_cast<std::uint16_t>(status_.projectile_allowance_17mm +
                                           remote_ammo_pending_rounds_);
            remote_ammo_pending_rounds_ = 0;
            remote_ammo_pending_ = false;
        }

        if (remote_hp_pending_ && now_ms >= remote_hp_apply_at_ms_)
        {
            if (status_.current_hp > 0)
            {
                status_.current_hp = static_cast<std::uint16_t>(std::min<int>(
                    status_.maximum_hp, static_cast<int>(status_.current_hp) +
                                            remote_hp_pending_heal_));
            }
            remote_hp_pending_heal_ = 0;
            remote_hp_pending_ = false;
        }

        bool fired_this_tick = false;
        status_.bullet_speed = 0.0f;
        fire_accumulator_ = 0.0;

        if (fired_this_tick)
        {
            status_.bullet_speed = 18.0f;
            status_.is_disengaged = 0;
            last_combat_activity_ms_ = now_ms;
        }

        heat_state_ = std::max(
            0.0,
            heat_state_ - static_cast<double>(status_.shooter_barrel_cooling_value) * dt_s);
        status_.shooter_17mm_1_barrel_heat =
            static_cast<std::uint16_t>(std::lround(heat_state_));

        if (!sim_input_.has_is_disengaged_override && status_.current_hp > 0 &&
            (now_ms - last_combat_activity_ms_) >= disengage_delay_ms_)
        {
            status_.is_disengaged = 1;
        }

        const auto previous_hp = status_.current_hp;
        ApplyPublishedOverrides();
        HandleHpTransition(previous_hp, status_.current_hp);

        status_.mode = has_last_intent_ ? 1 : 0;
        status_.game_status_nav = status_.game_progress == 4 ? 1 : 0;
        status_.maximum_hp = std::max<std::uint16_t>(1, status_.maximum_hp);
        status_.current_hp = static_cast<std::uint16_t>(std::min<int>(
            status_.maximum_hp, static_cast<int>(status_.current_hp)));
        status_.ally_7_robot_hp = status_.current_hp;
        status_.shooter_barrel_heat_limit =
            std::max<std::uint16_t>(1, status_.shooter_barrel_heat_limit);
        status_.header.stamp = this->now();
        sim_input_.header.stamp = status_.header.stamp;

        if (publish_status_)
        {
            status_pub_->publish(status_);
        }
        if (publish_sim_input_)
        {
            sim_input_pub_->publish(sim_input_);
        }
    }

    std::mutex mutex_{};
    rm_interfaces::msg::GimbalStatus status_{};
    rm_interfaces::msg::SentrySimInput sim_input_{};
    rm_interfaces::msg::SentryIntent last_intent_{};
    bool has_last_intent_{false};
    bool publish_status_{true};
    bool publish_sim_input_{true};
    bool posture_transition_pending_{false};
    std::uint8_t pending_posture_protocol_{0};
    std::uint64_t posture_apply_at_ms_{0};
    bool revive_pending_{false};
    std::uint64_t revive_apply_at_ms_{0};
    bool remote_ammo_pending_{false};
    std::uint64_t remote_ammo_apply_at_ms_{0};
    std::uint16_t remote_ammo_pending_rounds_{0};
    bool remote_hp_pending_{false};
    std::uint64_t remote_hp_apply_at_ms_{0};
    int remote_hp_pending_heal_{0};
    std::uint64_t last_tick_ms_{0};
    std::uint64_t last_combat_activity_ms_{0};
    std::uint64_t phase_start_ms_{0};
    std::uint64_t sim_posture_apply_delay_ms_{150};
    std::uint64_t disengage_delay_ms_{6000};
    double fire_rate_hz_{8.0};
    double fire_heat_per_shot_{10.0};
    double fire_accumulator_{0.0};
    double stage_time_accumulator_s_{0.0};
    double heat_state_{0.0};
    std::vector<Phase> phases_{};
    std::size_t current_phase_index_{0};
    std::string intent_topic_{};
    std::string status_topic_{};
    std::string sim_input_topic_{};
    std::string override_topic_{};
    std::string scenario_path_{};
    rclcpp::Subscription<rm_interfaces::msg::SentryIntent>::SharedPtr intent_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::SentrySimInput>::SharedPtr override_sub_{};
    rclcpp::Publisher<rm_interfaces::msg::GimbalStatus>::SharedPtr status_pub_{};
    rclcpp::Publisher<rm_interfaces::msg::SentrySimInput>::SharedPtr sim_input_pub_{};
    rclcpp::TimerBase::SharedPtr timer_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<SentryBtSimNode>());
    }
    catch (const std::exception& ex)
    {
        std::cerr << "sentry_bt_sim failed: " << ex.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
