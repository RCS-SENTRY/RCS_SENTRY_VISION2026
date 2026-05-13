#include "referee_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace
{
constexpr float kEnemyConfidenceEnter = 0.70f;
constexpr float kEnemyConfidenceExit = 0.45f;
constexpr float kEnemyFilterAlpha = 0.35f;
constexpr float kDefaultEnemyDistanceM = 12.0f;
constexpr int kDefaultSentryHpMax = 400;
constexpr int kOpeningAmmoFallbackRemainTimeSec = 415;
constexpr int kDefaultOpeningAmmo17 = 200;

bool HasBit(std::uint32_t value, unsigned bit)
{
    return ((value >> bit) & 0x1U) != 0U;
}

bool HasBit8(std::uint8_t value, unsigned bit)
{
    return ((value >> bit) & 0x1U) != 0U;
}

bool AnyBit(std::uint32_t value, const std::vector<int64_t>& bits)
{
    for (const auto bit : bits)
    {
        if (bit >= 0 && bit < 32 && HasBit(value, static_cast<unsigned>(bit)))
        {
            return true;
        }
    }
    return false;
}

bool AnyBit8(std::uint8_t value, const std::vector<int64_t>& bits)
{
    for (const auto bit : bits)
    {
        if (bit >= 0 && bit < 8 && HasBit8(value, static_cast<unsigned>(bit)))
        {
            return true;
        }
    }
    return false;
}

std::uint64_t SteadyNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

float BlendSignal(float previous, float input, float alpha)
{
    return previous + ((input - previous) * alpha);
}

struct MergedSnapshot
{
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
    bool under_attack{false};
    std::uint8_t armor_id{0};
    std::uint8_t hp_deduction_reason{0};
    bool on_supply{false};
    bool on_base{false};
    bool on_fortress{false};
    bool on_outpost{false};
    bool on_highground{false};
    std::uint32_t rfid_status{0};
    std::uint8_t rfid_status_2{0};
    std::uint8_t recovery_buff{0};
    Posture reported_posture{Posture::MOVE};
    Posture current_posture{Posture::MOVE};
    Posture pending_posture_target{Posture::MOVE};
    bool posture_switch_pending{false};
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
    std::array<std::uint64_t, 3> posture_accumulated_ms{{0, 0, 0}};
    std::array<bool, 3> posture_debuffed{{false, false, false}};
    std::uint64_t posture_debuff_threshold_ms{180000};
    std::uint64_t posture_debuff_rotate_margin_ms{15000};
    bool can_activate_energy_mechanism{false};
    bool can_claim_periodic_ammo{false};
    bool posture_switch_requested{false};
    std::uint64_t now_ms{0};
    std::string input_health_reason{};
    std::string health_data_reason{};
};
}  // namespace

RefereeInterface::RefereeInterface() = default;

void RefereeInterface::ConfigurePostureTiming(std::uint64_t posture_switch_cooldown_ms,
                                              std::uint64_t posture_feedback_stable_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    posture_switch_cooldown_ms_ = std::max<std::uint64_t>(1, posture_switch_cooldown_ms);
    posture_feedback_stable_ms_ = std::max<std::uint64_t>(0, posture_feedback_stable_ms);
}

void RefereeInterface::ConfigurePostureDebuff(std::uint64_t posture_debuff_threshold_ms,
                                              std::uint64_t posture_debuff_rotate_margin_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    posture_debuff_threshold_ms_ = std::max<std::uint64_t>(1, posture_debuff_threshold_ms);
    posture_debuff_rotate_margin_ms_ =
        std::min(posture_debuff_rotate_margin_ms, posture_debuff_threshold_ms_);
}

void RefereeInterface::ConfigureInputFreshness(std::uint64_t status_timeout_ms,
                                               std::uint64_t enemy_memory_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    status_timeout_ms_ = std::max<std::uint64_t>(1, status_timeout_ms);
    enemy_memory_ms_ = std::max<std::uint64_t>(1, enemy_memory_ms);
}

void RefereeInterface::ConfigureSimInput(bool enable_sim_input, std::uint64_t sim_input_timeout_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enable_sim_input_ = enable_sim_input;
    sim_input_timeout_ms_ = std::max<std::uint64_t>(1, sim_input_timeout_ms);
}

void RefereeInterface::ConfigureDecisionStatusInputs(
    std::uint64_t autoaim_status_timeout_ms, std::uint64_t nav_status_timeout_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);
    autoaim_status_timeout_ms_ = std::max<std::uint64_t>(1, autoaim_status_timeout_ms);
    nav_status_timeout_ms_ = std::max<std::uint64_t>(1, nav_status_timeout_ms);
}

void RefereeInterface::ConfigureRfidBits(
    const std::vector<int64_t>& base_bits,
    const std::vector<int64_t>& supply_bits,
    const std::vector<int64_t>& fortress_bits,
    const std::vector<int64_t>& outpost_bits,
    const std::vector<int64_t>& highground_bits,
    const std::vector<int64_t>& base_bits_2,
    const std::vector<int64_t>& supply_bits_2,
    const std::vector<int64_t>& fortress_bits_2,
    const std::vector<int64_t>& outpost_bits_2,
    const std::vector<int64_t>& highground_bits_2)
{
    std::lock_guard<std::mutex> lock(mutex_);
    rfid_base_bits_ = base_bits;
    rfid_supply_bits_ = supply_bits;
    rfid_fortress_bits_ = fortress_bits;
    rfid_outpost_bits_ = outpost_bits;
    rfid_highground_bits_ = highground_bits;
    rfid_base_bits_2_ = base_bits_2;
    rfid_supply_bits_2_ = supply_bits_2;
    rfid_fortress_bits_2_ = fortress_bits_2;
    rfid_outpost_bits_2_ = outpost_bits_2;
    rfid_highground_bits_2_ = highground_bits_2;
}

void RefereeInterface::UpdateFromStatus(const rm_interfaces::msg::GimbalStatus& status)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_status_ = status;
    has_status_ = true;
    last_status_ms_ = SteadyNowMs();
    ++frame_index_;
}

void RefereeInterface::UpdateFromSimInput(const rm_interfaces::msg::SentrySimInput& sim_input)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_sim_input_ = sim_input;
    has_sim_input_ = true;
    last_sim_input_ms_ = SteadyNowMs();
}

void RefereeInterface::UpdateFromAutoaimTargetStatus(
    const rm_interfaces::msg::AutoaimTargetStatus& target_status)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_autoaim_target_status_ = target_status;
    has_autoaim_target_status_ = true;
    last_autoaim_target_status_ms_ = SteadyNowMs();
}

void RefereeInterface::UpdateFromNavStatus(const rm_interfaces::msg::SentryNavStatus& nav_status)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_nav_status_ = nav_status;
    has_nav_status_ = true;
    last_nav_status_ms_ = SteadyNowMs();
}

void RefereeInterface::ObserveDecisionOutput(RobotContext& ctx)
{
    std::uint8_t posture_cmd = 0;
    std::uint8_t activate_energy_confirm = 0;
    std::uint8_t claim_periodic_ammo = 0;
    {
        std::lock_guard<std::mutex> lock(ctx.mtx);
        posture_cmd = ctx.posture_cmd_referee;
        activate_energy_confirm = ctx.activate_energy_confirm;
        claim_periodic_ammo = ctx.claim_periodic_ammo;
    }

    if (posture_cmd == 0 && activate_energy_confirm == 0 && claim_periodic_ammo == 0)
    {
        return;
    }

    Posture target = Posture::MOVE;
    const bool has_valid_posture_cmd =
        posture_cmd != 0 && ParsePostureProtocolValue(posture_cmd, target);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_valid_posture_cmd)
        {
            posture_switch_pending_ = true;
            pending_posture_target_ = target;
            posture_switch_request_pending_ = false;
        }
        if (activate_energy_confirm != 0)
        {
            energy_activation_pending_ = false;
        }
        if (claim_periodic_ammo != 0)
        {
            periodic_ammo_claim_pending_ = false;
        }
    }

    if (has_valid_posture_cmd)
    {
        std::lock_guard<std::mutex> ctx_lock(ctx.mtx);
        ctx.posture_switch_pending = true;
        ctx.pending_posture_target = target;
    }
}

void RefereeInterface::SyncToContext(RobotContext& ctx)
{
    std::lock_guard<std::mutex> snapshot_lock(mutex_);
    const std::uint64_t now_ms = SteadyNowMs();
    if (!has_status_)
    {
        return;
    }

    const std::uint64_t status_age_ms =
        (last_status_ms_ == 0 || now_ms < last_status_ms_) ? status_timeout_ms_ + 1
                                                           : (now_ms - last_status_ms_);
    const std::uint64_t sim_input_age_ms =
        (!has_sim_input_ || now_ms < last_sim_input_ms_) ? sim_input_timeout_ms_ + 1
                                                         : (now_ms - last_sim_input_ms_);
    const std::uint64_t autoaim_status_age_ms =
        (!has_autoaim_target_status_ || now_ms < last_autoaim_target_status_ms_)
            ? autoaim_status_timeout_ms_ + 1
            : (now_ms - last_autoaim_target_status_ms_);
    const std::uint64_t nav_status_age_ms =
        (!has_nav_status_ || now_ms < last_nav_status_ms_) ? nav_status_timeout_ms_ + 1
                                                           : (now_ms - last_nav_status_ms_);
    const bool status_fresh = status_age_ms <= status_timeout_ms_;
    const bool use_sim_input =
        enable_sim_input_ && has_sim_input_ && sim_input_age_ms <= sim_input_timeout_ms_;
    const bool autoaim_status_fresh =
        has_autoaim_target_status_ && autoaim_status_age_ms <= autoaim_status_timeout_ms_;
    const bool nav_status_fresh = has_nav_status_ && nav_status_age_ms <= nav_status_timeout_ms_;

    MergedSnapshot snapshot;
    snapshot.now_ms = now_ms;
    snapshot.frame_index = frame_index_;
    snapshot.referee_link_fresh = status_fresh;
    snapshot.sim_input_fresh = use_sim_input;
    snapshot.referee_status_age_ms = status_age_ms;
    snapshot.sim_input_age_ms = sim_input_age_ms;
    snapshot.autoaim_status_age_ms = autoaim_status_age_ms;
    snapshot.nav_status_age_ms = nav_status_age_ms;
    if (nav_status_fresh)
    {
        snapshot.nav_goal_active = latest_nav_status_.active;
        snapshot.nav_goal_reached = latest_nav_status_.reached;
        snapshot.nav_goal_failed = latest_nav_status_.failed;
        snapshot.current_goal_id = latest_nav_status_.goal_id;
    }
    snapshot.game_progress = latest_status_.game_progress;
    const bool referee_says_match_started = latest_status_.game_progress == 4;
    snapshot.match_started = status_fresh && referee_says_match_started;
    snapshot.stage_remain_time = std::max(0, latest_status_.stage_remain_time);
    snapshot.can_confirm_revive = latest_status_.can_confirm_revival != 0;
    snapshot.can_buy_immediate_revive = latest_status_.can_buy_instant_revival != 0;
    const int raw_current_hp = static_cast<int>(latest_status_.current_hp);
    const int raw_ally_sentry_hp = static_cast<int>(latest_status_.ally_7_robot_hp);
    const int raw_max_hp = static_cast<int>(latest_status_.maximum_hp);
    snapshot.hp_max = raw_max_hp > 0
                          ? raw_max_hp
                          : (has_last_valid_hp_ ? last_valid_hp_max_ : kDefaultSentryHpMax);
    snapshot.hp_max = std::max(1, snapshot.hp_max);

    if (raw_current_hp > 0)
    {
        snapshot.hp = raw_current_hp;
    }
    else if (raw_ally_sentry_hp > 0)
    {
        snapshot.hp = raw_ally_sentry_hp;
        snapshot.health_data_degraded = true;
        snapshot.health_data_reason =
            "自身 current_hp 为 0，但 ally_7_robot_hp 有效，使用哨兵队友血量字段兜底。";
    }
    else if (snapshot.can_confirm_revive || snapshot.can_buy_immediate_revive)
    {
        snapshot.hp = 0;
    }
    else if (raw_max_hp == 0)
    {
        snapshot.hp = has_last_valid_hp_ ? last_valid_hp_ : snapshot.hp_max;
        snapshot.health_data_degraded = true;
        snapshot.health_data_reason =
            has_last_valid_hp_
                ? "血量字段缺失，沿用上一帧有效哨兵血量。"
                : "血量字段缺失且没有历史值，按默认存活血量进入降级决策。";
    }
    else if (has_last_valid_hp_)
    {
        snapshot.hp = std::min(last_valid_hp_, snapshot.hp_max);
        snapshot.health_data_degraded = true;
        snapshot.health_data_reason =
            "当前血量为 0 但没有复活信号，沿用上一帧有效血量避免误判战亡。";
    }
    else
    {
        snapshot.hp = snapshot.hp_max;
        snapshot.health_data_degraded = true;
        snapshot.health_data_reason =
            "当前血量为 0 且没有复活信号，缺少历史值，按存活降级处理。";
    }

    snapshot.is_disengaged = latest_status_.is_disengaged != 0;
    snapshot.immediate_revive_cost = static_cast<int>(latest_status_.instant_revival_cost);
    snapshot.heat = static_cast<int>(latest_status_.shooter_17mm_1_barrel_heat);
    snapshot.heat_limit = std::max(1, static_cast<int>(latest_status_.shooter_barrel_heat_limit));
    snapshot.cooling = static_cast<int>(latest_status_.shooter_barrel_cooling_value);
    const int raw_ammo_17 = static_cast<int>(latest_status_.projectile_allowance_17mm);
    snapshot.ammo_17 = raw_ammo_17;
    if (snapshot.match_started && raw_ammo_17 == 0 &&
        snapshot.stage_remain_time >= kOpeningAmmoFallbackRemainTimeSec)
    {
        snapshot.ammo_17 = kDefaultOpeningAmmo17;
        snapshot.health_data_degraded = true;
        snapshot.health_data_reason +=
            " 开局 5 秒内允许发弹量为 0，按规则初始 200 发兜底，避免误触发兑弹。";
    }
    snapshot.gold = static_cast<int>(latest_status_.remaining_gold_coin);
    snapshot.exchanged_projectile_allowance =
        static_cast<int>(latest_status_.exchanged_projectile_allowance);
    snapshot.remote_exchange_projectile_count =
        static_cast<int>(latest_status_.remote_exchange_projectile_count);
    snapshot.remote_exchange_hp_count = static_cast<int>(latest_status_.remote_exchange_hp_count);
    snapshot.team_17mm_exchange_remain =
        static_cast<int>(latest_status_.team_17mm_exchange_remain);
    snapshot.chassis_power_now = latest_status_.chassis_power;
    snapshot.chassis_power_limit =
        std::max(1.0f, static_cast<float>(latest_status_.chassis_power_limit));
    snapshot.supercap_soc = std::clamp(latest_status_.remain_energy / 100.0f, 0.0f, 1.0f);
    snapshot.armor_id = latest_status_.armor_id;
    snapshot.hp_deduction_reason = latest_status_.hp_deduction_reason;
    snapshot.can_activate_energy_mechanism =
        latest_status_.can_activate_energy_mechanism != 0;

    std::uint8_t reported_posture_value = latest_status_.sentry_posture;

    bool raw_enemy_in_view = false;
    float raw_enemy_confidence = 0.0f;
    float raw_enemy_distance_m = kDefaultEnemyDistanceM;
    bool raw_can_claim_periodic_ammo = false;
    bool raw_posture_switch_requested = false;

    if (use_sim_input)
    {
        raw_enemy_in_view = latest_sim_input_.enemy_in_view;
        raw_enemy_confidence = std::clamp(latest_sim_input_.enemy_confidence, 0.0f, 1.0f);
        raw_enemy_distance_m = std::max(0.0f, latest_sim_input_.enemy_distance_m);
        snapshot.on_supply = latest_sim_input_.on_supply;
        snapshot.on_base = false;
        snapshot.on_fortress = latest_sim_input_.on_fortress;
        snapshot.on_outpost = latest_sim_input_.on_outpost;
        snapshot.on_highground = latest_sim_input_.on_highground;
        raw_can_claim_periodic_ammo = latest_sim_input_.can_claim_periodic_ammo;
        raw_posture_switch_requested = latest_sim_input_.posture_switch_requested;

        if (latest_sim_input_.has_hp_override)
        {
            snapshot.hp = latest_sim_input_.hp_override;
        }
        if (latest_sim_input_.has_ammo_17_override)
        {
            snapshot.ammo_17 = latest_sim_input_.ammo_17_override;
        }
        if (latest_sim_input_.has_heat_override)
        {
            snapshot.heat = latest_sim_input_.heat_override;
        }
        if (latest_sim_input_.has_gold_override)
        {
            snapshot.gold = latest_sim_input_.gold_override;
        }
        if (latest_sim_input_.has_stage_remain_time_override)
        {
            snapshot.stage_remain_time = std::max(0, latest_sim_input_.stage_remain_time_override);
        }
        if (latest_sim_input_.has_is_disengaged_override)
        {
            snapshot.is_disengaged = latest_sim_input_.is_disengaged_override;
        }
        if (latest_sim_input_.has_sentry_posture_override)
        {
            reported_posture_value = latest_sim_input_.sentry_posture_override;
        }
        if (latest_sim_input_.has_can_activate_energy_mechanism_override)
        {
            snapshot.can_activate_energy_mechanism =
                latest_sim_input_.can_activate_energy_mechanism_override;
        }
    }
    else
    {
        const std::uint32_t rfid = latest_status_.rfid_status;
        const std::uint8_t rfid_2 = latest_status_.rfid_status_2;
        snapshot.rfid_status = rfid;
        snapshot.rfid_status_2 = rfid_2;
        snapshot.recovery_buff = latest_status_.recovery_buff;
        snapshot.on_base = AnyBit(rfid, rfid_base_bits_) ||
                           AnyBit8(rfid_2, rfid_base_bits_2_);
        snapshot.on_supply = AnyBit(rfid, rfid_supply_bits_) ||
                             AnyBit8(rfid_2, rfid_supply_bits_2_);
        snapshot.on_fortress = AnyBit(rfid, rfid_fortress_bits_) ||
                               AnyBit8(rfid_2, rfid_fortress_bits_2_);
        snapshot.on_outpost = AnyBit(rfid, rfid_outpost_bits_) ||
                              AnyBit8(rfid_2, rfid_outpost_bits_2_);
        snapshot.on_highground = AnyBit(rfid, rfid_highground_bits_) ||
                                 AnyBit8(rfid_2, rfid_highground_bits_2_);
        if (autoaim_status_fresh)
        {
            snapshot.autoaim_has_target = latest_autoaim_target_status_.has_target;
            snapshot.autoaim_tracking = latest_autoaim_target_status_.tracking;
            snapshot.autoaim_fire_ready = latest_autoaim_target_status_.fire_ready;
            snapshot.autoaim_target_distance =
                std::max(0.0f, latest_autoaim_target_status_.target_distance);
            raw_enemy_in_view = latest_autoaim_target_status_.has_target ||
                                latest_autoaim_target_status_.tracking;
            raw_enemy_confidence =
                latest_autoaim_target_status_.fire_ready ? 0.90f
                : latest_autoaim_target_status_.tracking ? 0.75f
                : latest_autoaim_target_status_.has_target ? 0.72f
                                                           : 0.0f;
            raw_enemy_distance_m = snapshot.autoaim_target_distance;
        }
    }

    const bool raw_can_activate_energy = snapshot.can_activate_energy_mechanism;
    if (raw_can_activate_energy && !last_energy_available_raw_)
    {
        energy_activation_pending_ = true;
    }
    if (!raw_can_activate_energy)
    {
        energy_activation_pending_ = false;
    }
    last_energy_available_raw_ = raw_can_activate_energy;

    if (raw_can_claim_periodic_ammo && !last_periodic_ammo_raw_)
    {
        periodic_ammo_claim_pending_ = true;
    }
    if (!raw_can_claim_periodic_ammo)
    {
        periodic_ammo_claim_pending_ = false;
    }
    last_periodic_ammo_raw_ = raw_can_claim_periodic_ammo;

    if (raw_posture_switch_requested && !last_posture_switch_raw_ && snapshot.match_started)
    {
        posture_switch_request_pending_ = true;
    }
    last_posture_switch_raw_ = raw_posture_switch_requested;

    snapshot.can_activate_energy_mechanism =
        snapshot.match_started && energy_activation_pending_;
    snapshot.can_claim_periodic_ammo =
        snapshot.match_started && periodic_ammo_claim_pending_;
    snapshot.posture_switch_requested =
        snapshot.match_started && posture_switch_request_pending_;

    const bool reliable_enemy_sample =
        raw_enemy_in_view && raw_enemy_confidence >= kEnemyConfidenceExit &&
        (use_sim_input || autoaim_status_fresh);
    if (reliable_enemy_sample)
    {
        if (last_enemy_observation_ms_ == 0)
        {
            enemy_confidence_filtered_ = raw_enemy_confidence;
            enemy_distance_filtered_m_ = raw_enemy_distance_m;
        }
        else
        {
            enemy_confidence_filtered_ =
                BlendSignal(enemy_confidence_filtered_, raw_enemy_confidence, kEnemyFilterAlpha);
            enemy_distance_filtered_m_ =
                BlendSignal(enemy_distance_filtered_m_, raw_enemy_distance_m, kEnemyFilterAlpha);
        }
        last_enemy_observation_ms_ = now_ms;
        if (raw_enemy_confidence >= kEnemyConfidenceEnter)
        {
            enemy_seen_latched_ = true;
        }
    }
    else
    {
        enemy_confidence_filtered_ =
            BlendSignal(enemy_confidence_filtered_, 0.0f, kEnemyFilterAlpha);
    }

    if (last_enemy_observation_ms_ == 0 ||
        (!use_sim_input && !autoaim_status_fresh) ||
        (now_ms - last_enemy_observation_ms_) > enemy_memory_ms_ ||
        enemy_confidence_filtered_ < kEnemyConfidenceExit)
    {
        enemy_seen_latched_ = false;
    }
    snapshot.enemy_in_view =
        snapshot.match_started && snapshot.referee_link_fresh && enemy_seen_latched_;
    snapshot.enemy_confidence = snapshot.enemy_in_view ? enemy_confidence_filtered_ : 0.0f;
    snapshot.enemy_distance_m =
        snapshot.enemy_in_view ? enemy_distance_filtered_m_ : kDefaultEnemyDistanceM;

    snapshot.hp = std::clamp(snapshot.hp, 0, snapshot.hp_max);
    if (snapshot.hp > 0)
    {
        last_valid_hp_ = std::clamp(snapshot.hp, 1, snapshot.hp_max);
        last_valid_hp_max_ = snapshot.hp_max;
        has_last_valid_hp_ = true;
    }
    snapshot.heat = std::max(0, snapshot.heat);
    snapshot.ammo_17 = std::max(0, snapshot.ammo_17);
    snapshot.gold = std::max(0, snapshot.gold);
    snapshot.is_dead = snapshot.hp <= 0;
    if (!snapshot.referee_link_fresh)
    {
        snapshot.input_health_reason = "裁判/底盘状态超时，冻结为保守安全决策。";
    }
    else if (enable_sim_input_ && !snapshot.sim_input_fresh)
    {
        snapshot.input_health_reason =
            "裁判/底盘状态新鲜，但战术补充输入超时，敌情与点位信号降级。";
    }
    else if (enable_sim_input_)
    {
        snapshot.input_health_reason = "裁判/底盘状态与战术补充输入均新鲜。";
    }
    else
    {
        snapshot.input_health_reason = "裁判/底盘状态新鲜，未启用战术补充输入。";
    }
    if (snapshot.health_data_degraded)
    {
        snapshot.input_health_reason += " " + snapshot.health_data_reason;
    }

    if (!ParsePostureProtocolValue(reported_posture_value, snapshot.reported_posture))
    {
        snapshot.reported_posture = Posture::MOVE;
    }

    if (!referee_says_match_started)
    {
        posture_accumulated_ms_.fill(0);
        posture_accounting_initialized_ = false;
        last_posture_accounting_ms_ = now_ms;
    }
    else if (snapshot.referee_link_fresh)
    {
        if (!posture_accounting_initialized_)
        {
            last_posture_accounting_ms_ = now_ms;
            posture_accounting_initialized_ = true;
        }
        else if (!snapshot.is_dead && stable_posture_initialized_ &&
                 now_ms > last_posture_accounting_ms_)
        {
            posture_accumulated_ms_[PostureToIndex(stable_posture_)] +=
                (now_ms - last_posture_accounting_ms_);
            last_posture_accounting_ms_ = now_ms;
        }
        else
        {
            last_posture_accounting_ms_ = now_ms;
        }
    }
    else
    {
        last_posture_accounting_ms_ = now_ms;
    }

    if (!has_reported_posture_)
    {
        has_reported_posture_ = true;
        last_reported_posture_ = snapshot.reported_posture;
        last_reported_posture_change_ms_ = now_ms;
    }
    else if (snapshot.reported_posture != last_reported_posture_)
    {
        last_reported_posture_ = snapshot.reported_posture;
        last_reported_posture_change_ms_ = now_ms;
    }

    if (!stable_posture_initialized_)
    {
        stable_posture_ = snapshot.reported_posture;
        stable_posture_initialized_ = true;
    }
    else if (snapshot.reported_posture != stable_posture_ &&
             (now_ms - last_reported_posture_change_ms_) >= posture_feedback_stable_ms_)
    {
        stable_posture_ = snapshot.reported_posture;
    }

    if (posture_switch_pending_ && stable_posture_ == pending_posture_target_)
    {
        posture_switch_pending_ = false;
    }

    snapshot.current_posture = stable_posture_;
    snapshot.posture_switch_pending = posture_switch_pending_;
    snapshot.pending_posture_target = pending_posture_target_;
    snapshot.posture_accumulated_ms = posture_accumulated_ms_;
    snapshot.posture_debuff_threshold_ms = posture_debuff_threshold_ms_;
    snapshot.posture_debuff_rotate_margin_ms = posture_debuff_rotate_margin_ms_;
    for (std::size_t index = 0; index < snapshot.posture_debuffed.size(); ++index)
    {
        snapshot.posture_debuffed[index] =
            snapshot.posture_accumulated_ms[index] >= posture_debuff_threshold_ms_;
    }

    std::lock_guard<std::mutex> lock(ctx.mtx);
    ctx.frame_index = snapshot.frame_index;
    ctx.is_dead = snapshot.is_dead;
    ctx.match_started = snapshot.match_started;
    ctx.can_confirm_revive = snapshot.can_confirm_revive;
    ctx.can_buy_immediate_revive = snapshot.can_buy_immediate_revive;
    ctx.is_disengaged = snapshot.is_disengaged;
    ctx.immediate_revive_cost = snapshot.immediate_revive_cost;
    ctx.game_progress = snapshot.game_progress;
    ctx.stage_remain_time = snapshot.stage_remain_time;
    ctx.hp = snapshot.hp;
    ctx.hp_max = snapshot.hp_max;
    ctx.heat = snapshot.heat;
    ctx.heat_limit = snapshot.heat_limit;
    ctx.cooling = snapshot.cooling;
    ctx.ammo_17 = snapshot.ammo_17;
    ctx.gold = snapshot.gold;
    ctx.exchanged_projectile_allowance = snapshot.exchanged_projectile_allowance;
    ctx.remote_exchange_projectile_count = snapshot.remote_exchange_projectile_count;
    ctx.remote_exchange_hp_count = snapshot.remote_exchange_hp_count;
    ctx.team_17mm_exchange_remain = snapshot.team_17mm_exchange_remain;
    ctx.chassis_power_now = snapshot.chassis_power_now;
    ctx.chassis_power_limit = snapshot.chassis_power_limit;
    ctx.supercap_soc = snapshot.supercap_soc;
    ctx.enemy_in_view = snapshot.enemy_in_view;
    ctx.enemy_confidence = snapshot.enemy_confidence;
    ctx.enemy_distance_m = snapshot.enemy_distance_m;
    ctx.armor_id = snapshot.armor_id;
    ctx.hp_deduction_reason = snapshot.hp_deduction_reason;
    ctx.under_attack = snapshot.under_attack;
    ctx.on_supply = snapshot.on_supply;
    ctx.on_base = snapshot.on_base;
    ctx.on_fortress = snapshot.on_fortress;
    ctx.on_outpost = snapshot.on_outpost;
    ctx.on_highground = snapshot.on_highground;
    ctx.rfid_status = snapshot.rfid_status;
    ctx.rfid_status_2 = snapshot.rfid_status_2;
    ctx.recovery_buff = snapshot.recovery_buff;
    ctx.reported_posture = snapshot.reported_posture;
    ctx.current_posture = snapshot.current_posture;
    ctx.pending_posture_target = snapshot.pending_posture_target;
    ctx.posture_switch_pending = snapshot.posture_switch_pending;
    ctx.referee_link_fresh = snapshot.referee_link_fresh;
    ctx.sim_input_fresh = snapshot.sim_input_fresh;
    ctx.health_data_degraded = snapshot.health_data_degraded;
    ctx.referee_status_age_ms = snapshot.referee_status_age_ms;
    ctx.sim_input_age_ms = snapshot.sim_input_age_ms;
    ctx.nav_goal_active = snapshot.nav_goal_active;
    ctx.nav_goal_reached = snapshot.nav_goal_reached;
    ctx.nav_goal_failed = snapshot.nav_goal_failed;
    ctx.current_goal_id = snapshot.current_goal_id;
    ctx.nav_status_age_ms = snapshot.nav_status_age_ms;
    ctx.autoaim_has_target = snapshot.autoaim_has_target;
    ctx.autoaim_tracking = snapshot.autoaim_tracking;
    ctx.autoaim_fire_ready = snapshot.autoaim_fire_ready;
    ctx.autoaim_target_distance = snapshot.autoaim_target_distance;
    ctx.autoaim_status_age_ms = snapshot.autoaim_status_age_ms;
    ctx.posture_accumulated_ms = snapshot.posture_accumulated_ms;
    ctx.posture_debuffed = snapshot.posture_debuffed;
    ctx.posture_debuff_threshold_ms = snapshot.posture_debuff_threshold_ms;
    ctx.posture_debuff_rotate_margin_ms = snapshot.posture_debuff_rotate_margin_ms;
    ctx.can_activate_energy_mechanism = snapshot.can_activate_energy_mechanism;
    ctx.can_claim_periodic_ammo = snapshot.can_claim_periodic_ammo;
    ctx.posture_switch_requested = snapshot.posture_switch_requested;
    ctx.now_ms = snapshot.now_ms;
    ctx.input_health_reason = snapshot.input_health_reason;

    const std::uint64_t elapsed_since_posture_cmd =
        (ctx.last_posture_command_ms == 0 || snapshot.now_ms <= ctx.last_posture_command_ms)
            ? 0
            : (snapshot.now_ms - ctx.last_posture_command_ms);
    ctx.posture_cooldown_ok =
        (ctx.last_posture_command_ms == 0) ||
        (elapsed_since_posture_cmd >= posture_switch_cooldown_ms_);
    ctx.posture_cooldown_remaining_ms =
        ctx.posture_cooldown_ok
            ? 0
            : (posture_switch_cooldown_ms_ - elapsed_since_posture_cmd);
}

bool RefereeInterface::HasSnapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return has_status_;
}
