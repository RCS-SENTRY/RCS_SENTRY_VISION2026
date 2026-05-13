#include <filesystem>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "bt_compat.hpp"
#include "bt_setup.hpp"
#include "llm_interface.hpp"
#include "referee_interface.hpp"
#include "rm_interfaces/msg/autoaim_target_status.hpp"
#include "rm_interfaces/msg/sentry_decision_debug.hpp"
#include "rm_interfaces/msg/sentry_intent.hpp"
#include "rm_interfaces/msg/sentry_nav_status.hpp"
#include "rm_interfaces/msg/sentry_sim_input.hpp"
#include "robot_context.hpp"
#include "sentry_decision_protocol.h"

#if __has_include(<behaviortree_cpp/loggers/bt_file_logger_v2.h>)
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#define SENTRY_BT_HAS_BTCPP4_LOGGERS 1
#elif __has_include(<behaviortree_cpp_v3/loggers/bt_file_logger.h>)
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#define SENTRY_BT_HAS_BTCPP3_LOGGERS 1
#endif

namespace
{
const char* PendingPostureToString(const RobotContext& ctx)
{
    return ctx.posture_switch_pending ? PostureToString(ctx.pending_posture_target) : "NONE";
}

std::uint8_t GoalIdToProtocol(const std::string& goal)
{
    if (goal == "CURRENT_HOLD") return 0;
    if (goal == "SAFE_HOLD") return 1;
    if (goal == "WAIT_REVIVE") return 2;
    if (goal == "SAFE_RETREAT_A") return 3;
    if (goal == "SAFE_RETREAT_B") return 4;
    if (goal == "SUPPLY_LEFT") return 5;
    if (goal == "SUPPLY_RIGHT") return 6;
    if (goal == "FORTRESS_HOLD") return 7;
    if (goal == "OUTPOST_HOLD") return 8;
    if (goal == "SEARCH_AREA_A") return 14;
    if (goal == "SEARCH_AREA_B") return 15;
    if (goal == "MID_CROSS") return 18;
    if (goal == "BASE_HOME") return 19;
    if (goal == "BASE_HOLD") return 20;
    return 0;
}

std::uint8_t TacticalStateToIntentValue(TacticalState state)
{
    switch (state)
    {
        case TacticalState::HOLD:
            return 1;
        case TacticalState::ENGAGE:
            return 2;
        case TacticalState::RETREAT:
            return 3;
        case TacticalState::RESUPPLY:
            return 4;
        case TacticalState::SEARCH:
            return 5;
        case TacticalState::REPOSITION:
            return 6;
    }
    return 0;
}

std::uint8_t PostureToIntentValue(Posture posture)
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
    return 0;
}

std::uint8_t RuleActionTypeToIntentValue(const RobotContext& ctx)
{
    if (ctx.revive_cmd == SENTRY_REVIVE_CMD_CONFIRM_FREE)
    {
        return 4;
    }
    if (ctx.revive_cmd == SENTRY_REVIVE_CMD_CONFIRM_IMMEDIATE)
    {
        return 5;
    }

    switch (ctx.rule_action_type)
    {
        case RuleActionType::NONE:
            return 0;
        case RuleActionType::EXCHANGE_AMMO_AT_POINT:
            return 1;
        case RuleActionType::REMOTE_AMMO:
            return 2;
        case RuleActionType::REMOTE_HP:
            return 3;
        case RuleActionType::SWITCH_POSTURE:
            return 6;
        case RuleActionType::ACTIVATE_ENERGY:
            return 7;
        case RuleActionType::CLAIM_PERIODIC_AMMO:
            return 0;
    }
    return 0;
}

void AppendReason(std::ostringstream& oss, const std::string& label, const std::string& value)
{
    if (value.empty())
    {
        return;
    }
    if (oss.tellp() > 0)
    {
        oss << " | ";
    }
    oss << label << "=" << value;
}

std::uint64_t SteadyNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}
}  // namespace

class SentryBtNode : public rclcpp::Node
{
public:
    SentryBtNode()
      : Node("sentry_bt"),
        ctx_(std::make_shared<RobotContext>())
    {
        const double tick_hz = std::max(1.0, this->declare_parameter<double>("tick_hz", 20.0));
        const auto posture_switch_cooldown_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("posture_switch_cooldown_ms", 5000)));
        const auto posture_feedback_stable_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("posture_feedback_stable_ms", 200)));
        const auto posture_debuff_threshold_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("posture_debuff_threshold_ms", 180000)));
        const auto posture_debuff_rotate_margin_ms = static_cast<std::uint64_t>(std::max<int>(
            0, this->declare_parameter<int>("posture_debuff_rotate_margin_ms", 15000)));
        const auto status_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("status_timeout_ms", 300)));
        const auto enemy_memory_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("enemy_memory_ms", 800)));
        enable_sim_input_ = this->declare_parameter<bool>("enable_sim_input", false);
        const auto sim_input_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("sim_input_timeout_ms", 500)));
        const auto autoaim_status_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("autoaim_status_timeout_ms", 300)));
        const auto nav_status_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("nav_status_timeout_ms", 500)));
        debug_topic_ =
            this->declare_parameter<std::string>("debug_topic", "/sentry_bt/debug");
        intent_topic_ =
            this->declare_parameter<std::string>("intent_topic", "/sentry/intent");
        const std::string autoaim_target_status_topic =
            this->declare_parameter<std::string>(
                "autoaim_target_status_topic", "/autoaim/target_status");
        const std::string nav_status_topic =
            this->declare_parameter<std::string>("nav_status_topic", "/sentry/nav_status");
        const std::string main_lidar_topic =
            this->declare_parameter<std::string>("main_lidar_topic", "/cloud_registered");
        const std::string safety_debug_topic =
            this->declare_parameter<std::string>("safety_debug_topic", "/second_lidar_safety_debug");
        const std::string hold_reached_posture =
            this->declare_parameter<std::string>("hold_reached_posture", "DEFENSE");
        const bool enable_bt_file_log =
            this->declare_parameter<bool>("enable_bt_file_log", true);
        const std::string btlog_path =
            this->declare_parameter<std::string>("btlog_path", "/tmp/sentry_bt.btlog");
        const bool enable_groot_zmq =
            this->declare_parameter<bool>("enable_groot_zmq", false);
        const auto rfid_base_bits =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.base_bits", std::vector<int64_t>{0});
        const auto rfid_supply_bits =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.supply_bits", std::vector<int64_t>{19, 20});
        const auto rfid_fortress_bits =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.fortress_bits", std::vector<int64_t>{17});
        const auto rfid_outpost_bits =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.outpost_bits", std::vector<int64_t>{18});
        const auto rfid_highground_bits =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.highground_bits", std::vector<int64_t>{1, 2, 3, 4, 9, 10, 11, 12});
        const auto rfid_base_bits_2 =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.base_bits_2", std::vector<int64_t>{});
        const auto rfid_supply_bits_2 =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.supply_bits_2", std::vector<int64_t>{});
        const auto rfid_fortress_bits_2 =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.fortress_bits_2", std::vector<int64_t>{});
        const auto rfid_outpost_bits_2 =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.outpost_bits_2", std::vector<int64_t>{});
        const auto rfid_highground_bits_2 =
            this->declare_parameter<std::vector<int64_t>>(
                "rfid.highground_bits_2", std::vector<int64_t>{});
        ctx_->require_recovery_buff_for_confirm =
            this->declare_parameter<bool>("recovery.require_recovery_buff_for_confirm", false);
        ctx_->hp_resupply_enter_ratio = static_cast<float>(
            this->declare_parameter<double>("hp_resupply_enter_ratio", 0.60));
        ctx_->hp_resupply_exit_ratio = static_cast<float>(
            this->declare_parameter<double>("hp_resupply_exit_ratio", 0.85));
        ctx_->ammo_resupply_enter_count =
            this->declare_parameter<int>("ammo_resupply_enter_count", 80);
        ctx_->ammo_resupply_exit_count =
            this->declare_parameter<int>("ammo_resupply_exit_count", 120);
        ctx_->resupply_rfid_confirm_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>(
                                 "resupply_rfid_confirm_hold_ms", 300)));
        ctx_->resupply_goal_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("resupply_goal_timeout_ms", 12000)));
        ctx_->resupply_wait_recovery_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>(
                                 "resupply_wait_recovery_timeout_ms", 12000)));
        ctx_->resupply_candidate_switch_cooldown_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>(
                                 "resupply_candidate_switch_cooldown_ms", 1500)));
        ctx_->resupply_candidates = this->declare_parameter<std::vector<std::string>>(
            "resupply_candidates", std::vector<std::string>{
                                       "SUPPLY_LEFT", "SUPPLY_RIGHT"});
        if (!ctx_->resupply_candidates.empty())
        {
            ctx_->resupply_goal_current = ctx_->resupply_candidates.front();
        }
        ctx_->patrol_goals = this->declare_parameter<std::vector<std::string>>(
            "patrol_goals",
            std::vector<std::string>{"SEARCH_AREA_B", "MID_CROSS", "SEARCH_AREA_A"});
        if (ctx_->patrol_goals.empty())
        {
            ctx_->patrol_goals = {"SEARCH_AREA_B", "MID_CROSS", "SEARCH_AREA_A"};
        }
        ctx_->dead_return_home_enabled =
            this->declare_parameter<bool>("dead_return_home_enabled", true);
        ctx_->dead_return_goal =
            this->declare_parameter<std::string>("dead_return_goal", "SUPPLY_LEFT");
        ctx_->dead_return_rfid_confirm_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>(
                                 "dead_return_rfid_confirm_hold_ms", 300)));
        ctx_->dead_full_hp_exit_ratio = static_cast<float>(
            this->declare_parameter<double>("dead_full_hp_exit_ratio", 0.98));
        ctx_->dead_return_no_timeout =
            this->declare_parameter<bool>("dead_return_no_timeout", true);
        ctx_->tactical_state_min_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("tactical_state_min_hold_ms", 3000)));
        ctx_->internal_motion_min_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("internal_motion_min_hold_ms", 3000)));
        ctx_->goal_dwell_default_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("goal_dwell_default_ms", 12000)));
        ctx_->goal_dwell_search_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("goal_dwell_search_ms", 12000)));
        ctx_->goal_dwell_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("goal_dwell_hold_ms", 12000)));
        ctx_->goal_dwell_resupply_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("goal_dwell_resupply_ms", 0)));
        ctx_->goal_dwell_engage_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("goal_dwell_engage_ms", 0)));
        ctx_->spin_hysteresis_enabled =
            this->declare_parameter<bool>("spin_hysteresis_enabled", true);
        ctx_->spin_on_confirm_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("spin_on_confirm_ms", 0)));
        ctx_->spin_off_confirm_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("spin_off_confirm_ms", 0)));
        ctx_->spin_min_on_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("spin_min_on_ms", 0)));
        ctx_->spin_min_off_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("spin_min_off_ms", 0)));
        ctx_->spin_target_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("spin_target_hold_ms", 5000)));
        ctx_->spin_under_attack_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("spin_under_attack_hold_ms", 5000)));
        ctx_->spin_hp_drop_threshold =
            std::max<int>(0, this->declare_parameter<int>("spin_hp_drop_threshold", 0));
        ctx_->spin_hp_drop_triggers_under_attack =
            this->declare_parameter<bool>("spin_hp_drop_triggers_under_attack", false);
        ctx_->engage_target_max_distance_m = static_cast<float>(
            this->declare_parameter<double>("engage_target_max_distance_m", 7.0));
        ctx_->fire_policy_hysteresis_enabled =
            this->declare_parameter<bool>("fire_policy_hysteresis_enabled", true);
        ctx_->fire_policy_on_confirm_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("fire_policy_on_confirm_ms", 150)));
        ctx_->fire_policy_off_confirm_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("fire_policy_off_confirm_ms", 800)));
        ctx_->fire_policy_min_on_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("fire_policy_min_on_ms", 1500)));
        ctx_->fire_policy_min_off_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("fire_policy_min_off_ms", 500)));
        ctx_->fire_policy_target_hold_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("fire_policy_target_hold_ms", 1200)));
        ctx_->main_lidar_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("main_lidar_timeout_ms", 500)));
        ctx_->safety_debug_timeout_ms = static_cast<std::uint64_t>(
            std::max<int>(1, this->declare_parameter<int>("safety_debug_timeout_ms", 500)));
        ctx_->safety_collision_interrupt_ms = static_cast<std::uint64_t>(
            std::max<int>(0, this->declare_parameter<int>("safety_collision_interrupt_ms", 2500)));

        referee_.ConfigurePostureTiming(
            posture_switch_cooldown_ms, posture_feedback_stable_ms);
        referee_.ConfigurePostureDebuff(
            posture_debuff_threshold_ms, posture_debuff_rotate_margin_ms);
        referee_.ConfigureInputFreshness(status_timeout_ms, enemy_memory_ms);
        referee_.ConfigureSimInput(enable_sim_input_, sim_input_timeout_ms);
        referee_.ConfigureDecisionStatusInputs(autoaim_status_timeout_ms, nav_status_timeout_ms);
        referee_.ConfigureRfidBits(
            rfid_base_bits, rfid_supply_bits, rfid_fortress_bits, rfid_outpost_bits,
            rfid_highground_bits, rfid_base_bits_2, rfid_supply_bits_2,
            rfid_fortress_bits_2, rfid_outpost_bits_2, rfid_highground_bits_2);
        Posture parsed_hold_reached_posture = Posture::DEFENSE;
        if (ParsePosture(hold_reached_posture, parsed_hold_reached_posture))
        {
            ctx_->hold_reached_posture = parsed_hold_reached_posture;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Invalid hold_reached_posture=%s, using DEFENSE",
                hold_reached_posture.c_str());
        }

        RegisterAllNodes(factory_, ctx_);

        auto blackboard = BT::Blackboard::create();
        blackboard->set("ctx", ctx_);

        const auto tree_template_path =
            std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "tree" / "sentry_main.template.xml";
        const auto tree_path =
            std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "tree" / "sentry_main.xml";

        ExportTreeModelXML(factory_, tree_template_path, tree_path, false);
        tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_path, blackboard));

        SetupBtLoggers(enable_bt_file_log, btlog_path, enable_groot_zmq);

        gimbal_status_sub_ = this->create_subscription<rm_interfaces::msg::GimbalStatus>(
            "/gimbal_status", rclcpp::SensorDataQoS(),
            [this](const rm_interfaces::msg::GimbalStatus::SharedPtr msg) {
                referee_.UpdateFromStatus(*msg);
            });
        autoaim_target_status_sub_ =
            this->create_subscription<rm_interfaces::msg::AutoaimTargetStatus>(
                autoaim_target_status_topic, rclcpp::SensorDataQoS(),
                [this](const rm_interfaces::msg::AutoaimTargetStatus::SharedPtr msg) {
                    referee_.UpdateFromAutoaimTargetStatus(*msg);
                });
        nav_status_sub_ = this->create_subscription<rm_interfaces::msg::SentryNavStatus>(
            nav_status_topic, rclcpp::SensorDataQoS(),
            [this](const rm_interfaces::msg::SentryNavStatus::SharedPtr msg) {
                referee_.UpdateFromNavStatus(*msg);
            });
        main_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            main_lidar_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {
                std::lock_guard<std::mutex> lock(ctx_->mtx);
                ctx_->main_lidar_seen = true;
                ctx_->main_lidar_last_seen_ms = SteadyNowMs();
            });
        safety_debug_sub_ = this->create_subscription<std_msgs::msg::String>(
            safety_debug_topic, rclcpp::QoS(10),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(ctx_->mtx);
                ctx_->safety_debug_seen = true;
                ctx_->safety_debug_last_seen_ms = SteadyNowMs();
                ctx_->safety_emergency_active =
                    msg->data.find("emergency_any=true") != std::string::npos;
                ctx_->safety_obstacle_timeout =
                    msg->data.find("obstacle_timeout=true") != std::string::npos;
                if (ctx_->safety_emergency_active)
                {
                    ctx_->safety_interrupt_last_seen_ms = ctx_->safety_debug_last_seen_ms;
                    ctx_->safety_interrupt_reason =
                        "第二雷达 safety emergency，保持正常目标规划并交给导航避障/超时切点。";
                }
            });

        if (enable_sim_input_)
        {
            sim_input_sub_ = this->create_subscription<rm_interfaces::msg::SentrySimInput>(
                "/sentry_bt/sim_input", rclcpp::SensorDataQoS(),
                [this](const rm_interfaces::msg::SentrySimInput::SharedPtr msg) {
                    referee_.UpdateFromSimInput(*msg);
                });
        }

        debug_pub_ = this->create_publisher<rm_interfaces::msg::SentryDecisionDebug>(
            debug_topic_, rclcpp::SensorDataQoS());
        intent_pub_ = this->create_publisher<rm_interfaces::msg::SentryIntent>(
            intent_topic_, rclcpp::SensorDataQoS());

        const auto tick_period =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / tick_hz));
        tick_timer_ = this->create_wall_timer(tick_period, [this]() { this->TickDecision(); });

        RCLCPP_INFO(
            get_logger(),
            "sentry_bt is INTENT-ONLY. subscribing /gimbal_status, %s, %s%s, publishing %s and %s. "
            "It does NOT publish /gimbal_cmd or /nav_cmd.",
            autoaim_target_status_topic.c_str(), nav_status_topic.c_str(),
            enable_sim_input_ ? ", /sentry_bt/sim_input" : "",
            intent_topic_.c_str(), debug_topic_.c_str());
    }

private:
    void SetupBtLoggers(bool enable_bt_file_log, const std::string& btlog_path,
                        bool enable_groot_zmq)
    {
#if defined(SENTRY_BT_HAS_BTCPP4_LOGGERS)
        if (enable_bt_file_log)
        {
            bt_file_logger_ = std::make_unique<BT::FileLogger2>(*tree_, btlog_path);
        }
        if (enable_groot_zmq)
        {
            groot_logger_ = std::make_unique<BT::Groot2Publisher>(*tree_);
        }
#elif defined(SENTRY_BT_HAS_BTCPP3_LOGGERS)
        if (enable_bt_file_log)
        {
            bt_file_logger_ = std::make_unique<BT::FileLogger>(*tree_, btlog_path.c_str());
        }
        if (enable_groot_zmq)
        {
            groot_logger_ = std::make_unique<BT::PublisherZMQ>(*tree_);
        }
#else
        (void)enable_bt_file_log;
        (void)btlog_path;
        (void)enable_groot_zmq;
        RCLCPP_WARN(get_logger(), "BehaviorTree logger headers not available; BT logging disabled");
#endif
    }

    rm_interfaces::msg::SentryDecisionDebug BuildDebugMessage() const
    {
        rm_interfaces::msg::SentryDecisionDebug message;
        message.header.stamp = this->now();
        message.header.frame_id = "sentry_bt";

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        message.frame_index = ctx_->frame_index;
        message.bt_tick_index = ctx_->bt_tick_index;
        message.reported_posture = PostureToProtocolValue(ctx_->reported_posture);
        message.current_posture = PostureToProtocolValue(ctx_->current_posture);
        message.preferred_posture = PostureToProtocolValue(ctx_->preferred_posture);
        message.desired_posture = PostureToProtocolValue(ctx_->desired_posture);
        message.pending_posture_target =
            ctx_->posture_switch_pending ? PostureToProtocolValue(ctx_->pending_posture_target) : 0;
        message.posture_switch_pending = ctx_->posture_switch_pending;
        message.posture_cooldown_remaining_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                ctx_->posture_cooldown_remaining_ms, 0xFFFFFFFFULL));
        message.posture_debuff_threshold_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                ctx_->posture_debuff_threshold_ms, 0xFFFFFFFFULL));
        message.current_posture_remaining_before_debuff_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                RemainingBeforePostureDebuff(*ctx_, ctx_->current_posture), 0xFFFFFFFFULL));
        message.preferred_posture_remaining_before_debuff_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                RemainingBeforePostureDebuff(*ctx_, ctx_->preferred_posture), 0xFFFFFFFFULL));
        message.current_posture_debuffed = IsPostureDebuffed(*ctx_, ctx_->current_posture);
        message.preferred_posture_debuffed = IsPostureDebuffed(*ctx_, ctx_->preferred_posture);
        message.attack_posture_accumulated_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                GetAccumulatedPostureMs(*ctx_, Posture::ATTACK), 0xFFFFFFFFULL));
        message.defense_posture_accumulated_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                GetAccumulatedPostureMs(*ctx_, Posture::DEFENSE), 0xFFFFFFFFULL));
        message.move_posture_accumulated_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                GetAccumulatedPostureMs(*ctx_, Posture::MOVE), 0xFFFFFFFFULL));
        message.attack_posture_debuffed = IsPostureDebuffed(*ctx_, Posture::ATTACK);
        message.defense_posture_debuffed = IsPostureDebuffed(*ctx_, Posture::DEFENSE);
        message.move_posture_debuffed = IsPostureDebuffed(*ctx_, Posture::MOVE);
        message.tactical_state = TacticalStateToProtocolValue(ctx_->tactical_state);
        message.rule_action_type = RuleActionTypeToProtocolValue(ctx_->rule_action_type);
        message.desired_goal = ctx_->desired_goal;
        message.fire_policy = FirePolicyToProtocolValue(ctx_->desired_fire_policy);
        message.spin_mode = SpinModeToProtocolValue(ctx_->desired_spin_mode);
        message.supercap_mode = SupercapModeToProtocolValue(ctx_->desired_supercap_mode);
        message.hp = ctx_->hp;
        message.ammo_17 = ctx_->ammo_17;
        message.heat = ctx_->heat;
        message.enemy_in_view = ctx_->enemy_in_view;
        message.referee_link_fresh = ctx_->referee_link_fresh;
        message.sim_input_fresh = ctx_->sim_input_fresh;
        message.referee_status_age_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                ctx_->referee_status_age_ms, 0xFFFFFFFFULL));
        message.sim_input_age_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                ctx_->sim_input_age_ms, 0xFFFFFFFFULL));
        message.nav_goal_active = ctx_->nav_goal_active;
        message.nav_goal_reached = ctx_->nav_goal_reached;
        message.nav_goal_failed = ctx_->nav_goal_failed;
        message.current_goal_id = ctx_->current_goal_id;
        message.nav_status_age_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                ctx_->nav_status_age_ms, 0xFFFFFFFFULL));
        message.autoaim_has_target = ctx_->autoaim_has_target;
        message.autoaim_tracking = ctx_->autoaim_tracking;
        message.autoaim_fire_ready = ctx_->autoaim_fire_ready;
        message.autoaim_target_distance = ctx_->autoaim_target_distance;
        message.autoaim_status_age_ms =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(
                ctx_->autoaim_status_age_ms, 0xFFFFFFFFULL));
        message.revive_cmd = ctx_->revive_cmd;
        message.remote_ammo_req_inc = ctx_->remote_ammo_req_inc;
        message.remote_hp_req_inc = ctx_->remote_hp_req_inc;
        message.posture_cmd_referee = ctx_->posture_cmd_referee;
        message.activate_energy_confirm = ctx_->activate_energy_confirm;
        message.claim_periodic_ammo = ctx_->claim_periodic_ammo;
        message.tactical_reason = ctx_->tactical_reason;
        message.rule_reason = ctx_->rule_reason;
        message.goal_reason = ctx_->goal_reason;
        message.posture_reason = ctx_->posture_reason;
        message.spin_reason = ctx_->spin_reason;
        message.executor_summary = ctx_->executor_summary;
        message.last_rule_command = ctx_->last_rule_command;
        message.input_health_reason = ctx_->input_health_reason;
        message.last_nav_command = ctx_->last_nav_command;
        message.last_shooter_command = ctx_->last_shooter_command;
        return message;
    }

    void PublishDebugMessage()
    {
        debug_pub_->publish(BuildDebugMessage());
    }

    rm_interfaces::msg::SentryIntent BuildIntentMessage() const
    {
        rm_interfaces::msg::SentryIntent message;
        message.header.stamp = this->now();
        message.header.frame_id = "sentry_bt";

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        message.protocol_version = ctx_->protocol_version;
        message.tactical_state = TacticalStateToIntentValue(ctx_->tactical_state);
        message.goal_id = GoalIdToProtocol(ctx_->desired_goal);
        message.posture_intent = PostureToIntentValue(ctx_->desired_posture);
        message.fire_policy = FirePolicyToProtocolValue(ctx_->desired_fire_policy);
        message.spin_mode = SpinModeToProtocolValue(ctx_->desired_spin_mode);
        message.supercap_mode = SupercapModeToProtocolValue(ctx_->desired_supercap_mode);
        message.rule_action_type = RuleActionTypeToIntentValue(*ctx_);
        message.ammo_exchange_target_total = ctx_->ammo_exchange_target_total;
        message.request_revive_confirm =
            ctx_->revive_cmd == SENTRY_REVIVE_CMD_CONFIRM_FREE ? 1 : 0;
        message.request_instant_revive =
            ctx_->revive_cmd == SENTRY_REVIVE_CMD_CONFIRM_IMMEDIATE ? 1 : 0;
        message.request_remote_ammo_once = ctx_->remote_ammo_req_inc;
        message.request_remote_hp_once = ctx_->remote_hp_req_inc;
        message.request_posture_referee = ctx_->posture_cmd_referee;
        message.request_activate_energy = ctx_->activate_energy_confirm;

        std::ostringstream reason;
        AppendReason(reason, "executor", ctx_->executor_summary);
        AppendReason(reason, "tactical", ctx_->tactical_reason);
        AppendReason(reason, "rule", ctx_->rule_reason);
        AppendReason(reason, "goal", ctx_->goal_reason);
        AppendReason(reason, "posture", ctx_->posture_reason);
        AppendReason(reason, "spin", ctx_->spin_reason);
        if (ctx_->rule_action_type == RuleActionType::CLAIM_PERIODIC_AMMO ||
            ctx_->claim_periodic_ammo != 0)
        {
            AppendReason(reason, "claim_periodic_ammo", "disabled in upper-computer phase 1");
        }
        message.reason = reason.str();
        return message;
    }

    void PublishIntent()
    {
        intent_pub_->publish(BuildIntentMessage());
    }

    void MaybeLogDecisionChange()
    {
        std::string signature;
        std::string detail;

        {
            std::lock_guard<std::mutex> lock(ctx_->mtx);
            std::ostringstream sig;
            sig << TacticalStateToString(ctx_->tactical_state) << '|'
                << RuleActionTypeToString(ctx_->rule_action_type) << '|'
                << ctx_->desired_goal << '|'
                << PostureToString(ctx_->desired_posture) << '|'
                << SpinModeToString(ctx_->desired_spin_mode) << '|'
                << ctx_->posture_switch_pending << '|'
                << PendingPostureToString(*ctx_) << '|'
                << static_cast<int>(ctx_->revive_cmd) << '|'
                << static_cast<int>(ctx_->remote_ammo_req_inc) << '|'
                << static_cast<int>(ctx_->remote_hp_req_inc) << '|'
                << static_cast<int>(ctx_->posture_cmd_referee) << '|'
                << static_cast<int>(ctx_->activate_energy_confirm) << '|'
                << static_cast<int>(ctx_->claim_periodic_ammo);
            signature = sig.str();

            std::ostringstream oss;
            oss << "decision frame=" << ctx_->frame_index
                << " bt=" << ctx_->bt_tick_index
                << " tactical=" << TacticalStateToString(ctx_->tactical_state)
                << " rule=" << RuleActionTypeToString(ctx_->rule_action_type)
                << " goal=" << ctx_->desired_goal
                << " posture(report=" << PostureToString(ctx_->reported_posture)
                << ", current=" << PostureToString(ctx_->current_posture)
                << ", desired=" << PostureToString(ctx_->desired_posture)
                << ", pending=" << PendingPostureToString(*ctx_)
                << ", cooldown_ms=" << ctx_->posture_cooldown_remaining_ms
                << ", remain_before_debuff_ms="
                << RemainingBeforePostureDebuff(*ctx_, ctx_->current_posture)
                << ", debuffed="
                << (IsPostureDebuffed(*ctx_, ctx_->current_posture) ? "true" : "false") << ")"
                << " fire=" << FirePolicyToString(ctx_->desired_fire_policy)
                << " spin=" << SpinModeToString(ctx_->desired_spin_mode)
                << " input(referee_fresh=" << (ctx_->referee_link_fresh ? "true" : "false")
                << ", status_age_ms=" << ctx_->referee_status_age_ms
                << ", sim_fresh=" << (ctx_->sim_input_fresh ? "true" : "false")
                << ", sim_age_ms=" << ctx_->sim_input_age_ms << ")"
                << " pulses(revive=" << static_cast<int>(ctx_->revive_cmd)
                << ", remote_ammo=" << static_cast<int>(ctx_->remote_ammo_req_inc)
                << ", remote_hp=" << static_cast<int>(ctx_->remote_hp_req_inc)
                << ", posture=" << static_cast<int>(ctx_->posture_cmd_referee)
                << ", energy=" << static_cast<int>(ctx_->activate_energy_confirm)
                << ", claim=" << static_cast<int>(ctx_->claim_periodic_ammo) << ")"
                << " reasons[tactical=" << ctx_->tactical_reason
                << "; rule=" << ctx_->rule_reason
                << "; goal=" << ctx_->goal_reason
                << "; posture=" << ctx_->posture_reason
                << "; spin=" << ctx_->spin_reason << "]";
            detail = oss.str();
        }

        if (signature == last_decision_signature_)
        {
            return;
        }

        last_decision_signature_ = signature;
        RCLCPP_INFO(get_logger(), "%s", detail.c_str());
    }

    void TickDecision()
    {
        if (!referee_.HasSnapshot())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Waiting for first /gimbal_status message");
            return;
        }

        referee_.SyncToContext(*ctx_);
        llm_.Update(*ctx_);
        TickRootOnceCompat(*tree_);
        referee_.ObserveDecisionOutput(*ctx_);
        {
            std::lock_guard<std::mutex> lock(ctx_->mtx);
            ctx_->claim_periodic_ammo = 0;
            ctx_->last_shooter_command =
                "disabled: sentry_bt is intent-only and does not publish /gimbal_cmd";
            ctx_->last_nav_command =
                "disabled: sentry_bt is intent-only and does not publish /nav_cmd";
        }
        PublishIntent();
        PublishDebugMessage();
        MaybeLogDecisionChange();
    }

    BT::BehaviorTreeFactory factory_{};
    std::shared_ptr<RobotContext> ctx_{};
    RefereeInterface referee_{};
    LLMInterface llm_{};
    std::unique_ptr<BT::Tree> tree_{};
    rclcpp::Subscription<rm_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::SentrySimInput>::SharedPtr sim_input_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::AutoaimTargetStatus>::SharedPtr
        autoaim_target_status_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::SentryNavStatus>::SharedPtr nav_status_sub_{};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr main_lidar_sub_{};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_debug_sub_{};
    rclcpp::Publisher<rm_interfaces::msg::SentryDecisionDebug>::SharedPtr debug_pub_{};
    rclcpp::Publisher<rm_interfaces::msg::SentryIntent>::SharedPtr intent_pub_{};
    rclcpp::TimerBase::SharedPtr tick_timer_{};
    std::string debug_topic_{};
    std::string intent_topic_{};
    std::string last_decision_signature_{};
    bool enable_sim_input_{false};
    std::unique_ptr<BT::StatusChangeLogger> bt_file_logger_{};
    std::unique_ptr<BT::StatusChangeLogger> groot_logger_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<SentryBtNode>());
    }
    catch (const std::exception& ex)
    {
        std::cerr << "sentry_bt failed: " << ex.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
