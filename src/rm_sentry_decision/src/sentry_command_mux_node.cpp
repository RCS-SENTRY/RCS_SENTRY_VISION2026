#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/gimbal_status.hpp>
#include <rm_interfaces/msg/sentry_intent.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
constexpr std::uint8_t kFirePolicyHoldFire = 0;
constexpr std::uint8_t kPostureKeep = 0;
constexpr std::uint8_t kPostureMove = 1;
constexpr std::uint8_t kPostureAttack = 2;
constexpr std::uint8_t kPostureDefense = 3;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

const char* BoolString(bool value)
{
    return value ? "true" : "false";
}

std::uint8_t ClampPulse(std::uint8_t value)
{
    return value == 0 ? 0 : 1;
}

std::int8_t PostureIntentToStateSwitch(std::uint8_t posture_intent,
                                       const rm_interfaces::msg::GimbalCmd& autoaim_raw,
                                       int default_posture)
{
    (void)autoaim_raw;
    switch (posture_intent)
    {
        case kPostureMove:
            return 1;
        case kPostureAttack:
            return 2;
        case kPostureDefense:
            return 3;
        case kPostureKeep:
        default:
            return static_cast<std::int8_t>(default_posture);
    }
}

std::uint8_t PostureIntentToRefereePosture(std::uint8_t posture_intent)
{
    switch (posture_intent)
    {
        case kPostureAttack:
            return 1;
        case kPostureDefense:
            return 2;
        case kPostureMove:
            return 3;
        default:
            return 0;
    }
}
}  // namespace

class SentryCommandMuxNode : public rclcpp::Node
{
public:
    SentryCommandMuxNode() : Node("sentry_command_mux")
    {
        intent_timeout_sec_ = declare_parameter<double>("intent_timeout_sec", 0.5);
        autoaim_timeout_sec_ = declare_parameter<double>("autoaim_timeout_sec", 0.3);
        heat_margin_ = declare_parameter<int>("heat_margin", 20);
        default_posture_when_no_intent_ =
            declare_parameter<int>("default_posture_when_no_intent", 1);
        allow_fire_without_intent_ =
            declare_parameter<bool>("allow_fire_without_intent", false);
        const double publish_rate_hz = std::max(1.0, declare_parameter<double>("publish_rate_hz", 50.0));
        enable_intent_only_heartbeat_ =
            declare_parameter<bool>("enable_intent_only_heartbeat", true);
        const double intent_only_publish_rate_hz =
            std::max(1.0, declare_parameter<double>("intent_only_publish_rate_hz", 20.0));
        hold_angles_from_gimbal_status_ =
            declare_parameter<bool>("hold_angles_from_gimbal_status", true);
        spin_mux_hold_sec_ =
            std::max(0.0, declare_parameter<double>("spin_mux_hold_sec", 5.0));
        fire_hold_sec_ =
            std::max(0.0, declare_parameter<double>("fire_hold_sec", 0.5));
        enable_spin_stop_on_lidar_timeout_ =
            declare_parameter<bool>("enable_spin_stop_on_lidar_timeout", true);
        main_lidar_timeout_sec_ =
            std::max(0.05, declare_parameter<double>("main_lidar_timeout_sec", 0.5));
        enable_spin_stop_on_safety_emergency_ =
            declare_parameter<bool>("enable_spin_stop_on_safety_emergency", true);
        spin_stop_on_safety_timeout_ =
            declare_parameter<bool>("spin_stop_on_safety_timeout", true);
        safety_debug_timeout_sec_ =
            std::max(0.05, declare_parameter<double>("safety_debug_timeout_sec", 0.5));

        autoaim_raw_topic_ =
            declare_parameter<std::string>("autoaim_raw_cmd_topic", "/autoaim/gimbal_cmd_raw");
        intent_topic_ = declare_parameter<std::string>("intent_topic", "/sentry/intent");
        gimbal_status_topic_ =
            declare_parameter<std::string>("gimbal_status_topic", "/gimbal_status");
        main_lidar_heartbeat_topic_ =
            declare_parameter<std::string>("main_lidar_heartbeat_topic", "/cloud_registered");
        safety_debug_topic_ =
            declare_parameter<std::string>("safety_debug_topic", "/second_lidar_safety_debug");
        final_gimbal_cmd_topic_ =
            declare_parameter<std::string>("final_gimbal_cmd_topic", "/gimbal_cmd");
        debug_topic_ =
            declare_parameter<std::string>("debug_topic", "/sentry/command_mux_debug");

        auto qos = rclcpp::SensorDataQoS();
        autoaim_sub_ = create_subscription<rm_interfaces::msg::GimbalCmd>(
            autoaim_raw_topic_, qos,
            [this](rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
                last_autoaim_raw_ = *msg;
                last_autoaim_time_ = now();
                has_autoaim_raw_ = true;
                PublishMergedCommand();
            });
        intent_sub_ = create_subscription<rm_interfaces::msg::SentryIntent>(
            intent_topic_, qos,
            [this](rm_interfaces::msg::SentryIntent::SharedPtr msg) {
                last_intent_ = *msg;
                last_intent_time_ = now();
                has_intent_ = true;
            });
        status_sub_ = create_subscription<rm_interfaces::msg::GimbalStatus>(
            gimbal_status_topic_, qos,
            [this](rm_interfaces::msg::GimbalStatus::SharedPtr msg) {
                last_status_ = *msg;
                last_status_time_ = now();
                has_status_ = true;
            });
        main_lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            main_lidar_heartbeat_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr) {
                last_main_lidar_time_ = now();
                has_main_lidar_ = true;
            });
        safety_debug_sub_ = create_subscription<std_msgs::msg::String>(
            safety_debug_topic_, rclcpp::QoS(10),
            [this](std_msgs::msg::String::SharedPtr msg) {
                last_safety_debug_time_ = now();
                has_safety_debug_ = true;
                safety_emergency_active_ =
                    msg->data.find("emergency_any=true") != std::string::npos;
                safety_obstacle_timeout_ =
                    msg->data.find("obstacle_timeout=true") != std::string::npos;
            });

        cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>(
            final_gimbal_cmd_topic_, qos);
        debug_pub_ = create_publisher<std_msgs::msg::String>(
            debug_topic_, rclcpp::QoS(10));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(
                    1.0 / (enable_intent_only_heartbeat_ ? intent_only_publish_rate_hz
                                                          : publish_rate_hz))),
            [this]() { PublishMergedCommand(); });

        RCLCPP_INFO(
            get_logger(),
            "sentry_command_mux: %s + %s -> %s, debug=%s",
            autoaim_raw_topic_.c_str(), intent_topic_.c_str(),
            final_gimbal_cmd_topic_.c_str(), debug_topic_.c_str());
    }

private:
    bool IsFresh(bool has_msg, const rclcpp::Time& stamp, double timeout_sec) const
    {
        return has_msg && (now() - stamp).seconds() < timeout_sec;
    }

    bool SafetyOk() const
    {
        // Shooter/fire safety only: do not add chassis/nav/lidar/spin/goal conditions here.
        if (!has_status_)
        {
            return false;
        }
        return last_status_.current_hp > 0 &&
               last_status_.projectile_allowance_17mm > 0 &&
               last_status_.power_management_shooter_output == 1 &&
               static_cast<int>(last_status_.shooter_17mm_1_barrel_heat) + heat_margin_ <
                   static_cast<int>(last_status_.shooter_barrel_heat_limit);
    }

    bool SpinHardStop() const
    {
        return has_status_ &&
               (last_status_.game_progress != 4 ||
                (last_status_.current_hp == 0 &&
                 (last_status_.can_confirm_revival != 0 ||
                  last_status_.can_buy_instant_revival != 0)));
    }

    bool MainLidarTimedOut() const
    {
        return enable_spin_stop_on_lidar_timeout_ &&
               (!has_main_lidar_ ||
                (now() - last_main_lidar_time_).seconds() > main_lidar_timeout_sec_);
    }

    bool SafetySpinStopActive() const
    {
        if (!enable_spin_stop_on_safety_emergency_ || !has_safety_debug_)
        {
            return false;
        }
        const bool debug_fresh =
            (now() - last_safety_debug_time_).seconds() <= safety_debug_timeout_sec_;
        if (!debug_fresh)
        {
            return spin_stop_on_safety_timeout_;
        }
        return safety_emergency_active_ ||
               (spin_stop_on_safety_timeout_ && safety_obstacle_timeout_);
    }

    std::string SpinStopReason() const
    {
        if (SpinHardStop())
        {
            return "match/death hard stop";
        }
        return "none";
    }

    bool SpinMuxHoldActive() const
    {
        return spin_mux_latched_ &&
               (now() - last_spin_on_time_).seconds() <= spin_mux_hold_sec_;
    }

    void PublishMergedCommand()
    {
        const bool intent_fresh = IsFresh(has_intent_, last_intent_time_, intent_timeout_sec_);
        const bool autoaim_fresh =
            IsFresh(has_autoaim_raw_, last_autoaim_time_, autoaim_timeout_sec_);
        if (!autoaim_fresh && (!intent_fresh || !enable_intent_only_heartbeat_))
        {
            if (has_autoaim_raw_ || has_intent_)
            {
                rm_interfaces::msg::GimbalCmd out;
                PublishDebug(
                    "idle", intent_fresh, autoaim_fresh, false, false, false, false, false, out);
            }
            return;
        }

        const bool autoaim_fire = autoaim_fresh && last_autoaim_raw_.fire_control == 1;
        const bool decision_enables_autoaim =
            intent_fresh ? (last_intent_.fire_policy >= 2)
                         : allow_fire_without_intent_;
        const bool use_autoaim_raw = autoaim_fresh && decision_enables_autoaim;
        const bool safety_ok = SafetyOk();
        if (use_autoaim_raw && autoaim_fire)
        {
            last_autoaim_fire_time_ = now();
            has_recent_autoaim_fire_ = true;
        }
        const bool fire_hold_active =
            has_recent_autoaim_fire_ &&
            fire_hold_sec_ > 0.0 &&
            (now() - last_autoaim_fire_time_).seconds() <= fire_hold_sec_;
        const bool final_fire =
            use_autoaim_raw && (autoaim_fire || fire_hold_active) && safety_ok;

        rm_interfaces::msg::GimbalCmd out;
        const char* mode = use_autoaim_raw
                               ? "autoaim_merge"
                               : (autoaim_fresh ? "autoaim_disabled_by_fire_policy"
                                                : "intent_only_heartbeat");
        if (use_autoaim_raw)
        {
            out = last_autoaim_raw_;
        }
        else if (hold_angles_from_gimbal_status_ && has_status_)
        {
            out.target_yaw = static_cast<double>(last_status_.gimbal_yaw) * kDegToRad;
            out.target_pitch = static_cast<double>(last_status_.gimbal_pitch) * kDegToRad;
        }
        out.fire_control = final_fire ? 1 : 0;

        if (intent_fresh)
        {
            out.protocol_version = last_intent_.protocol_version;
            out.goal_id = last_intent_.goal_id;
            out.tactical_state = last_intent_.tactical_state;
            out.posture = last_intent_.posture_intent;
            out.fire_policy = last_intent_.fire_policy;
            out.spin_mode = last_intent_.spin_mode;
            out.supercap_mode = last_intent_.supercap_mode;
            out.rule_action_type = last_intent_.rule_action_type;
            out.ammo_exchange_target_total = last_intent_.ammo_exchange_target_total;
            out.remote_ammo_req_inc = ClampPulse(last_intent_.request_remote_ammo_once);
            out.remote_hp_req_inc = ClampPulse(last_intent_.request_remote_hp_once);
            out.activate_energy_confirm = ClampPulse(last_intent_.request_activate_energy);
            if (last_intent_.request_instant_revive != 0)
            {
                out.revive_cmd = 2;
            }
            else if (last_intent_.request_revive_confirm != 0)
            {
                out.revive_cmd = 1;
            }
            else
            {
                out.revive_cmd = 0;
            }

            out.state_switch = PostureIntentToStateSwitch(
                last_intent_.posture_intent, out,
                default_posture_when_no_intent_);
            out.posture_cmd_referee = last_intent_.request_posture_referee != 0
                ? last_intent_.request_posture_referee
                : PostureIntentToRefereePosture(last_intent_.posture_intent);
        }
        else
        {
            out.protocol_version = 0;
            out.goal_id = 0;
            out.tactical_state = 0;
            out.posture = kPostureKeep;
            out.fire_policy = kFirePolicyHoldFire;
            out.spin_mode = 0;
            out.supercap_mode = 0;
            out.rule_action_type = 0;
            out.ammo_exchange_target_total = 0;
            out.revive_cmd = 0;
            out.remote_ammo_req_inc = 0;
            out.remote_hp_req_inc = 0;
            out.activate_energy_confirm = 0;
            out.state_switch = static_cast<std::int8_t>(default_posture_when_no_intent_);
            out.posture_cmd_referee = PostureIntentToRefereePosture(
                static_cast<std::uint8_t>(default_posture_when_no_intent_));
        }

        const bool spin_hard_stop = SpinHardStop();

        if (spin_hard_stop)
        {
            out.spin_mode = 0;
            out.supercap_mode = 0;
            spin_mux_latched_ = false;
        }
        else if (intent_fresh)
        {
            if (last_intent_.spin_mode != 0)
            {
                spin_mux_latched_ = true;
                last_spin_on_time_ = now();
                out.spin_mode = 1;
            }
            else
            {
                spin_mux_latched_ = false;
                out.spin_mode = 0;
            }
        }
        else if (SpinMuxHoldActive())
        {
            out.spin_mode = 1;
        }
        if (out.spin_mode != 0 && !intent_fresh)
        {
            out.supercap_mode = 2;
        }
        out.claim_periodic_ammo = 0;
        if (!autoaim_fresh)
        {
            out.fire_control = 0;
        }

        cmd_pub_->publish(out);
        PublishDebug(mode, intent_fresh, autoaim_fresh, autoaim_fire, decision_enables_autoaim,
                     safety_ok, final_fire, fire_hold_active, out);
    }

    void PublishDebug(const std::string& mode, bool intent_fresh, bool autoaim_fresh, bool autoaim_fire,
                      bool decision_enables_autoaim, bool safety_ok, bool final_fire,
                      bool fire_hold_active,
                      const rm_interfaces::msg::GimbalCmd& out)
    {
        std_msgs::msg::String debug;
        std::ostringstream oss;
        oss << "mode=" << mode
            << " has_autoaim_raw=" << BoolString(has_autoaim_raw_)
            << " autoaim_fresh=" << BoolString(autoaim_fresh)
            << " has_intent=" << BoolString(has_intent_)
            << " intent_fresh=" << BoolString(intent_fresh)
            << " has_status=" << BoolString(has_status_)
            << " intent_only_heartbeat_enabled="
            << BoolString(enable_intent_only_heartbeat_)
            << " autoaim_fire=" << BoolString(autoaim_fire)
            << " fire_hold_active=" << BoolString(fire_hold_active)
            << " fire_hold_sec=" << fire_hold_sec_
            << " has_recent_autoaim_fire=" << BoolString(has_recent_autoaim_fire_)
            << " decision_enables_autoaim=" << BoolString(decision_enables_autoaim)
            << " decision_fire_policy_autoaim_switch="
            << static_cast<int>(intent_fresh ? last_intent_.fire_policy : kFirePolicyHoldFire)
            << " safety_ok=" << BoolString(safety_ok)
            << " final_fire=" << BoolString(final_fire)
            << " posture_intent="
            << static_cast<int>(intent_fresh ? last_intent_.posture_intent : kPostureKeep)
            << " state_switch_out=" << static_cast<int>(out.state_switch)
            << " goal_id=" << static_cast<int>(out.goal_id)
            << " tactical_state=" << static_cast<int>(out.tactical_state)
            << " fire_policy=" << static_cast<int>(out.fire_policy)
            << " spin_mode=" << static_cast<int>(out.spin_mode)
            << " spin_mux_latched=" << BoolString(spin_mux_latched_)
            << " spin_mux_hold_active=" << BoolString(SpinMuxHoldActive())
            << " spin_mux_hold_sec=" << spin_mux_hold_sec_
            << " spin_stop_reason=" << SpinStopReason()
            << " main_lidar_seen=" << BoolString(has_main_lidar_)
            << " main_lidar_timeout=" << BoolString(MainLidarTimedOut())
            << " safety_debug_seen=" << BoolString(has_safety_debug_)
            << " safety_emergency=" << BoolString(safety_emergency_active_)
            << " safety_obstacle_timeout=" << BoolString(safety_obstacle_timeout_)
            << " supercap_mode=" << static_cast<int>(out.supercap_mode)
            << " heat=" << (has_status_ ? last_status_.shooter_17mm_1_barrel_heat : 0)
            << " heat_limit=" << (has_status_ ? last_status_.shooter_barrel_heat_limit : 0)
            << " ammo=" << (has_status_ ? last_status_.projectile_allowance_17mm : 0)
            << " rule_action_type=" << static_cast<int>(out.rule_action_type)
            << " revive_cmd=" << static_cast<int>(out.revive_cmd)
            << " remote_hp_req_inc=" << static_cast<int>(out.remote_hp_req_inc)
            << " remote_ammo_req_inc=" << static_cast<int>(out.remote_ammo_req_inc)
            << " activate_energy_confirm=" << static_cast<int>(out.activate_energy_confirm)
            << " ammo_exchange_target_total=" << out.ammo_exchange_target_total
            << " posture_cmd_referee=" << static_cast<int>(out.posture_cmd_referee)
            << " pulse_note=upper mux passes 0/1 pulses only; Control must close monotonic 0x0120 counters"
            << " reason=" << (intent_fresh ? last_intent_.reason : "intent timeout or missing");
        debug.data = oss.str();
        debug_pub_->publish(debug);
    }

    double intent_timeout_sec_{0.5};
    double autoaim_timeout_sec_{0.3};
    int heat_margin_{20};
    int default_posture_when_no_intent_{1};
    bool allow_fire_without_intent_{false};
    bool enable_intent_only_heartbeat_{true};
    bool hold_angles_from_gimbal_status_{true};
    double spin_mux_hold_sec_{5.0};
    double fire_hold_sec_{0.08};
    bool enable_spin_stop_on_lidar_timeout_{true};
    bool enable_spin_stop_on_safety_emergency_{true};
    bool spin_stop_on_safety_timeout_{true};
    double main_lidar_timeout_sec_{0.5};
    double safety_debug_timeout_sec_{0.5};

    std::string autoaim_raw_topic_{};
    std::string intent_topic_{};
    std::string gimbal_status_topic_{};
    std::string main_lidar_heartbeat_topic_{};
    std::string safety_debug_topic_{};
    std::string final_gimbal_cmd_topic_{};
    std::string debug_topic_{};

    rm_interfaces::msg::GimbalCmd last_autoaim_raw_{};
    rm_interfaces::msg::SentryIntent last_intent_{};
    rm_interfaces::msg::GimbalStatus last_status_{};
    rclcpp::Time last_autoaim_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_intent_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_status_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_main_lidar_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_safety_debug_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_spin_on_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_autoaim_fire_time_{0, 0, get_clock()->get_clock_type()};
    bool has_autoaim_raw_{false};
    bool has_intent_{false};
    bool has_status_{false};
    bool has_main_lidar_{false};
    bool has_safety_debug_{false};
    bool safety_emergency_active_{false};
    bool safety_obstacle_timeout_{false};
    bool spin_mux_latched_{false};
    bool has_recent_autoaim_fire_{false};

    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr autoaim_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::SentryIntent>::SharedPtr intent_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::GimbalStatus>::SharedPtr status_sub_{};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr main_lidar_sub_{};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_debug_sub_{};
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_{};
    rclcpp::TimerBase::SharedPtr timer_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SentryCommandMuxNode>());
    rclcpp::shutdown();
    return 0;
}
