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
#include <std_msgs/msg/string.hpp>

namespace
{
constexpr std::uint8_t kFirePolicyHoldFire = 0;
constexpr std::uint8_t kPostureKeep = 0;
constexpr std::uint8_t kPostureMove = 1;
constexpr std::uint8_t kPostureAttack = 2;
constexpr std::uint8_t kPostureDefense = 3;

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
    switch (posture_intent)
    {
        case kPostureKeep:
            return autoaim_raw.state_switch != 0 ? autoaim_raw.state_switch
                                                 : static_cast<std::int8_t>(default_posture);
        case kPostureMove:
            return 1;
        case kPostureAttack:
            return 2;
        case kPostureDefense:
            return 3;
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

        autoaim_raw_topic_ =
            declare_parameter<std::string>("autoaim_raw_cmd_topic", "/autoaim/gimbal_cmd_raw");
        intent_topic_ = declare_parameter<std::string>("intent_topic", "/sentry/intent");
        gimbal_status_topic_ =
            declare_parameter<std::string>("gimbal_status_topic", "/gimbal_status");
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

        cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>(
            final_gimbal_cmd_topic_, qos);
        debug_pub_ = create_publisher<std_msgs::msg::String>(
            debug_topic_, rclcpp::QoS(10));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz)),
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

    void PublishMergedCommand()
    {
        if (!has_autoaim_raw_)
        {
            return;
        }

        const bool intent_fresh = IsFresh(has_intent_, last_intent_time_, intent_timeout_sec_);
        const bool autoaim_fresh =
            IsFresh(has_autoaim_raw_, last_autoaim_time_, autoaim_timeout_sec_);
        const bool autoaim_fire = last_autoaim_raw_.fire_control == 1;
        const bool decision_allows_fire =
            intent_fresh ? (last_intent_.fire_policy != kFirePolicyHoldFire)
                         : allow_fire_without_intent_;
        const bool safety_ok = SafetyOk();
        const bool final_fire =
            autoaim_fresh && autoaim_fire && decision_allows_fire && safety_ok;

        auto out = last_autoaim_raw_;
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
                last_intent_.posture_intent, last_autoaim_raw_,
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
        out.claim_periodic_ammo = 0;

        cmd_pub_->publish(out);
        PublishDebug(intent_fresh, autoaim_fresh, autoaim_fire, decision_allows_fire,
                     safety_ok, final_fire, out);
    }

    void PublishDebug(bool intent_fresh, bool autoaim_fresh, bool autoaim_fire,
                      bool decision_allows_fire, bool safety_ok, bool final_fire,
                      const rm_interfaces::msg::GimbalCmd& out)
    {
        std_msgs::msg::String debug;
        std::ostringstream oss;
        oss << "intent_fresh=" << BoolString(intent_fresh)
            << " autoaim_fresh=" << BoolString(autoaim_fresh)
            << " autoaim_fire=" << BoolString(autoaim_fire)
            << " decision_allows_fire=" << BoolString(decision_allows_fire)
            << " decision_fire_policy="
            << static_cast<int>(intent_fresh ? last_intent_.fire_policy : kFirePolicyHoldFire)
            << " safety_ok=" << BoolString(safety_ok)
            << " final_fire=" << BoolString(final_fire)
            << " posture_intent="
            << static_cast<int>(intent_fresh ? last_intent_.posture_intent : kPostureKeep)
            << " state_switch_out=" << static_cast<int>(out.state_switch)
            << " goal_id=" << static_cast<int>(out.goal_id)
            << " tactical_state=" << static_cast<int>(out.tactical_state)
            << " heat=" << (has_status_ ? last_status_.shooter_17mm_1_barrel_heat : 0)
            << " heat_limit=" << (has_status_ ? last_status_.shooter_barrel_heat_limit : 0)
            << " ammo=" << (has_status_ ? last_status_.projectile_allowance_17mm : 0)
            << " rule_action_type=" << static_cast<int>(out.rule_action_type)
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

    std::string autoaim_raw_topic_{};
    std::string intent_topic_{};
    std::string gimbal_status_topic_{};
    std::string final_gimbal_cmd_topic_{};
    std::string debug_topic_{};

    rm_interfaces::msg::GimbalCmd last_autoaim_raw_{};
    rm_interfaces::msg::SentryIntent last_intent_{};
    rm_interfaces::msg::GimbalStatus last_status_{};
    rclcpp::Time last_autoaim_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_intent_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_status_time_{0, 0, get_clock()->get_clock_type()};
    bool has_autoaim_raw_{false};
    bool has_intent_{false};
    bool has_status_{false};

    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr autoaim_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::SentryIntent>::SharedPtr intent_sub_{};
    rclcpp::Subscription<rm_interfaces::msg::GimbalStatus>::SharedPtr status_sub_{};
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
