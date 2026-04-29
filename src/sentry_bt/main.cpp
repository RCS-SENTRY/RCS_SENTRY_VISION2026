#include <filesystem>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bt_compat.hpp"
#include "bt_setup.hpp"
#include "llm_interface.hpp"
#include "nav_interface.hpp"
#include "referee_interface.hpp"
#include "robot_context.hpp"
#include "shooter_interface.hpp"

class SentryBtNode : public rclcpp::Node
{
public:
    SentryBtNode()
      : Node("sentry_bt"),
        ctx_(std::make_shared<RobotContext>()),
        shooter_(*this)
    {
        this->declare_parameter<double>("tick_hz", 20.0);

        RegisterAllNodes(factory_, ctx_);

        auto blackboard = BT::Blackboard::create();
        blackboard->set("ctx", ctx_);

        const auto tree_template_path =
            std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "tree" / "sentry_main.template.xml";
        const auto tree_path =
            std::filesystem::path(SENTRY_BT_SOURCE_DIR) / "tree" / "sentry_main.xml";

        ExportTreeModelXML(factory_, tree_template_path, tree_path, false);
        tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_path, blackboard));

        gimbal_status_sub_ = this->create_subscription<rm_interfaces::msg::GimbalStatus>(
            "/gimbal_status", rclcpp::SensorDataQoS(),
            [this](const rm_interfaces::msg::GimbalStatus::SharedPtr msg) {
                referee_.UpdateFromStatus(*msg);
            });

        const double tick_hz = std::max(1.0, this->get_parameter("tick_hz").as_double());
        const auto tick_period =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / tick_hz));
        tick_timer_ = this->create_wall_timer(tick_period, [this]() { this->TickDecision(); });

        RCLCPP_INFO(
            get_logger(),
            "sentry_bt initialized: subscribing /gimbal_status, publishing %s",
            shooter_.output_topic().c_str());
    }

private:
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
        tree_->tickRoot();
        shooter_.PublishCommand(*ctx_);
        navigator_.PublishCommand(*ctx_);

        std::lock_guard<std::mutex> lock(ctx_->mtx);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "frame=%llu tactical=%s rule=%s goal=%s posture=%s fire=%s spin=%s supercap=%s "
            "ammo_target_total=%u revive_cmd=%u remote_ammo_inc=%u remote_hp_inc=%u "
            "posture_cmd_referee=%u activate_energy=%u claim_periodic_ammo=%u",
            static_cast<unsigned long long>(ctx_->frame_index),
            TacticalStateToString(ctx_->tactical_state),
            RuleActionTypeToString(ctx_->rule_action_type),
            ctx_->desired_goal.c_str(),
            PostureToString(ctx_->desired_posture),
            FirePolicyToString(ctx_->desired_fire_policy),
            SpinModeToString(ctx_->desired_spin_mode),
            SupercapModeToString(ctx_->desired_supercap_mode),
            static_cast<unsigned>(ctx_->ammo_exchange_target_total),
            static_cast<unsigned>(ctx_->revive_cmd),
            static_cast<unsigned>(ctx_->remote_ammo_req_inc),
            static_cast<unsigned>(ctx_->remote_hp_req_inc),
            static_cast<unsigned>(ctx_->posture_cmd_referee),
            static_cast<unsigned>(ctx_->activate_energy_confirm),
            static_cast<unsigned>(ctx_->claim_periodic_ammo));
    }

    BT::BehaviorTreeFactory factory_{};
    std::shared_ptr<RobotContext> ctx_{};
    RefereeInterface referee_{};
    LLMInterface llm_{};
    ShooterInterface shooter_;
    NavInterface navigator_{};
    std::unique_ptr<BT::Tree> tree_{};
    rclcpp::Subscription<rm_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_sub_{};
    rclcpp::TimerBase::SharedPtr tick_timer_{};
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
