// =============================================================================
// hw_bridge_node.cpp — rm_hw_bridge 节点实现
// =============================================================================
// 职责一（下行）：订阅 /gimbal_cmd → 打包 VisionToNucFrame → CRC → 串口 TX
// 职责二（上行）：串口 RX → 帧解析/CRC校验 → 线程安全队列 → Timer 消费发布
//
// ★ 本节点是纯翻译层，不包含任何火控门控、延迟补偿或数据插值逻辑。
// ★ 线程规范：所有 publish() 仅在 Executor 线程（Timer 回调）中执行。
// =============================================================================
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// --------------- 自定义消息 ---------------
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/gimbal_status.hpp"
#include "rm_interfaces/msg/nav_cmd.hpp"

// --------------- 内部模块 ---------------
#include "rm_hw_bridge/protocol.hpp"
#include "rm_hw_bridge/crc16.hpp"
#include "rm_hw_bridge/serial_port.hpp"

// =============================================================================
// 线程安全队列 — 串口读线程 → Executor Timer 线程的数据桥梁
// =============================================================================
template <typename T>
class ThreadSafeQueue
{
public:
  void push(T item)
  {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (q_.size() >= MAX_SIZE) q_.pop();  // 丢弃最旧数据（背压策略）
      q_.push(std::move(item));
    }
    cv_.notify_one();
  }

  /// 非阻塞弹出所有待处理数据
  void drain(std::vector<T> & out)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    out.reserve(q_.size());
    while (!q_.empty()) {
      out.push_back(std::move(q_.front()));
      q_.pop();
    }
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return q_.empty();
  }

private:
  static constexpr size_t MAX_SIZE = 64;  // ~64ms @ 1kHz，超过即丢弃旧帧
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::queue<T> q_;
};

// =============================================================================
// 上行数据载体 — 从串口读线程传递到 Executor 线程的 POD
// =============================================================================
struct ParsedFrame
{
  NucToVisionFrame frame;       // memcpy 后的帧数据（栈上对齐安全）
  int64_t          stamp_ns;    // 解析时刻的 ROS 时间（nanoseconds since epoch）
};

// =============================================================================
// HwBridgeNode
// =============================================================================
class HwBridgeNode : public rclcpp::Node
{
public:
  HwBridgeNode() : Node("rm_hw_bridge")
  {
    // ---- 参数 ----
    this->declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 460800);
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<bool>("require_crc_check", false);

    const auto device = this->get_parameter("serial_device").as_string();
    const auto baud   = this->get_parameter("baudrate").as_int();
    frame_id_         = this->get_parameter("frame_id").as_string();
    require_crc_      = this->get_parameter("require_crc_check").as_bool();

    RCLCPP_INFO(get_logger(), "Opening serial: %s @ %ld bps", device.c_str(), static_cast<long>(baud));

    // ---- 串口（回调在读线程执行，仅做解析+入队，不 publish） ----
    serial_ = std::make_unique<rm_hw_bridge::SerialPort>(
      device, baud,
      [this](const uint8_t * data, size_t len) { serial_rx_callback(data, len); },
      get_logger());

    // ---- 下行订阅（Executor 线程安全） ----
    gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
      "/gimbal_cmd", rclcpp::SensorDataQoS(),
      [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
        on_gimbal_cmd(*msg);
      });

    nav_cmd_sub_ = this->create_subscription<rm_interfaces::msg::NavCmd>(
      "/nav_cmd", rclcpp::SensorDataQoS(),
      [this](const rm_interfaces::msg::NavCmd::SharedPtr msg) {
        on_nav_cmd(*msg);
      });

    // ---- 上行发布（仅由 Timer 在 Executor 线程中调用） ----
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS());
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS());
    gimbal_status_pub_ = this->create_publisher<rm_interfaces::msg::GimbalStatus>(
      "/gimbal_status", rclcpp::SensorDataQoS());

    // ---- 1000Hz Timer：从队列消费数据并 publish（Executor 线程） ----
    publish_timer_ = this->create_wall_timer(
      std::chrono::microseconds(1000),   // 1ms = 1000Hz
      [this]() { timer_publish_callback(); });

    RCLCPP_INFO(get_logger(), "rm_hw_bridge node initialized (timer-driven publish @ 1kHz).");
  }

  ~HwBridgeNode() override
  {
    // 析构时发送安全停止帧
    try {
      VisionToNucFrame safe{};
      safe.head[0] = 'S'; safe.head[1] = 'P';
      safe.crc16_TJ = rm_hw_bridge::compute_crc16(
        reinterpret_cast<uint8_t *>(&safe), sizeof(safe) - 2);
      serial_->write(reinterpret_cast<uint8_t *>(&safe), sizeof(safe));

      NavToNucFrame safe_nav{};
      safe_nav.crc16_TJ = rm_hw_bridge::compute_crc16(
        reinterpret_cast<uint8_t *>(&safe_nav), sizeof(safe_nav) - 2);
      serial_->write(reinterpret_cast<uint8_t *>(&safe_nav), sizeof(safe_nav));
    } catch (...) {}
  }

private:
  // =========================================================================
  //  Timer 回调 — Executor 线程中消费队列并 publish
  // =========================================================================
  void timer_publish_callback()
  {
    std::vector<ParsedFrame> frames;
    parsed_queue_.drain(frames);

    for (auto & pf : frames) {
      const auto stamp = rclcpp::Time(pf.stamp_ns);
      const auto * pkt = &pf.frame;

      publish_imu(pkt, stamp);
      publish_joint_state(pkt, stamp);
      publish_gimbal_status(pkt, stamp);

      // 周期性日志（~1Hz）
      log_counter_++;
      if (log_counter_ % 100 == 0) {
        RCLCPP_INFO(get_logger(),
          "[RX] mode=%d yaw=%.2f° pitch=%.2f° bullet_speed=%.1f m/s "
          "chassis=(%.2f, %.2f, %.2f)",
          pkt->mode_TJ, pkt->yaw_TJ, pkt->pitch_TJ, pkt->bullet_speed_TJ,
          pkt->chassis_vx, pkt->chassis_vy, pkt->chassis_wz);
      }
    }
  }

  // =========================================================================
  //  下行：ROS 2 → 串口（Executor 线程，安全）
  // =========================================================================
  void on_gimbal_cmd(const rm_interfaces::msg::GimbalCmd & msg)
  {
    VisionToNucFrame frame{};
    frame.mode_TJ          = static_cast<uint8_t>(msg.mode);
    frame.yaw_TJ           = static_cast<float>(msg.target_yaw);
    frame.yaw_vel_TJ       = static_cast<float>(msg.yaw_vel);
    frame.yaw_acc_TJ       = 0.0f;
    frame.pitch_TJ         = static_cast<float>(msg.target_pitch);
    frame.pitch_vel_TJ     = static_cast<float>(msg.pitch_vel);
    frame.pitch_acc_TJ     = 0.0f;
    frame.state_switch_TJ  = static_cast<int8_t>(msg.state_switch);
    frame.fire_control_TJ  = static_cast<int8_t>(msg.fire_control);
    frame.protocol_version = static_cast<uint8_t>(msg.protocol_version);
    frame.goal_id          = static_cast<uint8_t>(msg.goal_id);
    frame.tactical_state   = static_cast<uint8_t>(msg.tactical_state);
    frame.posture          = static_cast<uint8_t>(msg.posture);
    frame.fire_policy      = static_cast<uint8_t>(msg.fire_policy);
    frame.spin_mode        = static_cast<uint8_t>(msg.spin_mode);
    frame.supercap_mode    = static_cast<uint8_t>(msg.supercap_mode);
    frame.rule_action_type = static_cast<uint8_t>(msg.rule_action_type);
    frame.ammo_exchange_target_total =
      static_cast<uint16_t>(msg.ammo_exchange_target_total);
    frame.revive_cmd               = static_cast<uint8_t>(msg.revive_cmd);
    frame.remote_ammo_req_inc      = static_cast<uint8_t>(msg.remote_ammo_req_inc);
    frame.remote_hp_req_inc        = static_cast<uint8_t>(msg.remote_hp_req_inc);
    frame.posture_cmd_referee      = static_cast<uint8_t>(msg.posture_cmd_referee);
    frame.activate_energy_confirm  = static_cast<uint8_t>(msg.activate_energy_confirm);
    frame.claim_periodic_ammo      = static_cast<uint8_t>(msg.claim_periodic_ammo);
    frame.crc16_TJ = rm_hw_bridge::compute_crc16(
      reinterpret_cast<uint8_t *>(&frame), sizeof(frame) - 2);

    serial_->write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
  }

  void on_nav_cmd(const rm_interfaces::msg::NavCmd & msg)
  {
    NavToNucFrame frame{};
    frame.linear_x  = static_cast<float>(msg.linear_x);
    frame.linear_y  = static_cast<float>(msg.linear_y);
    frame.angular_z = static_cast<float>(msg.angular_z);
    frame.isReached = static_cast<int8_t>(msg.is_reached);
    frame.crc16_TJ = rm_hw_bridge::compute_crc16(
      reinterpret_cast<uint8_t *>(&frame), sizeof(frame) - 2);

    serial_->write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
  }

  // =========================================================================
  //  上行：串口读线程 → 解析 → 入队（不 publish！）
  // =========================================================================
  void serial_rx_callback(const uint8_t * data, size_t len)
  {
    rx_buffer_.insert(rx_buffer_.end(), data, data + len);

    const size_t MAX_BUF = 4096;
    if (rx_buffer_.size() > MAX_BUF) {
      rx_buffer_.erase(
        rx_buffer_.begin(),
        rx_buffer_.begin() + (rx_buffer_.size() - VISION_RX_FRAME_SIZE));
    }

    while (rx_buffer_.size() >= VISION_RX_FRAME_SIZE) {
      auto it = rx_buffer_.begin();
      for (; it != rx_buffer_.end() - 1; ++it) {
        if (*it == 'S' && *(it + 1) == 'P') break;
      }

      if (it == rx_buffer_.end() - 1) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 1);
        break;
      }

      size_t offset = static_cast<size_t>(std::distance(rx_buffer_.begin(), it));
      if (rx_buffer_.size() - offset < VISION_RX_FRAME_SIZE) {
        break;
      }

      const uint8_t * raw = rx_buffer_.data() + offset;
      if (try_parse_frame(raw, VISION_RX_FRAME_SIZE)) {
        rx_buffer_.erase(
          rx_buffer_.begin(), it + static_cast<long>(VISION_RX_FRAME_SIZE));
      } else {
        rx_buffer_.erase(rx_buffer_.begin(), it + 2);
      }
    }
  }

  // =========================================================================
  //  帧解析 — memcpy 替代 reinterpret_cast，CRC 强制校验
  // =========================================================================
  bool try_parse_frame(const uint8_t * data, size_t len)
  {
    if (len < VISION_RX_FRAME_SIZE) return false;

    // ---- 1. 帧头校验 ----
    if (data[0] != 'S' || data[1] != 'P') return false;

    // ---- 2. CRC16 校验 ----
    //    下位机暂未实现 CRC 时，require_crc_==false 可跳过校验（仅限调试！）
    if (!rm_hw_bridge::check_crc16(data, len)) {
      if (require_crc_) {
        // 严格模式：丢弃脏包
        RCLCPP_DEBUG(get_logger(), "CRC16 mismatch, dropping dirty packet");
        return false;
      } else {
        // 调试模式：放行但警告（每 5 秒打印一次）
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "CRC check bypassed for testing! Do not use in competition!");
      }
    }

    // ---- 3. 安全解包：memcpy 到对齐的栈变量 ----
    //    避免对 DMA/环形缓冲区中未对齐数据直接 reinterpret_cast
    NucToVisionFrame aligned_frame;
    std::memcpy(&aligned_frame, data, sizeof(NucToVisionFrame));

    // ---- 4. 记录解析时刻时间戳 ----
    int64_t stamp_ns = this->now().nanoseconds();

    // ---- 5. 入队（由 Timer 在 Executor 线程中消费） ----
    parsed_queue_.push({aligned_frame, stamp_ns});

    return true;
  }

  // =========================================================================
  //  发布函数 — 仅在 Timer 回调（Executor 线程）中调用
  // =========================================================================
  void publish_imu(const NucToVisionFrame * pkt, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = stamp;
    imu.header.frame_id = frame_id_;

    // 姿态 → 四元数
    // ★ 坐标系契约（与 STM32 下位机对齐）:
    //   yaw   = CCW+ (向左为正), 与 ROS REP-103 一致 → 直传
    //   pitch = 向上为负(向下为正), 与 ROS REP-103 FLU 一致 → 直传
    //   roll  = 右倾增大, 与 ROS 一致 → 直传
    const double yaw_rad   = static_cast<double>(pkt->yaw_TJ)   * M_PI / 180.0;
    const double pitch_rad = static_cast<double>(pkt->pitch_TJ)  * M_PI / 180.0;
    const double roll_rad  = static_cast<double>(pkt->roll_TJ)  * M_PI / 180.0;

    const double cy = std::cos(yaw_rad / 2.0), sy = std::sin(yaw_rad / 2.0);
    const double cp = std::cos(pitch_rad / 2.0), sp = std::sin(pitch_rad / 2.0);
    const double cr = std::cos(roll_rad / 2.0), sr = std::sin(roll_rad / 2.0);

    imu.orientation.w = cr * cp * cy + sr * sp * sy;
    imu.orientation.x = sr * cp * cy - cr * sp * sy;
    imu.orientation.y = cr * sp * cy + sr * cp * sy;
    imu.orientation.z = cr * cp * sy - sr * sp * cy;

    const float d2r = static_cast<float>(M_PI / 180.0);
    // 角速度符号与角度一致: 全部直传
    imu.angular_velocity.x = static_cast<float>(pkt->roll_vel_TJ)  * d2r;
    imu.angular_velocity.y = static_cast<float>(pkt->pitch_vel_TJ) * d2r;
    imu.angular_velocity.z = static_cast<float>(pkt->yaw_vel_TJ)   * d2r;

    for (int i = 0; i < 9; i++) {
      imu.orientation_covariance[i]         = (i % 4 == 0) ? 0.0 : -1.0;
      imu.angular_velocity_covariance[i]    = -1.0;
      imu.linear_acceleration_covariance[i] = -1.0;
    }

    imu_pub_->publish(imu);
  }

  void publish_joint_state(const NucToVisionFrame * pkt, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp    = stamp;
    js.header.frame_id = frame_id_;
    js.name  = {"gimbal_yaw", "gimbal_pitch"};
    // 全部直传，与 publish_imu() 坐标系契约一致
    js.position = {
      static_cast<double>(pkt->yaw_TJ)   * M_PI / 180.0,
      static_cast<double>(pkt->pitch_TJ) * M_PI / 180.0
    };
    js.velocity = {
      static_cast<double>(pkt->yaw_vel_TJ)   * M_PI / 180.0,
      static_cast<double>(pkt->pitch_vel_TJ) * M_PI / 180.0
    };
    joint_state_pub_->publish(js);
  }

  void publish_gimbal_status(const NucToVisionFrame * pkt, const rclcpp::Time & stamp)
  {
    rm_interfaces::msg::GimbalStatus status;
    status.header.stamp    = stamp;
    status.header.frame_id = frame_id_;

    status.gimbal_yaw          = pkt->yaw_TJ;
    status.gimbal_pitch        = pkt->pitch_TJ;
    status.gimbal_roll         = pkt->roll_TJ;
    status.gimbal_yaw_rate     = pkt->yaw_vel_TJ;
    status.gimbal_pitch_rate   = pkt->pitch_vel_TJ;
    status.gimbal_roll_rate    = pkt->roll_vel_TJ;
    status.bullet_speed        = pkt->bullet_speed_TJ;
    status.bullet_count        = pkt->bullet_count_TJ;
    status.mode                = pkt->mode_TJ;
    status.game_status_nav     = pkt->game_status_NAV;

    status.chassis_vx          = pkt->chassis_vx;
    status.chassis_vy          = pkt->chassis_vy;
    status.chassis_wz          = pkt->chassis_wz;
    status.chassis_power       = pkt->chassis_power;
    status.remain_energy       = pkt->remain_energy;
    status.supply_speed        = pkt->supply_speed;
    status.game_progress       = pkt->game_progress;
    status.stage_remain_time   = pkt->stage_remain_time;

    status.event_data                         = pkt->event_data;
    status.robot_id                           = pkt->robot_id;
    status.robot_level                        = pkt->robot_level;
    status.current_hp                         = pkt->current_HP;
    status.maximum_hp                         = pkt->maximum_HP;
    status.shooter_barrel_cooling_value       = pkt->shooter_barrel_cooling_value;
    status.shooter_barrel_heat_limit          = pkt->shooter_barrel_heat_limit;
    status.chassis_power_limit                = pkt->chassis_power_limit;
    status.power_management_gimbal_output     = pkt->power_management_gimbal_output;
    status.power_management_chassis_output    = pkt->power_management_chassis_output;
    status.power_management_shooter_output    = pkt->power_management_shooter_output;

    status.exchanged_projectile_allowance     = pkt->exchanged_projectile_allowance;
    status.remote_exchange_projectile_count   = pkt->remote_exchange_projectile_count;
    status.remote_exchange_hp_count           = pkt->remote_exchange_hp_count;
    status.can_confirm_revival                = pkt->can_confirm_revival;
    status.can_buy_instant_revival            = pkt->can_buy_instant_revival;
    status.instant_revival_cost               = pkt->instant_revival_cost;
    status.is_disengaged                      = pkt->is_disengaged;
    status.team_17mm_exchange_remain          = pkt->team_17mm_exchange_remain;
    status.sentry_posture                     = pkt->sentry_posture;
    status.can_activate_energy_mechanism      = pkt->can_activate_energy_mechanism;

    status.recovery_buff                      = pkt->recovery_buff;
    status.cooling_buff                       = pkt->cooling_buff;
    status.defence_buff                       = pkt->defence_buff;
    status.vulnerability_buff                 = pkt->vulnerability_buff;
    status.attack_buff                        = pkt->attack_buff;
    status.remaining_energy                   = pkt->remaining_energy;
    status.armor_id                           = pkt->armor_id;
    status.hp_deduction_reason                = pkt->HP_deduction_reason;

    status.projectile_allowance_17mm         = pkt->projectile_allowance_17mm;
    status.projectile_allowance_42mm         = pkt->projectile_allowance_42mm;
    status.remaining_gold_coin               = pkt->remaining_gold_coin;
    status.projectile_allowance_fortress     = pkt->projectile_allowance_fortress;

    status.buffer_energy                     = pkt->buffer_energy;
    status.rfid_status                       = pkt->rfid_status;
    status.rfid_status_2                     = pkt->rfid_status_2;
    status.shooter_17mm_1_barrel_heat       = pkt->shooter_17mm_1_barrel_heat;

    status.ally_1_robot_hp                  = pkt->ally_1_robot_HP;
    status.ally_2_robot_hp                  = pkt->ally_2_robot_HP;
    status.ally_3_robot_hp                  = pkt->ally_3_robot_HP;
    status.ally_4_robot_hp                  = pkt->ally_4_robot_HP;
    status.ally_reserved_hp                 = pkt->reserved;
    status.ally_7_robot_hp                  = pkt->ally_7_robot_HP;
    status.ally_outpost_hp                  = pkt->ally_outpost_HP;
    status.ally_base_hp                     = pkt->ally_base_HP;

    gimbal_status_pub_->publish(status);
  }

  // =========================================================================
  //  成员
  // =========================================================================
  std::unique_ptr<rm_hw_bridge::SerialPort> serial_;
  std::string frame_id_;
  bool require_crc_ = false;
  uint32_t log_counter_ = 0;

  // 上行：读线程 → Executor 线程的桥梁
  ThreadSafeQueue<ParsedFrame> parsed_queue_;
  std::vector<uint8_t> rx_buffer_;   // 仅在读线程访问

  // ROS 2 接口
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<rm_interfaces::msg::NavCmd>::SharedPtr   nav_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    joint_state_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HwBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
