// =============================================================================
// autoaim_node.hpp — 自瞄全流水线节点
// =============================================================================
// 核心流水线:
//   1. IMU SLERP 时间对齐
//   2. PnP: 2D 像素 → 相机系 3D
//   3. 坐标变换: 相机系 → 云台系 → 世界惯性系
//   4. IMM-UKF Tracker: 观测更新 + 前馈预测
//   5. Aimer: 物理前馈逆运动学 + 火控判定 + 重力补偿
//   6. 发布 GimbalCmd 控制指令
// =============================================================================
#ifndef RM_AUTOAIM__AUTOAIM_NODE_HPP_
#define RM_AUTOAIM__AUTOAIM_NODE_HPP_

#include <deque>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rm_interfaces/msg/autoaim_target_status.hpp>
#include <rm_interfaces/msg/armor_detections.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/gimbal_status.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>

#include "rm_autoaim/imm_ukf_tracker.hpp"
#include "rm_autoaim/aimer.hpp"

namespace rm_autoaim
{

// =============================================================================
// IMU 数据条目 — 用于时间对齐查找
// =============================================================================
struct ImuStamp
{
  rclcpp::Time stamp;
  cv::Quatd orientation;
};

enum class AutoaimTrackState
{
  LOST,
  TRACKING,
  TEMP_LOST
};

// =============================================================================
// AutoaimNode — 自瞄全流水线节点
// =============================================================================
class AutoaimNode : public rclcpp::Node
{
public:
  explicit AutoaimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ===========================================================================
  // 回调
  // ===========================================================================
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void on_gimbal_status(const rm_interfaces::msg::GimbalStatus::ConstSharedPtr msg);
  void on_armors(const rm_interfaces::msg::ArmorDetections::ConstSharedPtr msg);

  // ===========================================================================
  // 核心算法模块 (空间对齐)
  // ===========================================================================

  /// IMU 时间戳包围查找
  bool find_imu_bracket(
    const rclcpp::Time & target_t,
    ImuStamp & before,
    ImuStamp & after);

  /// 四元数 SLERP
  static cv::Quatd slerp(const cv::Quatd & q0, const cv::Quatd & q1, double t);

  /// PnP 求解 (IPPE + ITERATIVE 精炼)
  bool solve_pnp(
    const rm_interfaces::msg::ArmorDetection & armor,
    cv::Vec3d & rvec, cv::Vec3d & tvec);

  /// 坐标变换: P_cam → P_world
  cv::Point3d transform_to_world(
    const cv::Vec3d & p_cam,
    const cv::Quatd & q_gimbal_to_world);

  static const char * track_state_name(AutoaimTrackState state);

  void publish_fire_debug(
    const std::string & aim_source,
    bool alignment_ready,
    bool tracker_ready,
    bool ray_guard,
    bool prediction_guard,
    bool fallback_allows_fire,
    int fire_control,
    double model_prob_cv,
    double model_prob_ctrv,
    double target_distance,
    double cmd_yaw_err_deg,
    double cmd_pitch_err_deg,
    double yaw_window,
    double pitch_window,
    const std::string & reason,
    double age_sec);
  void publish_target_status(
    bool has_target,
    bool tracking,
    bool temp_lost,
    bool fire_ready,
    double target_distance,
    double yaw_error_deg,
    double pitch_error_deg,
    uint8_t aim_source,
    const std::string & reason);

  // ===========================================================================
  // 参数 — PnP / 坐标变换
  // ===========================================================================
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Matx33d R_cam_to_gimbal_;
  cv::Vec3d   t_cam_to_gimbal_;

  double armor_small_width_;
  double armor_small_height_;
  double armor_large_width_;
  double armor_large_height_;

  std::string world_frame_id_;
  double imu_buffer_duration_;
  bool pnp_refine_;
  bool use_imu_world_transform_ = false;
  bool enable_prediction_consistency_guard_ = true;
  double prediction_consistency_guard_deg_ = 25.0;
  bool enable_ray_consistency_guard_ = true;
  double ray_consistency_guard_deg_ = 10.0;
  bool allow_fire_on_prediction_fallback_ = true;
  bool allow_fire_on_ray_fallback_ = false;

  double temp_lost_timeout_sec_ = 0.30;
  double lost_timeout_sec_ = 0.80;
  bool hold_last_cmd_in_temp_lost_ = true;
  bool enable_target_status_ = true;

  // ===========================================================================
  // Tracker + Aimer
  // ===========================================================================
  UKFParams ukf_params_;
  ImmUkfTracker tracker_;
  Aimer aimer_;
  int tracker_frame_count_ = 0;
  bool last_ray_guard_active_ = false;
  bool last_prediction_guard_active_ = false;
  AutoaimTrackState track_state_{AutoaimTrackState::LOST};
  rclcpp::Time last_detection_time_;
  bool has_last_detection_{false};
  rm_interfaces::msg::GimbalCmd last_good_cmd_;
  bool has_last_good_cmd_{false};

  // 下位机反馈状态 (由 on_gimbal_status 更新)
  std::mutex gimbal_mutex_;
  double current_yaw_deg_   = 0.0;  // 下位机反馈 yaw (deg)
  double current_pitch_deg_ = 0.0;  // 下位机反馈 pitch (deg)
  double current_bullet_speed_ = 15.0;  // 下位机反馈弹速 (m/s)

  // ===========================================================================
  // IMU 环形缓冲区
  // ===========================================================================
  std::deque<ImuStamp> imu_buffer_;
  std::mutex imu_mutex_;
  static constexpr double kDefaultImuBufferSec = 2.0;

  // ===========================================================================
  // ROS 2 接口
  // ===========================================================================
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GimbalStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<rm_interfaces::msg::ArmorDetections>::SharedPtr armor_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr world_points_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fire_debug_pub_;
  rclcpp::Publisher<rm_interfaces::msg::AutoaimTargetStatus>::SharedPtr target_status_pub_;
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__AUTOAIM_NODE_HPP_
