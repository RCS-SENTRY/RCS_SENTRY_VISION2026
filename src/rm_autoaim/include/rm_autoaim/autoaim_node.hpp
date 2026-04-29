// =============================================================================
// autoaim_node.hpp — 自瞄全流水线节点
// =============================================================================
// 核心流水线:
//   1. IMU SLERP 时间对齐
//   2. PnP: 2D 像素 → 相机系 3D
//   3. 坐标变换: 相机系 → 云台系 → 世界惯性系
//   4. IMM-UKF Tracker: 观测更新 + 前馈预测
//   5. Aimer: 物理前馈逆运动学 + 三段火控 + 重力补偿
//   6. 发布 GimbalCmd 控制指令
// =============================================================================
#ifndef RM_AUTOAIM__AUTOAIM_NODE_HPP_
#define RM_AUTOAIM__AUTOAIM_NODE_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rm_interfaces/msg/armor_detections.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/gimbal_status.hpp>

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

  // ===========================================================================
  // Tracker + Aimer
  // ===========================================================================
  ImmUkfTracker tracker_;
  Aimer aimer_;
  int tracker_frame_count_ = 0;
  bool last_ray_guard_active_ = false;
  bool last_prediction_guard_active_ = false;

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
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__AUTOAIM_NODE_HPP_
