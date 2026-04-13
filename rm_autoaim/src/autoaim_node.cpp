// =============================================================================
// autoaim_node.cpp — 自瞄全流水线节点实现
// =============================================================================
// 核心流水线:
//   on_armors() 触发
//     → find_imu_bracket(T_img)         // 时间对齐
//     → slerp(Q_before, Q_after, α)     // SLERP 四元数插值
//     → solve_pnp(apexes)               // PnP 求解 P_cam
//     → transform_to_world(P_cam, Q)    // 相机系→当前云台系 / 世界惯性系
//     → tracker_.update(p_world, t)     // IMM-UKF 观测更新
//     → tracker_.predict(t + Δt)        // 前馈预测
//     → aimer_.solve(...)               // 物理逆运动学 + 三段火控
//     → publish(GimbalCmd)              // 发布控制指令
// =============================================================================
#include "rm_autoaim/autoaim_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>

using ArmorDetection = rm_interfaces::msg::ArmorDetection;
using ArmorDetections = rm_interfaces::msg::ArmorDetections;
using GimbalCmd = rm_interfaces::msg::GimbalCmd;
using GimbalStatus = rm_interfaces::msg::GimbalStatus;

namespace rm_autoaim
{

// =============================================================================
// 构造函数：参数加载 + 订阅/发布 + Aimer/Tracker 初始化
// =============================================================================
AutoaimNode::AutoaimNode(const rclcpp::NodeOptions & options)
: Node("rm_autoaim", options),
  tracker_(),
  aimer_()
{
  // ===================== PnP 参数 =====================

  this->declare_parameter<std::vector<double>>(
    "camera_matrix",
    {1000.0, 0.0, 320.0,
     0.0, 1000.0, 240.0,
     0.0, 0.0, 1.0});

  this->declare_parameter<std::vector<double>>(
    "dist_coeffs", {0.0, 0.0, 0.0, 0.0, 0.0});

  auto cm = this->get_parameter("camera_matrix").as_double_array();
  camera_matrix_ = cv::Mat(3, 3, CV_64F, cm.data()).clone();

  auto dc = this->get_parameter("dist_coeffs").as_double_array();
  dist_coeffs_ = cv::Mat(1, static_cast<int>(dc.size()), CV_64F, dc.data()).clone();

  if (std::abs(camera_matrix_.at<double>(0, 2) - 320.0) < 1e-3 &&
      std::abs(camera_matrix_.at<double>(1, 2) - 240.0) < 1e-3)
  {
    RCLCPP_WARN(
      get_logger(),
      "camera_matrix is still at built-in default principal point (320,240). "
      "If you intended to use calibrated params, check the ROS2 params file root key "
      "matches node name 'rm_autoaim'.");
  }

  // 相机→云台外参
  this->declare_parameter<std::vector<double>>(
    "r_cam_to_gimbal",
    {1.0, 0.0, 0.0,
     0.0, 1.0, 0.0,
     0.0, 0.0, 1.0});

  this->declare_parameter<std::vector<double>>(
    "t_cam_to_gimbal", {0.0, 0.0, 0.0});

  auto rcg = this->get_parameter("r_cam_to_gimbal").as_double_array();
  if (rcg.size() != 9) {
    RCLCPP_ERROR(get_logger(), "r_cam_to_gimbal must have exactly 9 elements, got %zu",
                 rcg.size());
    throw std::runtime_error("Invalid r_cam_to_gimbal parameter");
  }

  // ★ 防呆行主序赋值: YAML 数组 [r00,r01,r02, r10,r11,r12, r20,r21,r22]
  //   → 矩阵 M(i,j) = rcg[i*3 + j]
  //   绝不使用 Eigen::Map 或逗号初始化（它们默认列主序会导致转置）
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_cam_to_gimbal_(i, j) = rcg[i * 3 + j];
    }
  }

  // 验证: 仅保留必要的旋转矩阵合法性检查
  double det = cv::determinant(cv::Mat(R_cam_to_gimbal_));
  if (std::fabs(det - 1.0) > 0.01) {
    RCLCPP_WARN(get_logger(), "WARNING: R_cam_to_gimbal is NOT a valid rotation matrix! det=%.6f", det);
  }

  auto tcg = this->get_parameter("t_cam_to_gimbal").as_double_array();
  t_cam_to_gimbal_ = cv::Vec3d(tcg[0], tcg[1], tcg[2]);

  // 装甲板物理尺寸
  this->declare_parameter<double>("armor_small_width",  0.135);
  this->declare_parameter<double>("armor_small_height", 0.055);
  this->declare_parameter<double>("armor_large_width",  0.225);
  this->declare_parameter<double>("armor_large_height", 0.055);

  armor_small_width_  = this->get_parameter("armor_small_width").as_double();
  armor_small_height_ = this->get_parameter("armor_small_height").as_double();
  armor_large_width_  = this->get_parameter("armor_large_width").as_double();
  armor_large_height_ = this->get_parameter("armor_large_height").as_double();

  // 其他 PnP 参数
  this->declare_parameter<std::string>("world_frame_id", "gimbal_odom");
  this->declare_parameter<double>("imu_buffer_duration", kDefaultImuBufferSec);
  this->declare_parameter<bool>("pnp_refine_iterative", true);
  this->declare_parameter<bool>("use_imu_world_transform", false);
  this->declare_parameter<bool>("enable_prediction_consistency_guard", true);
  this->declare_parameter<double>("prediction_consistency_guard_deg", 25.0);
  this->declare_parameter<bool>("enable_ray_consistency_guard", true);
  this->declare_parameter<double>("ray_consistency_guard_deg", 10.0);
  this->declare_parameter<std::string>("imu_topic", "/imu/data");
  this->declare_parameter<std::string>("gimbal_status_topic", "/gimbal_status");

  world_frame_id_      = this->get_parameter("world_frame_id").as_string();
  imu_buffer_duration_ = this->get_parameter("imu_buffer_duration").as_double();
  pnp_refine_          = this->get_parameter("pnp_refine_iterative").as_bool();
  use_imu_world_transform_ = this->get_parameter("use_imu_world_transform").as_bool();
  enable_prediction_consistency_guard_ =
    this->get_parameter("enable_prediction_consistency_guard").as_bool();
  prediction_consistency_guard_deg_ =
    this->get_parameter("prediction_consistency_guard_deg").as_double();
  enable_ray_consistency_guard_ =
    this->get_parameter("enable_ray_consistency_guard").as_bool();
  ray_consistency_guard_deg_ =
    this->get_parameter("ray_consistency_guard_deg").as_double();
  const std::string imu_topic = this->get_parameter("imu_topic").as_string();
  const std::string gimbal_status_topic =
    this->get_parameter("gimbal_status_topic").as_string();

  // ===================== Aimer 参数 =====================

  AimerParams aimer_params;

  this->declare_parameter<double>("gravity", 9.81);
  this->declare_parameter<double>("bullet_speed_default", 15.0);
  this->declare_parameter<double>("fire_delay", 0.10);
  this->declare_parameter<double>("yaw_offset", 0.0);
  this->declare_parameter<bool>("pitch_invert", false);

  this->declare_parameter<double>("fire_tolerance", 0.03);
  this->declare_parameter<double>("armor_facing_tolerance", 30.0);
  this->declare_parameter<double>("ctrv_threshold", 0.5);
  this->declare_parameter<int>("fire_min_frames", 5);
  this->declare_parameter<double>("fire_max_distance", 8.0);

  this->declare_parameter<double>("pitch_offset_k0", 0.0);
  this->declare_parameter<double>("pitch_offset_k1", 0.0);
  this->declare_parameter<double>("pitch_offset_k2", 0.0);

  aimer_params.gravity          = this->get_parameter("gravity").as_double();
  aimer_params.bullet_speed     = this->get_parameter("bullet_speed_default").as_double();
  aimer_params.fire_delay       = this->get_parameter("fire_delay").as_double();
  aimer_params.yaw_offset       = this->get_parameter("yaw_offset").as_double();
  aimer_params.pitch_invert     = this->get_parameter("pitch_invert").as_bool();

  aimer_params.fire_tolerance   = this->get_parameter("fire_tolerance").as_double();
  aimer_params.armor_facing_tol = this->get_parameter("armor_facing_tolerance").as_double();
  aimer_params.ctrv_threshold   = this->get_parameter("ctrv_threshold").as_double();
  aimer_params.fire_min_frames  = this->get_parameter("fire_min_frames").as_int();
  aimer_params.fire_max_distance = this->get_parameter("fire_max_distance").as_double();

  aimer_params.pitch_offset_k0  = this->get_parameter("pitch_offset_k0").as_double();
  aimer_params.pitch_offset_k1  = this->get_parameter("pitch_offset_k1").as_double();
  aimer_params.pitch_offset_k2  = this->get_parameter("pitch_offset_k2").as_double();

  aimer_ = Aimer(aimer_params);

  // ===================== 订阅 =====================

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(msg); });

  // 下位机状态反馈 (用于火控 Gate 1)
  status_sub_ = this->create_subscription<GimbalStatus>(
    gimbal_status_topic, rclcpp::SensorDataQoS(),
    [this](const GimbalStatus::ConstSharedPtr msg) { on_gimbal_status(msg); });

  armor_sub_ = this->create_subscription<ArmorDetections>(
    "/detector/armors", rclcpp::SensorDataQoS(),
    [this](const ArmorDetections::ConstSharedPtr msg) { on_armors(msg); });

  // ===================== 发布 =====================

  world_points_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/autoaim/debug_world_points", rclcpp::SensorDataQoS());

  gimbal_cmd_pub_ = this->create_publisher<GimbalCmd>(
    "/gimbal_cmd", rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(),
    "rm_autoaim initialized. tracking_frame=%s bullet=%.1fm/s delay=%.3fs "
    "K[fx=%.1f fy=%.1f cx=%.1f cy=%.1f]",
    use_imu_world_transform_ ? "imu_world" : "current_gimbal",
    aimer_params.bullet_speed,
    aimer_params.fire_delay,
    camera_matrix_.at<double>(0, 0),
    camera_matrix_.at<double>(1, 1),
    camera_matrix_.at<double>(0, 2),
    camera_matrix_.at<double>(1, 2));
}

// =============================================================================
// IMU 回调：推入缓冲区 + 淘汰过期数据
// =============================================================================
void AutoaimNode::on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  ImuStamp entry;
  entry.stamp = rclcpp::Time(msg->header.stamp);
  entry.orientation = cv::Quatd(
    msg->orientation.w,
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z).normalize();

  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back(std::move(entry));

    rclcpp::Time newest = imu_buffer_.back().stamp;
    while (imu_buffer_.size() > 2 &&
           (newest - imu_buffer_.front().stamp).seconds() > imu_buffer_duration_)
    {
      imu_buffer_.pop_front();
    }
  }
}

// =============================================================================
// 云台状态回调：更新当前真实角度 (用于火控 Gate 1)
// =============================================================================
void AutoaimNode::on_gimbal_status(const GimbalStatus::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(gimbal_mutex_);
  current_yaw_deg_       = msg->gimbal_yaw;
  current_pitch_deg_     = msg->gimbal_pitch;
  current_bullet_speed_  = msg->bullet_speed;
}

// =============================================================================
// 装甲板检测回调 — 完整自瞄流水线
// =============================================================================
void AutoaimNode::on_armors(const ArmorDetections::ConstSharedPtr msg)
{
  if (msg->detections.empty()) {
    // ---- 无检测 → 发送 mode=0 idle 指令 ----
    // mode=0: 告知下位机当前无目标，下位机可切换到手动/巡逻模式
    // state_switch: 0=无效, 1=Move, 2=Attack, 3=Defense
    //   无目标时保持 Move (1)，让下位机回到巡逻姿态
    GimbalCmd cmd;
    cmd.target_yaw   = 0.0;
    cmd.target_pitch = 0.0;
    cmd.yaw_vel      = 0.0;
    cmd.pitch_vel    = 0.0;
    cmd.state_switch = 1;   // Move — 回到巡逻/默认姿态
    cmd.fire_control = 0;
    cmd.mode         = 0;   // idle — 无目标
    gimbal_cmd_pub_->publish(cmd);

    // 重置 Tracker (目标丢失)
    tracker_frame_count_ = 0;
    return;
  }

  // ======== Step 1: 时间对齐 — 查找包围 T_img 的两个 IMU 样本 ========
  rclcpp::Time t_img(msg->header.stamp);

  ImuStamp imu_before, imu_after;
  if (!find_imu_bracket(t_img, imu_before, imu_after)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No IMU bracket for stamp %.3f. Dropping frame.", t_img.seconds());
    return;
  }

  // ======== Step 2: SLERP 四元数插值 ========
  double dt_total  = (imu_after.stamp - imu_before.stamp).seconds();
  double dt_interp = (t_img - imu_before.stamp).seconds();
  double alpha = (dt_total > 1e-9) ? (dt_interp / dt_total) : 0.0;
  alpha = std::clamp(alpha, 0.0, 1.0);

  cv::Quatd q_gimbal_to_world = slerp(imu_before.orientation, imu_after.orientation, alpha);

  // ======== Step 3: PnP + 坐标变换 → 选最优装甲板 ========
  struct ArmorResult
  {
    cv::Point3d p_world;
    double confidence;
    double distance;
    double center_axis_error_px;
    int class_id;
    const ArmorDetection * armor_msg;
    cv::Point2f center_px;
  };
  std::vector<ArmorResult> results;

  const cv::Point2f optical_axis_px(
    static_cast<float>(camera_matrix_.at<double>(0, 2)),
    static_cast<float>(camera_matrix_.at<double>(1, 2)));

  for (const auto & armor : msg->detections) {
    cv::Vec3d rvec, tvec;
    if (!solve_pnp(armor, rvec, tvec)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "solvePnP failed for armor class=%d", armor.class_id);
      continue;
    }

    cv::Point3d p_world = transform_to_world(tvec, q_gimbal_to_world);

    cv::Point2f center_px(0.0f, 0.0f);
    for (const auto & apex : armor.apexes) {
      center_px.x += static_cast<float>(apex.x);
      center_px.y += static_cast<float>(apex.y);
    }
    center_px.x *= 0.25f;
    center_px.y *= 0.25f;

    ArmorResult ar;
    ar.p_world    = p_world;
    ar.confidence = armor.confidence;
    ar.distance   = std::sqrt(
      p_world.x * p_world.x + p_world.y * p_world.y + p_world.z * p_world.z);
    ar.center_axis_error_px = cv::norm(center_px - optical_axis_px);
    ar.class_id = armor.class_id;
    ar.armor_msg = &armor;
    ar.center_px = center_px;
    results.push_back(ar);
  }

  if (results.empty()) return;

  // 选最接近相机光轴的装甲板。
  // 旧仓库优先靠近图像中心；这里改为靠近相机主点(optical axis)更符合几何语义，
  // 可避免误检因 PnP 假近距而抢占目标。
  auto best_it = std::min_element(results.begin(), results.end(),
    [](const ArmorResult & a, const ArmorResult & b) {
      if (std::abs(a.center_axis_error_px - b.center_axis_error_px) > 1e-3) {
        return a.center_axis_error_px < b.center_axis_error_px;
      }
      if (std::abs(a.confidence - b.confidence) > 1e-6) {
        return a.confidence > b.confidence;
      }
      return a.distance < b.distance;
    });

  const auto & best = *best_it;

  // 发布调试点云
  for (const auto & res : results) {
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp    = msg->header.stamp;
    pt.header.frame_id = world_frame_id_;
    pt.point.x = res.p_world.x;
    pt.point.y = res.p_world.y;
    pt.point.z = res.p_world.z;
    world_points_pub_->publish(pt);
  }

  // ======== Step 4: Tracker 观测更新 ========
  double current_time = t_img.seconds();
  Eigen::Vector3d obs(best.p_world.x, best.p_world.y, best.p_world.z);

  if (!tracker_.isInitialized()) {
    tracker_.init(obs, current_time);
    tracker_frame_count_ = 1;
  } else {
    tracker_.update(obs, current_time);
    tracker_frame_count_++;
  }

  // ======== Step 5: Tracker 前馈预测 ========
  // 预测 fire_delay 秒后的状态 (系统延迟补偿)
  double future_time = current_time + aimer_.getParams().fire_delay;
  auto predicted_state = tracker_.predict(future_time);

  // ======== Step 6: Aimer 解算 + 火控判定 ========
  auto model_probs = tracker_.getModelProbabilities();

  double cur_yaw, cur_pitch, cur_bullet;
  {
    std::lock_guard<std::mutex> lock(gimbal_mutex_);
    cur_yaw    = current_yaw_deg_;
    cur_pitch  = current_pitch_deg_;
    cur_bullet = current_bullet_speed_;
  }

  auto predicted_aim = aimer_.solve(
    predicted_state,
    model_probs,
    cur_yaw,
    cur_pitch,
    tracker_frame_count_,
    cur_bullet,
    !use_imu_world_transform_);

  Aimer::StateVec direct_state = Aimer::StateVec::Zero();
  direct_state(0) = obs.x();
  direct_state(1) = obs.y();
  direct_state(2) = obs.z();
  auto direct_aim = aimer_.solve(
    direct_state,
    model_probs,
    cur_yaw,
    cur_pitch,
    0,
    cur_bullet,
    !use_imu_world_transform_);

  cv::Point2f center_px = best.center_px;

  std::vector<cv::Point2f> pixel_points = {center_px};
  std::vector<cv::Point2f> undistorted_points;
  cv::undistortPoints(pixel_points, undistorted_points, camera_matrix_, dist_coeffs_);
  cv::Vec3d ray_cam(
    static_cast<double>(undistorted_points.front().x),
    static_cast<double>(undistorted_points.front().y),
    1.0);
  cv::Vec3d ray_gimbal = R_cam_to_gimbal_ * ray_cam;
  double ray_norm = std::sqrt(
    ray_gimbal(0) * ray_gimbal(0) +
    ray_gimbal(1) * ray_gimbal(1) +
    ray_gimbal(2) * ray_gimbal(2));
  if (ray_norm > 1e-9) {
    ray_gimbal *= (best.distance / ray_norm);
  }

  Aimer::StateVec ray_state = Aimer::StateVec::Zero();
  if (use_imu_world_transform_) {
    cv::Quatd ray_quat(0.0, ray_gimbal(0), ray_gimbal(1), ray_gimbal(2));
    cv::Quatd ray_world_quat = q_gimbal_to_world * ray_quat * q_gimbal_to_world.inv();
    ray_state(0) = ray_world_quat.x;
    ray_state(1) = ray_world_quat.y;
    ray_state(2) = ray_world_quat.z;
  } else {
    ray_state(0) = ray_gimbal(0);
    ray_state(1) = ray_gimbal(1);
    ray_state(2) = ray_gimbal(2);
  }
  auto ray_aim = aimer_.solve(
    ray_state,
    model_probs,
    cur_yaw,
    cur_pitch,
    0,
    cur_bullet,
    !use_imu_world_transform_);

  double ray_pnp_gap_yaw_deg = std::abs(std::atan2(
    std::sin(direct_aim.target_yaw - ray_aim.target_yaw),
    std::cos(direct_aim.target_yaw - ray_aim.target_yaw))) * 180.0 / M_PI;
  double ray_pnp_gap_pitch_deg =
    std::abs(direct_aim.target_pitch - ray_aim.target_pitch) * 180.0 / M_PI;
  bool use_ray_fallback =
    enable_ray_consistency_guard_ &&
    (ray_pnp_gap_yaw_deg > ray_consistency_guard_deg_ ||
     ray_pnp_gap_pitch_deg > ray_consistency_guard_deg_);

  const auto & base_aim = use_ray_fallback ? ray_aim : direct_aim;

  double aim_gap_yaw_deg = std::abs(std::atan2(
    std::sin(predicted_aim.target_yaw - base_aim.target_yaw),
    std::cos(predicted_aim.target_yaw - base_aim.target_yaw))) * 180.0 / M_PI;
  double aim_gap_pitch_deg =
    std::abs(predicted_aim.target_pitch - base_aim.target_pitch) * 180.0 / M_PI;
  bool use_direct_fallback =
    enable_prediction_consistency_guard_ &&
    (aim_gap_yaw_deg > prediction_consistency_guard_deg_ ||
     aim_gap_pitch_deg > prediction_consistency_guard_deg_);

  const auto & aim = use_direct_fallback ? base_aim : predicted_aim;

  // ======== Step 7: 构建 GimbalCmd ========
  // 有目标 → mode=1 (auto_aim)
  // fire_control 由 Aimer 三段火控判定:
  //   Gate 1: 炮管对齐 (target vs current)
  //   Gate 2: 装甲板相位窗口 (Spin Killer)
  //   Gate 3: 收敛/距离保护
  GimbalCmd cmd;
  cmd.target_yaw   = aim.target_yaw;
  cmd.target_pitch = aim.target_pitch;
  cmd.yaw_vel      = aim.yaw_vel;
  cmd.pitch_vel    = aim.pitch_vel;
  cmd.fire_control = ((use_direct_fallback || use_ray_fallback) ? 0 : (aim.can_fire ? 1 : 0));
  cmd.state_switch = 2;  // Attack — 有目标即 Attack (下位机小陀螺+跟踪)
  cmd.mode         = 1;  // auto_aim — 有目标

  gimbal_cmd_pub_->publish(cmd);

  if (use_ray_fallback != last_ray_guard_active_) {
    if (use_ray_fallback) {
      RCLCPP_WARN(get_logger(),
        "Ray guard: pnp=(%.2f°, %.2f°) ray=(%.2f°, %.2f°) gap=(%.2f°, %.2f°) "
        "px=(%.1f, %.1f) fire=0",
        direct_aim.target_yaw * 180.0 / M_PI,
        direct_aim.target_pitch * 180.0 / M_PI,
        ray_aim.target_yaw * 180.0 / M_PI,
        ray_aim.target_pitch * 180.0 / M_PI,
        ray_pnp_gap_yaw_deg,
        ray_pnp_gap_pitch_deg,
        center_px.x, center_px.y);
    } else {
      RCLCPP_INFO(get_logger(), "Ray guard cleared");
    }
    last_ray_guard_active_ = use_ray_fallback;
  }

  if (use_direct_fallback != last_prediction_guard_active_) {
    if (use_direct_fallback) {
      RCLCPP_WARN(get_logger(),
        "Prediction guard: tracker=(%.2f°, %.2f°) base=(%.2f°, %.2f°) gap=(%.2f°, %.2f°) fire=0",
        predicted_aim.target_yaw * 180.0 / M_PI,
        predicted_aim.target_pitch * 180.0 / M_PI,
        base_aim.target_yaw * 180.0 / M_PI,
        base_aim.target_pitch * 180.0 / M_PI,
        aim_gap_yaw_deg,
        aim_gap_pitch_deg);
    } else {
      RCLCPP_INFO(get_logger(), "Prediction guard cleared");
    }
    last_prediction_guard_active_ = use_direct_fallback;
  }

  double cur_yaw_rad = cur_yaw * M_PI / 180.0;
  double cur_pitch_rad = cur_pitch * M_PI / 180.0;
  double cmd_yaw_err_deg = std::atan2(
    std::sin(aim.target_yaw - cur_yaw_rad),
    std::cos(aim.target_yaw - cur_yaw_rad)) * 180.0 / M_PI;
  double cmd_pitch_err_deg = (aim.target_pitch - cur_pitch_rad) * 180.0 / M_PI;

  const char * source =
    use_direct_fallback ? (use_ray_fallback ? "ray" : "pnp") :
    (use_ray_fallback ? "ray" : "tracker");

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Aim: src=%s cmd=(%.2f°, %.2f°) err=(%.2f°, %.2f°) fire=%d gate=[%s%s%s] frames=%d",
    source,
    aim.target_yaw * 180.0 / M_PI,
    aim.target_pitch * 180.0 / M_PI,
    cmd_yaw_err_deg,
    cmd_pitch_err_deg,
    aim.can_fire ? 1 : 0,
    aim.gate1_alignment ? "1" : "0",
    aim.gate2_facing ? "1" : "0",
    aim.gate3_ready ? "1" : "0",
    tracker_frame_count_);
}

// =============================================================================
// IMU 时间戳包围查找
// =============================================================================
bool AutoaimNode::find_imu_bracket(
  const rclcpp::Time & target_t,
  ImuStamp & before,
  ImuStamp & after)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);

  if (imu_buffer_.size() < 2) return false;

  auto it = std::lower_bound(
    imu_buffer_.begin(), imu_buffer_.end(), target_t,
    [](const ImuStamp & entry, const rclcpp::Time & t) {
      return entry.stamp < t;
    });

  if (it == imu_buffer_.begin()) return false;
  if (it == imu_buffer_.end())   return false;

  before = *(it - 1);
  after  = *it;
  return true;
}

// =============================================================================
// SLERP 四元数球面线性插值
// =============================================================================
cv::Quatd AutoaimNode::slerp(const cv::Quatd & q0, const cv::Quatd & q1, double t)
{
  double dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;

  cv::Quatd q1_adj = q1;
  if (dot < 0.0) {
    q1_adj = cv::Quatd(-q1.w, -q1.x, -q1.y, -q1.z);
    dot = -dot;
  }

  const double kThreshold = 0.9995;
  if (dot > kThreshold) {
    cv::Quatd result(
      q0.w + t * (q1_adj.w - q0.w),
      q0.x + t * (q1_adj.x - q0.x),
      q0.y + t * (q1_adj.y - q0.y),
      q0.z + t * (q1_adj.z - q0.z));
    double norm = std::sqrt(
      result.w * result.w + result.x * result.x +
      result.y * result.y + result.z * result.z);
    return cv::Quatd(
      result.w / norm, result.x / norm,
      result.y / norm, result.z / norm);
  }

  double theta_0 = std::acos(std::clamp(dot, -1.0, 1.0));
  double sin_theta_0 = std::sin(theta_0);
  double theta = theta_0 * t;

  double s0 = std::cos(theta) - dot * std::sin(theta) / sin_theta_0;
  double s1 = std::sin(theta) / sin_theta_0;

  return cv::Quatd(
    s0 * q0.w + s1 * q1_adj.w,
    s0 * q0.x + s1 * q1_adj.x,
    s0 * q0.y + s1 * q1_adj.y,
    s0 * q0.z + s1 * q1_adj.z);
}

// =============================================================================
// PnP 求解 (IPPE + ITERATIVE 精炼)
// =============================================================================
bool AutoaimNode::solve_pnp(
  const ArmorDetection & armor,
  cv::Vec3d & rvec, cv::Vec3d & tvec)
{
  static const std::unordered_set<int> kBigArmorClassIds = {
    21, 22, 23, 24, 29, 30, 31, 32, 33, 34, 35, 36, 37,
  };
  bool is_large = kBigArmorClassIds.count(armor.class_id) > 0;

  double half_w = (is_large ? armor_large_width_ : armor_small_width_) / 2.0;
  double half_h = (is_large ? armor_large_height_ : armor_small_height_) / 2.0;

  std::vector<cv::Point3d> object_points = {
    {-half_w,  half_h, 0.0},
    { half_w,  half_h, 0.0},
    { half_w, -half_h, 0.0},
    {-half_w, -half_h, 0.0},
  };

  std::vector<cv::Point2d> image_points;
  for (int i = 0; i < 4; i++) {
    image_points.emplace_back(armor.apexes[i].x, armor.apexes[i].y);
  }

  // 阶段 1: IPPE
  std::vector<cv::Vec3d> rvecs, tvecs;
  bool ok = cv::solvePnPGeneric(
    object_points, image_points,
    camera_matrix_, dist_coeffs_,
    rvecs, tvecs,
    false, cv::SOLVEPNP_IPPE);

  if (!ok || rvecs.empty()) return false;

  rvec = rvecs[0];
  tvec = tvecs[0];

  // 阶段 2 (可选): ITERATIVE 精炼
  if (pnp_refine_) {
    ok = cv::solvePnP(
      object_points, image_points,
      camera_matrix_, dist_coeffs_,
      rvec, tvec,
      true, cv::SOLVEPNP_ITERATIVE);
    if (!ok) return false;
  }

  return true;
}

// =============================================================================
// 坐标变换: P_cam → P_gimbal → P_world
// 历史命名保留:
//   - use_imu_world_transform=false 时这里只做相机→当前云台系变换，返回的是 p_gimbal
//   - use_imu_world_transform=true  时才继续旋转到世界/惯性系
// =============================================================================
cv::Point3d AutoaimNode::transform_to_world(
  const cv::Vec3d & p_cam,
  const cv::Quatd & q_gimbal_to_world)
{
  cv::Vec3d p_gimbal = R_cam_to_gimbal_ * p_cam + t_cam_to_gimbal_;

  if (!use_imu_world_transform_) {
    return cv::Point3d(p_gimbal(0), p_gimbal(1), p_gimbal(2));
  }

  cv::Quatd p_quat(0.0, p_gimbal(0), p_gimbal(1), p_gimbal(2));
  cv::Quatd p_world_quat = q_gimbal_to_world * p_quat * q_gimbal_to_world.inv();
  return cv::Point3d(p_world_quat.x, p_world_quat.y, p_world_quat.z);
}

}  // namespace rm_autoaim

// =============================================================================
// main — 标准 ROS 2 节点入口
// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_autoaim::AutoaimNode>());
  rclcpp::shutdown();
  return 0;
}
