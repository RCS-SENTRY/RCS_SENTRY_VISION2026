// =============================================================================
// autoaim_node.cpp — 自瞄全流水线节点实现
// =============================================================================
// 核心流水线:
//   on_armors() 触发
//     → find_imu_bracket(T_img)         // 时间对齐
//     → slerp(Q_before, Q_after, α)     // SLERP 四元数插值
//     → solve_pnp(apexes)               // PnP 求解 P_cam
//     → transform_to_world(P_cam, Q)    // 相机系→当前云台系 / 世界惯性系
//     → CsuArmorTracker/raw_pnp         // 输出可击打 AimTarget
//     → ManualCompensator               // yaw/pitch LUT
//     → aimer_.solve(AimTarget, ...)    // 逆运动学 + 火控判定
//     → publish(GimbalCmd)              // 发布控制指令
// =============================================================================
#include "rm_autoaim/autoaim_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>

using ArmorDetection = rm_interfaces::msg::ArmorDetection;
using ArmorDetections = rm_interfaces::msg::ArmorDetections;
using AutoaimTargetStatus = rm_interfaces::msg::AutoaimTargetStatus;
using GimbalCmd = rm_interfaces::msg::GimbalCmd;
using GimbalStatus = rm_interfaces::msg::GimbalStatus;

namespace rm_autoaim
{

namespace
{

double clamp01(double value, const char * name, rclcpp::Logger logger)
{
  double clamped = std::clamp(value, 0.0, 1.0);
  if (clamped != value) {
    RCLCPP_WARN(logger, "%s=%.6f is outside [0,1], clamped to %.6f", name, value, clamped);
  }
  return clamped;
}

double positive_or_default(double value, double fallback, const char * name, rclcpp::Logger logger)
{
  if (value <= 0.0) {
    RCLCPP_WARN(logger, "%s=%.6f must be > 0, using safe default %.6f", name, value, fallback);
    return fallback;
  }
  return value;
}

std::vector<std::pair<double, double>> pair_lut_from_flat(
  const std::vector<double> & flat,
  const std::vector<std::pair<double, double>> & fallback,
  const char * name,
  rclcpp::Logger logger)
{
  if (flat.empty()) return fallback;
  if (flat.size() % 2 != 0) {
    RCLCPP_WARN(logger, "%s must contain distance/offset pairs, using fallback LUT", name);
    return fallback;
  }
  std::vector<std::pair<double, double>> lut;
  lut.reserve(flat.size() / 2);
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    lut.emplace_back(flat[i], flat[i + 1]);
  }
  return lut;
}

std::string vec_to_debug(const Eigen::Vector3d & v)
{
  std::ostringstream ss;
  ss << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
  return ss.str();
}

}  // namespace

// =============================================================================
// 构造函数：参数加载 + 订阅/发布 + Aimer/Tracker 初始化
// =============================================================================
AutoaimNode::AutoaimNode(const rclcpp::NodeOptions & options)
: Node("rm_autoaim", options),
  ukf_params_(),
  tracker_(ukf_params_),
  csu_tracker_(csu_tracker_params_),
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
  this->declare_parameter<double>("temp_lost_timeout_sec", 0.30);
  this->declare_parameter<double>("lost_timeout_sec", 0.80);
  this->declare_parameter<bool>("hold_last_cmd_in_temp_lost", true);
  this->declare_parameter<bool>("allow_fire_on_prediction_fallback", true);
  this->declare_parameter<bool>("allow_fire_on_ray_fallback", false);
  this->declare_parameter<bool>("enable_target_status", true);
  this->declare_parameter<std::string>("target_status_topic", "/autoaim/target_status");
  this->declare_parameter<std::string>("imu_topic", "/imu/data");
  this->declare_parameter<std::string>("gimbal_status_topic", "/gimbal_status");
  this->declare_parameter<std::string>("tracker_backend", "csu_tracker");
  this->declare_parameter<bool>("enable_raw_pnp_fallback", true);
  this->declare_parameter<bool>("raw_pnp_fallback_on_tracker_lost", true);
  this->declare_parameter<double>("fallback_jump_guard_m", 1.0);

  this->declare_parameter<bool>("manual_compensator.enabled", true);
  this->declare_parameter<std::vector<double>>(
    "manual_compensator.pitch_lut",
    {2.0, -0.222, 3.0, -0.222, 4.0, -0.222, 5.0, -0.222});
  this->declare_parameter<std::vector<double>>(
    "manual_compensator.yaw_lut",
    {2.0, -0.002, 5.0, -0.002});
  this->declare_parameter<double>("manual_compensator.clamp_min_distance", 0.5);
  this->declare_parameter<double>("manual_compensator.clamp_max_distance", 8.0);

  this->declare_parameter<double>("tracker.max_match_distance", 0.5);
  this->declare_parameter<double>("tracker.max_lost_time", 0.3);
  this->declare_parameter<int>("tracker.min_detect_count", 2);
  this->declare_parameter<int>("tracker.min_tracking_count_for_fire", 3);
  this->declare_parameter<double>("tracker.fire_max_distance", 8.0);
  this->declare_parameter<bool>("tracker.allow_fire_on_raw_pnp_fallback", false);
  this->declare_parameter<bool>("tracker.allow_fire_on_csu_tracker", true);

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
  temp_lost_timeout_sec_ = positive_or_default(
    this->get_parameter("temp_lost_timeout_sec").as_double(), 0.30,
    "temp_lost_timeout_sec", get_logger());
  lost_timeout_sec_ = positive_or_default(
    this->get_parameter("lost_timeout_sec").as_double(), 0.80,
    "lost_timeout_sec", get_logger());
  if (lost_timeout_sec_ < temp_lost_timeout_sec_) {
    RCLCPP_WARN(
      get_logger(),
      "lost_timeout_sec=%.3f is smaller than temp_lost_timeout_sec=%.3f; using temp_lost_timeout_sec",
      lost_timeout_sec_, temp_lost_timeout_sec_);
    lost_timeout_sec_ = temp_lost_timeout_sec_;
  }
  hold_last_cmd_in_temp_lost_ =
    this->get_parameter("hold_last_cmd_in_temp_lost").as_bool();
  allow_fire_on_prediction_fallback_ =
    this->get_parameter("allow_fire_on_prediction_fallback").as_bool();
  allow_fire_on_ray_fallback_ =
    this->get_parameter("allow_fire_on_ray_fallback").as_bool();
  enable_target_status_ =
    this->get_parameter("enable_target_status").as_bool();
  tracker_backend_ = this->get_parameter("tracker_backend").as_string();
  enable_raw_pnp_fallback_ =
    this->get_parameter("enable_raw_pnp_fallback").as_bool();
  raw_pnp_fallback_on_tracker_lost_ =
    this->get_parameter("raw_pnp_fallback_on_tracker_lost").as_bool();
  fallback_jump_guard_m_ = positive_or_default(
    this->get_parameter("fallback_jump_guard_m").as_double(), 1.0,
    "fallback_jump_guard_m", get_logger());

  csu_tracker_params_.max_match_distance = positive_or_default(
    this->get_parameter("tracker.max_match_distance").as_double(), 0.5,
    "tracker.max_match_distance", get_logger());
  csu_tracker_params_.max_lost_time = positive_or_default(
    this->get_parameter("tracker.max_lost_time").as_double(), 0.3,
    "tracker.max_lost_time", get_logger());
  csu_tracker_params_.min_detect_count = std::max(
    1, static_cast<int>(this->get_parameter("tracker.min_detect_count").as_int()));
  csu_tracker_params_.min_tracking_count_for_fire = std::max(
    1, static_cast<int>(
      this->get_parameter("tracker.min_tracking_count_for_fire").as_int()));
  csu_tracker_.configure(csu_tracker_params_);

  const auto pitch_lut = pair_lut_from_flat(
    this->get_parameter("manual_compensator.pitch_lut").as_double_array(),
    {{2.0, -0.222}, {3.0, -0.222}, {4.0, -0.222}, {5.0, -0.222}},
    "manual_compensator.pitch_lut", get_logger());
  const auto yaw_lut = pair_lut_from_flat(
    this->get_parameter("manual_compensator.yaw_lut").as_double_array(),
    {{2.0, -0.002}, {5.0, -0.002}},
    "manual_compensator.yaw_lut", get_logger());
  manual_compensator_.configure(
    this->get_parameter("manual_compensator.enabled").as_bool(),
    pitch_lut,
    yaw_lut,
    this->get_parameter("manual_compensator.clamp_min_distance").as_double(),
    this->get_parameter("manual_compensator.clamp_max_distance").as_double());

  const std::string target_status_topic =
    this->get_parameter("target_status_topic").as_string();
  const std::string imu_topic = this->get_parameter("imu_topic").as_string();
  const std::string gimbal_status_topic =
    this->get_parameter("gimbal_status_topic").as_string();

  // ===================== UKF 参数 =====================

  const UKFParams ukf_defaults;
  this->declare_parameter<double>("ukf.alpha", ukf_defaults.alpha);
  this->declare_parameter<double>("ukf.beta", ukf_defaults.beta);
  this->declare_parameter<double>("ukf.kappa", ukf_defaults.kappa);
  this->declare_parameter<double>("ukf.q_pos", ukf_defaults.q_pos);
  this->declare_parameter<double>("ukf.q_vel", ukf_defaults.q_vel);
  this->declare_parameter<double>("ukf.q_r", ukf_defaults.q_r);
  this->declare_parameter<double>("ukf.q_phi", ukf_defaults.q_phi);
  this->declare_parameter<double>("ukf.q_omega", ukf_defaults.q_omega);
  this->declare_parameter<double>("ukf.r_pos", ukf_defaults.r_pos);
  this->declare_parameter<double>("ukf.markov_00", ukf_defaults.markov_transition[0][0]);
  this->declare_parameter<double>("ukf.markov_11", ukf_defaults.markov_transition[1][1]);
  this->declare_parameter<double>("ukf.init_prob_cv", ukf_defaults.initial_model_prob[0]);

  ukf_params_.alpha = positive_or_default(
    this->get_parameter("ukf.alpha").as_double(), ukf_defaults.alpha,
    "ukf.alpha", get_logger());
  ukf_params_.beta = this->get_parameter("ukf.beta").as_double();
  ukf_params_.kappa = this->get_parameter("ukf.kappa").as_double();
  ukf_params_.q_pos = positive_or_default(
    this->get_parameter("ukf.q_pos").as_double(), ukf_defaults.q_pos,
    "ukf.q_pos", get_logger());
  ukf_params_.q_vel = positive_or_default(
    this->get_parameter("ukf.q_vel").as_double(), ukf_defaults.q_vel,
    "ukf.q_vel", get_logger());
  ukf_params_.q_r = positive_or_default(
    this->get_parameter("ukf.q_r").as_double(), ukf_defaults.q_r,
    "ukf.q_r", get_logger());
  ukf_params_.q_phi = positive_or_default(
    this->get_parameter("ukf.q_phi").as_double(), ukf_defaults.q_phi,
    "ukf.q_phi", get_logger());
  ukf_params_.q_omega = positive_or_default(
    this->get_parameter("ukf.q_omega").as_double(), ukf_defaults.q_omega,
    "ukf.q_omega", get_logger());
  ukf_params_.r_pos = positive_or_default(
    this->get_parameter("ukf.r_pos").as_double(), ukf_defaults.r_pos,
    "ukf.r_pos", get_logger());

  const double markov_00 = clamp01(
    this->get_parameter("ukf.markov_00").as_double(), "ukf.markov_00", get_logger());
  const double markov_11 = clamp01(
    this->get_parameter("ukf.markov_11").as_double(), "ukf.markov_11", get_logger());
  const double init_prob_cv = clamp01(
    this->get_parameter("ukf.init_prob_cv").as_double(), "ukf.init_prob_cv", get_logger());

  ukf_params_.markov_transition[0][0] = markov_00;
  ukf_params_.markov_transition[0][1] = 1.0 - markov_00;
  ukf_params_.markov_transition[1][0] = 1.0 - markov_11;
  ukf_params_.markov_transition[1][1] = markov_11;
  ukf_params_.initial_model_prob[0] = init_prob_cv;
  ukf_params_.initial_model_prob[1] = 1.0 - init_prob_cv;

  tracker_ = ImmUkfTracker(ukf_params_);

  RCLCPP_INFO(
    get_logger(),
    "[UKF PARAMS LOADED]\n"
    "alpha=%.6f\n"
    "beta=%.6f\n"
    "kappa=%.6f\n"
    "q_pos=%.6f\n"
    "q_vel=%.6f\n"
    "q_r=%.6f\n"
    "q_phi=%.6f\n"
    "q_omega=%.6f\n"
    "r_pos=%.6f\n"
    "markov=[[%.6f, %.6f], [%.6f, %.6f]]\n"
    "init_prob=[%.6f, %.6f]",
    ukf_params_.alpha,
    ukf_params_.beta,
    ukf_params_.kappa,
    ukf_params_.q_pos,
    ukf_params_.q_vel,
    ukf_params_.q_r,
    ukf_params_.q_phi,
    ukf_params_.q_omega,
    ukf_params_.r_pos,
    ukf_params_.markov_transition[0][0],
    ukf_params_.markov_transition[0][1],
    ukf_params_.markov_transition[1][0],
    ukf_params_.markov_transition[1][1],
    ukf_params_.initial_model_prob[0],
    ukf_params_.initial_model_prob[1]);

  // ===================== Aimer 参数 =====================

  AimerParams aimer_params;

  this->declare_parameter<double>("gravity", 9.81);
  this->declare_parameter<double>("bullet_speed_default", 15.0);
  this->declare_parameter<double>("fire_delay", 0.10);
  this->declare_parameter<double>("yaw_offset", 0.0);
  this->declare_parameter<bool>("pitch_invert", false);

  this->declare_parameter<double>("fire_tolerance", 0.03);
  this->declare_parameter<bool>("use_dynamic_fire_window", true);
  this->declare_parameter<double>("shooting_range_width", 0.135);
  this->declare_parameter<double>("shooting_range_height", 0.055);
  this->declare_parameter<double>("min_fire_window_deg", 1.0);
  this->declare_parameter<double>("max_fire_window_deg", 3.0);
  this->declare_parameter<int>("fire_min_frames", 3);
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
  aimer_params.use_dynamic_fire_window =
    this->get_parameter("use_dynamic_fire_window").as_bool();
  aimer_params.shooting_range_width = positive_or_default(
    this->get_parameter("shooting_range_width").as_double(), 0.135,
    "shooting_range_width", get_logger());
  aimer_params.shooting_range_height = positive_or_default(
    this->get_parameter("shooting_range_height").as_double(), 0.055,
    "shooting_range_height", get_logger());
  aimer_params.min_fire_window_deg =
    this->get_parameter("min_fire_window_deg").as_double();
  aimer_params.max_fire_window_deg =
    this->get_parameter("max_fire_window_deg").as_double();
  aimer_params.fire_min_frames  = this->get_parameter("fire_min_frames").as_int();
  aimer_params.fire_max_distance = this->get_parameter("fire_max_distance").as_double();
  aimer_params.fire_min_frames = std::max(
    aimer_params.fire_min_frames,
    static_cast<int>(this->get_parameter("tracker.min_tracking_count_for_fire").as_int()));
  aimer_params.fire_max_distance = this->get_parameter("tracker.fire_max_distance").as_double();

  aimer_params.pitch_offset_k0  = this->get_parameter("pitch_offset_k0").as_double();
  aimer_params.pitch_offset_k1  = this->get_parameter("pitch_offset_k1").as_double();
  aimer_params.pitch_offset_k2  = this->get_parameter("pitch_offset_k2").as_double();

  aimer_ = Aimer(aimer_params);

  // ===================== 订阅 =====================

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(msg); });

  // 下位机状态反馈 (用于火控对齐窗口)
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

  gimbal_cmd_raw_pub_ = this->create_publisher<GimbalCmd>(
    "/autoaim/gimbal_cmd_raw", rclcpp::SensorDataQoS());

  fire_debug_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/autoaim/debug_fire_gate", rclcpp::SensorDataQoS());

  if (enable_target_status_) {
    target_status_pub_ = this->create_publisher<AutoaimTargetStatus>(
      target_status_topic, rclcpp::SensorDataQoS());
  }

  RCLCPP_INFO(get_logger(),
    "rm_autoaim initialized. backend=%s tracking_frame=%s bullet=%.1fm/s delay=%.3fs "
    "K[fx=%.1f fy=%.1f cx=%.1f cy=%.1f]",
    tracker_backend_.c_str(),
    use_imu_world_transform_ ? "imu_world" : "current_gimbal",
    aimer_params.bullet_speed,
    aimer_params.fire_delay,
    camera_matrix_.at<double>(0, 0),
    camera_matrix_.at<double>(1, 1),
    camera_matrix_.at<double>(0, 2),
    camera_matrix_.at<double>(1, 2));
}

const char * AutoaimNode::track_state_name(AutoaimTrackState state)
{
  switch (state) {
    case AutoaimTrackState::LOST:
      return "LOST";
    case AutoaimTrackState::TRACKING:
      return "TRACKING";
    case AutoaimTrackState::TEMP_LOST:
      return "TEMP_LOST";
  }
  return "UNKNOWN";
}

std::string AutoaimNode::armor_type_from_detection(const ArmorDetection & armor)
{
  const std::string label = armor.label;
  if (label.find("large") != std::string::npos || label.find("big") != std::string::npos) {
    return "large";
  }

  static const std::unordered_set<int> kBigArmorClassIds = {
    21, 22, 23, 24, 29, 30, 31, 32, 33, 34, 35, 36, 37,
  };
  return kBigArmorClassIds.count(armor.class_id) > 0 ? "large" : "small";
}

void AutoaimNode::publish_fire_debug(
  const std::string & backend,
  const std::string & aim_source,
  int target_id,
  const std::string & armor_type,
  const Eigen::Vector3d & raw_pnp_pos,
  const Eigen::Vector3d & tracker_pos,
  const Eigen::Vector3d & selected_pos,
  int fire_control,
  double target_distance,
  double yaw_cmd,
  double pitch_cmd,
  double cmd_yaw_err_deg,
  double cmd_pitch_err_deg,
  double manual_yaw_offset,
  double manual_pitch_offset,
  double yaw_window,
  double pitch_window,
  const std::string & tracker_state,
  const std::string & reason,
  double age_sec)
{
  if (!fire_debug_pub_) return;

  std_msgs::msg::String debug_msg;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "backend=" << backend
     << " source=" << aim_source
     << " target_id=" << target_id
     << " armor_type=" << armor_type
     << " raw_pnp_pos=" << vec_to_debug(raw_pnp_pos)
     << " tracker_pos=" << vec_to_debug(tracker_pos)
     << " selected_pos=" << vec_to_debug(selected_pos)
     << " distance=" << target_distance
     << " yaw_cmd=" << yaw_cmd
     << " pitch_cmd=" << pitch_cmd
     << " yaw_err_deg=" << cmd_yaw_err_deg
     << " pitch_err_deg=" << cmd_pitch_err_deg
     << " manual_yaw_offset=" << manual_yaw_offset
     << " manual_pitch_offset=" << manual_pitch_offset
     << " fire_window=[" << yaw_window * 180.0 / M_PI
     << "," << pitch_window * 180.0 / M_PI << "]"
     << " tracker_state=" << tracker_state
     << " fire_control=" << fire_control
     << " state=" << track_state_name(track_state_)
     << " source=" << aim_source
     << " frames=" << tracker_frame_count_;
  if (!reason.empty()) {
    ss << " reason=" << reason;
  }
  if (age_sec >= 0.0) {
    ss << " age=" << age_sec;
  }
  debug_msg.data = ss.str();
  fire_debug_pub_->publish(debug_msg);
}

void AutoaimNode::publish_target_status(
  bool has_target,
  bool tracking,
  bool temp_lost,
  bool fire_ready,
  double target_distance,
  double yaw_error_deg,
  double pitch_error_deg,
  uint8_t aim_source,
  const std::string & reason)
{
  if (!target_status_pub_) return;

  AutoaimTargetStatus status;
  status.header.stamp = this->now();
  status.header.frame_id = world_frame_id_;
  status.has_target = has_target;
  status.tracking = tracking;
  status.temp_lost = temp_lost;
  status.fire_ready = fire_ready;
  status.target_distance = static_cast<float>(std::max(0.0, target_distance));
  status.yaw_error_deg = static_cast<float>(yaw_error_deg);
  status.pitch_error_deg = static_cast<float>(pitch_error_deg);
  status.aim_source = aim_source;
  status.reason = reason;
  target_status_pub_->publish(status);
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
// 云台状态回调：更新当前真实角度 (用于火控对齐窗口)
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
  rclcpp::Time t_img(msg->header.stamp);
  const double current_time = t_img.seconds();

  this->get_parameter("tracker_backend", tracker_backend_);
  this->get_parameter("enable_raw_pnp_fallback", enable_raw_pnp_fallback_);
  this->get_parameter("raw_pnp_fallback_on_tracker_lost", raw_pnp_fallback_on_tracker_lost_);

  auto publish_cmd = [&](const GimbalCmd & cmd) {
    gimbal_cmd_pub_->publish(cmd);
    if (gimbal_cmd_raw_pub_ &&
        std::string(gimbal_cmd_raw_pub_->get_topic_name()) !=
        std::string(gimbal_cmd_pub_->get_topic_name()))
    {
      gimbal_cmd_raw_pub_->publish(cmd);
    }
  };

  auto publish_no_target_cmd = [&](const std::string & reason) {
    const double age_sec =
      has_last_detection_ ? (t_img - last_detection_time_).seconds() : -1.0;

    const bool within_temp_hold =
      age_sec >= 0.0 && age_sec < temp_lost_timeout_sec_;
    const bool within_lost_timeout =
      age_sec >= 0.0 && age_sec < lost_timeout_sec_;
    const bool can_enter_temp_lost =
      has_last_detection_ &&
      (track_state_ == AutoaimTrackState::TRACKING ||
       track_state_ == AutoaimTrackState::TEMP_LOST) &&
      within_lost_timeout;

    if (can_enter_temp_lost) {
      track_state_ = AutoaimTrackState::TEMP_LOST;
      const AimTarget held_target =
        csu_tracker_.markLost(current_time, aimer_.getParams().fire_delay);

      GimbalCmd cmd;
      if (hold_last_cmd_in_temp_lost_ && has_last_good_cmd_) {
        cmd = last_good_cmd_;
        if (!within_temp_hold) {
          cmd.yaw_vel = 0.0;
          cmd.pitch_vel = 0.0;
        }
      } else {
        cmd.target_yaw = 0.0;
        cmd.target_pitch = 0.0;
        cmd.yaw_vel = 0.0;
        cmd.pitch_vel = 0.0;
      }
      cmd.fire_control = 0;
      cmd.mode = 1;
      cmd.state_switch = 2;
      publish_cmd(cmd);

      publish_fire_debug(
        tracker_backend_,
        "none",
        csu_tracker_.targetId(),
        csu_tracker_.targetArmorType(),
        Eigen::Vector3d::Zero(),
        held_target.valid ? held_target.position_gimbal : Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        0,
        0.0,
        cmd.target_yaw,
        cmd.target_pitch,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        tracker_state_name(csu_tracker_.state()),
        within_temp_hold ? "temp_lost" : (reason.empty() ? "lost_grace" : reason),
        age_sec);
      publish_target_status(
        false, false, true, false, 0.0, 0.0, 0.0, 0,
        within_temp_hold ? "temp_lost" : (reason.empty() ? "lost_grace" : reason));
      return;
    }

    track_state_ = AutoaimTrackState::LOST;
    tracker_frame_count_ = 0;
    tracker_ = ImmUkfTracker(ukf_params_);
    csu_tracker_.reset();

    GimbalCmd cmd;
    cmd.target_yaw   = 0.0;
    cmd.target_pitch = 0.0;
    cmd.yaw_vel      = 0.0;
    cmd.pitch_vel    = 0.0;
    cmd.state_switch = 1;   // Move — 回到巡逻/默认姿态
    cmd.fire_control = 0;
    cmd.mode         = 0;   // idle — 无目标
    publish_cmd(cmd);

    publish_fire_debug(
      tracker_backend_,
      "none",
      -1,
      "none",
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      0,
      0.0,
      cmd.target_yaw,
      cmd.target_pitch,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      tracker_state_name(csu_tracker_.state()),
      reason.empty() ? "no_detection" : reason,
      age_sec);
    publish_target_status(
      false, false, false, false, 0.0, 0.0, 0.0, 0,
      reason.empty() ? "no_detection" : reason);
  };

  if (msg->detections.empty()) {
    publish_no_target_cmd("no_detection");
    return;
  }

  // ======== Step 1: 时间对齐 — 查找包围 T_img 的两个 IMU 样本 ========
  cv::Quatd q_gimbal_to_world(1.0, 0.0, 0.0, 0.0);
  ImuStamp imu_before, imu_after;
  if (use_imu_world_transform_ && !find_imu_bracket(t_img, imu_before, imu_after)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No IMU bracket for stamp %.3f. Dropping frame.", t_img.seconds());
    publish_fire_debug(
      tracker_backend_,
      "none",
      -1,
      "none",
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      "LOST",
      "no_imu",
      has_last_detection_ ? (t_img - last_detection_time_).seconds() : -1.0);
    publish_target_status(false, false, false, false, 0.0, 0.0, 0.0, 0, "no_imu");
    return;
  }

  // ======== Step 2: SLERP 四元数插值 ========
  if (use_imu_world_transform_) {
    double dt_total  = (imu_after.stamp - imu_before.stamp).seconds();
    double dt_interp = (t_img - imu_before.stamp).seconds();
    double alpha = (dt_total > 1e-9) ? (dt_interp / dt_total) : 0.0;
    alpha = std::clamp(alpha, 0.0, 1.0);
    q_gimbal_to_world = slerp(imu_before.orientation, imu_after.orientation, alpha);
  }

  // ======== Step 3: PnP + 坐标变换 → 选最优装甲板 ========
  std::vector<ArmorObservation> observations;

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

    cv::Point3d p = transform_to_world(tvec, q_gimbal_to_world);

    cv::Point2f center_px(0.0f, 0.0f);
    for (const auto & apex : armor.apexes) {
      center_px.x += static_cast<float>(apex.x);
      center_px.y += static_cast<float>(apex.y);
    }
    center_px.x *= 0.25f;
    center_px.y *= 0.25f;

    ArmorObservation obs;
    obs.position_gimbal = Eigen::Vector3d(p.x, p.y, p.z);
    obs.distance = obs.position_gimbal.norm();
    obs.confidence = armor.confidence;
    obs.class_id = armor.class_id;
    obs.label = armor.label;
    obs.armor_type = armor_type_from_detection(armor);
    obs.center_axis_error_px = cv::norm(center_px - optical_axis_px);
    observations.push_back(obs);
  }

  if (observations.empty()) {
    publish_no_target_cmd("pnp_failed");
    return;
  }

  const auto selected_opt = csu_tracker_.select(observations);
  if (!selected_opt) {
    publish_no_target_cmd("select_failed");
    return;
  }
  const auto best = *selected_opt;

  // 发布调试点云
  for (const auto & res : observations) {
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp    = msg->header.stamp;
    pt.header.frame_id = world_frame_id_;
    pt.point.x = res.position_gimbal.x();
    pt.point.y = res.position_gimbal.y();
    pt.point.z = res.position_gimbal.z();
    world_points_pub_->publish(pt);
  }

  AimTarget raw_target;
  raw_target.position_gimbal = best.position_gimbal;
  raw_target.velocity_gimbal = Eigen::Vector3d::Zero();
  raw_target.distance = best.distance;
  raw_target.confidence = best.confidence;
  raw_target.source = "raw_pnp";
  raw_target.valid = raw_target.position_gimbal.allFinite() && raw_target.distance > 0.05;

  // ======== Step 4: Tracker backend ========
  AimTarget tracker_target;
  if (tracker_backend_ == "legacy_imm") {
    Eigen::Vector3d obs = best.position_gimbal;
    if (!tracker_.isInitialized()) {
      tracker_.init(obs, current_time);
      tracker_frame_count_ = 1;
    } else {
      tracker_.update(obs, current_time);
      tracker_frame_count_++;
    }
    auto predicted_state = tracker_.predict(current_time + aimer_.getParams().fire_delay);
    tracker_target.position_gimbal = predicted_state.head<3>();
    tracker_target.velocity_gimbal = predicted_state.segment<3>(3);
    tracker_target.distance = tracker_target.position_gimbal.norm();
    tracker_target.confidence = best.confidence;
    tracker_target.source = "legacy_imm";
    tracker_target.valid = tracker_target.position_gimbal.allFinite() && tracker_target.distance > 0.05;
    track_state_ = AutoaimTrackState::TRACKING;
  } else if (tracker_backend_ == "raw_pnp") {
    tracker_target = raw_target;
    tracker_frame_count_ = 1;
    track_state_ = AutoaimTrackState::TRACKING;
  } else {
    tracker_backend_ = "csu_tracker";
    tracker_target = csu_tracker_.update(best, current_time, aimer_.getParams().fire_delay);
    tracker_frame_count_ = csu_tracker_.trackingFrames();
    track_state_ =
      csu_tracker_.state() == CsuTrackerState::TEMP_LOST ? AutoaimTrackState::TEMP_LOST :
      csu_tracker_.state() == CsuTrackerState::LOST ? AutoaimTrackState::LOST :
      AutoaimTrackState::TRACKING;
  }

  last_detection_time_ = t_img;
  has_last_detection_ = true;

  double cur_yaw, cur_pitch, cur_bullet;
  {
    std::lock_guard<std::mutex> lock(gimbal_mutex_);
    cur_yaw    = current_yaw_deg_;
    cur_pitch  = current_pitch_deg_;
    cur_bullet = current_bullet_speed_;
  }

  AimTarget selected_target = tracker_target;
  const bool tracker_bad =
    !tracker_target.valid ||
    !tracker_target.position_gimbal.allFinite() ||
    tracker_target.distance > 20.0 ||
    (tracker_backend_ == "csu_tracker" &&
     (csu_tracker_.state() == CsuTrackerState::LOST ||
      (raw_pnp_fallback_on_tracker_lost_ && csu_tracker_.state() == CsuTrackerState::TEMP_LOST) ||
      (tracker_target.position_gimbal - raw_target.position_gimbal).norm() > fallback_jump_guard_m_));

  std::string reason;
  if ((tracker_backend_ == "raw_pnp") ||
      (enable_raw_pnp_fallback_ && tracker_bad && raw_target.valid))
  {
    selected_target = raw_target;
    selected_target.source =
      tracker_backend_ == "raw_pnp" ? "raw_pnp" : "raw_pnp_fallback";
    if (tracker_backend_ != "raw_pnp") {
      reason = "tracker_bad_raw_pnp_fallback";
    }
  }

  const double manual_pitch_offset = manual_compensator_.pitchOffset(selected_target.distance);
  const double manual_yaw_offset = manual_compensator_.yawOffset(selected_target.distance);

  auto aim = aimer_.solve(
    selected_target,
    cur_yaw,
    cur_pitch,
    tracker_frame_count_,
    cur_bullet,
    !use_imu_world_transform_,
    manual_yaw_offset,
    manual_pitch_offset);

  const bool allow_fire_on_raw =
    this->get_parameter("tracker.allow_fire_on_raw_pnp_fallback").as_bool();
  const bool allow_fire_on_csu =
    this->get_parameter("tracker.allow_fire_on_csu_tracker").as_bool();
  const bool backend_allows_fire =
    (selected_target.source == "csu_tracker" && allow_fire_on_csu) ||
    ((selected_target.source == "raw_pnp" || selected_target.source == "raw_pnp_fallback") &&
     allow_fire_on_raw);
  const bool tracker_tracking =
    tracker_backend_ == "raw_pnp" ? true :
    tracker_backend_ == "legacy_imm" ? tracker_.isInitialized() :
    csu_tracker_.state() == CsuTrackerState::TRACKING;

  // ======== Step 7: 构建 GimbalCmd ========
  // 有目标 → mode=1 (auto_aim)
  // fire_control 由 Aimer 的对齐窗口 + Tracker 收敛/距离判定给出。
  GimbalCmd cmd;
  cmd.target_yaw   = aim.target_yaw;
  cmd.target_pitch = aim.target_pitch;
  cmd.yaw_vel      = aim.yaw_vel;
  cmd.pitch_vel    = aim.pitch_vel;
  cmd.fire_control =
    (selected_target.valid &&
     tracker_tracking &&
     backend_allows_fire &&
     aim.can_fire) ? 1 : 0;
  cmd.state_switch = 2;  // Attack — 有目标即 Attack (下位机小陀螺+跟踪)
  cmd.mode         = 1;  // auto_aim — 有目标

  publish_cmd(cmd);
  last_good_cmd_ = cmd;
  has_last_good_cmd_ = true;

  double cur_yaw_rad = cur_yaw * M_PI / 180.0;
  double cur_pitch_rad = cur_pitch * M_PI / 180.0;
  double cmd_yaw_err_deg = std::atan2(
    std::sin(aim.target_yaw - cur_yaw_rad),
    std::cos(aim.target_yaw - cur_yaw_rad)) * 180.0 / M_PI;
  double cmd_pitch_err_deg = (aim.target_pitch - cur_pitch_rad) * 180.0 / M_PI;

  if (reason.empty()) {
    if (!selected_target.valid) {
      reason = "invalid_target";
    } else if (!tracker_tracking) {
      reason = "tracker_not_tracking";
    } else if (!backend_allows_fire) {
      reason = "backend_fire_disabled";
    } else if (!aim.alignment_ready) {
      reason = "not_aligned";
    } else if (!aim.tracker_ready) {
      reason = "tracker_frames_or_distance";
    } else {
      reason = "ok";
    }
  }

  publish_fire_debug(
    tracker_backend_,
    selected_target.source,
    best.class_id,
    best.armor_type,
    raw_target.position_gimbal,
    tracker_target.valid ? tracker_target.position_gimbal : Eigen::Vector3d::Zero(),
    selected_target.valid ? selected_target.position_gimbal : Eigen::Vector3d::Zero(),
    cmd.fire_control,
    aim.target_distance,
    aim.target_yaw,
    aim.target_pitch,
    cmd_yaw_err_deg,
    cmd_pitch_err_deg,
    manual_yaw_offset,
    manual_pitch_offset,
    aim.yaw_window,
    aim.pitch_window,
    tracker_backend_ == "legacy_imm" ? "LEGACY_IMM" : tracker_state_name(csu_tracker_.state()),
    reason,
    0.0);
  const std::uint8_t aim_source_id =
    (selected_target.source == "raw_pnp" || selected_target.source == "raw_pnp_fallback") ? 2 : 1;
  publish_target_status(
    true,
    tracker_tracking,
    csu_tracker_.state() == CsuTrackerState::TEMP_LOST,
    cmd.fire_control == 1,
    aim.target_distance,
    cmd_yaw_err_deg,
    cmd_pitch_err_deg,
    aim_source_id,
    selected_target.source + ":" + reason);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Aim: backend=%s src=%s cmd=(%.2f°, %.2f°) err=(%.2f°, %.2f°) fire=%d "
    "state=%s frames=%d manual=(%.3f, %.3f) win=(%.2f°, %.2f°) reason=%s",
    tracker_backend_.c_str(),
    selected_target.source.c_str(),
    aim.target_yaw * 180.0 / M_PI,
    aim.target_pitch * 180.0 / M_PI,
    cmd_yaw_err_deg,
    cmd_pitch_err_deg,
    cmd.fire_control,
    tracker_backend_ == "legacy_imm" ? "LEGACY_IMM" : tracker_state_name(csu_tracker_.state()),
    tracker_frame_count_,
    manual_yaw_offset,
    manual_pitch_offset,
    aim.yaw_window * 180.0 / M_PI,
    aim.pitch_window * 180.0 / M_PI,
    reason.c_str());
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
  bool is_large = armor_type_from_detection(armor) == "large";

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

  auto reprojection_error = [&](const cv::Vec3d & candidate_rvec, const cv::Vec3d & candidate_tvec) {
    std::vector<cv::Point2d> projected;
    cv::projectPoints(
      object_points, candidate_rvec, candidate_tvec,
      camera_matrix_, dist_coeffs_, projected);
    double err = 0.0;
    for (std::size_t i = 0; i < projected.size(); ++i) {
      err += cv::norm(projected[i] - image_points[i]);
    }
    return err / std::max<std::size_t>(1, projected.size());
  };

  int best_idx = 0;
  double best_err = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < tvecs.size(); ++i) {
    if (!std::isfinite(tvecs[i][2]) || tvecs[i][2] <= 0.0) continue;
    const double err = reprojection_error(rvecs[i], tvecs[i]);
    if (err < best_err) {
      best_err = err;
      best_idx = static_cast<int>(i);
    }
  }

  rvec = rvecs[best_idx];
  tvec = tvecs[best_idx];

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
