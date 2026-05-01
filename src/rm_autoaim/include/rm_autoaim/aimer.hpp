// =============================================================================
// aimer.hpp — 物理前馈逆运动学 + 军事级三段火控
// =============================================================================
// 职责: 接收 Tracker 预测状态 → 解算瞄准角度 → 判定开火窗口
// 设计: 纯数学类，零 ROS 依赖，可离线单元测试
//
// 状态向量约定 (与 ImmUkfTracker 一致):
//   x = [xc, yc, zc, vx, vy, vz, r, phi, omega]^T
//
// 角度契约:
//   - inverseKinematics() 输出的是 hit_pos 所在参考系下的视线角
//     * 当前云台系输入 -> 相对当前枪口前向的相对角
//     * 世界/惯性系输入 -> 世界/惯性系下的绝对角
//   - solve() 负责把该视线角转换为最终 /gimbal_cmd 绝对角命令
//   - 最终 target_yaw / target_pitch 必须与 GimbalStatus.gimbal_yaw /
//     gimbal_pitch 处于同一参考系、同一正方向、同一零点语义
//
// 下位机协议正方向:
//   - yaw   = CCW+ (向左为正), 与 ROS REP-103 FLU 一致
//   - pitch = 默认抬头为负(向下为正), pitch_invert=true 时在发布前取反
// =============================================================================
#ifndef RM_AUTOAIM__AIMER_HPP_
#define RM_AUTOAIM__AIMER_HPP_

#include <array>
#include <cmath>
#include <algorithm>
#include <utility>

#include <Eigen/Dense>

namespace rm_autoaim
{

// =============================================================================
// Aimer 参数配置 — 所有阈值均可从 ROS 参数服务器加载
// =============================================================================
struct AimerParams
{
  // ---- 物理常量 ----
  double gravity         = 9.81;    // 重力加速度 (m/s²)
  double bullet_speed    = 15.0;    // 默认弹丸速度 (m/s)
  double fire_delay      = 0.10;    // 系统延迟 (s), 含图像+传输+云台响应

  // ---- 坐标系 ----
  double yaw_offset      = 0.0;     // yaw 常量偏置 (rad), 正=向左补, 负=向右补
  bool   pitch_invert    = false;   // 若协议 pitch 正方向与数学解算相反，则在生成命令前取反

  // ---- 火控 Gate 1: 炮管对齐容差 ----
  double fire_tolerance  = 0.03;    // yaw/pitch 对齐容差 (rad, ≈1.7°)
  bool   use_dynamic_fire_window = true;
  double shooting_range_width = 0.135;
  double shooting_range_height = 0.055;
  double min_fire_window_deg = 1.0;
  double max_fire_window_deg = 3.0;

  // ---- 火控 Gate 2: 装甲板相位窗口 (Spin Killer) ----
  double armor_facing_tol = 30.0;   // 装甲板法线对齐容差 (deg)
  double ctrv_threshold   = 0.5;    // CTRV 模型概率高于此值才启用 Gate 2

  // ---- 火控 Gate 3: 收敛/距离保护 ----
  int    fire_min_frames  = 3;      // Tracker 最少收敛帧数
  double fire_max_distance = 8.0;   // 最大开火距离 (m)

  // ---- 弹道补偿 LUT 系数 (预留) ----
  // 当前使用解析模型，后续可替换为查表
  double pitch_offset_k0 = 0.0;     // 常数项 (rad)
  double pitch_offset_k1 = 0.0;     // 一次项系数
  double pitch_offset_k2 = 0.0;     // 二次项系数
};

// =============================================================================
// Aimer — 解算与火控类
// =============================================================================
class Aimer
{
public:
  using StateVec = Eigen::Matrix<double, 9, 1>;

  explicit Aimer(const AimerParams & params = AimerParams())
  : params_(params) {}

  /// 获取参数 (只读)
  const AimerParams & getParams() const { return params_; }

  // ===========================================================================
  // 核心接口: solve() — 一次调用完成 瞄准解算 + 火控判定
  // ===========================================================================
  struct AimResult
  {
    double target_yaw;      // 最终绝对 yaw 命令 (rad), 语义与 GimbalStatus.gimbal_yaw 完全一致
    double target_pitch;    // 最终绝对 pitch 命令 (rad), 语义与 GimbalStatus.gimbal_pitch 完全一致
    double yaw_vel;         // yaw 前馈角速度 (rad/s)
    double pitch_vel;       // pitch 前馈角速度 (rad/s)
    bool   gate1_alignment; // Gate 1: 炮管对齐
    bool   gate2_facing;    // Gate 2: 装甲板相位窗口
    bool   gate3_ready;     // Gate 3: 收敛/距离保护
    bool   can_fire;        // 火控判定: true = 允许开火
    double t_flight;        // 子弹飞行时间 (s)
    double hit_phi;         // 预测的击中时装甲板相位角 (rad)
    double target_distance;  // 预测状态 3D 距离 (m)
    double yaw_window;       // Gate 1 yaw 实际窗口 (rad)
    double pitch_window;     // Gate 1 pitch 实际窗口 (rad)
  };

  /// 完整解算
  /// @param predicted_state   Tracker 预测的 9D 状态
  /// @param model_probs       IMM 模型概率 [P_CV, P_CTRV]
  /// @param current_yaw_deg   下位机反馈的当前真实 yaw (deg)
  /// @param current_pitch_deg 下位机反馈的当前真实 pitch (deg)
  /// @param tracker_frames    Tracker 已更新帧数
  /// @param bullet_speed      实时弹丸速度 (m/s), 0=使用默认值
  /// @param input_in_current_gimbal_frame
  ///                         true: predicted_state 在当前云台系, inverseKinematics 输出相对角,
  ///                               solve() 会与当前 gimbal_status 组合成绝对命令
  ///                         false: predicted_state 已在世界/惯性系, inverseKinematics 直接输出绝对角
  AimResult solve(
    const StateVec & predicted_state,
    const std::array<double, 2> & model_probs,
    double current_yaw_deg,
    double current_pitch_deg,
    int tracker_frames,
    double bullet_speed = 0.0,
    bool input_in_current_gimbal_frame = false) const;

  // ===========================================================================
  // 物理前馈逆运动学
  // ===========================================================================

  /// 预测目标在子弹飞行后的位置 (前馈补偿)
  /// @param state        预测状态 (已包含 fire_delay 的前推)
  /// @param bullet_speed 弹丸速度
  /// @param[out] t_flight 飞行时间
  /// @return 预测的击中位置 [x, y, z]
  Eigen::Vector3d predictHitPosition(
    const StateVec & state,
    double bullet_speed,
    double & t_flight) const;

  /// 逆运动学: 3D 位置 → yaw + pitch
  /// @param hit_pos  预测击中位置
  /// @param[out] yaw   hit_pos 所在参考系下的水平方位角 (rad, CCW+ 向左为正)
  /// @param[out] pitch hit_pos 所在参考系下的俯仰角 (rad, 数学约定: 正=向下, 负=向上)
  void inverseKinematics(
    const Eigen::Vector3d & hit_pos,
    double & yaw, double & pitch) const;

  /// 重力下坠补偿 (LUT 接口)
  /// @param distance  水平距离 (m)
  /// @return pitch 抬高量 (rad, 正=抬头)
  double getPitchOffset(double distance) const;

  // ===========================================================================
  // 三段火控
  // ===========================================================================

  /// Gate 1: 炮管对齐容差
  /// 比较解算角度与下位机当前真实角度
  bool gateBarrelAlignment(
    double target_yaw, double target_pitch,
    double current_yaw_deg, double current_pitch_deg,
    double distance) const;

  /// 获取当前距离对应的 yaw/pitch 射击窗口 (rad)
  std::pair<double, double> getFireWindows(double distance) const;

  /// Gate 2: 装甲板相位窗口 (Spin Killer)
  /// 预测子弹击中时装甲板法线与射线夹角
  bool gateArmorFacing(
    const StateVec & state,
    double target_yaw,
    double t_flight,
    double p_ctrv) const;

  /// Gate 3: 收敛/距离保护
  bool gateConvergenceAndRange(
    const StateVec & state,
    int tracker_frames) const;

private:
  AimerParams params_;
};

// =============================================================================
// ================================ 实现 =======================================
// =============================================================================

inline Aimer::AimResult Aimer::solve(
  const StateVec & predicted_state,
  const std::array<double, 2> & model_probs,
  double current_yaw_deg,
  double current_pitch_deg,
  int tracker_frames,
  double bullet_speed,
  bool input_in_current_gimbal_frame) const
{
  AimResult result;

  double v_bullet = (bullet_speed > 2.0) ? bullet_speed : params_.bullet_speed;
  double current_yaw = current_yaw_deg * M_PI / 180.0;
  double current_pitch = current_pitch_deg * M_PI / 180.0;

  // ---- Step 1: 预测子弹飞行后的击中位置 ----
  double t_flight = 0.0;
  Eigen::Vector3d hit_pos = predictHitPosition(predicted_state, v_bullet, t_flight);
  result.t_flight = t_flight;

  // ---- Step 2: 逆运动学 → tracker 输入参考系下的视线角 ----
  double frame_yaw, frame_pitch;
  inverseKinematics(hit_pos, frame_yaw, frame_pitch);

  // ---- Step 3: 重力下坠补偿 ----
  // getPitchOffset > 0 (需要抬枪补偿重力下坠)
  // 在当前约定中 (pitch 负=向上), 抬枪 = 使 pitch 更负 → 减去补偿量
  double d_h = std::sqrt(hit_pos(0) * hit_pos(0) + hit_pos(1) * hit_pos(1));
  frame_pitch -= getPitchOffset(d_h);

  // ---- Step 4: 变成最终下位机绝对角命令 ----
  double command_yaw = frame_yaw + params_.yaw_offset;
  double command_pitch = params_.pitch_invert ? -frame_pitch : frame_pitch;
  if (input_in_current_gimbal_frame) {
    result.target_yaw = std::atan2(
      std::sin(current_yaw + command_yaw),
      std::cos(current_yaw + command_yaw));
    result.target_pitch = current_pitch + command_pitch;
  } else {
    result.target_yaw = std::atan2(std::sin(command_yaw), std::cos(command_yaw));
    result.target_pitch = command_pitch;
  }

  // ---- Step 5: 前馈角速度 (从状态向量导出) ----
  double xc = predicted_state(0), yc = predicted_state(1);
  double vx = predicted_state(3), vy = predicted_state(4);
  double r_sq = xc * xc + yc * yc;
  result.yaw_vel   = (r_sq > 1e-4) ? ((xc * vy - yc * vx) / r_sq) : 0.0;
  result.pitch_vel = 0.0;

  // ---- Step 6: 预测击中时的装甲板相位角 (用于 Gate 2) ----
  double phi_now   = predicted_state(7);   // 当前相位角 (rad)
  double omega     = predicted_state(8);   // 角速度 (rad/s)
  result.hit_phi   = phi_now + omega * t_flight;

  // ---- Step 7: 三段火控判定 ----
  // Gate 1 比较最终绝对命令 vs 下位机当前绝对姿态
  // Gate 2 继续使用 tracker 所在参考系下的 yaw, 保持与 state/phi 语义一致
  result.target_distance = std::sqrt(
    predicted_state(0) * predicted_state(0) +
    predicted_state(1) * predicted_state(1) +
    predicted_state(2) * predicted_state(2));
  auto fire_windows = getFireWindows(result.target_distance);
  result.yaw_window = fire_windows.first;
  result.pitch_window = fire_windows.second;

  bool gate1 = gateBarrelAlignment(
    result.target_yaw, result.target_pitch, current_yaw_deg, current_pitch_deg,
    result.target_distance);
  bool gate2 = gateArmorFacing(predicted_state, command_yaw, t_flight, model_probs[1]);
  bool gate3 = gateConvergenceAndRange(predicted_state, tracker_frames);

  result.gate1_alignment = gate1;
  result.gate2_facing = gate2;
  result.gate3_ready = gate3;
  result.can_fire = gate1 && gate2 && gate3;

  return result;
}

// =============================================================================
// 物理前馈: 预测子弹飞行时间后的目标位置
// =============================================================================
inline Eigen::Vector3d Aimer::predictHitPosition(
  const StateVec & state,
  double bullet_speed,
  double & t_flight) const
{
  const double xc = state(0);
  const double yc = state(1);
  const double zc = state(2);
  const double vx = state(3);
  const double vy = state(4);
  const double vz = state(5);

  // 水平距离
  double d_h = std::sqrt(xc * xc + yc * yc);

  // 迭代法求解飞行时间 (最多 3 次迭代)
  // 初始猜测: t = d_h / v_bullet (忽略重力影响)
  t_flight = d_h / bullet_speed;

  for (int i = 0; i < 3; ++i) {
    // 目标在 t_flight 后的位置 (匀速直线近似)
    double x_hit = xc + vx * t_flight;
    double y_hit = yc + vy * t_flight;
    double z_hit = zc + vz * t_flight;

    // 更新距离估算
    double d_new = std::sqrt(x_hit * x_hit + y_hit * y_hit + z_hit * z_hit);
    t_flight = d_new / bullet_speed;
  }

  // 最终预测位置
  Eigen::Vector3d hit_pos;
  hit_pos(0) = xc + vx * t_flight;
  hit_pos(1) = yc + vy * t_flight;
  hit_pos(2) = zc + vz * t_flight;

  return hit_pos;
}

// =============================================================================
// 逆运动学: 3D 位置 → (yaw, pitch)
// =============================================================================
inline void Aimer::inverseKinematics(
  const Eigen::Vector3d & hit_pos,
  double & yaw, double & pitch) const
{
  // Yaw: 水平面方位角 (CCW+, 向左为正, 与 FLU 一致)
  yaw = std::atan2(hit_pos(1), hit_pos(0));

  // Pitch: FLU 约定 (正=向下, 负=向上)
  // atan2(-z, d_h): 目标在上方(z>0) → pitch<0(向上看) ✓
  double d_h = std::sqrt(hit_pos(0) * hit_pos(0) + hit_pos(1) * hit_pos(1));
  pitch = std::atan2(-hit_pos(2), d_h);
}

// =============================================================================
// 重力下坠补偿 (LUT 接口)
// =============================================================================
// 当前实现: 解析弹道模型 + 多项式校正
//   Δh = ½ g t²
//   Δα = arctan(Δh / d)
//
// 预留: 可替换为查表或多项式拟合
//   return k0 + k1*d + k2*d² + ...
// =============================================================================
inline double Aimer::getPitchOffset(double distance) const
{
  if (distance < 0.1) return 0.0;

  // 解析弹道模型
  double t_flight = distance / params_.bullet_speed;
  double drop = 0.5 * params_.gravity * t_flight * t_flight;
  double comp = std::atan2(drop, distance);

  // 加上 LUT 多项式校正 (参数从 YAML 加载, 默认为 0)
  double lut_offset = params_.pitch_offset_k0 +
                      params_.pitch_offset_k1 * distance +
                      params_.pitch_offset_k2 * distance * distance;

  return comp + lut_offset;
}

// =============================================================================
// Gate 1: 炮管对齐容差
// =============================================================================
// 比较解算角度 (数学约定, rad) 与下位机反馈角度 (deg)
// 注意: GimbalStatus 中 yaw/pitch 单位为 deg
// =============================================================================
inline bool Aimer::gateBarrelAlignment(
  double target_yaw, double target_pitch,
  double current_yaw_deg, double current_pitch_deg,
  double distance) const
{
  // 下位机反馈: deg → rad (全部直传, 坐标系已对齐)
  double cur_yaw   = current_yaw_deg * M_PI / 180.0;
  double cur_pitch = current_pitch_deg * M_PI / 180.0;

  double err_yaw   = std::fabs(std::atan2(std::sin(target_yaw - cur_yaw),
                                           std::cos(target_yaw - cur_yaw)));
  double err_pitch = std::fabs(target_pitch - cur_pitch);

  auto fire_windows = getFireWindows(distance);
  return (err_yaw < fire_windows.first) &&
         (err_pitch < fire_windows.second);
}

inline std::pair<double, double> Aimer::getFireWindows(double distance) const
{
  if (!params_.use_dynamic_fire_window || distance <= 1e-3) {
    return {params_.fire_tolerance, params_.fire_tolerance};
  }

  double yaw_window = std::atan2(params_.shooting_range_width / 2.0, distance);
  double pitch_window = std::atan2(params_.shooting_range_height / 2.0, distance);

  double min_window = params_.min_fire_window_deg * M_PI / 180.0;
  double max_window = params_.max_fire_window_deg * M_PI / 180.0;
  if (min_window > max_window) std::swap(min_window, max_window);

  yaw_window = std::clamp(yaw_window, min_window, max_window);
  pitch_window = std::clamp(pitch_window, min_window, max_window);

  return {yaw_window, pitch_window};
}

// =============================================================================
// Gate 2: 装甲板相位窗口 (The Spin Killer)
// =============================================================================
// 原理:
//   CTRV 模型下，目标在绕中心旋转。装甲板法线方向随相位角 φ 变化。
//   子弹击中时，装甲板法线方向为:
//     n = [cos(φ_hit + π/2), sin(φ_hit + π/2), 0]^T
//   (法线垂直于半径方向)
//
//   枪管射线方向:
//     d = [cos(yaw)·cos(pitch), sin(yaw)·cos(pitch), sin(pitch)]^T
//
//   夹角 θ = arccos(n · d)
//   只有 θ < armor_facing_tol 时才开火
//
//   当目标不旋转 (CV 模型权重高) 时, 此门控自动旁路
// =============================================================================
inline bool Aimer::gateArmorFacing(
  const StateVec & state,
  double target_yaw,
  double t_flight,
  double p_ctrv) const
{
  // 如果 CTRV 概率低于阈值 → 目标不在旋转 → 旁路此门控
  if (p_ctrv < params_.ctrv_threshold) return true;

  // 预测击中时的相位角
  double phi   = state(7);   // 当前相位
  double omega = state(8);   // 角速度
  double phi_hit = phi + omega * t_flight;

  // 装甲板法线方向 (水平面, 垂直于半径)
  double n_x = std::cos(phi_hit + M_PI / 2.0);
  double n_y = std::sin(phi_hit + M_PI / 2.0);

  // 枪管射线方向 (水平面投影, 忽略 pitch 影响简化计算)
  double d_x = std::cos(target_yaw);
  double d_y = std::sin(target_yaw);

  // 法线与射线的夹角
  double cos_theta = n_x * d_x + n_y * d_y;
  cos_theta = std::clamp(cos_theta, -1.0, 1.0);
  double theta_deg = std::acos(cos_theta) * 180.0 / M_PI;

  return theta_deg < params_.armor_facing_tol;
}

// =============================================================================
// Gate 3: 收敛保护 + 距离保护
// =============================================================================
inline bool Aimer::gateConvergenceAndRange(
  const StateVec & state,
  int tracker_frames) const
{
  // 收敛帧数检查
  if (tracker_frames < params_.fire_min_frames) return false;

  // 距离检查
  double xc = state(0), yc = state(1), zc = state(2);
  double dist = std::sqrt(xc * xc + yc * yc + zc * zc);
  if (dist > params_.fire_max_distance) return false;

  return true;
}

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__AIMER_HPP_
