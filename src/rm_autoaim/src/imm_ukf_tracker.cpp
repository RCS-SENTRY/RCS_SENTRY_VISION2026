// =============================================================================
// imm_ukf_tracker.cpp — IMM-UKF 装甲板追踪器实现
// =============================================================================
// 纯数学层，零 ROS 依赖。所有矩阵运算基于 Eigen3。
//
// 审查修复记录:
//   (2026-04-04) P0-Fix 1: Sigma 点 φ 均值改用方向向量法
//   (2026-04-04) P0-Fix 2: Sigma 点观测传播统一选板
//   (2026-04-04) P1-Fix 1: ukfUpdate 先马氏距离选板 → 再重算
//   (2026-04-04) P1-Fix 2: alpha 从 1e-3 调大到 0.1
//   (2026-04-06) P2-Fix 1: init() 重置 model_probs / likelihood / target_plate
//   (2026-04-06) P2-Fix 2: update() 中 dt > 0.5s 强制 re-init (防幽灵时间戳)
// =============================================================================
#include "rm_autoaim/imm_ukf_tracker.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cassert>

namespace rm_autoaim
{

// =============================================================================
//  构造函数
// =============================================================================

ImmUkfTracker::ImmUkfTracker() : params_() { initInternal(); }

ImmUkfTracker::ImmUkfTracker(const UKFParams & params) : params_(params) { initInternal(); }

void ImmUkfTracker::initInternal()
{
  constexpr int NS = 2 * NX + 1;  // Sigma 点数量 = 19

  // ---- 初始化模型概率 ----
  model_probs_[0] = params_.initial_model_prob[0];
  model_probs_[1] = params_.initial_model_prob[1];

  // ---- 初始化马尔可夫转移矩阵 ----
  for (int i = 0; i < NUM_MODELS; i++)
    for (int j = 0; j < NUM_MODELS; j++)
      markov_matrix_(i, j) = params_.markov_transition[i][j];

  // ---- 初始化每个 UKF 滤波器 ----
  for (auto & f : filters_) {
    f.x.setZero();
    f.P  = StateMat::Identity();
    f.Q  = StateMat::Identity();
    f.w_mean = Eigen::VectorXd(NS);
    f.w_cov  = Eigen::VectorXd(NS);
    f.S = MeasureMat::Identity();
    f.K = Eigen::Matrix<double, NX, NZ>::Zero();
    f.z_sigma = Eigen::Matrix<double, NZ, NS>::Zero();
    f.sigma_pred = Eigen::Matrix<double, NX, NS>::Zero();
    f.target_plate = 0;
    computeWeights(f);
  }

  // ---- 配置过程噪声 Q ----
  auto buildQ = [&](StateMat & Q) {
    Q = StateMat::Zero();
    Q(0, 0) = params_.q_pos   * params_.q_pos;
    Q(1, 1) = params_.q_pos   * params_.q_pos;
    Q(2, 2) = params_.q_pos   * params_.q_pos;
    Q(3, 3) = params_.q_vel   * params_.q_vel;
    Q(4, 4) = params_.q_vel   * params_.q_vel;
    Q(5, 5) = params_.q_vel   * params_.q_vel;
    Q(6, 6) = params_.q_r     * params_.q_r;
    Q(7, 7) = params_.q_phi   * params_.q_phi;
    Q(8, 8) = params_.q_omega * params_.q_omega;
  };
  for (auto & f : filters_) buildQ(f.Q);

  fused_state_.setZero();
  fused_cov_ = StateMat::Identity();
}

// =============================================================================
//  公开接口
// =============================================================================

void ImmUkfTracker::init(const Eigen::Vector3d & initial_pos, double timestamp)
{
  for (auto & f : filters_) {
    // [P2-Fix 1] 强制覆盖位置、速度、角速度
    f.x.setZero();
    f.x.head<3>() = initial_pos;  // 位置直接覆为观测值
    // f.x(3..5) = 0.0  速度置零 (setZero 已处理)
    f.x(6) = 0.20;   // 默认半径 0.2m
    // f.x(7) = 0.0    (setZero 已处理)
    // f.x(8) = 0.0    (setZero 已处理)

    // 位置小方差 (已锁定到观测)，速度大方差 (待估计)
    f.P = StateMat::Identity();
    f.P.block<3, 3>(0, 0) *= 0.01;   // 位置: 0.01 (10cm std)
    f.P.block<3, 3>(3, 3) *= 1.0;    // 速度: 1.0  (1 m/s std)
    f.P(6, 6) = 0.04;                // 半径: 0.2m std
    f.P(7, 7) = 1.0;                 // φ: ~57° std
    f.P(8, 8) = 1.0;                 // ω: ~1 rad/s std

    // [P2-Fix 1] 重置每个滤波器的辅助状态
    f.target_plate = 0;
    f.likelihood = 1.0;
  }

  // [P2-Fix 1] 重置 IMM 模型概率为初始值
  model_probs_[0] = params_.initial_model_prob[0];
  model_probs_[1] = params_.initial_model_prob[1];

  fused_state_ = filters_[0].x;
  fused_cov_   = filters_[0].P;
  last_timestamp_ = timestamp;
  initialized_ = true;
}

ImmUkfTracker::StateVec ImmUkfTracker::predict(double timestamp)
{
  if (!initialized_) return StateVec::Zero();
  double dt = timestamp - last_timestamp_;
  if (dt <= 0.0 || dt > 1.0) dt = 0.001;

  // ★ Save & Restore: predict 是纯查询函数，绝不能污染滤波器内部状态
  // 保存当前所有模型的状态向量 x 和协方差矩阵 P
  std::array<StateVec, NUM_MODELS> saved_x;
  std::array<StateMat, NUM_MODELS> saved_P;
  for (int m = 0; m < NUM_MODELS; m++) {
    saved_x[m] = filters_[m].x;
    saved_P[m] = filters_[m].P;
  }

  immInteraction();

  for (int m = 0; m < NUM_MODELS; m++) {
    ukfPredict(filters_[m], dt, m);
  }

  immFusion();
  auto result = fused_state_;

  // ★ 核心修复: 恢复滤波器内部状态，消除时间轴漂移
  for (int m = 0; m < NUM_MODELS; m++) {
    filters_[m].x = saved_x[m];
    filters_[m].P = saved_P[m];
  }

  return result;
}

void ImmUkfTracker::update(const Eigen::Vector3d & measured_pos, double timestamp)
{
  if (!initialized_) {
    init(measured_pos, timestamp);
    return;
  }

  double dt = timestamp - last_timestamp_;

  // [P2-Fix 2] 幽灵时间戳拦截: dt > 0.5s 视为目标丢失或 Bag 异常
  //   强制 re-init，将这段数据视为一段新的轨迹
  if (dt > 0.5) {
    init(measured_pos, timestamp);
    return;
  }

  if (dt <= 0.0) dt = 0.001;  // 重复/倒序时间戳保护
  last_timestamp_ = timestamp;

  immInteraction();
  immFiltering(MeasureVec(measured_pos), dt);
  immUpdateProbabilities();
  immFusion();
}

std::array<double, NUM_MODELS> ImmUkfTracker::getModelProbabilities() const
{
  return {model_probs_[0], model_probs_[1]};
}

std::array<double, NUM_MODELS> ImmUkfTracker::getLastLikelihoods() const
{
  return {filters_[0].likelihood, filters_[1].likelihood};
}

// =============================================================================
//  Merwe Scaled UT — 权重计算
// =============================================================================
void ImmUkfTracker::computeWeights(UKFFilter & filter) const
{
  constexpr int NS = 2 * NX + 1;
  const double a = params_.alpha;
  const double b = params_.beta;
  const double k = params_.kappa;
  const double lam = a * a * (NX + k) - NX;

  filter.w_mean(0) = lam / (NX + lam);
  filter.w_cov(0)  = lam / (NX + lam) + (1.0 - a * a + b);

  const double w = 1.0 / (2.0 * (NX + lam));
  for (int i = 1; i < NS; i++) {
    filter.w_mean(i) = w;
    filter.w_cov(i)  = w;
  }
}

// =============================================================================
//  Sigma 点生成 (含 Cholesky 保护)
// =============================================================================
void ImmUkfTracker::generateSigmaPoints(
  const StateVec & x, const StateMat & P,
  Eigen::Matrix<double, NX, 2 * NX + 1> & sigma) const
{
  const double a = params_.alpha;
  const double k = params_.kappa;
  const double lam = a * a * (NX + k) - NX;
  const double scale = std::sqrt(NX + lam);

  // Cholesky 分解 L·L^T = P，含 Jitter 保护
  StateMat P_safe = P;
  Eigen::LLT<StateMat> llt(P_safe);
  if (llt.info() != Eigen::Success) {
    double jitter = 1e-6;
    for (int attempt = 0; attempt < 10; attempt++) {
      P_safe += jitter * StateMat::Identity();
      llt.compute(P_safe);
      if (llt.info() == Eigen::Success) break;
      jitter *= 10.0;
    }
  }
  StateMat L = llt.matrixL();

  sigma.col(0) = x;
  for (int i = 0; i < NX; i++) {
    sigma.col(i + 1)      = x + scale * L.col(i);
    sigma.col(i + 1 + NX) = x - scale * L.col(i);
  }
}

// =============================================================================
//  CV 模型 — 匀速直线运动
// =============================================================================
ImmUkfTracker::StateVec ImmUkfTracker::processModelCV(
  const StateVec & x, double dt) const
{
  StateVec x_new = x;
  x_new(0) += x(3) * dt;
  x_new(1) += x(4) * dt;
  x_new(2) += x(5) * dt;
  x_new(7) += x(8) * dt;
  x_new(7) = normalize_angle(x_new(7));
  return x_new;
}

// =============================================================================
//  CTRV 模型 — 匀速转弯 (小陀螺专用，含 L'Hôpital 除零保护)
// =============================================================================
ImmUkfTracker::StateVec ImmUkfTracker::processModelCTRV(
  const StateVec & x, double dt) const
{
  StateVec x_new = x;
  const double vx = x(3), vy = x(4), omega = x(8);
  const double speed = std::sqrt(vx * vx + vy * vy);

  x_new(2) += x(5) * dt;
  x_new(6) = x(6);

  if (std::abs(omega) > EPS) {
    const double heading = std::atan2(vy, vx);
    const double new_heading = heading + omega * dt;
    const double ratio = speed / omega;

    x_new(0) = x(0) + ratio * (std::sin(new_heading) - std::sin(heading));
    x_new(1) = x(1) + ratio * (-std::cos(new_heading) + std::cos(heading));
    x_new(3) = speed * std::cos(new_heading);
    x_new(4) = speed * std::sin(new_heading);
  } else {
    x_new(0) = x(0) + vx * dt;
    x_new(1) = x(1) + vy * dt;
  }

  x_new(7) = normalize_angle(x(7) + omega * dt);
  x_new(8) = x(8);
  return x_new;
}

// =============================================================================
//  观测模型辅助函数
// =============================================================================
std::vector<ImmUkfTracker::MeasureVec> ImmUkfTracker::getAllArmorPositions(
  const StateVec & x) const
{
  constexpr int N_PLATES = 4;
  constexpr double DPHI = 2.0 * PI / N_PLATES;
  const double r = std::abs(x(6));

  std::vector<MeasureVec> plates(N_PLATES);
  for (int i = 0; i < N_PLATES; i++) {
    double angle = x(7) + i * DPHI;
    plates[i] = MeasureVec(
      x(0) + r * std::cos(angle),
      x(1) + r * std::sin(angle),
      x(2));
  }
  return plates;
}

ImmUkfTracker::MeasureVec ImmUkfTracker::observationModel(const StateVec & x) const
{
  const double r = std::abs(x(6));
  if (r < 1e-4) {
    return x.head<3>();
  }

  const double dir_to_cam = std::atan2(-x(1), -x(0));

  constexpr int N_PLATES = 4;
  constexpr double DPHI = 2.0 * PI / N_PLATES;
  int best_k = 0;
  double best_diff = std::abs(normalize_angle(x(7) - dir_to_cam));
  for (int k = 1; k < N_PLATES; k++) {
    double diff = std::abs(normalize_angle(x(7) + k * DPHI - dir_to_cam));
    if (diff < best_diff) {
      best_diff = diff;
      best_k = k;
    }
  }

  const double phi_obs = x(7) + best_k * DPHI;
  return MeasureVec(
    x(0) + r * std::cos(phi_obs),
    x(1) + r * std::sin(phi_obs),
    x(2));
}

// =============================================================================
//  [P0-Fix 1 + P0-Fix 2] 单模型 UKF 预测步骤
// =============================================================================
//  修复要点:
//    1. φ(7) 均值使用方向向量法 (sin/cos 加权 + atan2)
//    2. 所有 Sigma 点的观测传播统一使用同一块装甲板 (基于 x_pred 选板)
// =============================================================================
void ImmUkfTracker::ukfPredict(UKFFilter & filter, double dt, int model_idx)
{
  constexpr int NS = 2 * NX + 1;
  constexpr int N_PLATES = 4;
  constexpr double DPHI = 2.0 * PI / N_PLATES;

  // ======== Step 1: 生成 Sigma 点 ========
  Eigen::Matrix<double, NX, NS> sigma;
  generateSigmaPoints(filter.x, filter.P, sigma);

  // ======== Step 2: 通过过程模型传播 Sigma 点 ========
  Eigen::Matrix<double, NX, NS> & sigma_pred = filter.sigma_pred;
  for (int i = 0; i < NS; i++) {
    if (model_idx == 0) {
      sigma_pred.col(i) = processModelCV(sigma.col(i), dt);
    } else {
      sigma_pred.col(i) = processModelCTRV(sigma.col(i), dt);
    }
    sigma_pred(7, i) = normalize_angle(sigma_pred(7, i));
  }

  // ======== Step 3: 预测状态均值 ========
  // 非角度分量: 算术加权平均
  StateVec x_pred = StateVec::Zero();
  for (int i = 0; i < NS; i++) {
    x_pred += filter.w_mean(i) * sigma_pred.col(i);
  }

  // [P0-Fix 1] 角度分量 φ(7): 方向向量加权平均
  //   φ_mean = atan2(Σ w_i · sin(φ_i), Σ w_i · cos(φ_i))
  //   ω(8) 是角速度 (rad/s)，不是周期量，算术平均正确
  {
    double sin_sum = 0.0, cos_sum = 0.0;
    for (int i = 0; i < NS; i++) {
      sin_sum += filter.w_mean(i) * std::sin(sigma_pred(7, i));
      cos_sum += filter.w_mean(i) * std::cos(sigma_pred(7, i));
    }
    x_pred(7) = normalize_angle(std::atan2(sin_sum, cos_sum));
  }

  // ======== Step 4: 预测协方差 ========
  StateMat P_pred = StateMat::Zero();
  for (int i = 0; i < NS; i++) {
    StateVec diff = sigma_pred.col(i) - x_pred;
    diff(7) = normalize_angle(diff(7));
    P_pred += filter.w_cov(i) * diff * diff.transpose();
  }
  P_pred += filter.Q * dt;

  // ======== Step 5: 写回 ========
  filter.x = x_pred;
  filter.P = P_pred;

  // ======== Step 6: [P0-Fix 2] 统一选板 + 观测 Sigma 点传播 ========
  // Step 6a: 基于均值 x_pred 确定追踪哪块板
  int target_plate = 0;
  const double r_pred = std::abs(x_pred(6));
  if (r_pred >= 1e-4) {
    const double dir_to_cam = std::atan2(-x_pred(1), -x_pred(0));
    double best_diff = std::abs(normalize_angle(x_pred(7) - dir_to_cam));
    for (int k = 1; k < N_PLATES; k++) {
      double diff = std::abs(normalize_angle(x_pred(7) + k * DPHI - dir_to_cam));
      if (diff < best_diff) {
        best_diff = diff;
        target_plate = k;
      }
    }
  }
  filter.target_plate = target_plate;

  // Step 6b: 所有 Sigma 点使用同一块板计算观测值
  //   z_sigma.col(i) = center_i + r_i · [cos(φ_i + k·π/2), sin(φ_i + k·π/2), 0]^T
  for (int i = 0; i < NS; i++) {
    const double r_i = std::abs(sigma_pred(6, i));
    const double angle_i = sigma_pred(7, i) + target_plate * DPHI;
    filter.z_sigma.col(i) = MeasureVec(
      sigma_pred(0, i) + r_i * std::cos(angle_i),
      sigma_pred(1, i) + r_i * std::sin(angle_i),
      sigma_pred(2, i));
  }

  // ======== Step 7: 预测观测均值 ========
  filter.z_pred = MeasureVec::Zero();
  for (int i = 0; i < NS; i++) {
    filter.z_pred += filter.w_mean(i) * filter.z_sigma.col(i);
  }

  // ======== Step 8: 新息协方差 S ========
  MeasureMat S = MeasureMat::Zero();
  for (int i = 0; i < NS; i++) {
    MeasureVec zdiff = filter.z_sigma.col(i) - filter.z_pred;
    S += filter.w_cov(i) * zdiff * zdiff.transpose();
  }
  const double r_var = params_.r_pos * params_.r_pos;
  S += r_var * MeasureMat::Identity();
  filter.S = S;

  // ======== Step 9: 互协方差 T & 卡尔曼增益 K ========
  Eigen::Matrix<double, NX, NZ> T = Eigen::Matrix<double, NX, NZ>::Zero();
  for (int i = 0; i < NS; i++) {
    StateVec xdiff = sigma_pred.col(i) - x_pred;
    xdiff(7) = normalize_angle(xdiff(7));
    MeasureVec zdiff = filter.z_sigma.col(i) - filter.z_pred;
    T += filter.w_cov(i) * xdiff * zdiff.transpose();
  }
  filter.K = T * S.inverse();
}

// =============================================================================
//  [P1-Fix 1] 单模型 UKF 更新步骤
// =============================================================================
//  修复要点:
//    1. 先用马氏距离匹配装甲板 → 确定目标板
//    2. 基于确定的板重算 z_sigma / z_pred / S / K
//    3. 然后用正确的 K 和 S 执行状态更新
// =============================================================================
void ImmUkfTracker::ukfUpdate(UKFFilter & filter, const MeasureVec & z, int model_idx)
{
  constexpr int NS = 2 * NX + 1;
  constexpr int N_PLATES = 4;
  constexpr double DPHI = 2.0 * PI / N_PLATES;

  // ======== Step 1: [P1-Fix 1] 先用马氏距离匹配装甲板 ========
  //   使用 predict 阶段已有的 S^{-1} 做初始匹配 (足够准确)
  MeasureMat S_inv_est = filter.S.inverse();

  auto plates = getAllArmorPositions(filter.x);
  int best_plate = filter.target_plate;  // 默认: predict 阶段确定的板
  double min_mahal = (z - filter.z_pred).transpose() * S_inv_est * (z - filter.z_pred);

  for (int k = 0; k < N_PLATES; k++) {
    if (k == filter.target_plate) continue;
    double d = (z - plates[k]).transpose() * S_inv_est * (z - plates[k]);
    if (d < min_mahal) {
      min_mahal = d;
      best_plate = k;
    }
  }

  // ======== Step 2: 如果板号变了，基于新板号重算 z_sigma / z_pred / S / K ========
  if (best_plate != filter.target_plate) {
    filter.target_plate = best_plate;

    // 重算所有 Sigma 点的观测 (统一使用新板号)
    for (int i = 0; i < NS; i++) {
      const double r_i = std::abs(filter.sigma_pred(6, i));
      const double angle_i = filter.sigma_pred(7, i) + best_plate * DPHI;
      filter.z_sigma.col(i) = MeasureVec(
        filter.sigma_pred(0, i) + r_i * std::cos(angle_i),
        filter.sigma_pred(1, i) + r_i * std::sin(angle_i),
        filter.sigma_pred(2, i));
    }

    // 重算 z_pred
    filter.z_pred = MeasureVec::Zero();
    for (int i = 0; i < NS; i++) {
      filter.z_pred += filter.w_mean(i) * filter.z_sigma.col(i);
    }

    // 重算 S
    MeasureMat S = MeasureMat::Zero();
    for (int i = 0; i < NS; i++) {
      MeasureVec zdiff = filter.z_sigma.col(i) - filter.z_pred;
      S += filter.w_cov(i) * zdiff * zdiff.transpose();
    }
    S += (params_.r_pos * params_.r_pos) * MeasureMat::Identity();
    filter.S = S;

    // 重算 T 和 K
    Eigen::Matrix<double, NX, NZ> T = Eigen::Matrix<double, NX, NZ>::Zero();
    for (int i = 0; i < NS; i++) {
      StateVec xdiff = filter.sigma_pred.col(i) - filter.x;
      xdiff(7) = normalize_angle(xdiff(7));
      MeasureVec zdiff = filter.z_sigma.col(i) - filter.z_pred;
      T += filter.w_cov(i) * xdiff * zdiff.transpose();
    }
    filter.K = T * S.inverse();
  }

  // ======== Step 3: 计算创新向量 ========
  MeasureVec y = z - filter.z_pred;

  // ======== Step 4: 状态更新 ========
  StateVec x_upd = filter.x + filter.K * y;
  x_upd(7) = normalize_angle(x_upd(7));

  // ======== Step 5: 协方差更新 ========
  StateMat P_upd = filter.P - filter.K * filter.S * filter.K.transpose();
  filter.P = 0.5 * (P_upd + P_upd.transpose());  // 确保对称
  filter.x = x_upd;

  // ======== Step 6: 半径约束 r ≥ 0 ========
  if (filter.x(6) < 0) {
    filter.x(6) = std::abs(filter.x(6));
    filter.x(7) = normalize_angle(filter.x(7) + PI);
  }

  // ======== Step 7: 计算似然度 (IMM 模型概率更新用) ========
  MeasureMat S_inv = filter.S.inverse();
  double exponent = -0.5 * static_cast<double>(y.transpose() * S_inv * y);
  double log_det = std::log(filter.S.determinant());
  double log_likelihood =
    -0.5 * (NZ * std::log(2.0 * PI) + log_det) + exponent;

  filter.likelihood = std::exp(std::max(log_likelihood, -500.0));
}

// =============================================================================
//  IMM Step 1: 交互 / 混合
// =============================================================================
void ImmUkfTracker::immInteraction()
{
  double c[2];
  for (int j = 0; j < NUM_MODELS; j++) {
    c[j] = 0.0;
    for (int i = 0; i < NUM_MODELS; i++) {
      c[j] += markov_matrix_(i, j) * model_probs_[i];
    }
  }

  if (std::abs(c[0]) < 1e-15) c[0] = 1e-15;
  if (std::abs(c[1]) < 1e-15) c[1] = 1e-15;

  double mu_mix[2][2];
  for (int i = 0; i < NUM_MODELS; i++)
    for (int j = 0; j < NUM_MODELS; j++)
      mu_mix[i][j] = markov_matrix_(i, j) * model_probs_[i] / c[j];

  for (int j = 0; j < NUM_MODELS; j++) {
    StateVec x_mixed = StateVec::Zero();
    for (int i = 0; i < NUM_MODELS; i++) {
      x_mixed += mu_mix[i][j] * filters_[i].x;
    }
    // 角度加权 (方向向量法)
    double sin_sum = 0.0, cos_sum = 0.0;
    for (int i = 0; i < NUM_MODELS; i++) {
      sin_sum += mu_mix[i][j] * std::sin(filters_[i].x(7));
      cos_sum += mu_mix[i][j] * std::cos(filters_[i].x(7));
    }
    x_mixed(7) = normalize_angle(std::atan2(sin_sum, cos_sum));

    StateMat P_mixed = StateMat::Zero();
    for (int i = 0; i < NUM_MODELS; i++) {
      StateVec diff = filters_[i].x - x_mixed;
      diff(7) = normalize_angle(diff(7));
      P_mixed += mu_mix[i][j] * (filters_[i].P + diff * diff.transpose());
    }

    mixed_states_[j] = x_mixed;
    mixed_covs_[j]   = P_mixed;
  }

  for (int j = 0; j < NUM_MODELS; j++) {
    filters_[j].x = mixed_states_[j];
    filters_[j].P = mixed_covs_[j];
  }
}

// =============================================================================
//  IMM Step 2: 模型条件滤波 (predict + update)
// =============================================================================
void ImmUkfTracker::immFiltering(const MeasureVec & z, double dt)
{
  for (int m = 0; m < NUM_MODELS; m++) {
    ukfPredict(filters_[m], dt, m);
    ukfUpdate(filters_[m], z, m);
  }
}

// =============================================================================
//  IMM Step 3: 模型概率更新
// =============================================================================
void ImmUkfTracker::immUpdateProbabilities()
{
  double c[2];
  for (int j = 0; j < NUM_MODELS; j++) {
    c[j] = 0.0;
    for (int i = 0; i < NUM_MODELS; i++) {
      c[j] += markov_matrix_(i, j) * model_probs_[i];
    }
  }

  double unnorm[2];
  double sum = 0.0;
  for (int j = 0; j < NUM_MODELS; j++) {
    unnorm[j] = filters_[j].likelihood * c[j];
    if (unnorm[j] < 1e-300) unnorm[j] = 1e-300;
    sum += unnorm[j];
  }

  if (sum < 1e-300) sum = 1.0;
  for (int j = 0; j < NUM_MODELS; j++) {
    model_probs_[j] = unnorm[j] / sum;
  }
}

// =============================================================================
//  IMM Step 4: 状态融合 (角度用方向向量法)
// =============================================================================
void ImmUkfTracker::immFusion()
{
  fused_state_ = StateVec::Zero();
  for (int j = 0; j < NUM_MODELS; j++) {
    fused_state_ += model_probs_[j] * filters_[j].x;
  }

  // 角度加权 (方向向量法)
  double sin_sum = 0.0, cos_sum = 0.0;
  for (int j = 0; j < NUM_MODELS; j++) {
    sin_sum += model_probs_[j] * std::sin(filters_[j].x(7));
    cos_sum += model_probs_[j] * std::cos(filters_[j].x(7));
  }
  fused_state_(7) = normalize_angle(std::atan2(sin_sum, cos_sum));

  fused_cov_ = StateMat::Zero();
  for (int j = 0; j < NUM_MODELS; j++) {
    StateVec diff = filters_[j].x - fused_state_;
    diff(7) = normalize_angle(diff(7));
    fused_cov_ += model_probs_[j] * (filters_[j].P + diff * diff.transpose());
  }
}

}  // namespace rm_autoaim