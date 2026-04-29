// =============================================================================
// imm_ukf_tracker.hpp — IMM-UKF 装甲板追踪器 (纯数学层，零 ROS 依赖)
// =============================================================================
// 算法: 交互式多模型 (IMM) × 无迹卡尔曼滤波 (UKF)
// 模型: CV (Constant Velocity) + CTRV (Constant Turn Rate and Velocity)
//
// ====== 状态向量设计 (9维) ======
//
//   x = [ xc, yc, zc, vx, vy, vz, r, phi, omega ]^T
//         │    │    │   │   │   │   │   │     │
//         │    │    │   │   │   │   │   │     └── 装甲板自旋角速度 (rad/s)
//         │    │    │   │   │   │   │   └──────── 装甲板相位角 (rad)
//         │    │    │   │   │   │   └──────────── 旋转半径 / 小陀螺半径 (m)
//         │    │    │   │   │   └──────────────── Z 方向线速度 (m/s)
//         │    │    │   │   └──────────────────── Y 方向线速度 (m/s)
//         │    │    │   └──────────────────────── X 方向线速度 (m/s)
//         │    │    └──────────────────────────── 目标中心 Z (m)
//         │    └───────────────────────────────── 目标中心 Y (m)
//         └────────────────────────────────────── 目标中心 X (m)
//
// ====== 设计哲学 ======
//
//   1. 中心化状态: (xc, yc, zc) 描述的是**机器人底盘旋转中心**在相机坐标系
//      下的位置，而非某一块装甲板的位置。这样即使装甲板切换（前后左右），
//      中心坐标保持平滑，滤波器不会因为观测跳变而发散。
//
//   2. 装甲板观测模型: 第 i 块装甲板的位置为
//         armor_i = center + r * [cos(phi_i), sin(phi_i), 0]^T
//      其中 phi_i = phi + i * (2π/N)。观测时遍历所有装甲板方向，选择与
//      实际观测最近的作为匹配目标。
//
//   3. 9 维而非 7 维: 虽然 z 方向通常运动很小，保留 vz 使得滤波器可以
//      追踪哨兵升降机构等垂直运动。比强行投影到 2D 更鲁棒。
//
//   4. CV 模型: 匀速直线运动，用于目标直线接近/远离。
//   5. CTRV 模型: 匀速转弯，精确匹配小陀螺运动模式。
//      IMM 自动根据似然度分配模型概率，无需手动切换。
//
// =============================================================================
#ifndef RM_AUTOAIM__IMM_UKF_TRACKER_HPP_
#define RM_AUTOAIM__IMM_UKF_TRACKER_HPP_

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <vector>
#include <string>

namespace rm_autoaim
{

// =============================================================================
// 编译期常量
// =============================================================================
constexpr int   NX  = 9;      // 状态维度
constexpr int   NZ  = 3;      // 观测维度 (x, y, z)
constexpr int   NUM_MODELS = 2; // CV + CTRV
constexpr double PI  = 3.14159265358979323846;
constexpr double EPS = 1e-6;

// =============================================================================
// 工具函数
// =============================================================================

/// 角度归一化到 [-π, π)
inline double normalize_angle(double angle)
{
  while (angle >= PI)  angle -= 2.0 * PI;
  while (angle < -PI)  angle += 2.0 * PI;
  return angle;
}

/// 对整个向量的指定行做角度归一化 (用于 Sigma 点)
template <typename Derived>
inline void normalize_angle_inplace(Eigen::MatrixBase<Derived> & v, int row)
{
  v(row) = normalize_angle(v(row));
}

// =============================================================================
// UKF 参数配置
// =============================================================================
struct UKFParams
{
  // Merwe Scaled UT 参数
  double alpha = 0.1;    // 扩散参数 (≥0.1 才能有效捕获 CTRV 非线性)
  double beta  = 2.0;    // 先验知识参数 (高斯分布最优值)
  double kappa = 0.0;    // 次要缩放参数

  // 过程噪声标准差
  double q_pos   = 0.05;   // 位置过程噪声 (m)
  double q_vel   = 0.5;    // 速度过程噪声 (m/s)
  double q_r     = 0.005;  // 半径过程噪声 (m)
  double q_phi   = 0.05;   // 相位角过程噪声 (rad)
  double q_omega = 0.1;    // 角速度过程噪声 (rad/s)

  // 观测噪声标准差
  double r_pos = 0.03;     // 位置观测噪声 (m)

  // IMM 马尔可夫转移概率矩阵 (2×2)
  // M[i][j] = P(model_j | model_i)
  // M[0] = CV→{CV, CTRV}  M[1] = CTRV→{CV, CTRV}
  double markov_transition[2][2] = {
    {0.95, 0.05},   // CV → CV(95%), CV → CTRV(5%)
    {0.05, 0.95}    // CTRV → CV(5%), CTRV → CTRV(95%)
  };

  // 初始模型概率
  double initial_model_prob[2] = {0.6, 0.4};  // CV=60%, CTRV=40%
};

// =============================================================================
// IMM-UKF Tracker — 主类
// =============================================================================
class ImmUkfTracker
{
public:
  // ---- 类型别名 ----
  using StateVec   = Eigen::Matrix<double, NX, 1>;
  using StateMat   = Eigen::Matrix<double, NX, NX>;
  using MeasureVec = Eigen::Matrix<double, NZ, 1>;
  using MeasureMat = Eigen::Matrix<double, NZ, NZ>;

  // ===========================================================================
  //  公开接口
  // ===========================================================================

  /// 默认构造 (使用默认参数)
  ImmUkfTracker();

  /// 带参数构造
  explicit ImmUkfTracker(const UKFParams & params);

  /// 初始化追踪器
  /// @param initial_pos  首次观测到的装甲板 3D 位置 (相机坐标系)
  /// @param timestamp    当前时间戳 (秒)
  void init(const Eigen::Vector3d & initial_pos, double timestamp);

  /// 纯预测 (不更新) — 用于弹道解算中的状态外推
  /// @param timestamp  目标时间戳 (秒)
  /// @return 预测的中心位置状态向量 (9维)
  StateVec predict(double timestamp);

  /// 带观测的更新步骤 — 接收新的装甲板位置并更新滤波器
  /// @param measured_pos  观测到的装甲板 3D 位置 (相机坐标系)
  /// @param timestamp     当前时间戳 (秒)
  void update(const Eigen::Vector3d & measured_pos, double timestamp);

  /// 获取当前融合状态
  StateVec getState() const { return fused_state_; }

  /// 获取当前融合协方差
  StateMat getCovariance() const { return fused_cov_; }

  /// 获取模型概率 [P_CV, P_CTRV]
  std::array<double, NUM_MODELS> getModelProbabilities() const;

  /// 是否已初始化
  bool isInitialized() const { return initialized_; }

  /// 调试: 获取最近一次更新的似然度
  std::array<double, NUM_MODELS> getLastLikelihoods() const;

private:
  // ===========================================================================
  //  UKF 内部数据结构 (每个模型一个)
  // ===========================================================================
  struct UKFFilter
  {
    StateVec   x = StateVec::Zero();   // 状态估计
    StateMat   P = StateMat::Identity(); // 状态协方差
    StateMat   Q = StateMat::Identity(); // 过程噪声协方差

    // Sigma 点权重
    Eigen::VectorXd w_mean;              // 均值权重 (2*NX+1)
    Eigen::VectorXd w_cov;               // 协方差权重 (2*NX+1)

    // 预测阶段的中间量 (用于 update)
    Eigen::Matrix<double, NX, 2 * NX + 1> sigma_pred =
      Eigen::Matrix<double, NX, 2 * NX + 1>::Zero(); // 传播后的 Sigma 点
    MeasureVec z_pred = MeasureVec::Zero(); // 预测观测
    Eigen::Matrix<double, NZ, NZ> S;        // 新息协方差
    Eigen::Matrix<double, NX, NZ> K;        // 卡尔曼增益
    Eigen::Matrix<double, NZ, 2 * NX + 1> z_sigma; // 观测 Sigma 点
    int target_plate = 0;                    // 当前追踪的装甲板编号 (0-3)

    double likelihood = 1.0;  // 最近一次更新的似然度
  };

  // ===========================================================================
  //  核心算法模块
  // ===========================================================================

  /// Merwe Scaled UT: 计算权重
  void computeWeights(UKFFilter & filter) const;

  /// 生成 Sigma 点 (含 Cholesky 保护)
  /// @param[in]  x       均值
  /// @param[in]  P       协方差
  /// @param[out] sigma   输出 Sigma 点矩阵 [NX × (2*NX+1)]
  void generateSigmaPoints(
    const StateVec & x,
    const StateMat & P,
    Eigen::Matrix<double, NX, 2 * NX + 1> & sigma) const;

  /// CV 模型状态转移函数 f_cv(x, dt)
  StateVec processModelCV(const StateVec & x, double dt) const;

  /// CTRV 模型状态转移函数 f_ctrv(x, dt)
  StateVec processModelCTRV(const StateVec & x, double dt) const;

  /// 观测函数 h(x) — 中心状态 → 装甲板位置
  /// 使用最近装甲板匹配: 返回距离最近的装甲板观测
  MeasureVec observationModel(const StateVec & x) const;

  /// 获取所有装甲板候选位置 (4块装甲板)
  std::vector<MeasureVec> getAllArmorPositions(const StateVec & x) const;

  /// 单个模型的 UKF 预测步骤
  void ukfPredict(UKFFilter & filter, double dt, int model_idx);

  /// 单个模型的 UKF 更新步骤
  void ukfUpdate(UKFFilter & filter, const MeasureVec & z, int model_idx);

  // ===========================================================================
  //  IMM 模块
  // ===========================================================================

  /// IMM Step 1: 计算混合概率
  void immInteraction();

  /// IMM Step 2: 模型条件滤波 (predict + update)
  void immFiltering(const MeasureVec & z, double dt);

  /// IMM Step 3: 模型概率更新
  void immUpdateProbabilities();

  /// IMM Step 4: 状态融合
  void immFusion();

  // ===========================================================================
  //  内部初始化
  // ===========================================================================
  void initInternal();

  // ===========================================================================
  //  私有成员
  // ===========================================================================
  UKFParams params_;

  std::array<UKFFilter, NUM_MODELS> filters_;  // [0]=CV, [1]=CTRV
  std::array<double, NUM_MODELS> model_probs_;  // 模型概率

  // IMM 交互的中间量
  std::array<StateVec, NUM_MODELS> mixed_states_;
  std::array<StateMat, NUM_MODELS> mixed_covs_;

  // 融合输出
  StateVec fused_state_;
  StateMat fused_cov_;

  double last_timestamp_ = 0.0;
  bool initialized_ = false;

  // 马尔可夫转移矩阵 (Eigen 格式)
  Eigen::Matrix<double, NUM_MODELS, NUM_MODELS> markov_matrix_;
};

}  // namespace rm_autoaim

#endif  // RM_AUTOAIM__IMM_UKF_TRACKER_HPP_