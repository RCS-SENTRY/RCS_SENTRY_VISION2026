#!/usr/bin/env python3
"""
tune_ukf.py — IMM-UKF 数据驱动参数寻优器
=============================================================================

纯离线脚本，零 ROS 依赖。从 SQLite3 (ROSbag) 读取观测数据，
用 Python 忠实复现 C++ IMM-UKF 算法，网格搜索最优 Q/R 参数。

代价函数:
    J = w_lag * Lag + w_jitter * Jitter

    Lag   = 预测轨迹 X_pred(t) vs 未来观测 Z_raw(t+dt) 的 RMSE
    Jitter = 预测速度的方差 (高频抖动惩罚)

用法:
    python3 tools/tune_ukf.py
    python3 tools/tune_ukf.py --bag /path/to/bag.db3
    python3 tools/tune_ukf.py --quick   # 粗搜索
    python3 tools/tune_ukf.py --fine    # 精搜索

注意事项:
    当前使用 /autoaim/debug_world_points 的中心轨迹作为伪观测。
    这是最优参数下的滤波结果，作为 tuning proxy 是可行的，
    但理想情况下应录制约 raw PnP 观测话题。
=============================================================================
"""

import argparse
import math
import os
import sqlite3
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional

import matplotlib
matplotlib.use('Agg')  # 无头后端
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import cho_factor, cho_solve, cholesky, solve_triangular

# ============================================================================
# 常量 (与 C++ imm_ukf_tracker.hpp 一致)
# ============================================================================
NX = 9       # 状态维度
NZ = 3       # 观测维度
N_MODELS = 2 # CV + CTRV
N_PLATES = 4 # 装甲板数量
N_SIGMA = 2 * NX + 1  # 19

PI = 3.14159265358979323846
DPHI = 2.0 * PI / N_PLATES  # π/2

# ============================================================================
# 工具函数
# ============================================================================
def normalize_angle(a):
    while a >= PI:  a -= 2.0 * PI
    while a < -PI:  a += 2.0 * PI
    return a

vec_normalize_angle = np.vectorize(normalize_angle)

# ============================================================================
# Bag 解析器
# ============================================================================
BAG_DB = ""

def read_topic(db_path, topic_name):
    """从 db3 读取指定话题的 (timestamp, raw_bytes) 列表"""
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    row = c.fetchone()
    if not row:
        conn.close()
        return []
    tid = row[0]
    c.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (tid,))
    rows = c.fetchall()
    conn.close()
    return rows

def parse_point_stamped(data):
    """解析 CDR PointStamped → (stamp_sec, x, y, z)"""
    data = bytes(data)
    off = 4
    sec = struct.unpack_from('<i', data, off)[0]
    nsec = struct.unpack_from('<I', data, off+4)[0]
    stamp = sec + nsec * 1e-9
    off += 8
    slen = struct.unpack_from('<I', data, off)[0]
    off += 4 + slen
    off = (off + 3) & ~3
    x, y, z = struct.unpack_from('<ddd', data, off)
    return stamp, np.array([x, y, z])

def extract_observations(db_path):
    """提取观测序列: [(t, xyz), ...]"""
    raw = read_topic(db_path, "/autoaim/debug_world_points")
    obs = []
    for ts, data in raw:
        try:
            t, xyz = parse_point_stamped(data)
            if np.linalg.norm(xyz) < 100:
                obs.append((t, xyz))
        except:
            pass
    return obs

# ============================================================================
# IMM-UKF Python 实现 (忠实复现 C++ 逻辑)
# ============================================================================

@dataclass
class UKFParams:
    """可调参数 — 这是要搜索的变量"""
    alpha: float = 0.1
    beta: float  = 2.0
    kappa: float = 0.0

    q_pos: float   = 0.05
    q_vel: float   = 0.5
    q_r: float     = 0.005
    q_phi: float   = 0.05
    q_omega: float = 0.1

    r_pos: float = 0.03

    markov_00: float = 0.95
    markov_11: float = 0.95
    init_prob_cv: float = 0.6

class UKFFilter:
    """单个 UKF 滤波器"""
    def __init__(self):
        self.x = np.zeros(NX)       # 状态
        self.P = np.eye(NX)         # 协方差
        self.Q = np.eye(NX)         # 过程噪声
        self.w_m = np.zeros(N_SIGMA)  # 均值权重
        self.w_c = np.zeros(N_SIGMA)  # 协方差权重
        self.sigma_pred = np.zeros((NX, N_SIGMA))
        self.z_pred = np.zeros(NZ)
        self.S = np.eye(NZ)
        self.K = np.zeros((NX, NZ))
        self.z_sigma = np.zeros((NZ, N_SIGMA))
        self.target_plate = 0
        self.likelihood = 1.0

class ImmUkfPython:
    """Python 版 IMM-UKF (忠实复现 C++ imm_ukf_tracker.cpp)"""

    def __init__(self, params: UKFParams):
        self.params = params
        self.filters = [UKFFilter() for _ in range(N_MODELS)]
        self.model_probs = np.array([params.init_prob_cv, 1.0 - params.init_prob_cv])
        self.fused_x = np.zeros(NX)
        self.fused_P = np.eye(NX)
        self.last_t = 0.0
        self.initialized = False

        # 马尔可夫转移矩阵
        self.markov = np.array([
            [params.markov_00, 1.0 - params.markov_00],
            [1.0 - params.markov_11, params.markov_11]
        ])

        # 初始化
        for f in self.filters:
            self._compute_weights(f)
            self._build_Q(f)

    def _build_Q(self, f: UKFFilter):
        p = self.params
        f.Q = np.diag([
            p.q_pos**2, p.q_pos**2, p.q_pos**2,
            p.q_vel**2, p.q_vel**2, p.q_vel**2,
            p.q_r**2, p.q_phi**2, p.q_omega**2
        ])

    def _compute_weights(self, f: UKFFilter):
        a, b, k = self.params.alpha, self.params.beta, self.params.kappa
        lam = a * a * (NX + k) - NX
        f.w_m[0] = lam / (NX + lam)
        f.w_c[0] = lam / (NX + lam) + (1.0 - a*a + b)
        w = 1.0 / (2.0 * (NX + lam))
        f.w_m[1:] = w
        f.w_c[1:] = w

    @staticmethod
    def _nearest_pd(A):
        """Find the nearest positive-definite matrix (Higham 2002)"""
        B = (A + A.T) / 2.0
        _, S, V = np.linalg.svd(B)
        H = V.T @ np.diag(S) @ V
        A2 = (B + H) / 2.0
        A3 = (A2 + A2.T) / 2.0
        if np.all(np.linalg.eigvalsh(A3) > 0):
            return A3
        # Fallback: eigenvalue clamping
        eigvals, eigvecs = np.linalg.eigh(A3)
        eigvals = np.maximum(eigvals, 1e-6)
        return eigvecs @ np.diag(eigvals) @ eigvecs.T

    def _generate_sigma(self, x, P):
        a, k = self.params.alpha, self.params.kappa
        lam = a * a * (NX + k) - NX
        scale = math.sqrt(NX + lam)

        # Cholesky with robust PD enforcement
        P_safe = (P + P.T) / 2.0  # ensure symmetric
        try:
            L = np.linalg.cholesky(P_safe)
        except np.linalg.LinAlgError:
            # Try jitter first
            for j in [1e-6, 1e-5, 1e-4, 1e-3, 1e-2]:
                try:
                    L = np.linalg.cholesky(P_safe + j * np.eye(NX))
                    break
                except np.linalg.LinAlgError:
                    continue
            else:
                # Last resort: nearest PD
                P_safe = self._nearest_pd(P_safe)
                L = np.linalg.cholesky(P_safe)

        sigma = np.zeros((NX, N_SIGMA))
        sigma[:, 0] = x
        for i in range(NX):
            sigma[:, i+1]      = x + scale * L[:, i]
            sigma[:, i+1+NX]   = x - scale * L[:, i]
        return sigma

    # ====== 过程模型 ======

    @staticmethod
    def _cv_model(x, dt):
        x_new = x.copy()
        x_new[0] += x[3] * dt
        x_new[1] += x[4] * dt
        x_new[2] += x[5] * dt
        x_new[7] += x[8] * dt
        x_new[7] = normalize_angle(x_new[7])
        return x_new

    @staticmethod
    def _ctrv_model(x, dt):
        x_new = x.copy()
        vx, vy, omega = x[3], x[4], x[8]
        speed = math.sqrt(vx*vx + vy*vy)

        x_new[2] += x[5] * dt
        x_new[6] = x[6]

        if abs(omega) > 1e-6:
            heading = math.atan2(vy, vx)
            new_heading = heading + omega * dt
            ratio = speed / omega
            x_new[0] = x[0] + ratio * (math.sin(new_heading) - math.sin(heading))
            x_new[1] = x[1] + ratio * (-math.cos(new_heading) + math.cos(heading))
            x_new[3] = speed * math.cos(new_heading)
            x_new[4] = speed * math.sin(new_heading)
        else:
            x_new[0] = x[0] + vx * dt
            x_new[1] = x[1] + vy * dt

        x_new[7] = normalize_angle(x[7] + omega * dt)
        x_new[8] = x[8]
        return x_new

    # ====== 观测模型 ======

    @staticmethod
    def _get_all_plates(x):
        r = abs(x[6])
        plates = []
        for i in range(N_PLATES):
            angle = x[7] + i * DPHI
            plates.append(np.array([
                x[0] + r * math.cos(angle),
                x[1] + r * math.sin(angle),
                x[2]
            ]))
        return plates

    @staticmethod
    def _observation_model(x):
        r = abs(x[6])
        if r < 1e-4:
            return x[:3].copy()

        dir_to_cam = math.atan2(-x[1], -x[0])
        best_k = 0
        best_diff = abs(normalize_angle(x[7] - dir_to_cam))
        for k in range(1, N_PLATES):
            diff = abs(normalize_angle(x[7] + k * DPHI - dir_to_cam))
            if diff < best_diff:
                best_diff = diff
                best_k = k

        phi_obs = x[7] + best_k * DPHI
        return np.array([
            x[0] + r * math.cos(phi_obs),
            x[1] + r * math.sin(phi_obs),
            x[2]
        ])

    # ====== UKF Predict ======

    def _ukf_predict(self, f: UKFFilter, dt, model_idx):
        # Step 1: Generate sigma points
        sigma = self._generate_sigma(f.x, f.P)

        # Step 2: Propagate through process model
        for i in range(N_SIGMA):
            if model_idx == 0:
                f.sigma_pred[:, i] = self._cv_model(sigma[:, i], dt)
            else:
                f.sigma_pred[:, i] = self._ctrv_model(sigma[:, i], dt)
            f.sigma_pred[7, i] = normalize_angle(f.sigma_pred[7, i])

        # Step 3: Predicted mean (arithmetic for non-angle, circular for phi)
        x_pred = f.sigma_pred @ f.w_m  # (NX, NS) @ (NS,) → (NX,)

        # Circular mean for phi(7)
        sin_sum = np.sum(f.w_m * np.sin(f.sigma_pred[7, :]))
        cos_sum = np.sum(f.w_m * np.cos(f.sigma_pred[7, :]))
        x_pred[7] = normalize_angle(math.atan2(sin_sum, cos_sum))

        # Step 4: Predicted covariance
        P_pred = np.zeros((NX, NX))
        for i in range(N_SIGMA):
            diff = f.sigma_pred[:, i] - x_pred
            diff[7] = normalize_angle(diff[7])
            P_pred += f.w_c[i] * np.outer(diff, diff)
        P_pred += f.Q * dt

        f.x = x_pred
        f.P = P_pred

        # Step 5: [P0-Fix 2] Unified plate selection + observation sigma
        target_plate = 0
        r_pred = abs(x_pred[6])
        if r_pred >= 1e-4:
            dir_to_cam = math.atan2(-x_pred[1], -x_pred[0])
            best_diff = abs(normalize_angle(x_pred[7] - dir_to_cam))
            for k in range(1, N_PLATES):
                diff = abs(normalize_angle(x_pred[7] + k * DPHI - dir_to_cam))
                if diff < best_diff:
                    best_diff = diff
                    target_plate = k
        f.target_plate = target_plate

        for i in range(N_SIGMA):
            r_i = abs(f.sigma_pred[6, i])
            angle_i = f.sigma_pred[7, i] + target_plate * DPHI
            f.z_sigma[:, i] = np.array([
                f.sigma_pred[0, i] + r_i * math.cos(angle_i),
                f.sigma_pred[1, i] + r_i * math.sin(angle_i),
                f.sigma_pred[2, i]
            ])

        # Step 6: Predicted observation
        f.z_pred = f.z_sigma @ f.w_m  # (NZ, NS) @ (NS,) → (NZ,)

        # Step 7: Innovation covariance S
        S = np.zeros((NZ, NZ))
        for i in range(N_SIGMA):
            zdiff = f.z_sigma[:, i] - f.z_pred
            S += f.w_c[i] * np.outer(zdiff, zdiff)
        S += (self.params.r_pos ** 2) * np.eye(NZ)
        f.S = S

        # Step 8: Cross covariance & Kalman gain
        T = np.zeros((NX, NZ))
        for i in range(N_SIGMA):
            xdiff = f.sigma_pred[:, i] - f.x
            xdiff[7] = normalize_angle(xdiff[7])
            zdiff = f.z_sigma[:, i] - f.z_pred
            T += f.w_c[i] * np.outer(xdiff, zdiff)
        f.K = T @ np.linalg.inv(S)

    # ====== UKF Update ======

    def _ukf_update(self, f: UKFFilter, z):
        # Step 1: [P1-Fix] Mahalanobis plate matching
        S_inv_est = np.linalg.inv(f.S)
        plates = self._get_all_plates(f.x)
        best_plate = f.target_plate
        min_mahal = (z - f.z_pred) @ S_inv_est @ (z - f.z_pred)

        for k in range(N_PLATES):
            if k == f.target_plate:
                continue
            d = (z - plates[k]) @ S_inv_est @ (z - plates[k])
            if d < min_mahal:
                min_mahal = d
                best_plate = k

        # Step 2: If plate changed, recompute z_sigma/z_pred/S/K
        if best_plate != f.target_plate:
            f.target_plate = best_plate
            for i in range(N_SIGMA):
                r_i = abs(f.sigma_pred[6, i])
                angle_i = f.sigma_pred[7, i] + best_plate * DPHI
                f.z_sigma[:, i] = np.array([
                    f.sigma_pred[0, i] + r_i * math.cos(angle_i),
                    f.sigma_pred[1, i] + r_i * math.sin(angle_i),
                    f.sigma_pred[2, i]
                ])

            f.z_pred = f.z_sigma @ f.w_m  # (NZ, NS) @ (NS,) → (NZ,)

            S = np.zeros((NZ, NZ))
            for i in range(N_SIGMA):
                zdiff = f.z_sigma[:, i] - f.z_pred
                S += f.w_c[i] * np.outer(zdiff, zdiff)
            S += (self.params.r_pos ** 2) * np.eye(NZ)
            f.S = S

            T = np.zeros((NX, NZ))
            for i in range(N_SIGMA):
                xdiff = f.sigma_pred[:, i] - f.x
                xdiff[7] = normalize_angle(xdiff[7])
                zdiff = f.z_sigma[:, i] - f.z_pred
                T += f.w_c[i] * np.outer(xdiff, zdiff)
            f.K = T @ np.linalg.inv(S)

        # Step 3: Innovation
        y = z - f.z_pred

        # Step 4: State update
        x_upd = f.x + f.K @ y
        x_upd[7] = normalize_angle(x_upd[7])

        # Step 5: Covariance update
        P_upd = f.P - f.K @ f.S @ f.K.T
        f.P = 0.5 * (P_upd + P_upd.T)  # enforce symmetry
        f.x = x_upd

        # Step 6: Radius constraint r >= 0
        if f.x[6] < 0:
            f.x[6] = abs(f.x[6])
            f.x[7] = normalize_angle(f.x[7] + PI)

        # Step 7: Likelihood
        S_inv = np.linalg.inv(f.S)
        exponent = -0.5 * (y @ S_inv @ y)
        sign, log_det = np.linalg.slogdet(f.S)
        log_likelihood = -0.5 * (NZ * math.log(2.0 * PI) + log_det) + exponent
        f.likelihood = math.exp(max(log_likelihood, -500.0))

    # ====== IMM Steps ======

    def _imm_interaction(self):
        c = np.zeros(N_MODELS)
        for j in range(N_MODELS):
            c[j] = np.sum(self.markov[:, j] * self.model_probs)

        c = np.maximum(c, 1e-15)
        mu_mix = np.zeros((N_MODELS, N_MODELS))
        for i in range(N_MODELS):
            for j in range(N_MODELS):
                mu_mix[i, j] = self.markov[i, j] * self.model_probs[i] / c[j]

        for j in range(N_MODELS):
            x_mixed = np.zeros(NX)
            for i in range(N_MODELS):
                x_mixed += mu_mix[i, j] * self.filters[i].x

            # Circular mean for phi
            sin_sum = sum(mu_mix[i, j] * math.sin(self.filters[i].x[7]) for i in range(N_MODELS))
            cos_sum = sum(mu_mix[i, j] * math.cos(self.filters[i].x[7]) for i in range(N_MODELS))
            x_mixed[7] = normalize_angle(math.atan2(sin_sum, cos_sum))

            P_mixed = np.zeros((NX, NX))
            for i in range(N_MODELS):
                diff = self.filters[i].x - x_mixed
                diff[7] = normalize_angle(diff[7])
                P_mixed += mu_mix[i, j] * (self.filters[i].P + np.outer(diff, diff))

            self.filters[j].x = x_mixed
            self.filters[j].P = P_mixed

    def _imm_update_probs(self):
        c = np.zeros(N_MODELS)
        for j in range(N_MODELS):
            c[j] = np.sum(self.markov[:, j] * self.model_probs)

        unnorm = np.array([self.filters[j].likelihood * c[j] for j in range(N_MODELS)])
        unnorm = np.maximum(unnorm, 1e-300)
        total = unnorm.sum()
        if total < 1e-300:
            total = 1.0
        self.model_probs = unnorm / total

    def _imm_fusion(self):
        self.fused_x = np.zeros(NX)
        for j in range(N_MODELS):
            self.fused_x += self.model_probs[j] * self.filters[j].x

        sin_sum = sum(self.model_probs[j] * math.sin(self.filters[j].x[7]) for j in range(N_MODELS))
        cos_sum = sum(self.model_probs[j] * math.cos(self.filters[j].x[7]) for j in range(N_MODELS))
        self.fused_x[7] = normalize_angle(math.atan2(sin_sum, cos_sum))

        self.fused_P = np.zeros((NX, NX))
        for j in range(N_MODELS):
            diff = self.filters[j].x - self.fused_x
            diff[7] = normalize_angle(diff[7])
            self.fused_P += self.model_probs[j] * (self.filters[j].P + np.outer(diff, diff))

    # ====== Public API ======

    def init(self, pos, timestamp):
        """[P2-Fix 1] 全量初始化: 位置/速度/协方差/模型概率/似然度/板号"""
        for f in self.filters:
            # 强制覆盖: 位置=观测值, 速度=0, 半径=默认值
            f.x = np.zeros(NX)
            f.x[:3] = pos
            f.x[6] = 0.2  # default radius

            # 位置小方差 (已锁定), 速度大方差 (待估计)
            f.P = np.eye(NX)
            f.P[:3, :3] *= 0.01
            f.P[3:6, 3:6] *= 1.0
            f.P[6, 6] = 0.04
            f.P[7, 7] = 1.0
            f.P[8, 8] = 1.0

            # [P2-Fix 1] 重置辅助状态
            f.target_plate = 0
            f.likelihood = 1.0

        # [P2-Fix 1] 重置 IMM 模型概率为初始值
        self.model_probs = np.array([self.params.init_prob_cv,
                                     1.0 - self.params.init_prob_cv])

        self.fused_x = self.filters[0].x.copy()
        self.fused_P = self.filters[0].P.copy()
        self.last_t = timestamp
        self.initialized = True

    def update(self, z, timestamp):
        if not self.initialized:
            self.init(z, timestamp)
            return

        dt = timestamp - self.last_t

        # [P2-Fix 2] 幽灵时间戳拦截: dt > 0.5s → 强制 re-init
        if dt > 0.5:
            self.init(z, timestamp)
            return

        if dt <= 0:
            dt = 0.001  # 重复/倒序时间戳保护
        self.last_t = timestamp

        self._imm_interaction()
        # Filter: predict + update
        for m in range(N_MODELS):
            self._ukf_predict(self.filters[m], dt, m)
            self._ukf_update(self.filters[m], z)
        self._imm_update_probs()
        self._imm_fusion()


# ============================================================================
# 运行单次 UKF 回放
# ============================================================================

def run_ukf_replay(observations: List[Tuple[float, np.ndarray]],
                   params: UKFParams) -> Dict:
    """
    给定观测序列和参数，运行 IMM-UKF 回放。
    返回:
        times:       观测时间序列
        z_raw:       原始观测 (NX3)
        x_filtered:  滤波后中心位置 (NX3)
        x_vel:       滤波速度 (NX3)
        model_probs: 模型概率 (NX2)
    """
    tracker = ImmUkfPython(params)

    n = len(observations)
    times = np.zeros(n)
    z_raw = np.zeros((n, 3))
    x_filtered = np.zeros((n, 3))
    x_vel = np.zeros((n, 3))
    model_probs = np.zeros((n, 2))

    for i, (t, xyz) in enumerate(observations):
        tracker.update(xyz, t)
        times[i] = t
        z_raw[i] = xyz
        x_filtered[i] = tracker.fused_x[:3]
        x_vel[i] = tracker.fused_x[3:6]
        model_probs[i] = tracker.model_probs.copy()

    return {
        'times': times,
        'z_raw': z_raw,
        'x_filtered': x_filtered,
        'x_vel': x_vel,
        'model_probs': model_probs,
    }


# ============================================================================
# 代价函数
# ============================================================================

def compute_cost(result: Dict, w_lag: float = 1.0, w_jitter: float = 1.0,
                 lag_dt: float = 0.05) -> Tuple[float, float, float]:
    """
    J = w_lag * Lag + w_jitter * Jitter

    Lag:   预测位置与未来 lag_dt 秒后真实观测的 RMSE (相位延迟惩罚)
    Jitter: 预测速度的方差 (高频抖动惩罚)

    Returns: (J_total, lag_rmse, jitter_std)
    """
    times = result['times']
    x_filt = result['x_filtered']
    x_vel = result['x_vel']
    z_raw = result['z_raw']
    n = len(times)

    # ---- Lag ----
    lag_errors = []
    for i in range(n):
        t_target = times[i] + lag_dt
        # 找最近的未来观测
        j = np.searchsorted(times, t_target)
        if j >= n:
            break
        # 当前滤波位置 vs 未来观测
        err = np.linalg.norm(x_filt[i] - z_raw[j])
        lag_errors.append(err)

    lag_rmse = np.sqrt(np.mean(np.array(lag_errors)**2)) if lag_errors else 0.0

    # ---- Jitter ----
    # 速度的标准差 (高抖动 = 速度方差大 = 非物理)
    if n > 2:
        vel_mag = np.sqrt(np.sum(x_vel**2, axis=1))
        jitter_std = np.std(vel_mag)
    else:
        jitter_std = 0.0

    J = w_lag * lag_rmse + w_jitter * jitter_std
    return J, lag_rmse, jitter_std


# ============================================================================
# 网格搜索
# ============================================================================

def grid_search(observations: List[Tuple[float, np.ndarray]],
                quick: bool = False, fine: bool = False):
    """网格搜索最优 Q/R 参数"""

    if quick:
        q_pos_range   = [0.01, 0.1]
        q_vel_range   = [0.1, 1.0]
        q_omega_range = [0.05, 0.5]
        r_pos_range   = [0.01, 0.1]
    elif fine:
        q_pos_range   = np.linspace(0.02, 0.08, 4)
        q_vel_range   = np.linspace(0.2, 0.8, 4)
        q_omega_range = np.linspace(0.05, 0.3, 4)
        r_pos_range   = np.linspace(0.01, 0.08, 4)
    else:
        q_pos_range   = [0.01, 0.03, 0.05, 0.1, 0.2]
        q_vel_range   = [0.1, 0.3, 0.5, 1.0, 2.0]
        q_omega_range = [0.05, 0.1, 0.2, 0.5, 1.0]
        r_pos_range   = [0.01, 0.03, 0.05, 0.1, 0.2]

    total_combos = len(q_pos_range) * len(q_vel_range) * len(q_omega_range) * len(r_pos_range)
    print(f"\n{'='*70}")
    print(f"  网格搜索: {total_combos} 种参数组合")
    print(f"{'='*70}")

    results = []
    combo = 0
    t_start = time.time()

    for q_pos in q_pos_range:
        for q_vel in q_vel_range:
            for q_omega in q_omega_range:
                for r_pos in r_pos_range:
                    combo += 1
                    params = UKFParams(
                        q_pos=q_pos, q_vel=q_vel, q_omega=q_omega,
                        r_pos=r_pos
                    )

                    try:
                        result = run_ukf_replay(observations, params)
                        J, lag, jitter = compute_cost(result)
                        results.append({
                            'params': params,
                            'J': J, 'lag': lag, 'jitter': jitter,
                            'result': result
                        })
                    except Exception as e:
                        # 数值不稳定 → 给极差分数
                        results.append({
                            'params': params,
                            'J': 999.0, 'lag': 999.0, 'jitter': 999.0,
                            'result': None
                        })

                    if combo % max(1, total_combos // 20) == 0 or combo == total_combos:
                        elapsed = time.time() - t_start
                        pct = 100.0 * combo / total_combos
                        print(f"  [{pct:5.1f}%] {combo}/{total_combos} | "
                              f"{elapsed:.1f}s elapsed | "
                              f"Best J so far: {min(r['J'] for r in results):.4f}")

    # 排序
    results.sort(key=lambda r: r['J'])

    # 输出 Top 3
    print(f"\n{'='*70}")
    print(f"  搜索完成! Top 3 参数组合:")
    print(f"{'='*70}")
    for rank, r in enumerate(results[:3], 1):
        p = r['params']
        print(f"\n  #{rank}  J={r['J']:.4f}  (Lag={r['lag']:.4f}, Jitter={r['jitter']:.4f})")
        print(f"       q_pos={p.q_pos:.4f}  q_vel={p.q_vel:.4f}  "
              f"q_omega={p.q_omega:.4f}  r_pos={p.r_pos:.4f}")

    return results


# ============================================================================
# 可视化
# ============================================================================

def plot_results(observations, results: list, output_dir: str = "/tmp/ukf_tune"):
    """绘制 3 张对比图"""
    os.makedirs(output_dir, exist_ok=True)

    times = np.array([o[0] for o in observations])
    z_raw = np.array([o[1] for o in observations])
    # 相对时间 (秒)
    t_rel = times - times[0]

    # 取 Top 3 (过滤掉 J=999 的数值爆炸情况)
    valid = [r for r in results if r['J'] < 100]
    if len(valid) < 1:
        print("⚠️ 无有效结果可绘制!")
        return

    top3 = valid[:min(3, len(valid))]
    colors = ['#e74c3c', '#2ecc71', '#3498db']  # 红/绿/蓝

    # ========== 图 1: 位置对比 ==========
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    labels = ['X (m)', 'Y (m)', 'Z (m)']

    for ax_i, (ax, lbl) in enumerate(zip(axes, labels)):
        # 原始观测
        ax.plot(t_rel, z_raw[:, ax_i], 'k.', markersize=1, alpha=0.3, label='Raw obs')

        # Top 3 滤波结果
        for rank, r in enumerate(top3):
            p = r['params']
            res = r['result']
            ax.plot(t_rel, res['x_filtered'][:, ax_i],
                    color=colors[rank], linewidth=1.2, alpha=0.8,
                    label=f"#{rank+1} J={r['J']:.3f}")

        ax.set_ylabel(lbl)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('IMM-UKF Position Tracking — Parameter Comparison', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, '1_position_comparison.png'), dpi=150)
    print(f"  图1 已保存: {output_dir}/1_position_comparison.png")

    # ========== 图 2: 速度波形 ==========
    fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    vel_labels = ['Vx (m/s)', 'Vy (m/s)', 'Vz (m/s)']

    for ax_i, (ax, lbl) in enumerate(zip(axes, vel_labels)):
        for rank, r in enumerate(top3):
            res = r['result']
            ax.plot(t_rel, res['x_vel'][:, ax_i],
                    color=colors[rank], linewidth=1.0, alpha=0.8,
                    label=f"#{rank+1}")

        ax.set_ylabel(lbl)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('IMM-UKF Velocity Estimation', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, '2_velocity_comparison.png'), dpi=150)
    print(f"  图2 已保存: {output_dir}/2_velocity_comparison.png")

    # ========== 图 3: IMM 模型概率 ==========
    fig, ax = plt.subplots(figsize=(14, 5))

    for rank, r in enumerate(top3):
        res = r['result']
        ax.plot(t_rel, res['model_probs'][:, 0],  # P(CV)
                color=colors[rank], linewidth=1.0, alpha=0.8,
                label=f"#{rank+1} P(CV)")
        ax.plot(t_rel, res['model_probs'][:, 1],  # P(CTRV)
                color=colors[rank], linewidth=1.0, alpha=0.8, linestyle='--')

    ax.set_ylabel('Model Probability')
    ax.set_xlabel('Time (s)')
    ax.set_ylim(-0.05, 1.05)
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title('IMM Model Probabilities (solid=P_CV, dashed=P_CTRV)')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, '3_model_probabilities.png'), dpi=150)
    print(f"  图3 已保存: {output_dir}/3_model_probabilities.png")

    # ========== 热力图 (Lag vs Jitter 散点) ==========
    fig, ax = plt.subplots(figsize=(8, 6))
    js = [r['J'] for r in valid]
    lags = [r['lag'] for r in valid]
    jitters = [r['jitter'] for r in valid]
    sc = ax.scatter(lags, jitters, c=js, cmap='viridis_r', s=20, alpha=0.7)
    plt.colorbar(sc, label='Cost J')
    # 标记 Top 3
    for rank, r in enumerate(top3):
        ax.scatter(r['lag'], r['jitter'], c=colors[rank], s=100,
                   edgecolors='black', linewidth=2, zorder=5,
                   label=f"#{rank+1} J={r['J']:.3f}")
    ax.set_xlabel('Lag RMSE (m) ← lower = less delay')
    ax.set_ylabel('Velocity Jitter (m/s) ← lower = smoother')
    ax.set_title('Parameter Space: Lag vs Jitter')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, '4_lag_vs_jitter.png'), dpi=150)
    print(f"  图4 已保存: {output_dir}/4_lag_vs_jitter.png")


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='IMM-UKF 参数寻优器')
    parser.add_argument('--bag', type=str, default=BAG_DB, help='Bag db3 文件路径')
    parser.add_argument('--quick', action='store_true', help='粗搜索 (少量组合)')
    parser.add_argument('--fine', action='store_true', help='精搜索 (中量组合)')
    parser.add_argument('--output', type=str, default='/tmp/ukf_tune', help='输出目录')
    parser.add_argument('--w-lag', type=float, default=1.0, help='Lag 惩罚权重')
    parser.add_argument('--w-jitter', type=float, default=1.0, help='Jitter 惩罚权重')
    args = parser.parse_args()

    print("=" * 70)
    print("  IMM-UKF 数据驱动参数寻优器")
    print("=" * 70)

    # 1. 提取观测
    print(f"\n[1] 从 Bag 提取观测数据...")
    print(f"    Bag: {args.bag}")
    observations = extract_observations(args.bag)
    if len(observations) < 10:
        print(f"  ⚠️ 观测数据太少 ({len(observations)} 条)!")
        sys.exit(1)

    times = [o[0] for o in observations]
    dur = times[-1] - times[0]
    print(f"    观测数: {len(observations)}, 时长: {dur:.2f}s, 频率: {len(observations)/dur:.1f} Hz")

    xyzs = np.array([o[1] for o in observations])
    print(f"    X: {xyzs[:,0].min():.3f} ~ {xyzs[:,0].max():.3f} m")
    print(f"    Y: {xyzs[:,1].min():.3f} ~ {xyzs[:,1].max():.3f} m")
    print(f"    Z: {xyzs[:,2].min():.3f} ~ {xyzs[:,2].max():.3f} m")

    # 2. 先用默认参数跑一次 baseline
    print(f"\n[2] Baseline (默认参数)...")
    default_params = UKFParams()
    baseline = run_ukf_replay(observations, default_params)
    J_base, lag_base, jit_base = compute_cost(baseline, args.w_lag, args.w_jitter)
    print(f"    J={J_base:.4f}  Lag={lag_base:.4f}  Jitter={jit_base:.4f}")

    # 3. 网格搜索
    print(f"\n[3] 网格搜索...")
    results = grid_search(observations, quick=args.quick, fine=args.fine)

    # 4. 可视化
    print(f"\n[4] 生成可视化图表...")
    plot_results(observations, results, args.output)

    # 5. 输出最优参数的 YAML 片段
    best = results[0]
    bp = best['params']
    print(f"\n{'='*70}")
    print(f"  🏆 最优参数 (可直接写入 params.yaml):")
    print(f"{'='*70}")
    print(f"""
# IMM-UKF 最优参数 (自动寻优 J={best['J']:.4f})
ukf:
  alpha: {bp.alpha}
  beta: {bp.beta}
  kappa: {bp.kappa}
  q_pos: {bp.q_pos}
  q_vel: {bp.q_vel}
  q_r: {bp.q_r}
  q_phi: {bp.q_phi}
  q_omega: {bp.q_omega}
  r_pos: {bp.r_pos}
  markov_00: {bp.markov_00}
  markov_11: {bp.markov_11}
""")

    # Baseline vs Best 对比
    print(f"  Baseline: J={J_base:.4f}  Lag={lag_base:.4f}  Jitter={jit_base:.4f}")
    print(f"  Best:     J={best['J']:.4f}  Lag={best['lag']:.4f}  Jitter={best['jitter']:.4f}")
    improvement = (J_base - best['J']) / max(J_base, 1e-6) * 100
    print(f"  改善: {improvement:.1f}%")

    print(f"\n图表已保存到: {args.output}/")
    print("完成!")


if __name__ == '__main__':
    main()