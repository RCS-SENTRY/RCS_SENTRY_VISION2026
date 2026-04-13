# XMU RCS Sentry — RMUC 自瞄系统 (ROS 2 Humble)

> **XMU RM 视觉组 · 2026 赛季**
> 哨兵全自动自瞄系统：海康驱动 → YOLO 检测 → IMM-UKF 追踪 → Aimer 火控 → 串口下发

---

## 目录

| # | 章节 | 内容 |
|---|------|------|
| 1 | [架构总览](#1-架构总览) | 节点拓扑、数据流、包职责 |
| 2 | [快速编译与点火](#2-快速编译与点火-build--bringup) | colcon build、一键启动、单机调试 |
| 3 | [标定与准星校准](#3-标定与准星校准-calibration) | 相机内参、弹道 LUT、pitch 补偿 |
| 4 | [实车数据采集](#4-实车数据采集-rosbag-recording) | 录制命令、必需话题、靶车机动规范 |
| 5 | [自动化炼丹调参](#5-自动化炼丹调参-auto-tuning) | tune_ukf.py 用法、波形判读、参数写入 |
| 6 | [参数速查表](#6-参数速查表) | params.yaml 关键字段 |

---

## 1. 架构总览

```
┌─────────────┐    ┌──────────────┐    ┌──────────────┐    ┌─────────────┐
│ rm_hik_driver│───>│  rm_vision  │───>│  rm_autoaim  │───>│ rm_hw_bridge│
│  (海康相机)   │   │  (YOLO检测)   │    │ (追踪+火控)   │    │  (串口桥)   │
└─────────────┘    └──────────────┘    └──────────────┘    └──────┬──────┘
     图像                装甲板               云台指令            UART
   /camera/image    /detector/armors     /gimbal_cmd         ↓ ↑
                                                               STM32
```

### 包职责

| 包名 | 职责 | 关键源文件 |
|------|------|-----------|
| `rm_interfaces` | 自定义消息定义 (`GimbalCmd`, `GimbalStatus`, `ArmorDetection` 等) | `msg/*.msg` |
| `rm_hik_driver` | 海康工业相机驱动，发布 `sensor_msgs/Image` | `src/hik_camera_node.cpp` |
| `rm_vision` | OpenVINO YOLO 推理，输出装甲板检测 | `src/yolo_detector.cpp`, `src/vision_detector_node.cpp` |
| `rm_autoaim` | IMM-UKF 追踪 + Aimer 火控解算 | `src/imm_ukf_tracker.cpp`, `src/autoaim_node.cpp` |
| `rm_hw_bridge` | 串口协议翻译 (ROS 2 ↔ STM32) | `src/hw_bridge_node.cpp` |
| `rm_bringup` | Launch 一键启动编排 | `launch/sentry_bringup.launch.py` |

### IMM-UKF 追踪器核心

状态向量 **x** ∈ ℝ⁹：
```
[x, y, z, vx, vy, vz, r, φ, ω]
 位置   速度    半径  相位  角速度
```

两个运动模型通过 IMM 交互：
- **CV** (Constant Velocity) — 匀速直线，追踪平移目标
- **CTRV** (Constant Turn Rate and Velocity) — 匀速转弯，专克小陀螺

---

## 2. 快速编译与点火 (Build & Bringup)

### 2.1 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- OpenVINO 2024.x (推理引擎)
- Eigen 3, OpenCV 4, Boost.Asio (串口)
- 海康 MVS SDK (相机驱动)

### 2.2 编译

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY

# 清理旧产物 (可选但推荐)
rm -rf build/ install/ log/

# 编译 (symlink-install 方便改参数后免重编)
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

> **常见问题**: 如果报 `rm_interfaces` 找不到，确认 `colcon build` 没有 `--packages-up-to` 漏掉接口包。

### 2.3 一键启动

```bash
# ===== 实车模式 (串口+相机+视觉+自瞄 全链路) =====
ros2 launch rm_bringup sentry_bringup.launch.py

# 启动时序:
#   T+0.0s: rm_hw_bridge (串口连接 STM32)
#   T+2.0s: rm_hik_driver (等待串口就绪后再开相机)
#   T+4.0s: rm_vision + rm_autoaim (同时启动)
```

### 2.4 单机无车调试模式 

不需要实车、不需要串口、**只要有一台海康相机或 Bag 回放就能调**：

```bash
# use_serial=false: 跳过 rm_hw_bridge, 相机立即启动
ros2 launch rm_bringup sentry_bringup.launch.py use_serial:=false
```

启动时序简化为：
```
T+0.0s: rm_hik_driver (立即启动)
T+2.0s: rm_vision + rm_autoaim
```

### 2.5 其他 Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_serial` | `true` | 是否启动串口节点 |
| `serial_device` | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | `460800` | 串口波特率 |
| `color_ignore` | `1` | 忽略敌方颜色: 0=红, 1=蓝, -1=不忽略 |
| `target_color` | `red` | 目标颜色: red / blue / all |
| `publish_debug_image` | `false` | 是否发布调试图像 |
| `model_path` | `model/yolo11.xml` | OpenVINO 模型路径 (.xml) |

---

## 3. 标定与准星校准 (Calibration)

### 3.1 相机内参

使用 OpenCV 棋盘格标定，将结果填入 `rm_autoaim/config/params.yaml`：

```yaml
autoaim_node:
  ros__parameters:
    camera_matrix: [
      fx,  0.0, cx,
      0.0, fy,  cy,
      0.0, 0.0, 1.0
    ]
    dist_coeffs: [k1, k2, p1, p2, k3]
```

> **提示**: 标定精度直接影响 PnP 解算的距离和角度。建议使用 9×6 棋盘，采集 ≥30 张不同位姿的图片。

### 3.2 相机-云台外参

```yaml
    # 旋转矩阵 (相机坐标系 → 云台坐标系)
    r_cam_to_gimbal: [
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    ]
    # 平移向量 (单位: m)
    t_cam_to_gimbal: [0.0, 0.0, 0.0]
```

### 3.3 弹道补偿 LUT (工程查表法) 

弹丸受重力和空气阻力影响，在不同距离下需要不同的 pitch 补偿量。
系统使用 **解析弹道模型 + 经验 LUT 修正** 的混合方案：

```
pitch_offset = k0 + k1 × d + k2 × d² + 解析弹道模型
```

其中 `d` 为目标距离 (m)，`k0/k1/k2` 为标定系数。

#### 标定流程

1. **准备**: 哨兵固定，目标装甲板放置在已知距离 (2m, 3m, 5m, 7m)
2. **记录**: 对每个距离手动调整 pitch 直到命中，记录「解算 pitch - 实际命中 pitch」的偏差
3. **拟合**: 用最小二乘法拟合 `Δpitch = k0 + k1*d + k2*d²`
4. **填入** `params.yaml`:

```yaml
    pitch_offset_k0: -0.005    # 基础偏移 (rad)
    pitch_offset_k1:  0.003    # 线性项
    pitch_offset_k2:  0.0008   # 二次项 (空气阻力)
```

> **实战经验**: 先在 5m 处调好 `k0`，然后逐步拉远到 7m 调 `k1`/`k2`。
> 如果弹速有变化 (如裁判系统限功率)，需要重新标定！

### 3.4 火控 Gate 参数

系统有三道火控门控，防止误开火：

| Gate | 参数 | 默认值 | 含义 |
|------|------|--------|------|
| Gate 1: 对齐容差 | `fire_tolerance` | 0.03 rad (≈1.7°) | 解算角度与云台反馈差值 |
| Gate 2: 相位窗口 | `armor_facing_tolerance` | 30° | 装甲板法线与枪管夹角 |
| Gate 2: CTRV 阈值 | `ctrv_threshold` | 0.5 | CTRV 概率高于此值才启用相位窗口 |
| Gate 3: 收敛保护 | `fire_min_frames` | 5 | 最少收敛帧数 |
| Gate 3: 距离保护 | `fire_max_distance` | 8.0 m | 最大开火距离 |

---

## 4. 实车数据采集 (ROSbag Recording)

### 4.1 标准录制命令

```bash
# 创建录制目录
mkdir -p ~/bag2 && cd ~/bag2

# 录制所有自瞄相关话题
ros2 bag record \
  -o autoaim_$(date +%Y%m%d_%H%M%S) \
  /detector/armors \            # ★ YOLO 检测结果 (必需)
  /autoaim/debug_world_points \ # ★ 追踪器输出的世界坐标 (调参核心)
  /autoaim/gimbal_cmd \         # 下发云台指令
  /autoaim/model_probabilities \ # IMM 模型概率
  /camera/image \               # 原始图像 (可选, 文件大)
  /diagnostics \                # 系统诊断
  /tf                           # TF 变换
```

> **最小调参集**: 只需要 `/detector/armors` 和 `/autoaim/debug_world_points` 两个话题即可运行 `tune_ukf.py`。

### 4.2 靶车机动规范 ⭐

**录制数据的品质直接决定了离线调参的天花板。** 请操作员按以下流程采集：

| 阶段 | 机动动作 | 持续时间 | 目的 |
|------|----------|----------|------|
| **A. 静止** | 靶车完全不动 | 5s | 建立 Q/R baseline，验证静态精度 |
| **B. 匀速平移** | 靶车匀速左右/前后移动 | 10s | 激发 CV 模型，标定 `q_vel` |
| **C. 急停急起** | 靶车突然启动/刹车 | 10s | 考验 tracker 的响应速度，标定 `q_pos` |
| **D. 小陀螺** | 靶车高速旋转 (≥2 rad/s) | 15s | **核心测试**，激发 CTRV 模型，标定 `q_omega` |
| **E. 混合** | 随机平移+旋转组合 | 15s | 验证 IMM 切换能力 |

> **关键指标**: 小陀螺阶段至少占总时长的 **30%**。如果 CTRV 数据不足，调参结果会严重偏向 CV 模型，导致实战中小陀螺追踪失败。

### 4.3 录制注意事项

- 确保录制前已完成标定 (内参+弹道)
- 弹速设置应与比赛一致
- 每次录制时长 **60-90 秒**为宜 (太短数据不够，太长可能有异常跳变)
- 录制结束后检查 bag 大小，确认没有丢帧

---

## 5. 自动化炼丹调参 (Auto-Tuning)

### 5.1 概述

`tune_ukf.py` 是一套纯离线的参数寻优工具：
- 从 ROSbag (SQLite3) 读取观测数据
- 用 Python **忠实复现** C++ IMM-UKF 算法
- 网格搜索最优 Q/R 参数组合
- 输出 4 张诊断图 + YAML 参数片段

### 5.2 运行

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY

# 粗搜索 (16 种组合, ~20s)
python3 tools/tune_ukf.py --quick

# 标准搜索 (625 种组合, ~5min)
python3 tools/tune_ukf.py

# 精搜索 (在粗搜索最优解附近细化, ~3min)
python3 tools/tune_ukf.py --fine

# 指定 bag 文件
python3 tools/tune_ukf.py --bag /path/to/your/bag.db3

# 调整代价函数权重
python3 tools/tune_ukf.py --w-lag 2.0 --w-jitter 1.0  # 更看重响应速度
python3 tools/tune_ukf.py --w-lag 1.0 --w-jitter 2.0  # 更看重平滑性
```

### 5.3 代价函数

```
J = w_lag × Lag + w_jitter × Jitter
```

| 分量 | 含义 | 物理意义 |
|------|------|----------|
| **Lag** | `X_pred(t)` vs `Z_raw(t+Δt)` 的 RMSE | 预测延迟 (越小 = 跟得越紧) |
| **Jitter** | 滤波速度 `‖v‖` 的标准差 | 高频抖动 (越小 = 越平滑) |

两者存在 trade-off：Lag 小 → Jitter 大 (过于灵敏)，反之亦然。最优参数在 Pareto 前沿上。

### 5.4 输出文件

所有图表保存在 `/tmp/ukf_tune/` (可通过 `--output` 修改)：

| 图表 | 文件名 | 看什么 |
|------|--------|--------|
| **位置对比** | `1_position_comparison.png` | X/Y/Z 三轴：黑点=原始观测，彩色线=滤波结果 |
| **速度波形** | `2_velocity_comparison.png` | Vx/Vy/Vz：越平滑越好，不应有高频毛刺 |
| **模型概率** | `3_model_probabilities.png` | P(CV) 和 P(CTRV) 随时间变化 |
| **参数空间** | `4_lag_vs_jitter.png` | 所有参数组合的 Lag-Jitter 散点图 |

### 5.5 如何看懂模型概率波形 

这是判断小陀螺追踪是否成功的**核心诊断工具**。

#### 正常波形 (追踪成功)

```
P(CV)   ████████          ████████          ████████
 1.0 ┤  ████████          ████████          ████████
     │  ████████          ████████          ████████
 0.5 ┤          ████████          ████████
     │          ████████          ████████
 0.0 ┤          ████████          ████████
        静止/平移   小陀螺   平移    小陀螺    平移
```

**判据**：
- 目标平移时：P(CV) ≈ 0.8~1.0 (CV 模型主导)
- 目标小陀螺时：P(CTRV) 快速跳到 ≈0.8~1.0 (CTRV 模型接管)
- **切换延迟 < 0.3s** (约 10 帧 @30Hz)
- 切换过程干脆利落，没有来回振荡

#### 异常波形 (需要调参)

```
P(CV)   ████████░░░░░░████████░░░░████████  ← 振荡!
 1.0 ┤  ████████░░░░░░████████░░░░████████
     │  ████████░░████████░░░░████████░░░░  ← 切换模糊!
 0.5 ┤          ████      ████      ████
     │          ████      ████      ████
 0.0 ┤
```

**诊断**：
| 现象 | 可能原因 | 修复方向 |
|------|----------|----------|
| 小陀螺时 P(CV)/P(CTRV) 来回振荡 | `q_omega` 太小，CTRV 跟不上转速 | 增大 `q_omega` |
| 始终 P(CV)≈1.0，CTRV 从未激活 | `markov_00` 太大或观测噪声 `r_pos` 太小 | 降低 `markov_00`，增大 `r_pos` |
| 切换延迟 >0.5s | `q_vel` 太大淹没了转弯信息 | 降低 `q_vel`，增大 `q_omega` |
| 静止时 P(CTRV) 不归零 | `markov_11` 太大 (CTRV 自锁) | 降低 `markov_11` |

### 5.6 将最优参数写入系统

脚本会在终端输出 YAML 片段，直接复制到 `rm_autoaim/config/params.yaml`：

```yaml
ukf:
  alpha: 0.1
  beta: 2.0
  kappa: 0.0
  q_pos: 0.1        # 位置过程噪声
  q_vel: 0.1        # 速度过程噪声
  q_r: 0.005        # 半径过程噪声
  q_phi: 0.05       # 相位过程噪声
  q_omega: 0.5      # 角速度过程噪声 (小陀螺关键参数!)
  r_pos: 0.01       # 观测噪声 (越小越信任观测)
  markov_00: 0.95   # CV→CV 转移概率
  markov_11: 0.95   # CTRV→CTRV 转移概率
```

写入后重新编译部署即可：

```bash
cd ~/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
colcon build --packages-select rm_autoaim --symlink-install
```

> **注意**: 使用了 `--symlink-install` 时，如果只修改 `config/params.yaml`，不需要重新编译，重启节点即可生效。

---

## 6. 参数速查表

### rm_autoaim (`config/params.yaml`)

```yaml
# ===== 相机 =====
camera_matrix: [...]        # 3×3 内参矩阵 (行优先)
dist_coeffs: [...]          # [k1, k2, p1, p2, k3]
r_cam_to_gimbal: [...]      # 3×3 外参旋转
t_cam_to_gimbal: [...]      # 3×1 外参平移

# ===== 物理 =====
gravity: 9.81               # 重力加速度
bullet_speed_default: 15.0  # 弹丸速度
fire_delay: 0.10            # 系统总延迟 (s)
pitch_invert: true          # 下位机 pitch 方向取反

# ===== 弹道补偿 =====
pitch_offset_k0: 0.0        # 基础偏移
pitch_offset_k1: 0.0        # 线性项
pitch_offset_k2: 0.0        # 二次项

# ===== 火控 =====
fire_tolerance: 0.03        # 对齐容差 (rad)
armor_facing_tolerance: 30  # 相位窗口 (deg)
ctrv_threshold: 0.5         # CTRV 概率阈值
fire_min_frames: 5          # 最少收敛帧
fire_max_distance: 8.0      # 最大开火距离 (m)
```

### rm_hw_bridge (`config/params.yaml`)

```yaml
serial_device: /dev/ttyUSB0  # 串口设备
baudrate: 460800             # 波特率
```

### rm_vision (`config/params.yaml`)

```yaml
model_path: "model/yolo11.xml"    # OpenVINO 模型
target_color: "red"               # 目标颜色
color_ignore: 1                   # 忽略颜色 (0=红, 1=蓝)
publish_debug_image: false        # 调试图像
```

---

## 附录: 常见问题排查

| 问题 | 排查步骤 |
|------|----------|
| 串口连接失败 | `ls /dev/ttyUSB*` 确认设备存在；`sudo chmod 666 /dev/ttyUSB0` |
| 相体打不开 | 检查 MVS SDK 是否安装；是否有其他进程占用相机 |
| 检测不到装甲板 | 检查 `target_color` 是否与敌方颜色匹配；开启 `publish_debug_image:=true` |
| 追踪发散 | 运行 `tune_ukf.py` 重新调参；检查 bag 中是否有大量丢帧 |
| 小陀螺打不准 | 查看模型概率图，确认 CTRV 能正常激活；增大 `q_omega` |
| Pitch 偏了 | 重新标定弹道 LUT (`pitch_offset_k0/k1/k2`) |
| 开机就开火 | 增大 `fire_min_frames` (收敛保护) |

---

## 贡献者

XMU RCS RM 视觉组

## License

MIT