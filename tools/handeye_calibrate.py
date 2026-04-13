#!/usr/bin/env python3
"""
handeye_calibrate.py — ROS 2 手眼标定求解工具 (修订版)

读取 handeye_capture.py 采集的 (图像 + IMU四元数) 数据,
调用 cv2.calibrateHandEye() 求解相机到云台的外参 (R, t),
输出可直接粘贴到 rm_autoaim/config/params.yaml 的格式。

关键设计:
  - 云台仅做旋转运动, 平移分量为零 → t_cam2gimbal 的估计可能不稳定
  - 旋转估计 R_cam2gimbal 通常是可靠的
  - 使用旋转矩阵正交性 ||R^T R - I|| 作为方法选择标准

用法:
  python3 tools/handeye_calibrate.py handeye_data/ --params-yaml rm_autoaim/config/params.yaml

前置:
  - 已用 handeye_capture.py 采集了 >=15 组数据
  - 需要相机内参 (从 params.yaml 读取或命令行指定)
"""

import os
import sys
import json
import math
import argparse

import cv2
import numpy as np


# =========================================================================
# 工具函数
# =========================================================================

def load_camera_matrix_from_yaml(yaml_path):
    """从 rm_autoaim/config/params.yaml 读取相机内参"""
    import yaml
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    params = config.get('autoaim_node', {}).get('ros__parameters', {})
    cm = params.get('camera_matrix', None)
    dc = params.get('dist_coeffs', None)

    if cm is None or dc is None:
        raise ValueError(f"无法从 {yaml_path} 读取 camera_matrix / dist_coeffs")

    camera_matrix = np.array(cm, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(dc, dtype=np.float64).reshape(1, -1)
    return camera_matrix, dist_coeffs


def quat_to_rotation_matrix(w, x, y, z):
    """四元数 (w,x,y,z) → 3×3 旋转矩阵 (体轴 ZYX)"""
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)    ],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)    ],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    return R


def rotation_matrix_to_euler_zyx(R):
    """旋转矩阵 → (yaw, pitch, roll) degrees (ZYX intrinsic convention)"""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll  = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = 0.0

    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))


def orthogonality_error(R):
    """旋转矩阵的正交性误差 ||R^T R - I||_F (越小越好, 理想值 0)"""
    return np.linalg.norm(R.T @ R - np.eye(3), 'fro')


def determinant_error(R):
    """行列式与 1 的偏差 |det(R) - 1|"""
    return abs(np.linalg.det(R) - 1.0)


def is_valid_rotation(R, tol=0.01):
    """检查 R 是否为有效的旋转矩阵"""
    return orthogonality_error(R) < tol and determinant_error(R) < tol * 0.1


# =========================================================================
# 主函数
# =========================================================================

def main():
    parser = argparse.ArgumentParser(
        description='手眼标定求解 — 输出 R_camera2gimbal 和 t_camera2gimbal')
    parser.add_argument('input_dir', help='handeye_capture.py 输出目录')
    parser.add_argument('--cols', type=int, default=11,
                        help='标定板内角点列数 (默认: 11)')
    parser.add_argument('--rows', type=int, default=8,
                        help='标定板内角点行数 (默认: 8)')
    parser.add_argument('--square', type=float, default=15.0,
                        help='方格边长 mm (默认: 15)')
    parser.add_argument('--params-yaml', default=None,
                        help='rm_autoaim params.yaml 路径 (自动读取内参)')
    parser.add_argument('--fx', type=float, default=None, help='fx (手动指定)')
    parser.add_argument('--fy', type=float, default=None, help='fy (手动指定)')
    parser.add_argument('--cx', type=float, default=None, help='cx (手动指定)')
    parser.add_argument('--cy', type=float, default=None, help='cy (手动指定)')
    parser.add_argument('--method', default=None,
                        choices=['PARK', 'HORAUD', 'TSAI', 'ANDREFF', 'DANIILIDIS', 'ALL'],
                        help='指定算法 (默认: 尝试所有并选最优)')
    args = parser.parse_args()

    # ---- 加载相机内参 ----
    if args.params_yaml:
        camera_matrix, dist_coeffs = load_camera_matrix_from_yaml(args.params_yaml)
        print(f"从 {args.params_yaml} 加载内参:")
    elif all(v is not None for v in [args.fx, args.fy, args.cx, args.cy]):
        camera_matrix = np.array([
            [args.fx, 0, args.cx],
            [0, args.fy, args.cy],
            [0, 0, 1],
        ], dtype=np.float64)
        dist_coeffs = np.zeros((1, 5), dtype=np.float64)
        print("使用手动指定内参:")
    else:
        print("错误: 必须指定 --params-yaml 或 --fx/--fy/--cx/--cy")
        sys.exit(1)

    print(f"  camera_matrix =\n{camera_matrix}")
    print(f"  dist_coeffs = {dist_coeffs.flatten()}")
    print()

    pattern_size = (args.cols, args.rows)

    # ---- 3D 标定板角点 (mm) ----
    obj_points_3d = np.zeros((args.cols * args.rows, 3), np.float32)
    for i in range(args.rows):
        for j in range(args.cols):
            obj_points_3d[i * args.cols + j] = (j * args.square, i * args.square, 0)

    # ---- 加载数据 ----
    R_gimbal2world_list = []  # R_gripper2base
    t_gimbal2world_list = []  # t_gripper2base (全零, 纯旋转)
    rvecs_target2cam = []     # R_target2cam (Rodrigues)
    tvecs_target2cam = []     # t_target2cam

    idx = 1
    valid = 0
    while True:
        img_path = os.path.join(args.input_dir, f"{idx}.jpg")
        meta_path = os.path.join(args.input_dir, f"{idx}.json")

        if not os.path.exists(img_path) or not os.path.exists(meta_path):
            break

        img = cv2.imread(img_path)
        if img is None:
            print(f"[{idx}] 图像读取失败, 跳过")
            idx += 1
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCornersSB(gray, pattern_size)
        if not found:
            print(f"[{idx}] 棋盘格未检测到, 跳过")
            idx += 1
            continue

        corners = cv2.cornerSubPix(
            gray, corners, (5, 5), (-1, -1),
            criteria=(cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 1e-3),
        )

        with open(meta_path, 'r') as f:
            meta = json.load(f)
        w, x, y, z = meta['w'], meta['x'], meta['y'], meta['z']

        R_gimbal2world = quat_to_rotation_matrix(w, x, y, z)

        success, rvec, tvec = cv2.solvePnP(
            obj_points_3d, corners, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE
        )

        if not success:
            print(f"[{idx}] solvePnP 失败, 跳过")
            idx += 1
            continue

        R_gimbal2world_list.append(R_gimbal2world)
        t_gimbal2world_list.append(np.zeros((3, 1), dtype=np.float64))
        rvecs_target2cam.append(rvec)
        tvecs_target2cam.append(tvec)

        valid += 1
        yaw, pitch, roll = rotation_matrix_to_euler_zyx(R_gimbal2world)
        print(f"[{idx}] OK | euler(y={yaw:+.1f} p={pitch:+.1f} r={roll:+.1f}) "
              f"| t_cam={tvec.flatten().round(1)} mm")

        idx += 1

    print(f"\n有效数据: {valid} 组")

    if valid < 5:
        print("错误: 有效数据不足 5 组, 无法标定。请至少采集 15 组。")
        sys.exit(1)

    # ---- 诊断: 检查旋转角度变化量 ----
    print(f"\n{'='*60}")
    print("数据质量诊断:")
    print(f"{'='*60}")

    rot_diffs = []
    for i in range(len(R_gimbal2world_list) - 1):
        R_rel = R_gimbal2world_list[i].T @ R_gimbal2world_list[i + 1]
        angle_deg = math.degrees(math.acos(
            max(-1.0, min(1.0, (np.trace(R_rel) - 1.0) / 2.0))
        ))
        rot_diffs.append(angle_deg)

    if rot_diffs:
        print(f"  相邻帧旋转角度: mean={np.mean(rot_diffs):.1f}° "
              f"min={np.min(rot_diffs):.1f}° max={np.max(rot_diffs):.1f}°")
        if np.max(rot_diffs) < 3.0:
            print("  ⚠️  所有旋转角度 < 3°, 数据可能不足以区分不同方法")
            print("  建议: 转动云台时增大角度变化, 至少覆盖 ±20° 范围")
        elif np.max(rot_diffs) < 10.0:
            print("  ⚠️  旋转角度偏小, 建议增大云台转动范围以提高精度")

    # ---- 手眼标定 ----
    print(f"\n{'='*60}")
    print("运行 cv2.calibrateHandEye() ...")
    print(f"{'='*60}")

    methods = {
        'PARK':       cv2.CALIB_HAND_EYE_PARK,
        'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
        'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
        'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
        'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    results = {}

    for name, method_id in methods.items():
        try:
            R_cg, t_cg = cv2.calibrateHandEye(
                R_gripper2base=R_gimbal2world_list,
                t_gripper2base=t_gimbal2world_list,
                R_target2cam=rvecs_target2cam,
                t_target2cam=tvecs_target2cam,
                method=method_id,
            )

            orth_err = orthogonality_error(R_cg)
            det_err = determinant_error(R_cg)
            t_norm = np.linalg.norm(t_cg)
            valid_rot = is_valid_rotation(R_cg)

            results[name] = {
                'R': R_cg, 't': t_cg,
                'orth_err': orth_err,
                'det_err': det_err,
                't_norm': t_norm,
                'valid': valid_rot,
            }

            status = "✓" if valid_rot else "✗"
            print(f"  {name:12s} {status} | orth_err={orth_err:.2e} "
                  f"det_err={det_err:.2e} |t|={t_norm*1000:.1f}mm")
            print(f"  {'':12s}   R diag = {np.diag(R_cg).flatten()}")
            print(f"  {'':12s}   t = [{t_cg[0,0]*1000:.1f}, "
                  f"{t_cg[1,0]*1000:.1f}, {t_cg[2,0]*1000:.1f}] mm")

        except Exception as e:
            print(f"  {name:12s} ✗ | 失败: {e}")

    if not results:
        print("\n所有方法均失败!")
        sys.exit(1)

    # ---- 选择最佳结果 ----
    print(f"\n{'='*60}")
    print("选择最佳结果:")
    print(f"{'='*60}")

    # 策略: 在正交性有效的结果中, 选 orth_err 最小的
    valid_results = {k: v for k, v in results.items() if v['valid']}

    if valid_results:
        best_name = min(valid_results, key=lambda k: valid_results[k]['orth_err'])
        best = valid_results[best_name]
        print(f"  最佳方法: {best_name} (orth_err={best['orth_err']:.2e})")
    else:
        # 没有有效的, 选 orth_err 最小的 (可能需要放宽容差)
        best_name = min(results, key=lambda k: results[k]['orth_err'])
        best = results[best_name]
        print(f"  ⚠️  没有方法通过正交性检验, 选最小的: {best_name} "
              f"(orth_err={best['orth_err']:.2e})")
        print(f"  建议重新采集更多数据, 增大云台转动角度")

    best_R = best['R']
    best_t = best['t']

    # ---- 计算相机安装偏角 ----
    # 理想安装: 相机 x→前(云台-y), y→下(云台-z), z→右(云台x)
    # R_ideal_cam2gimbal = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
    R_ideal = np.array([
        [0, -1, 0],
        [0, 0, -1],
        [1,  0, 0],
    ], dtype=np.float64)

    # R_cam_actual2gimbal = R_cam2gimbal
    # R_cam_ideal2actual = R_cam2gimbal @ R_ideal^T
    # (即从理想安装到实际安装的偏差)
    R_offset = best_R @ R_ideal.T
    yaw_off, pitch_off, roll_off = rotation_matrix_to_euler_zyx(R_offset)

    # ---- 输出结果 ----
    R_flat = best_R.flatten().tolist()
    t_flat = best_t.flatten().tolist()

    r_str = ", ".join(f"{v:.10g}" for v in R_flat)
    t_str = ", ".join(f"{v:.6f}" for v in t_flat)

    print(f"\n{'='*60}")
    print("标定结果 (可直接粘贴到 rm_autoaim/config/params.yaml):")
    print(f"{'='*60}")
    print(f"""
    # ---- 相机→云台外参 (手眼标定) ----
    # 相机安装偏角: yaw={yaw_off:.2f}° pitch={pitch_off:.2f}° roll={roll_off:.2f}°
    # 标定方法: {best_name} | 有效数据: {valid} 组
    # 正交性误差: {best['orth_err']:.2e}
    r_cam_to_gimbal: [{r_str}]
    t_cam_to_gimbal: [{t_str}]
""")

    # ---- 平移可靠性警告 ----
    t_norm_mm = best['t_norm'] * 1000
    if t_norm_mm > 500:
        print(f"  ⚠️  |t| = {t_norm_mm:.0f} mm, 明显过大!")
        print(f"  纯旋转手眼标定的平移估计通常不稳定")
        print(f"  建议: 测量相机到云台旋转中心的实际距离, 手动设置 t_cam_to_gimbal")
        print(f"  R_cam_to_gimbal (旋转) 仍然是可靠的")
        print()

    # ---- 保存到文件 ----
    result_path = os.path.join(args.input_dir, "result.yaml")
    with open(result_path, 'w') as f:
        f.write(f"# Hand-Eye Calibration Result\n")
        f.write(f"# Method: {best_name} | Samples: {valid}\n")
        f.write(f"# Orthogonality error: {best['orth_err']:.2e}\n")
        f.write(f"# Camera offset from ideal: "
                f"yaw={yaw_off:.2f} pitch={pitch_off:.2f} roll={roll_off:.2f} deg\n")
        f.write(f"# |t| = {t_norm_mm:.1f} mm "
                f"({'reliable' if t_norm_mm < 500 else 'UNRELIABLE - consider manual measurement'})\n\n")
        f.write(f"r_cam_to_gimbal: [{r_str}]\n")
        f.write(f"t_cam_to_gimbal: [{t_str}]\n")

    print(f"结果已保存到: {result_path}")

    # ---- 验证 ----
    print(f"\n{'='*60}")
    print("验证:")
    print(f"  det(R)       = {np.linalg.det(best_R):.10f} (应为 1.0)")
    print(f"  ||R^TR - I|| = {orthogonality_error(best_R):.2e} (应 < 0.01)")
    print(f"  |t|          = {t_norm_mm:.1f} mm")
    print(f"  R = ")
    for row in best_R:
        print(f"      [{', '.join(f'{v:+.8f}' for v in row)}]")
    print(f"  t = [{', '.join(f'{v*1000:.2f} mm' for v in t_flat)}]")
    print(f"  安装偏角: yaw={yaw_off:.2f}° pitch={pitch_off:.2f}° roll={roll_off:.2f}°")

    # ---- 额外: 尝试用已知的合理 t 重标定 ----
    # 如果用户提供了 --t-manual 参数, 可以用已知的 t 固定下来
    # 这里只给提示
    if t_norm_mm > 500:
        print(f"""
{'='*60}
建议操作:
  1. 先使用上面输出的 R_cam_to_gimbal (旋转部分可靠)
  2. 手动测量相机到云台旋转中心的物理距离
  3. 根据相机安装方向, 手动填写 t_cam_to_gimbal
     例如: 相机在云台前方 140mm, 左侧 10mm, 上方 50mm:
     t_cam_to_gimbal: [-0.01, 0.05, 0.14]
     (具体符号取决于坐标系约定)
{'='*60}""")


if __name__ == '__main__':
    main()