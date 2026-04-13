#!/usr/bin/env python3
"""
analyze_bag.py — 使用 ROS 2 Python 库分析 bag 中 Tracker 输出质量
"""
import sys
import os
import math

# 设置 ROS 2 环境
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/dist-packages')
os.environ['ROS_DOMAIN_ID'] = '0'

import sqlite3
import struct

BAG_DB = "/home/rm/bag2/autoaim_test_20260406_114723/autoaim_test_20260406_114723_0.db3"

def read_raw_messages(db_path, topic_name):
    """从 db3 读取指定话题的 raw bytes"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    row = cursor.fetchone()
    if not row:
        conn.close()
        return []
    topic_id = row[0]
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,))
    rows = cursor.fetchall()
    conn.close()
    return rows

def parse_cdr_point_stamped(data):
    """正确解析 CDR PointStamped"""
    # CDR encapsulation: 1 byte kind + 1 byte options + 2 bytes padding? No...
    # Actually: byte 0 = encapsulation kind, byte 1 = options
    # Then CDR data: alignment depends on type
    
    # Let's try: offset starts at 4 (after ROS message header)
    # Standard CDR: first 4 bytes = encapsulation header
    off = 4  # after encapsulation header (4 bytes)
    
    # Header.stamp: sec(int32) + nanosec(uint32) = 8 bytes
    sec = struct.unpack_from('<i', data, off)[0]
    nsec = struct.unpack_from('<I', data, off+4)[0]
    stamp = sec + nsec * 1e-9
    off += 8
    
    # frame_id: uint32 length + chars + pad to 4-byte boundary
    slen = struct.unpack_from('<I', data, off)[0]
    off += 4
    frame_id = data[off:off+slen].decode('utf-8', errors='ignore')
    off += slen
    # Align to 4 bytes
    off = (off + 3) & ~3
    
    # Point: x, y, z (float64 each) — no extra 8-byte align needed
    x = struct.unpack_from('<d', data, off)[0]; off += 8
    y = struct.unpack_from('<d', data, off)[0]; off += 8
    z = struct.unpack_from('<d', data, off)[0]; off += 8
    
    return stamp, x, y, z

def parse_cdr_gimbal_cmd(data):
    """解析 GimbalCmd CDR — 无 Header, 直接是数据字段"""
    # GimbalCmd.msg: float64 x4, int8 x2, uint8 x1 = 35 bytes
    off = 4  # after CDR encap header
    
    # float64 target_yaw, target_pitch, yaw_vel, pitch_vel
    tyaw, tpitch, yvel, pvel = struct.unpack_from('<dddd', data, off)
    off += 32
    
    # int8 state_switch
    state = struct.unpack_from('<b', data, off)[0]; off += 1
    # int8 fire_control
    fire = struct.unpack_from('<b', data, off)[0]; off += 1
    # uint8 mode
    mode = data[off]; off += 1
    
    return 0.0, tyaw, tpitch, fire, state, mode  # no stamp

def parse_cdr_armor_detections(data):
    """解析 ArmorDetections CDR — 提取数量和时间戳"""
    off = 4
    
    sec = struct.unpack_from('<i', data, off)[0]
    nsec = struct.unpack_from('<I', data, off+4)[0]
    stamp = sec + nsec * 1e-9
    off += 8
    
    slen = struct.unpack_from('<I', data, off)[0]
    off += 4 + slen
    off = (off + 3) & ~3
    
    # armors array: uint32 count
    count = struct.unpack_from('<I', data, off)[0]
    
    return stamp, count

def calc_std(vals):
    n = len(vals)
    if n < 2: return 0
    mean = sum(vals) / n
    return math.sqrt(sum((x-mean)**2 for x in vals) / n)

def main():
    print("=" * 60)
    print("  IMM-UKF Tracker 性能离线分析")
    print("=" * 60)
    
    # ---- 1. Tracker 输出 ----
    print("\n[1] /autoaim/debug_world_points")
    raw = read_raw_messages(BAG_DB, "/autoaim/debug_world_points")
    
    points = []
    parse_errors = 0
    for _, data in raw:
        try:
            t, x, y, z = parse_cdr_point_stamped(bytes(data))
            # Sanity check: RoboMaster 场地 ~28m x 15m, z 一般 0~0.5m
            if abs(x) > 100 or abs(y) > 100 or abs(z) > 100:
                parse_errors += 1
                continue
            points.append((t, x, y, z))
        except Exception as e:
            parse_errors += 1
    
    print(f"  总消息: {len(raw)}, 解析成功: {len(points)}, 解析失败: {parse_errors}")
    
    if points and len(points) > 2:
        dur = points[-1][0] - points[0][0]
        print(f"  时长: {dur:.2f}s, 平均频率: {len(points)/dur:.1f} Hz")
        
        xs = [p[1] for p in points]
        ys = [p[2] for p in points]
        zs = [p[3] for p in points]
        
        print(f"  X: min={min(xs):.3f} max={max(xs):.3f} mean={sum(xs)/len(xs):.3f} σ={calc_std(xs):.3f} m")
        print(f"  Y: min={min(ys):.3f} max={max(ys):.3f} mean={sum(ys)/len(ys):.3f} σ={calc_std(ys):.3f} m")
        print(f"  Z: min={min(zs):.3f} max={max(zs):.3f} mean={sum(zs)/len(zs):.3f} σ={calc_std(zs):.3f} m")
        
        # 跳变分析
        jumps = []
        for i in range(1, len(points)):
            dt = points[i][0] - points[i-1][0]
            if dt < 1e-6: continue
            dx = points[i][1] - points[i-1][1]
            dy = points[i][2] - points[i-1][2]
            dz = points[i][3] - points[i-1][3]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            jumps.append(dist)
        
        if jumps:
            print(f"\n  帧间位移: mean={sum(jumps)/len(jumps):.4f} max={max(jumps):.4f} m")
            big = sum(1 for j in jumps if j > 0.1)
            print(f"  大跳变(>0.1m): {big}/{len(jumps)} ({100*big/len(jumps):.1f}%)")
        
        # 差分速度
        vels = []
        for i in range(1, len(points)):
            dt = points[i][0] - points[i-1][0]
            if dt < 1e-6: continue
            dx = points[i][1] - points[i-1][1]
            dy = points[i][2] - points[i-1][2]
            v = math.sqrt((dx/dt)**2 + (dy/dt)**2)
            vels.append(v)
        
        if vels:
            print(f"\n  差分速度: mean={sum(vels)/len(vels):.3f} max={max(vels):.3f} σ={calc_std(vels):.3f} m/s")
        
        # 平滑度 (二阶差分)
        accels = []
        for i in range(2, len(points)):
            dt1 = points[i][0] - points[i-1][0]
            dt2 = points[i-1][0] - points[i-2][0]
            if dt1 < 1e-6 or dt2 < 1e-6: continue
            ax = (points[i][1] - 2*points[i-1][1] + points[i-2][1]) / (dt1*dt2)
            ay = (points[i][2] - 2*points[i-1][2] + points[i-2][2]) / (dt1*dt2)
            accels.append(math.sqrt(ax*ax + ay*ay))
        
        if accels:
            smooth = sum(accels) / len(accels)
            print(f"  平滑度(加速度均值): {smooth:.2f} m/s²")
    else:
        print("  ⚠️ 无有效 Tracker 数据!")
    
    # ---- 2. GimbalCmd ----
    print(f"\n[2] /gimbal_cmd")
    raw_cmd = read_raw_messages(BAG_DB, "/gimbal_cmd")
    cmds = []
    for _, data in raw_cmd:
        try:
            c = parse_cdr_gimbal_cmd(bytes(data))
            cmds.append(c)
        except:
            pass
    print(f"  总消息: {len(cmds)}")
    if cmds:
        fires = sum(1 for c in cmds if c[3] == 1)
        print(f"  开火率: {fires}/{len(cmds)} ({100*fires/len(cmds):.1f}%)")
        yaws = [c[1]*180/math.pi for c in cmds]
        pitches = [c[2]*180/math.pi for c in cmds]
        print(f"  Yaw: {min(yaws):.2f}° ~ {max(yaws):.2f}°")
        print(f"  Pitch: {min(pitches):.2f}° ~ {max(pitches):.2f}°")
    
    # ---- 3. ArmorDetections ----
    print(f"\n[3] /detector/armors")
    raw_det = read_raw_messages(BAG_DB, "/detector/armors")
    dets = []
    for _, data in raw_det:
        try:
            d = parse_cdr_armor_detections(bytes(data))
            dets.append(d)
        except:
            pass
    if dets:
        dur_d = dets[-1][0] - dets[0][0] if len(dets) > 1 else 1
        counts = [d[1] for d in dets]
        print(f"  总帧: {len(dets)}, 频率: {len(dets)/dur_d:.1f} Hz")
        print(f"  每帧检测数: min={min(counts)} max={max(counts)} mean={sum(counts)/len(counts):.1f}")
        empty = sum(1 for c in counts if c == 0)
        print(f"  空帧: {empty}/{len(dets)} ({100*empty/len(dets):.1f}%)")
    
    # ---- 综合诊断 ----
    print(f"\n{'='*60}")
    print("综合诊断与调参建议")
    print("=" * 60)
    
    if not points or len(points) < 10:
        print("⚠️ Tracker 几乎无输出!")
        print("  → 可能原因: IMU 时间戳问题, 或 UKF 发散")
        print("  → 建议: 检查 fake_sensor_publisher 是否正确运行")
        return
    
    # 诊断
    issues = []
    
    if jumps and sum(1 for j in jumps if j > 0.1) / len(jumps) > 0.05:
        issues.append("跳变频繁 (>5% 帧有 >0.1m 跳变)")
        print("  ⚠️ 跳变频繁 — 可能原因: 装甲板切换或 PnP 不稳定")
    
    if vels and calc_std(vels) > 2.0:
        issues.append("速度标准差过大")
        print("  ⚠️ 速度波动大 — UKF 滤波不足")
    
    if accels and sum(accels)/len(accels) > 50:
        issues.append("平滑度差")
        print("  ⚠️ 轨迹不平滑 — 需要增大过程噪声或观测噪声")
    
    if cmds:
        fire_rate = sum(1 for c in cmds if c[3]==1) / len(cmds)
        if fire_rate < 0.1:
            issues.append("开火率低")
            print(f"  ⚠️ 开火率仅 {fire_rate*100:.1f}% — Gate 条件可能过严")
        elif fire_rate > 0.8:
            print(f"  ⚠️ 开火率 {fire_rate*100:.1f}% 过高 — Gate 可能太松")
    
    if not issues:
        print("  ✅ Tracker 状态良好!")
    else:
        print(f"\n  发现 {len(issues)} 个问题:")
        for i, issue in enumerate(issues, 1):
            print(f"    {i}. {issue}")

if __name__ == '__main__':
    main()