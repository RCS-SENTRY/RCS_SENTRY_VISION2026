# Dual Lidar Safety Limiter

## 背景

PB2025 主导航链以 `gimbal_yaw / gimbal_yaw_fake` 为语义核心，主雷达 `front_mid360` 走 Point-LIO + terrain map。第二雷达固定在旋转云台上，如果直接用普通 `ObstacleLayer` 写入 local costmap，云台旋转会把扫过区域全部标成障碍（marking-only 且不清除），形成大面积鬼障碍。

为避免污染主 costmap，双雷达第一阶段采用 **safety limiter**：

- 主雷达链路完全不变。
- 第二雷达只作为近场安全壳，限制 `/cmd_vel` 的线速度分量。
- local costmap 的 second lidar layer 默认关闭，仅用于 debug。

## 为什么不直接写 local costmap

- 第二雷达固定在旋转云台上，点云 frame 语义随云台旋转变化。
- PB2025 本地规划围绕 `gimbal_yaw` 语义工作，普通 ObstacleLayer 无法识别旋转语义差异。
- marking-only 且无 clearing 会把旋转过程中的误点长期留在 costmap。
- safety limiter 更适合作为双雷达避障的第一阶段方案。

## 推荐模式

默认双雷达避障模式：

- `enable_second_lidar:=true`
- `enable_second_lidar_filter:=true`
- `enable_second_lidar_safety_limiter:=true`
- `enable_second_lidar_costmap:=false`

仅 debug 才允许打开 costmap：

- `enable_second_lidar_costmap:=true`

## 安全限速逻辑

- 将 `/second_lidar_obstacle_cloud` 按方位划分为 front/back/left/right。
- 统计最近障碍距离和点数。
- 若距离进入 caution/slow/emergency 区间，分别对对应方向速度缩放或归零。
- `emergency_stop=true` 时，紧急距离触发后整体线速度归零，但不影响角速度。

## 常用调试

```bash
ros2 topic hz /second_livox/lidar
ros2 topic hz /second_lidar_obstacle_cloud
ros2 topic echo /second_lidar_obstacle_debug

ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_safe
ros2 topic echo /second_lidar_safety_debug

ros2 param get /local_costmap/local_costmap second_lidar_obstacle_layer.enabled
```

若发现云台旋转导致鬼障碍，第一步检查 `enable_second_lidar_costmap` 是否误开，确认 local costmap 的 `second_lidar_obstacle_layer.enabled` 为 `false`。
