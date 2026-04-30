# 低带宽调试指南

## 为什么 ToDesk + RViz + 点云会卡

- 远程桌面需要持续视频编码。
- RViz 点云、costmap、TF 显示会占用 GPU 和 CPU。
- 点云 topic 带宽高，尤其 raw cloud、registered cloud、terrain map 同时显示时。
- costmap 高频发布会增加网络与渲染负担。
- 图像 debug 流也会吃掉大量带宽。

## 推荐方式

机器人端只跑服务：

```bash
tmux new -s sentry
ros2 launch rm_bringup sentry_bringup.launch.py \
  navigation_only:=true \
  use_serial:=true \
  serial_device:=/dev/rm_serial \
  baudrate:=460800 \
  enable_navigation:=true \
  slam:=False \
  map:=/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml \
  enable_small_gicp:=false \
  use_rviz:=false
```

本地电脑加入同一局域网，使用相同 `ROS_DOMAIN_ID`，再开 RViz。只显示必要 topic。

## RViz 降负载

- Queue Size 设为 `1`。
- Decay Time 设为 `0`。
- 点云一次只看一个。
- 不同时显示 raw cloud、registered cloud、`/terrain_map`、`/terrain_map_ext`。
- 关闭图像 debug 流。
- costmap 按需打开，确认后关闭。

## 常用命令

```bash
ros2 topic hz /odometry
ros2 topic hz /terrain_map
ros2 topic hz /cmd_vel
ros2 topic hz /nav_cmd
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

## rosbag

```bash
ros2 bag record /tf /tf_static /odometry /terrain_map /terrain_map_ext /cmd_vel /nav_cmd /local_costmap/costmap
```

离线分析时再打开 RViz 和点云显示，不把渲染压力放在机器人端。
