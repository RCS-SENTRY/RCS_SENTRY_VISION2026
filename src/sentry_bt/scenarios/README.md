# sentry_bt 调试场景

这些 YAML 文件供 `sentry_bt.launch.py mode:=debug scenario:=...` 使用。

| 场景 | 主要覆盖 |
|---|---|
| `nominal_patrol.yaml` | 无敌巡航、转高点、发现敌人、据点保持 |
| `engage_resupply.yaml` | 交战后低弹、去补给、补弹兑换、周期补弹领取 |
| `critical_hp_remote_repair.yaml` | 脱战低血远程补血、补血后重新接敌 |
| `revive_cycle.yaml` | 死亡、立即复活、免费确认复活窗口 |
| `posture_stress.yaml` | 姿态反馈抖动、姿态冷却、姿态累计衰减 |
| `energy_activation.yaml` | 能量机关激活机会、一次性触发确认 |
| `input_loss_failsafe.yaml` | 裁判/底盘状态输入丢失后的安全冻结与恢复 |
| `rfid_field_points.yaml` | RFID 位模拟基地、补给区、堡垒、高地等实车状态路径 |
| `rule_tuning_matrix.yaml` | 离线规则压力矩阵：热量/功率/低血/补弹/复活/姿态衰减/部分数据缺失断言 |

常用命令：

```bash
ros2 launch sentry_bt sentry_bt.launch.py mode:=debug scenario:=nominal_patrol.yaml
ros2 launch sentry_bt sentry_bt.launch.py mode:=debug scenario:=input_loss_failsafe.yaml watch_view:=io
ros2 launch sentry_bt sentry_bt.launch.py mode:=debug scenario:=rfid_field_points.yaml enable_sim_input:=false
ros2 run sentry_bt sentry_bt_scenario_runner
```

`enable_sim_input:=false` 会让决策节点忽略 `/sentry_bt/sim_input`，改用 `/gimbal_status`
中的 RFID、event、裁判系统字段推导场地状态，更接近实际上车路径。
