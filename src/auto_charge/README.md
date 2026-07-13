# Auto Charge

## 模拟低电量测试

先启动 Nav2，再启动本节点。使用独立测试话题，避免影响真实电池数据：

```bash
source install/setup.bash
ros2 run auto_charge auto_charge_manager --ros-args \
  -p battery_topic:=/test_battery_status \
  -p confirm_count:=1 \
  -p staging_x:=1.0 \
  -p staging_y:=2.0 \
  -p staging_yaw:=0.0
```

另开终端，模拟 10% 电量并触发返航：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /test_battery_status \
  sensor_msgs/msg/BatteryState \
  "{percentage: 0.10, power_supply_status: 2, present: true}"
```

测试前请将 `staging_x`、`staging_y` 和 `staging_yaw` 改为安全的充电桩预停靠位置。
