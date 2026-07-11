<<<<<<< HEAD
# Nav2 Waypoint Runner C++

This is a standalone ROS2 C++ package for sending waypoint lists to Nav2's
`/follow_waypoints` action.

It matches the current robot Nav2 setup:

- waypoint frame defaults to `map`
- action defaults to `/follow_waypoints`
- accepts `geometry_msgs/msg/PoseArray` on `/nav2_waypoints`
- accepts `nav_msgs/msg/Path` on `/nav2_waypoint_path`
- accepts single `geometry_msgs/msg/PoseStamped` appends on `/nav2_waypoint_append`

## Build

```bash
cd /home/wheeltec/nav2_waypoint_runner_cpp
source /home/wheeltec/wheeltec_ros2/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run Subscriber

Start Nav2 first, then run:

```bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
source /home/wheeltec/nav2_waypoint_runner_cpp/install/setup.bash
ros2 run nav2_waypoint_runner_cpp waypoint_subscriber
```

Useful parameters:

```bash
ros2 run nav2_waypoint_runner_cpp waypoint_subscriber --ros-args \
  -p default_frame:=map \
  -p waypoint_topic:=/nav2_waypoints \
  -p action_name:=/follow_waypoints \
  -p cancel_on_new_goal:=true
```

## Publish Example Waypoints

Each waypoint is `x y yaw` in the `map` frame:

```bash
source /home/wheeltec/nav2_waypoint_runner_cpp/install/setup.bash
ros2 run nav2_waypoint_runner_cpp publish_waypoints -- \
  --frame map \
  -1.03 3.43 1.57 \
  -4.85 5.13 0.0
```

You can also publish from the CLI:

```bash
ros2 topic pub --once /nav2_waypoints geometry_msgs/msg/PoseArray \
"header:
  frame_id: map
poses:
- position: {x: -1.03, y: 3.43, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
- position: {x: -4.85, y: 5.13, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```
=======
# Agricultural_vehicle_nav_backend
>>>>>>> 1451cc0fc0300b6de4b1fb0902c717ecb0859890
