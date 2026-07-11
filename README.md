waypoint_checker接收前端发布点
waypoint_subscriber处理转换坐标后的点并交给nav2
publish_waypoints为测试程序

该程序能够接受的前端传入点形式：
{
  "points": [
    {
      "pixel_x": 100.0,
      "pixel_y": 300.0,
      "name": "waypoint_1"
    },
    {
      "pixel_x": 180.0,
      "pixel_y": 250.0,
      "name": "waypoint_2"
    },
    {
      "pixel_x": 260.0,
      "pixel_y": 190.0,
      "yaw": 1.57,
      "name": "waypoint_3"
    }
  ]
}
注意：点坐标一定是原地图像素坐标，若UI界面有缩放，注意转换后再传入后端。
可选择是否带yaw，如果没有yaw，本后端将会按照相邻点方位自动计算yaw
