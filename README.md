# GO2 ROS2 工作空间

Unitree GO2 机器狗的 ROS2 Humble 软件集合：WebRTC 通信接口、slam_toolbox 建图/定位、
测试工具与可视化仿真，全部使用标准 ROS 消息，可直接对接 rviz2 / SLAM / Nav2。

## 包一览

| 包 | 说明 | 文档 |
|---|---|---|
| [`go2_webrtc_ros2`](src/go2_webrtc_ros2/README.md) | 机器狗通信接口单节点（数据读取 + 运动控制，标准消息） | [README](src/go2_webrtc_ros2/README.md) |
| [`go2_slam_toolbox`](src/go2_slam_toolbox/README.md) | slam_toolbox 建图封装（新建图/继续建图/定位，自动 rviz） | [README](src/go2_slam_toolbox/README.md) |
| [`go2_webrtc_test`](src/go2_webrtc_test/README.md) | 遥控/指令测试工具（键盘遥控、指令台） | [README](src/go2_webrtc_test/README.md) |

> 可视化仿真平台 `go2_sim`（与 `go2_node` 接口一致，不接真机即可测试整套链路）
> 独立维护，需要时可放入 `src/` 一并构建。

## 依赖

- ROS2 Humble（含 rviz2、slam_toolbox、robot_state_publisher、nav2_map_server）
- `pip install unitree_webrtc_connect`（GO2 WebRTC SDK）

## 构建与使用

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 连接机器狗（数据 + 控制接口）
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161

# 建图（自动启动 rviz，激光红点/地图/姿态）
ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=new
```

各包的详细话题/指令/参数说明见对应 README。

## License

MIT（见 [LICENSE](LICENSE)）
