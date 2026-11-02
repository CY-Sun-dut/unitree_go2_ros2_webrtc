# GO2 ROS2 工作空间

Unitree GO2 机器狗的 ROS2 Humble 软件集合。本项目通过 GO2 的 **WebRTC 通道**与真机通信，
将机器狗的传感数据与运动控制封装为**标准 ROS2 消息/话题接口**，可直接对接
rviz2 / slam_toolbox / Nav2 等生态工具。

**设计原则**

- **一条连接、一个节点**：`go2_node` 单节点持有唯一的 WebRTC 会话，数据读取与运动控制统一收口；
- **全标准消息**：雷达点云/激光/IMU/相机/里程计等均为 `sensor_msgs`/`nav_msgs` 标准类型，
  控制使用 `geometry_msgs/Twist` 等标准指令话题，第三方包零改动接入；
- **功能可裁剪**：读取与控制的每项功能都有独立开关参数，按需启动；
- **可视化友好**：内置 rviz 配置，建图/定位流程自动拉起可视化（激光红点/地图/姿态）。

---

## 📋 任务进度

**✅ 已完成**

- [x] **ROS2 通信及控制接口** —— 基于 WebRTC 连接 GO2 真机：
      数据读取（雷达点云、2D 激光、IMU、相机、里程计、关节、电池、运动状态）+
      运动控制（前后左右/旋转移动、站立/趴下/阻尼/恢复等状态切换、步态切换、运动模式切换），
      全部标准消息，带 rviz 可视化配置
- [x] **基于 slam_toolbox 的二维建图** —— 新建图 / 旧图继续建图（鼠标点选初始位姿，防里程计不匹配）/
      加载地图纯定位，自动启动 rviz 可视化，地图保存为栅格图 + 位姿图

**📌 待完成**

- [ ] **二维导航** —— 基于 Nav2 的定位与路径规划：全局/局部规划、动态避障、目标点导航
- [ ] **三维 SLAM 建图** —— 基于雷达点云的三维建图、地图管理与可视化
- [ ] **键盘操作** —— 键盘遥控工具（手动控制机器狗移动、指令交互），待实测后合入

---

## 包一览

| 包 | 说明 | 文档 |
|---|---|---|
| [`go2_webrtc_ros2`](src/go2_webrtc_ros2/README.md) | 机器狗通信接口单节点 `go2_node`：数据读取 + 运动控制，全部标准消息 | [README](src/go2_webrtc_ros2/README.md) |
| [`go2_slam_toolbox`](src/go2_slam_toolbox/README.md) | slam_toolbox 二维建图封装：新建图/继续建图/定位，自动 rviz | [README](src/go2_slam_toolbox/README.md) |

**go2_webrtc_ros2 接口速查**（详细说明见 [包 README](src/go2_webrtc_ros2/README.md)）：

| 数据话题 | 消息类型 | 控制话题 | 消息类型 |
|---|---|---|---|
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` | `/go2/cmd/vel` | `geometry_msgs/Twist` |
| `/go2/scan` | `sensor_msgs/LaserScan` | `/go2/cmd/state` | `std_msgs/String` |
| `/go2/imu` | `sensor_msgs/Imu` | `/go2/cmd/gait` | `std_msgs/String` |
| `/go2/camera/image_raw` 等 | `sensor_msgs/Image` 等 | `/go2/cmd/gait_id` | `std_msgs/Int32` |
| `/go2/odom` + TF | `nav_msgs/Odometry` | `/go2/cmd/mode` | `std_msgs/String` |
| `/go2/joint_states`、`/go2/battery` 等 | `sensor_msgs/JointState` 等 | `/go2/cmd/result` | `std_msgs/String`（结果反馈） |

## 目录结构

```text
ros2_ws/
├── src/
│   ├── go2_webrtc_ros2/          通信接口单节点（数据读取 + 运动控制）
│   │   ├── go2_webrtc_ros2/      节点源码（连接/转换/接口）
│   │   └── rviz/go2.rviz         rviz 预置配置
│   └── go2_slam_toolbox/         slam_toolbox 建图/定位封装
│       ├── go2_slam_toolbox/     适配层（话题转播 + 续建流程）
│       ├── config/               slam_toolbox 参数（建图/定位）
│       ├── launch/               一键启动（含自动 rviz）
│       └── rviz/go2_slam.rviz    建图可视化配置
├── README.md
└── LICENSE
```

---

## 依赖

### 1. 系统与 ROS2

- Ubuntu 22.04 + **ROS2 Humble**（桌面版自带 rviz2、robot_state_publisher）
- `slam_toolbox`、`nav2_map_server`、`nav2_lifecycle_manager`（建图包使用）：

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2
```

### 2. GO2 WebRTC 通信库（引用）

机器狗通信层基于开源项目 **[unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect)**
（Unitree GO2 的 WebRTC 通信库）封装，本项目的连接/数据流/指令通道均通过该库实现：

```bash
pip install unitree_webrtc_connect
# 或从源码安装：
git clone https://github.com/legion1581/unitree_webrtc_connect.git
pip install ./unitree_webrtc_connect
```

其 Python 依赖（numpy、opencv、aiortc 等）随该库一并安装。

### 3. 设备 AES 密钥（固件 ≥ 1.1.15）

每台设备需要独立密钥，通过官方工具获取后以参数传入（**不要写入代码/文档**）：

```bash
unitree-fetch-aes-key --email <账号> --password <密码> --device-type Go2
ros2 run go2_webrtc_ros2 go2_node --ros-args -p aes_key:=<32位hex>
```

---

## 快速开始

### ① 新建工作空间并克隆仓库

```bash
# 新建工作空间文件夹
mkdir -p ~/ros2_ws

# 克隆仓库（仓库根即工作空间，ROS2 包位于 src/ 下）
git clone https://github.com/CY-Sun-dut/unitree_go2_ros2_webrtc.git ~/ros2_ws
# SSH 方式：git clone git@github.com:CY-Sun-dut/unitree_go2_ros2_webrtc.git ~/ros2_ws
```

> 如果 `~/ros2_ws` 已有其他包：把仓库内容并入即可（保持 `src/<包名>` 结构）。

### ② 安装依赖

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2
pip install unitree_webrtc_connect        # GO2 WebRTC SDK（详见「依赖」章节）
```

### ③ 编译工作空间

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
# 建议把上面两行 source 写入 ~/.bashrc，免去每次手动执行
```

> 更新代码：`cd ~/ros2_ws && git pull && colcon build`（增量编译，只重建改动的包）。

### ④ 运行

```bash
# 连接机器狗（与狗同一局域网；默认 IP 192.168.123.161）
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161

# 另开终端：查看数据
ros2 topic echo /go2/imu
ros2 topic hz /go2/lidar/points

# 移动控制（持续发布；停止发布 0.5s 自动停车）
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{linear: {x: 0.3}}"
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: stand_up}"

# 建图（自动启动 rviz：激光红点/地图/姿态）
ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=new
```

**典型工作流**：`go2_node`（数据链）→ `go2_slam_toolbox` 建图并保存地图
（栅格图 `.yaml/.pgm` + 位姿图 `.posegraph/.data`）→ 后续用于继续建图/定位/导航。

**rviz 可视化**：

```bash
rviz2 -d $(ros2 pkg prefix go2_webrtc_ros2)/share/go2_webrtc_ros2/rviz/go2.rviz
```

## 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_ip` | `192.168.123.161` | 机器狗 IP（`local_sta`/`local_ap`/`remote` 三种连接方式） |
| `robot_ns` | `go2` | 话题命名空间前缀 |
| `read_lidar` `read_scan` `read_imu` `read_camera` `read_odom` `read_state` | true | 各数据流开关 |
| `enable_control` | true | 控制接口开关 |
| `publish_map_tf` | `true` | map→odom 占位 TF（**跑 SLAM 时置 false**） |
| `aes_key` | 空 | 设备密钥（固件 ≥1.1.15） |

完整参数、话题、指令语义见各包 README。

---

## 引用与致谢

- **[unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect)** ——
  Unitree GO2 WebRTC 通信库（作者 legion1581，MIT License）。
  本项目的机器狗通信层（WebRTC 连接、数据流解析、指令通道）基于该库封装，特此引用并致谢。

## License

MIT（见 [LICENSE](LICENSE)）
