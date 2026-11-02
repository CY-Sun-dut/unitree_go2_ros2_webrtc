# go2_webrtc_ros2 — Unitree GO2 机器人 ROS2 单节点接口 · 使用说明

本包把 [unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect) 封装为
**一个 ROS2 节点**（`go2_node`），通过**一条 WebRTC 连接**完成 GO2 机器狗的：

- **数据读取**：雷达点云、2D 激光、IMU、相机、里程计、关节、电池、运动状态；
- **运动控制**：前后/左右平移/旋转、状态切换（站立/趴下/阻尼/恢复）、步态切换、运动模式切换。

**全部接口均为 ROS 标准消息类型**，可直接用 `ros2 topic`、rviz2、slam_toolbox、Nav2 等使用；
读取与控制功能均可通过参数**单独开关**，用 `ros2 run` 即可启动。

---

## 1. 功能概览

| 模块 | 内容 | 接口形式 |
|---|---|---|
| 读取（8 组开关） | 雷达点云、2D 激光、IMU、相机、里程计+TF、关节/电池/速度/状态、链路状态 | 话题（标准消息） |
| 控制（总开关） | 移动（前后左右+旋转）、状态切换（站立/趴下/阻尼/恢复…）、步态切换、运动模式切换 | 话题（标准消息） |

遥控器摇杆输入、音频、灯光音量等与运动/建图/导航无关的内容**不在本接口范围内**。
旧版多节点接口（`bridge` + 8 数据节点 + 6 指令节点）已由本单节点取代。

---

## 2. 依赖与安装

**依赖**：ROS2 Humble、Python 3.10、`unitree_webrtc_connect`（含 aiortc / numpy / opencv / lz4）。

```bash
# 1) 安装 SDK（已安装可跳过）
pip install unitree_webrtc_connect
# 或从源码目录安装：pip install ~/unitree/unitree_webrtc_connect

# 2) 放入工作空间并编译
cp -r go2_webrtc_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_webrtc_ros2
source install/setup.bash
```

> **AES 密钥**：固件 ≥ 1.1.15 需要每设备密钥，用
> `unitree-fetch-aes-key --email ... --password ... --device-type Go2` 获取（32 位 hex），
> 启动时用 `-p aes_key:=<32hex>` 传入，否则连不上。

---

## 3. 快速开始（ros2 run）

```bash
# 最常用：数据 + 控制全开
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161

# 只读数据、不开控制
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161 \
  -p enable_control:=false

# 只开雷达点云 + 激光 + 里程计（建图导航最小集）
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161 \
  -p read_imu:=false -p read_camera:=false -p read_state:=false \
  -p enable_control:=false

# 功能开关一览（默认全部 true）
#   read_lidar  read_scan  read_imu  read_camera  read_odom  read_state
#   enable_control
```

**RViz 可视化**（点云/激光/里程计/相机，含预置配置）：

```bash
rviz2 -d $(ros2 pkg prefix go2_webrtc_ros2)/share/go2_webrtc_ros2/rviz/go2.rviz
```

**查看链路状态**：

```bash
ros2 topic echo /go2/status     # {"connected": true, "detail": "connected", ...}
```

---

## 4. 参数总表

所有参数均通过 `--ros-args -p 名称:=值` 传入。下文以命名空间 `robot_ns:=go2` 为例（话题前缀 `/go2/...`）。

### 4.1 连接参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_ip` | `192.168.123.161` | 机器人 IP（LocalSTA/LocalAP 模式） |
| `connection_method` | `local_sta` | `local_sta`（局域网）/ `local_ap`（直连热点）/ `remote`（云遥操作） |
| `serial_number` | 空 | 序列号连接 / remote 模式必填 |
| `username` `password` | 空 | remote 模式账号（Unitree 账号） |
| `aes_key` | 空 | 固件 ≥1.1.15 每设备 AES 密钥（32 位 hex） |
| `decoder` | `native` | 雷达点云解码器（`native` / `libvoxel`） |
| `reconnect_delay` | `5.0` | 断线自动重连间隔（s） |
| `lidar_switch` | `true` | 连接后自动打开 L2 雷达（`rt/utlidar/switch`） |

### 4.2 功能开关

| 参数 | 默认 | 控制的输出 |
|---|---|---|
| `read_lidar` | `true` | `/go2/lidar/points` |
| `read_scan` | `true` | `/go2/scan`（自动订阅点云与位姿，内部合成） |
| `read_imu` | `true` | `/go2/imu`、`/go2/imu/temperature` |
| `read_camera` | `true` | `/go2/camera/*`（同时开启视频轨） |
| `read_odom` | `true` | `/go2/odom`、`/tf` |
| `read_state` | `true` | `/go2/joint_states`、`/go2/battery`、`/go2/sport/*` |
| `enable_control` | `true` | 全部 `/go2/cmd/*` 控制接口 |

### 4.3 坐标系与 TF

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_ns` | `go2` | 话题命名空间前缀 |
| `odom_frame` / `base_frame` / `map_frame` | `odom` / `base_link` / `map` | 里程计 TF 链 |
| `imu_frame` / `camera_frame` / `lidar_frame` / `scan_frame` | `imu_link` / `camera_link` / `odom` / `base_link` | 各消息 header.frame_id |
| `publish_tf` | `true` | 发布 `odom → base_link` 动态 TF |
| `publish_map_tf` | `true` | 发布 `map → odom` 占位静态 TF；**外部 SLAM 发 map→odom 时必须置 false** |

> TF 树：`map → odom → base_link`。相机/IMU 的 frame 仅写入消息 header，不发布其 TF。

### 4.4 移动控制参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `cmd_rate` | `50.0` | 速度指令发送频率（Hz） |
| `vel_timeout` | `0.5` | 超过该时间未收到 `cmd/vel` 自动停车（s） |
| `max_linear_vel` | `1.0` | 线速度限幅（m/s） |
| `max_angular_vel` | `1.5` | 角速度限幅（rad/s） |
| `invert_x` `invert_y` `invert_z` | `false` | 方向取反（固件符号不一致时用） |

### 4.5 激光合成 / 相机参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `scan_robot_centric` | `true` | 扫描以机器人为中心（SLAM/Nav2 需要；false=odom 系投影） |
| `scan_num_beams` | `360` | 光束数（±π 均分） |
| `scan_range_min` / `scan_range_max` | `0.15` / `12.0` | 量程（m） |
| `scan_z_min` / `scan_z_max` | `0.05` / `0.60` | 参与投影的点云高度带（m，滤地面/天花板） |
| `jpeg_quality` | `80` | 相机 JPEG 质量 |
| `camera_fx` `camera_fy` `camera_cx` `camera_cy` | `0` | 相机内参（>0 时填入 CameraInfo，cx/cy=0 取图像中心） |

---

## 5. 读取数据接口（话题 → 标准消息）

频率为机器人推送上限（实测）；时间戳默认为本机接收时刻。命名空间以 `/go2` 为例。

### 5.1 传感器数据

| 话题 | 消息类型 | 频率 | 内容 |
|---|---|---|---|
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` | ~7.6 Hz | **L2 雷达点云**（体素地图，x/y/z float32，frame=`odom`，约 2–6 万点/帧） |
| `/go2/scan` | `sensor_msgs/LaserScan` | ~7.6 Hz | **2D 激光**（点云投影合成，360 束 ±π，0.15–12 m，frame=`base_link`） |
| `/go2/imu` | `sensor_msgs/Imu` | 20 Hz | **IMU**：`orientation`(w,x,y,z)、`angular_velocity`(rad/s)、`linear_acceleration`(m/s²) |
| `/go2/imu/temperature` | `sensor_msgs/Temperature` | 20 Hz | IMU 温度（℃） |
| `/go2/camera/image_compressed` | `sensor_msgs/CompressedImage` | ~14 Hz | **相机** JPEG（1280×720） |
| `/go2/camera/image_raw` | `sensor_msgs/Image` | ~14 Hz | 相机原始帧（bgr8 1280×720） |
| `/go2/camera/camera_info` | `sensor_msgs/CameraInfo` | ~14 Hz | 相机内参（`camera_fx` 等参数提供时填充） |
| `/go2/odom` | `nav_msgs/Odometry` | ~18.7 Hz | **里程计**：雷达里程计位姿（`pose`，twist 为 0） |
| `/tf` | `tf2_msgs/TFMessage` | ~18.7 Hz | `odom → base_link`（与 odom 同源） |
| `/tf_static` | `tf2_msgs/TFMessage` | 一次性 | `map → odom` 占位（`publish_map_tf:=false` 关闭） |

### 5.2 本体状态

| 话题 | 消息类型 | 频率 | 内容 |
|---|---|---|---|
| `/go2/joint_states` | `sensor_msgs/JointState` | 20 Hz | 12 关节角 `position`[rad]，`name` 序：FR/FL/RR/RL × hip/thigh/calf |
| `/go2/battery` | `sensor_msgs/BatteryState` | 20 Hz | `voltage`(V)、`current`(A，负=放电)、`percentage`(0–1)、`temperature`(℃) |
| `/go2/sport/velocity` | `geometry_msgs/TwistStamped` | 20 Hz | 机体速度 `linear`(m/s) + `angular.z` 偏航角速度（固件原值） |
| `/go2/sport/gait_type` | `std_msgs/Int32` | 20 Hz | **当前步态 id**（切换步态后用它回读确认） |
| `/go2/sport/mode` | `std_msgs/Int32` | 20 Hz | 运动状态机模式（固件原值） |
| `/go2/status` | `std_msgs/String` | 1 Hz | 链路状态 JSON：`{"connected":bool,"detail":"...","stamp":...}` |

---

## 6. 指令接口（标准消息 → 机器人）

> **约定**（REP-103 机体坐标系）：`linear.x` **+前** −后、`linear.y` **+左** −右、
> `angular.z` **+逆时针** −顺时针。若实际方向相反，用 `invert_x/y/z:=true` 取反。
>
> 所有指令的执行结果都会发布到 `/go2/cmd/result`（见 6.5）。

### 6.1 移动 — `/go2/cmd/vel`（`geometry_msgs/Twist`）

实现：内部以 50 Hz 流式发送 SDK `Move`(1008) 速度指令（前后/左右/旋转合成）。

| 字段 | 单位 | 含义 | 限幅 |
|---|---|---|---|
| `linear.x` | m/s | 前进(+) / 后退(−) | `max_linear_vel` |
| `linear.y` | m/s | 左移(+) / 右移(−) | `max_linear_vel` |
| `angular.z` | rad/s | 逆时针旋转(+) / 顺时针(−) | `max_angular_vel` |

**调用方式**（持续发布=持续移动；停止发布 0.5 s 自动停车）：

```bash
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{linear: {x: 0.3}}"     # 前进
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{linear: {x: -0.2}}"    # 后退
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{linear: {y: 0.3}}"     # 左移
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{linear: {y: -0.3}}"    # 右移
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{angular: {z: 0.5}}"    # 逆时针旋转
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{angular: {z: -0.5}}"   # 顺时针旋转
# 斜向+转向可组合，例如：
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{linear: {x: 0.3, y: 0.1}, angular: {z: 0.2}}"
# 停车：Ctrl+C 停止发布（0.5s 自动停），或显式：
ros2 topic pub --once /go2/cmd/vel geometry_msgs/Twist "{}"
```

### 6.2 状态切换 — `/go2/cmd/state`（`std_msgs/String`）

发布一次即执行；切换前节点**自动发停车指令**（安全）。取值：

| 取值（别名） | 功能 | SDK 指令(id) |
|---|---|---|
| `stand_up`（stand / 站立） | **站立** | StandUp **1004** |
| `stand_down`（down / lie / 趴下） | **趴下** | StandDown **1005** |
| `balance_stand`（balance / 平衡站立） | 平衡站立（移动前建议先执行） | BalanceStand **1002** |
| `damp`（阻尼 / 放松） | **阻尼模式**（全身放松，安全状态） | Damp **1001** |
| `recovery_stand`（recovery / 恢复） | **从阻尼/倒地恢复站立** | RecoveryStand **1006** |
| `stop`（停止） | 停止移动 | StopMove **1003** |
| `sit` | 坐下（附加） | Sit **1009** |
| `rise_sit` | 坐姿站起（附加） | RiseSit **1010** |

**调用方式**：

```bash
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: stand_up}"        # 站立
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: stand_down}"      # 趴下
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: damp}"            # 阻尼
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: recovery_stand}"  # 阻尼后恢复
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: balance_stand}"   # 平衡站立
```

### 6.3 步态切换（可移动状态）— `/go2/cmd/gait` + `/go2/cmd/gait_id`

**`/go2/cmd/gait`**（`std_msgs/String`）：按名称切换步态（MCF 开关式指令，固件 ≥1.1.7）。
取值（中文别名同样可用）：

| 取值（别名） | 步态 | SDK 指令(id) |
|---|---|---|
| `default`（默认 / trot / 小跑） | **默认步态（小跑）** | TrotRun **1062** |
| `classic`（经典 / 经典行走） | **经典步态** | ClassicWalk **2049** |
| `normal`（常规 / static / 静态行走） | **常规（静态）步态** | StaticWalk **1061** |
| `economic`（经济） | 经济步态 | EconomicGait **1063** |
| `free_walk`（自由行走） | 自由行走 | FreeWalk **2045** |
| `free_bound`（自由跳跃） | 自由跳跃 | FreeBound **2046** |
| `free_jump`（自由跳高） | 自由跳高 | FreeJump **2047** |
| `free_avoid`（避障行走） | 避障行走 | FreeAvoid **2048** |

- 语法：`"{data: <步态名>"}` = 进入该步态；`"{data: '<步态名> off>'}"` = 退出该步态（发 `{"data":false}`）。
- 切换后用 `/go2/sport/gait_type` 回读确认。

```bash
ros2 topic pub --once /go2/cmd/gait std_msgs/String "{data: default}"     # 进入默认步态
ros2 topic pub --once /go2/cmd/gait std_msgs/String "{data: classic}"     # 进入经典步态
ros2 topic pub --once /go2/cmd/gait std_msgs/String "{data: normal}"      # 进入常规（静态）步态
ros2 topic pub --once /go2/cmd/gait std_msgs/String "{data: classic off}" # 退出经典步态
ros2 topic echo /go2/sport/gait_type                                      # 回读当前步态 id
```

**`/go2/cmd/gait_id`**（`std_msgs/Int32`）：normal 运动模式下的 `SwitchGait`(1011) **原始步态 id**
直通口，适用于表外步态/固件差异：

```bash
ros2 topic pub --once /go2/cmd/gait_id std_msgs/Int32 "{data: 2}"
```

### 6.4 运动模式切换 — `/go2/cmd/mode`（`std_msgs/String`）

切换 motion_switcher 运动模式（`normal` / `ai` / `mcf`，`mfc` 为兼容拼写）。
切换耗时约 5–10 s；节点以回读为判据自动重试 3 次，结果发 `/go2/cmd/result`。

```bash
ros2 topic pub --once /go2/cmd/mode std_msgs/String "{data: normal}"   # 切回 normal 移动模式
ros2 topic pub --once /go2/cmd/mode std_msgs/String "{data: mcf}"      # 切到 mcf 多控模式
```

> **实测提示**：开机默认 `mcf`；执行过特殊动作后机器人可能"锁定"，移动无响应时切
> `normal` 解锁。6.3 的**按名步态指令是 MCF API**；normal 模式下请用 `gait_id`。

### 6.5 执行结果反馈 — `/go2/cmd/result`（`std_msgs/String`）

节点对每条状态/步态/模式指令回发一条 JSON：

```json
{"cmd": "state:stand_up", "ok": true, "code": 0, "detail": "站立 StandUp: "}
```

`ok=true` 表示 SDK 返回码为 0；`detail` 含 SDK 原始返回内容/失败原因。

```bash
ros2 topic echo /go2/cmd/result
```

---

## 7. 典型操作流程（完整命令序列）

```bash
# ① 启动（数据 + 控制）
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161
# 看到 "✔ WebRTC 已连接，数据转发中" 即就绪

# ② 另开终端：确认数据在流动
ros2 topic hz /go2/imu /go2/lidar/points /go2/odom

# ③ 进入可移动状态（步态）
ros2 topic pub --once /go2/cmd/gait std_msgs/String "{data: default}"
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: balance_stand}"

# ④ 移动（持续发布，Ctrl+C 停止即自动停车）
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{linear: {x: 0.3}}"   # 前进 3s
#   （Ctrl+C）→ 0.5s 自动停车
ros2 topic pub -r 20 /go2/cmd/vel geometry_msgs/Twist "{angular: {z: 0.5}}"  # 逆时针转

# ⑤ 切换步态
ros2 topic pub --once /go2/cmd/gait std_msgs/String "{data: classic}"

# ⑥ 结束：趴下 / 阻尼 / 恢复
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: stand_down}"
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: damp}"
ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: recovery_stand}"
```

---

## 8. 在自己的节点中使用（Python / C++ 示例）

### 8.1 Python 节点示例

```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

# ── 读取（订阅标准消息）──────────────────────────────────────────
node.create_subscription(Imu, '/go2/imu', on_imu, 10)
node.create_subscription(PointCloud2, '/go2/lidar/points', on_cloud, 5)

# ── 移动（持续发布；停止发布 0.5s 自动停车）──────────────────────
vel_pub = node.create_publisher(Twist, '/go2/cmd/vel', 10)
msg = Twist()
msg.linear.x = 0.3          # 前进 m/s
msg.angular.z = 0.2         # 逆时针 rad/s
vel_pub.publish(msg)

# ── 状态 / 步态 / 模式（发布一次即执行，结果见 /go2/cmd/result）──
state_pub = node.create_publisher(String, '/go2/cmd/state', 10)
state_pub.publish(String(data='stand_up'))       # 站立

gait_pub = node.create_publisher(String, '/go2/cmd/gait', 10)
gait_pub.publish(String(data='classic'))         # 经典步态

mode_pub = node.create_publisher(String, '/go2/cmd/mode', 10)
mode_pub.publish(String(data='normal'))          # 切回 normal 模式
```

**典型调用顺序**：`gait`(进入步态) → `state:balance_stand` → 发 `/go2/cmd/vel` →
停止发布（自动停）或 `state:stop` → `state:stand_down` / `state:damp`。

### 8.2 C++ 节点示例（rclcpp）

**① 构建配置**。`package.xml` 增加依赖：

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```

`CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(go2_client_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(go2_client_example src/go2_client_example.cpp)
ament_target_dependencies(go2_client_example
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

install(TARGETS go2_client_example DESTINATION lib/${PROJECT_NAME})
ament_package()
```

**② 示例源码** `src/go2_client_example.cpp`（完整可编译）：
订阅 IMU/里程计/执行结果，启动 2s 后进入默认步态并平衡站立，持续发布速度
前进 5s，然后停车趴下 —— 覆盖"读取 + 移动 + 状态/步态指令"全部用法：

```cpp
// go2_client_example.cpp —— 在自己的 C++ 节点中使用 go2_webrtc_ros2 接口
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Go2Client : public rclcpp::Node
{
public:
  Go2Client()
  : Node("go2_client")
  {
    // ── 读取：订阅标准消息 ──────────────────────────────────────
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/go2/imu", 10,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000, "imu wz=%.3f rad/s",
          msg->angular_velocity.z);
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/go2/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000, "odom x=%.2f y=%.2f",
          msg->pose.pose.position.x, msg->pose.pose.position.y);
      });

    // 指令执行结果：{"cmd","ok","code","detail"}（JSON 字符串）
    result_sub_ = create_subscription<std_msgs::msg::String>(
      "/go2/cmd/result", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        RCLCPP_INFO(get_logger(), "result: %s", msg->data.c_str());
      });

    // ── 控制：发布话题 ────────────────────────────────────
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/go2/cmd/vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/go2/cmd/state", 10);
    gait_pub_ = create_publisher<std_msgs::msg::String>("/go2/cmd/gait", 10);

    // 移动：持续发布（20~50Hz 均可），停止发布 0.5s 后自动停车
    vel_timer_ = create_wall_timer(20ms, [this]() {
      if (!moving_) {
        return;
      }
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.3;    // 前后：+前 −后 [m/s]
      cmd.linear.y = 0.0;    // 左右：+左 −右 [m/s]
      cmd.angular.z = 0.1;   // 旋转：+逆时针 −顺时针 [rad/s]
      vel_pub_->publish(cmd);
    });

    // 启动 2s 后（连接就绪）：进入默认步态 + 平衡站立（一次性指令）
    start_timer_ = create_wall_timer(2s, [this]() {
      start_timer_->cancel();
      std_msgs::msg::String msg;
      msg.data = "default";           // 步态：default/classic/normal/...
      gait_pub_->publish(msg);
      msg.data = "balance_stand";     // 状态：stand_up/stand_down/damp/...
      state_pub_->publish(msg);
      moving_ = true;
    });

    // 启动 7s 后：停车 + 趴下
    stop_timer_ = create_wall_timer(7s, [this]() {
      stop_timer_->cancel();
      moving_ = false;
      vel_pub_->publish(geometry_msgs::msg::Twist());   // 零速
      std_msgs::msg::String msg;
      msg.data = "stand_down";
      state_pub_->publish(msg);
    });
  }

private:
  bool moving_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_pub_;
  rclcpp::TimerBase::SharedPtr vel_timer_, start_timer_, stop_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2Client>());
  rclcpp::shutdown();
  return 0;
}
```

**③ 编译 / 运行**：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_client_example
source install/setup.bash

# 终端1：go2_node（数据 + 控制接口）
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161
# 终端2：本示例
ros2 run go2_client_example go2_client_example
```

**要点**：

- 话题名前缀 `/go2` 对应 `robot_ns` 参数；改命名空间时同步改话题名，
  或用 `--ros-args -r /go2/cmd/vel:=/robot1/cmd/vel` 重映射。
- 移动是"持续发布"语义：定时器停止发布 0.5s（`vel_timeout`）后自动停车，
  节点退出前建议发一帧零速（示例中 `stop_timer_` 里的写法）。
- 状态/步态/模式为"发布一次即执行"，执行结果统一回 `/go2/cmd/result`；
  也可用命令行代替：`ros2 topic pub --once /go2/cmd/state std_msgs/String "{data: stand_up}"`。

---

## 9. 与 SLAM / 导航配合

本包输出即 slam_toolbox / Nav2 所需输入：`/go2/scan` + TF `odom→base_link`。

```bash
# 建图/定位时必须关掉占位 map→odom（让 slam_toolbox 发布）
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161 \
  -p publish_map_tf:=false
```

与 `go2_slam_toolbox` 包配合的详细用法见该包 README（`scan_topic: /go2/scan`、
`odom_frame: odom`、`base_frame: base_link` 均为默认值，无需改动）。

---

## 10. 运行特性与注意事项

- **单连接**：GO2 只接受一个 WebRTC 对端——手机 App 或第二个 `go2_node` 会顶掉当前连接。
- **断线自动重连**：每 `reconnect_delay`(5s) 重试；断链期间指令结果为 `ok=false, "WebRTC 未连接"`。
- **移动安全**：`cmd/vel` 停止发布 `vel_timeout`(0.5s) 自动发零速；状态切换前自动停车；
  速度/角速度有 `max_linear_vel`/`max_angular_vel` 限幅。
- **步态指令为 MCF API**（固件 ≥1.1.7，机器人处于 `mcf` 模式时有效）；normal 模式用 `/go2/cmd/gait_id`。
- **点云即体素地图**：`/go2/lidar/points` 是被占体素中心（5 cm 栅格），不是原始扫描线点云。
- **2D 激光为投影合成**：按 `scan_z_min..z_max` 高度带取最近距离，供 2D SLAM/Nav2 使用。
- **时间戳**：默认本机接收时刻；TF/odom/scan 同源同时戳，可直接进 SLAM。
- **QoS**：全部 Reliable（深度 5–10），可直接对接 rviz2 / Nav2 / rosbag。
- **日志**：启动打印一行接口确认，之后静默（仅警告/错误），适合后台运行。

---

## 11. 故障排查

| 现象 | 处理 |
|---|---|
| 一直 "连接断开/失败" | 检查 `robot_ip`/网段；固件 ≥1.1.15 必须传 `aes_key`；确认没有手机 App 占用连接 |
| 有连接但无点云 | `lidar_switch:=true`；确认雷达旋转正常；`decoder` 换 `libvoxel` 试试 |
| `/go2/scan` 无输出 | 需要先收到机器人位姿（`/go2/odom`）；检查 `scan_z_min/z_max` 是否覆盖目标高度 |
| 移动无响应 | 看 `/go2/cmd/result`；先 `state:balance_stand`；执行过动作后 `mode:=normal` 解锁；检查 `/go2/sport/gait_type` 是否已进入步态 |
| 移动/旋转方向相反 | 对应 `invert_x:=true` / `invert_y:=true` / `invert_z:=true` |
| 速度太快/太慢 | 调 `max_linear_vel`/`max_angular_vel`；步态本身也影响实际速度 |
| 图像占用太高 | 不订阅 `/go2/camera/image_raw`（仅用 compressed），或 `read_camera:=false` |
| 与 SLAM TF 冲突 | 启动时加 `-p publish_map_tf:=false` |

---

## 12. 话题总览（速查）

```text
读取
  /go2/lidar/points            sensor_msgs/PointCloud2     雷达点云
  /go2/scan                    sensor_msgs/LaserScan       2D 激光
  /go2/imu                     sensor_msgs/Imu             IMU
  /go2/imu/temperature         sensor_msgs/Temperature     IMU 温度
  /go2/camera/image_compressed sensor_msgs/CompressedImage 相机 JPEG
  /go2/camera/image_raw        sensor_msgs/Image           相机原始帧
  /go2/camera/camera_info      sensor_msgs/CameraInfo      相机内参
  /go2/odom                    nav_msgs/Odometry           里程计
  /tf  /tf_static              tf2_msgs/TFMessage          odom→base_link / map→odom
  /go2/joint_states            sensor_msgs/JointState      12 关节角
  /go2/battery                 sensor_msgs/BatteryState    电池
  /go2/sport/velocity          geometry_msgs/TwistStamped  机体速度
  /go2/sport/gait_type         std_msgs/Int32              当前步态 id
  /go2/sport/mode              std_msgs/Int32              当前运动模式
  /go2/status                  std_msgs/String             链路状态 JSON

控制
  /go2/cmd/vel                 geometry_msgs/Twist         移动（前后左右+旋转）
  /go2/cmd/state               std_msgs/String             状态切换（站立/趴下/阻尼/恢复…）
  /go2/cmd/gait                std_msgs/String             步态切换（默认/经典/常规/…）
  /go2/cmd/gait_id             std_msgs/Int32              SwitchGait 原始 id
  /go2/cmd/mode                std_msgs/String             运动模式（normal/ai/mcf）
  /go2/cmd/result              std_msgs/String             指令执行结果（节点发布）
```
