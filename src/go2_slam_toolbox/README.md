# go2_slam_toolbox — GO2 slam_toolbox 建图包 · 使用手册

> 本文档两部分：**上篇 = 本包详细使用教程**（封装后的日常用法）；
> **下篇 = slam_toolbox 原生使用教程**（官方 launch / 服务 / 完整配置文件）。
> 后端为成熟 2D SLAM `slam_toolbox`（Ceres 位姿图 + 内置回环）；本包提供与原
> go2_slam 一致的启动与指令习惯，`go2_nav` 导航包与现有 rviz 配置**无需改动**。
> （原自研包 `go2_slam` 保留未删，随时可切回。）

```
go2 数据链(/go2/scan + TF odom->base_link)
        │
        ▼
slam_toolbox(async 建图 / localization 定位) ──► TF map->odom、/map
        │                                        │
        ▼                                        ▼
tb_adapter(转播层) ──► /slam/map(latched)   /go2/slam/pose   /go2/slam/status

建图控制/保存 = rviz SlamToolboxPlugin 面板（或 /slam_toolbox/* 原生服务）
```

---

# 上篇 · 本包使用教程

## 1. 安装

```bash
sudo apt install ros-humble-slam-toolbox          # 后端
cp -r go2_slam_toolbox ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
```

## 2. 前置：数据链路（每次建图前先起）

```bash
ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161 \
  -p publish_map_tf:=false -p read_camera:=false
```
⚠ **必须 `publish_map_tf:=false`**：map→odom 由 slam_toolbox 发布，数据链的占位 TF 会与之冲突。
数据链提供本包唯一需要的输入：`/go2/scan`（激光）+ TF `odom→base_link`（里程计）。

## 3. 三种模式详细流程

### 3.1 新建图（完整会话示例）

```bash
# 终端1：数据链（§2）
# 终端2：建图 —— 启动即开始建图，并自动打开 rviz 可视化
ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=new
# 不需要可视化时：rviz:=false
```

**rviz 自动启动**（配置 `rviz/go2_slam.rviz`），已打开的显示：

| 显示项 | 话题 | 样式 |
|---|---|---|
| LaserScan | `/go2/scan` | **红色点、点大小 0.03 m** |
| SlamMap | `/slam/map` | slam 输出的地图 |
| SlamPose | `/go2/slam/pose` | slam 输出的机器人姿态（绿色坐标轴） |
| Graph | `/slam_toolbox/graph_visualization` | 位姿图/回环（辅助观察） |

然后全程在 **rviz** 里操作：
1. `Panels → Add New Panel → SlamToolboxPlugin` —— 面板里 **Start/Pause**（暂停恢复建图）、
   **Save/Serialize**（保存地图与位姿图）、Clear（清图）；
2. 遥控器慢速走一圈 → 实时看图 → 面板暂停 → 面板保存。

> 没有 rviz 时保存也可用原生服务（见下篇 §11）：
> `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '<路径>'}}"`

### 3.2 在旧图上扩展建图（鼠标点选初始位姿）

```bash
ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=extend \
  map_file:=<包目录>/map/go2_map
```

启动后 rviz 自动打开（含 §3.1 全部显示），并额外显示**旧图**（`/slam/old_map`，
半透明参照层）供点选。**初始状态用鼠标确定机器人位于地图中的哪个位置**：

1. 点击 rviz 工具栏 **"2D Pose Estimate"（2D 位姿估计）**；
2. 在**旧图上点击机器人当前所在位置**，按住拖动箭头指向机器人朝向，松开；
3. tb_adapter 收到 `/initialpose` 后，按该位姿（`match_type=2` START_AT_GIVEN_POSE）
   自动载入旧图**继续建图** —— 起点/朝向与旧图严格对齐，防止里程计不匹配；
4. 正常走动建图，SlamToolboxPlugin 面板随时暂停/保存。

> ⚠ 未点选初始位姿前**不会载入旧图**（日志有明确提示）；只认第一次点选，
> 点错了需重启 launch 重新点。不想用鼠标时用旧行为：
> `extend_wait_pose:=false map_start_x:=<x> map_start_y:=<y> map_start_yaw:=<yaw>`
> （`map_file` = save_map 产物的位姿图 stem（同名 `.posegraph/.data`）；
> match_type 由 `extend_match_type` 决定，默认 1 从首节点续建）。

### 3.3 加载地图纯定位

```bash
ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=localization \
  map_file:=<包目录>/map/go2_map
```
- 输出 TF map→odom 与 `/go2/slam/pose`（导航可用）；地图不更新；
- 初始位姿默认 (0,0,0)（`config/mapper_params_localization.yaml` 的 `map_start_pose` 可改），
  或启动后 rviz **"2D Pose Estimate"** 随时修正；
- 定位模式**不自动启动 rviz**（需要时自行打开）：
  `rviz2 -d $(ros2 pkg prefix go2_slam_toolbox)/share/go2_slam_toolbox/rviz/go2_slam.rviz`。

## 4. 建图控制与保存（rviz 面板，无服务指令）

本包**不提供** start/stop/save 服务 —— 启动即建图，控制与保存统一走 rviz 面板：

| 操作 | rviz 面板（`Panels → SlamToolboxPlugin`） | 原生服务（无 rviz 时的等价命令） |
|---|---|---|
| 暂停/恢复建图 | 面板 **Start / Pause** 按钮 | `/slam_toolbox/pause_new_measurements`（toggle） |
| 保存地图 | 面板 **Save / Serialize** 按钮 | `/slam_toolbox/save_map`（栅格）+ `/slam_toolbox/serialize_map`（位姿图） |
| 清空当前图 | 面板 **Clear** | `/slam_toolbox/clear` |
| 手动回环优化 | — | `/slam_toolbox/loop_closure` |

**保存产物（栅格 + 位姿图各一份）**：
| 文件 | 用途 |
|---|---|
| `<stem>.yaml` + `<stem>.pgm` | 栅格地图：rviz 显示、map_server、go2_nav 直接用 |
| `<stem>.posegraph` + `<stem>.data` | **位姿图**：extend/localization 的 `map_file` 输入 |

保存路径由你在面板/命令里指定（例 `<包目录>/map/go2_map`，map 文件夹可先建好）。

## 5. 话题参考

| 话题 | 类型 | 说明 |
|---|---|---|
| `/slam/map` | `nav_msgs/OccupancyGrid` (latched) | 地图（适配层转播；slam_toolbox 原生 `/map` 同时存在） |
| `/go2/slam/pose` | `geometry_msgs/PoseStamped` | map 系位姿（20Hz，TF 转换） |
| `/go2/slam/status` | `std_msgs/String` | JSON：backend/mode/paused/pose |
| `/slam_toolbox/graph_visualization` | `visualization_msgs/MarkerArray` | 位姿图/回环可视化（原生） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | rviz 修正位姿（slam_toolbox 订阅） |

## 6. 参数参考

**launch 参数**：
| 参数 | 默认 | 说明 |
|---|---|---|
| `mode` | new | new / extend / localization |
| `map_file` | 空 | extend/localization：位姿图 stem |
| `extend_wait_pose` | true | extend：true=等 rviz 鼠标点选初始位姿后载入旧图；false=直接用 `map_start_*` |
| `map_start_x/y/yaw` | 0 | extend(`extend_wait_pose:=false`) 用 |
| `extend_match_type` | 1 | extend(`extend_wait_pose:=false`)：1=从首节点续建 2=给定位姿续建 |
| `rviz` | true | new/extend 自动启动 rviz（localization 永不自动启动） |
| `robot_ns` | go2 | 命名空间 |

**slam_toolbox 调参**（`config/mapper_params.yaml`，详见下篇 §4.3 注释）：常用
`max_laser_range`（8.0，配合成激光）、`minimum_travel_distance/heading`（0.25）、
`do_loop_closing`（true）、`map_update_interval`（2.0）、`resolution`（0.05）。

## 7. rviz

**new/extend 模式自动启动 rviz**（`rviz/go2_slam.rviz`；`rviz:=false` 关闭，
localization 永不自动启动）。配置已打开：

| 显示项 | 话题 | 样式 |
|---|---|---|
| LaserScan | `/go2/scan` | **红色点（FlatColor 255;0;0）、点大小 0.03 m** |
| SlamMap | `/slam/map` | slam 输出地图 |
| SlamPose | `/go2/slam/pose` | slam 输出机器人姿态（绿色坐标轴） |
| OldMap | `/slam/old_map` | 旧图参照（extend 自动加载，半透明） |
| InitialPose | `/initialpose` | 鼠标点选的初始位姿（橙色箭头） |
| Graph | `/slam_toolbox/graph_visualization` | 位姿图/回环边 |

工具栏含 **"2D Pose Estimate"（2D 位姿估计）**（发布 `/initialpose`）。
需要其他显示（TF、MarkerArray 等）可自行添加；`go2_nav.rviz` 配置仍可直接用
（话题名未变）。

## 8. 故障排查

| 现象 | 处理 |
|---|---|
| TF 报错/地图跳变 | 检查数据链是否带了 `publish_map_tf:=false` |
| `save_map` result=1 | 尚未收到地图（先走动几步） |
| rviz 面板没找到 | `Panels → Add New Panel → SlamToolboxPlugin`（slam_toolbox 自带插件） |
| extend 起点对不上 | 用 rviz "2D Pose Estimate" 在旧图上重新点选机器人位姿后重启 launch；或 `extend_wait_pose:=false` 后核对 `map_start_x/y/yaw` |
| 定位漂 | rviz "2D Pose Estimate" 重设初值 |
| 想看回环质量 | rviz 加 MarkerArray `/slam_toolbox/graph_visualization` |

---

# 下篇 · slam_toolbox 原生使用教程

## 9. 概念

- **位姿图（pose graph）**：节点=关键帧位姿，边=帧间匹配约束；回环边闭合后全局优化。
- **产物两种**：栅格图（yaml/pgm，可视化导航用）+ 位姿图（posegraph/data，续建/定位用）。
- **模式**：`mapping`（建图）与 `localization`（加载位姿图只定位）——是两个不同节点。

## 10. 官方 launch 文件

| 文件 | 用途 |
|---|---|
| `online_async_launch.py` | **异步建图（推荐）**：处理与建图并行，实时性好 |
| `online_sync_launch.py` | 同步建图：逐帧严格处理，慢速高精 |
| `localization_launch.py` | 定位模式 |
| `offline_launch.py` | 离线处理 rosbag（不联机） |
| `merge_maps_kinematic_launch.py` | 多段地图合并 |

```bash
# 原生建图（用本包提供的配置文件，可直接复用）：
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(ros2 pkg prefix go2_slam_toolbox)/share/go2_slam_toolbox/config/mapper_params.yaml

# 原生定位：
ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=$(ros2 pkg prefix go2_slam_toolbox)/share/go2_slam_toolbox/config/mapper_params_localization.yaml
```

## 11. 原生服务全表

| 服务 | 类型 | 请求 → 响应 | 示例 |
|---|---|---|---|
| `/slam_toolbox/pause_new_measurements` | `slam_toolbox/Pause` | 空 → `bool status`（**toggle 切换**） | `ros2 service call /slam_toolbox/pause_new_measurements slam_toolbox/srv/Pause` |
| `/slam_toolbox/save_map` | `slam_toolbox/SaveMap` | `std_msgs/String name` → `uint8 result`(0成功/1无图/255失败) | `... SaveMap "{name: {data: '/home/scy/mymap'}}"` |
| `/slam_toolbox/serialize_map` | `slam_toolbox/SerializePoseGraph` | `string filename` → `uint8 result` | `... SerializePoseGraph "{filename: '/home/scy/mymap'}"` |
| `/slam_toolbox/deserialize_map` | `slam_toolbox/DeserializePoseGraph` | `string filename` + `int8 match_type` + `geometry_msgs/Pose2D initial_pose` → 空 | `... DeserializePoseGraph "{filename: '/home/scy/mymap', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"` |
| `/slam_toolbox/loop_closure` | `slam_toolbox/LoopClosure` | 空 | 手动触发回环优化 |
| `/slam_toolbox/clear` | `slam_toolbox/Clear` | 空 | 清空当前图 |
| `/slam_toolbox/clear_queue` | `slam_toolbox/ClearQueue` | 空 | 清空测量队列 |
| `/slam_toolbox/add_submap` | `slam_toolbox/AddSubmap` | `string filename` | 添加子图 |
| `/slam_toolbox/toggle_interactive` | `slam_toolbox/ToggleInteractive` | 空 | 交互模式（配合 rviz 插件） |

**`match_type` 取值**（deserialize_map）：
| 值 | 含义 | 适用 |
|---|---|---|
| 1 `START_AT_FIRST_NODE` | 从图首节点续建 | **扩展建图**（狗从旧图起点进入） |
| 2 `START_AT_GIVEN_POSE` | 从给定位姿（`initial_pose`）续建 | 扩展建图（给准确位姿） |
| 3 `LOCALIZE_AT_POSE` | 定位到给定位姿 | 运行中切换到定位 |

## 12. 完整配置文件（建图用，与本包 `config/mapper_params.yaml` 相同）

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # 坐标系与话题（go2 数据链）
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /go2/scan
    mode: mapping

    debug_logging: false
    throttle_scans: 1                 # 每帧都处理
    transform_publish_period: 0.02    # map->odom 发布周期 (s)
    map_update_interval: 2.0          # /map 发布周期 (s)
    resolution: 0.05                  # 栅格分辨率 (m)——与体素一致
    min_laser_range: 0.15             # 合成 scan 近端过滤 (m)
    max_laser_range: 8.0              # 有效量程 (m)——远端稀疏,8m 内最稳
    minimum_time_interval: 0.2        # 帧间最小时间 (s)
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000       # 大图序列化需要大栈

    # 扫描匹配
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.25     # 走 0.25m 才建新关键帧（四足碎步多，取小）
    minimum_travel_heading: 0.25      # 转 0.25rad 才建新关键帧
    scan_buffer_size: 10              # 匹配缓存帧数
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5   # 帧间关联最大距离 (m)

    # 回环
    do_loop_closing: true
    loop_match_minimum_chain_size: 10       # 回环链最小长度
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45  # 回环接受阈值（防误闭环可调高）
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    loop_search_maximum_distance: 3.0       # 回环搜索半径 (m)

    # 相关性搜索
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

## 13. 定位配置文件（`config/mapper_params_localization.yaml` 节选）

```yaml
slam_toolbox:
  ros__parameters:
    # … 与上文相同的 solver 段 …
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /go2/scan
    mode: localization
    map_file_name: /home/scy/ros2_ws/src/go2_slam_toolbox/map/go2_map   # ← 位姿图 stem
    map_start_pose: [0.0, 0.0, 0.0]     # [x, y, yaw] 启动时机器人在 map 中的位姿
    # 其余匹配参数与建图配置一致
```

## 14. 原生典型工作流

```bash
# A. 建图会话
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<mapping.yaml>
# 走动建图 …
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/scy/mymap'}"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/scy/mymap'}}"

# B. 定位会话
ros2 launch slam_toolbox localization_launch.py slam_params_file:=<localization.yaml>
# （map_file_name 指向 mymap；rviz 2D Pose Estimate 设初值）

# C. 离线处理 bag
ros2 launch slam_toolbox offline_launch.py slam_params_file:=<mapping.yaml>
ros2 bag play my_scan_bag
```

## 15. 原生 rviz 插件

`Panels → Add New Panel → SlamToolboxPlugin`：交互式开关暂停/触发回环/清图；
配合 MarkerArray `/slam_toolbox/graph_visualization` 观察位姿图与回环边。
官方示例配置：`/opt/ros/humble/share/slam_toolbox/config/slam_toolbox_default.rviz`。

---

# 附 · 与原包（go2_slam）对照

| 原 go2_slam | 本包 | 备注 |
|---|---|---|
| `ros2 launch go2_slam go2_slam.launch.py mode:=new` | `ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=new` | 仅包名不同 |
| `/slam/start_mapping` 等指令服务 | **已移除** | 启动即建图；控制/保存用 rviz 面板或 `/slam_toolbox/*` 原生服务 |
| `map_file:=<yaml>` | `map_file:=<位姿图 stem>` | ⚠ 输入格式不同 |
| `/slam/map` `/go2/slam/pose` | **完全相同** | go2_nav/rviz 无感切换 |
| 运行时 `/slam/mode` 切模式 | 不支持（launch 参数决定） | slam_toolbox 建图/定位是不同节点 |
| 距离置信/动态扩图（自研特性） | 无 | 需要时切回 go2_slam（未删） |
