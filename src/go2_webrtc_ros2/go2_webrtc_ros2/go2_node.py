"""GO2 机器人 ROS2 单节点接口（数据读取 + 运动控制，全部标准消息）。

    ros2 run go2_webrtc_ros2 go2_node --ros-args -p robot_ip:=192.168.123.161

一个节点 = 一条 WebRTC 连接 + 全部对外接口：

  读取（每项可用参数单独开关，默认全开）
    /<ns>/lidar/points            sensor_msgs/PointCloud2   L2 雷达体素点云
    /<ns>/scan                    sensor_msgs/LaserScan     2D 激光（点云投影合成）
    /<ns>/imu                     sensor_msgs/Imu           姿态/角速度/线加速度
    /<ns>/imu/temperature         sensor_msgs/Temperature   IMU 温度
    /<ns>/odom                    nav_msgs/Odometry         雷达里程计
    /tf  /tf_static               tf2_msgs/TFMessage        odom→base_link / map→odom
    /<ns>/camera/image_compressed sensor_msgs/CompressedImage  相机 JPEG
    /<ns>/camera/image_raw        sensor_msgs/Image         相机 bgr8
    /<ns>/camera/camera_info      sensor_msgs/CameraInfo    相机内参
    /<ns>/joint_states            sensor_msgs/JointState    12 关节角
    /<ns>/battery                 sensor_msgs/BatteryState  电池
    /<ns>/sport/velocity          geometry_msgs/TwistStamped 机体速度
    /<ns>/sport/gait_type         std_msgs/Int32            当前步态 id
    /<ns>/sport/mode              std_msgs/Int32            当前运动模式
    /<ns>/status                  std_msgs/String           链路状态 JSON (1Hz)

  控制（enable_control:=true 时可用，全部话题+标准消息）
    /<ns>/cmd/vel      geometry_msgs/Twist   移动（前后/平移/旋转）
    /<ns>/cmd/state    std_msgs/String       状态切换：stand_up/stand_down/damp/...
    /<ns>/cmd/gait     std_msgs/String       步态切换：default/classic/normal/...
    /<ns>/cmd/gait_id  std_msgs/Int32        SwitchGait(1011) 原始 id
    /<ns>/cmd/mode     std_msgs/String       运动模式：normal/ai/mcf
    /<ns>/cmd/result   std_msgs/String       （节点发布）指令执行结果 JSON

详细说明见包内 README.md。
"""

import json
import logging
import queue
import re
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (BatteryState, CameraInfo, CompressedImage, Image,
                             Imu, JointState, LaserScan, PointCloud2,
                             Temperature)
from std_msgs.msg import Int32, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .convert import (LaserScanner, fnum, fvec, inum, quat_to_rot, u8_array,
                      xyz_to_pointcloud2)
from .webrtc_link import WebRTCSource

# ── 机器人侧 RTC 话题（SDK 数据流 / 指令通道）────────────────────────
RT_SPORT_STATE = "rt/lf/sportmodestate"        # 20Hz 运动状态 + IMU
RT_LOW_STATE = "rt/lf/lowstate"                # 20Hz 电机 + 电池
RT_ROBOT_POSE = "rt/utlidar/robot_pose"        # 18.7Hz 雷达里程计位姿
RT_VOXEL = "rt/utlidar/voxel_map"              # 雷达点云（体素地图）
RT_VOXEL_COMPRESSED = "rt/utlidar/voxel_map_compressed"
RT_SPORT_MOD = "rt/api/sport/request"          # 运动指令（请求/应答）
RT_MOTION_SWITCHER = "rt/api/motion_switcher/request"   # 运动模式切换

# ── 常用运动指令 api_id（SPORT_CMD / SPORT_CMD_MCF）──────────────────
MOVE_API = 1008            # Move        {"x","y","z"}  速度指令（无应答）
SWITCH_GAIT_API = 1011     # SwitchGait  {"data": id}   normal 模式步态 id
STOP_MOVE_API = 1003       # StopMove

# 状态切换：名称 -> (api_id, 说明)
STATE_CMDS = {
    "stand_up":       (1004, "站立 StandUp"),
    "stand_down":     (1005, "趴下 StandDown"),
    "balance_stand":  (1002, "平衡站立 BalanceStand"),
    "damp":           (1001, "阻尼/全身放松 Damp"),
    "recovery_stand": (1006, "恢复站立 RecoveryStand（从阻尼/倒地恢复）"),
    "stop":           (1003, "停止移动 StopMove"),
    "sit":            (1009, "坐下 Sit"),
    "rise_sit":       (1010, "坐姿站起 RiseSit"),
}
STATE_ALIASES = {
    "stand": "stand_up", "up": "stand_up", "站立": "stand_up",
    "down": "stand_down", "lie": "stand_down", "趴下": "stand_down",
    "balance": "balance_stand", "平衡站立": "balance_stand",
    "阻尼": "damp", "放松": "damp",
    "recovery": "recovery_stand", "恢复": "recovery_stand", "恢复站立": "recovery_stand",
    "停止": "stop",
}

# 步态切换（MCF 开关式指令，{"data": bool}）：名称 -> (api_id, 说明)
GAITS = {
    "default":     (1062, "默认步态（小跑 TrotRun）"),
    "trot":        (1062, "小跑 TrotRun"),
    "classic":     (2049, "经典行走 ClassicWalk"),
    "normal":      (1061, "常规/静态行走 StaticWalk"),
    "static":      (1061, "静态行走 StaticWalk"),
    "static_walk": (1061, "静态行走 StaticWalk"),
    "economic":    (1063, "经济步态 EconomicGait"),
    "free_walk":   (2045, "自由行走 FreeWalk"),
    "free_bound":  (2046, "自由跳跃 FreeBound"),
    "free_jump":   (2047, "自由跳高 FreeJump"),
    "free_avoid":  (2048, "避障行走 FreeAvoid"),
}
GAIT_ALIASES = {
    "默认": "default", "默认步态": "default", "小跑": "trot",
    "经典": "classic", "经典步态": "classic", "经典行走": "classic",
    "常规": "normal", "常规步态": "normal", "静态": "static", "静态行走": "static",
    "经济": "economic", "经济步态": "economic",
    "自由行走": "free_walk", "自由跳跃": "free_bound",
    "自由跳高": "free_jump", "避障行走": "free_avoid",
}

# Unitree GO2 电机顺序：每腿 3 关节，FR -> FL -> RR -> RL
JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


class Go2Node(Node):
    def __init__(self):
        super().__init__("go2")

        # ── 参数声明 ─────────────────────────────────────────────────
        # 连接
        self.declare_parameter("robot_ip", "192.168.123.161")
        self.declare_parameter("connection_method", "local_sta")  # local_sta|local_ap|remote
        self.declare_parameter("serial_number", "")
        self.declare_parameter("username", "")
        self.declare_parameter("password", "")
        self.declare_parameter("aes_key", "")
        self.declare_parameter("decoder", "native")
        self.declare_parameter("reconnect_delay", 5.0)
        self.declare_parameter("lidar_switch", True)
        # 命名空间 / 坐标系
        self.declare_parameter("robot_ns", "go2")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("lidar_frame", "odom")
        self.declare_parameter("scan_frame", "base_link")
        # 功能开关（读取）
        self.declare_parameter("read_lidar", True)
        self.declare_parameter("read_scan", True)
        self.declare_parameter("read_imu", True)
        self.declare_parameter("read_camera", True)
        self.declare_parameter("read_odom", True)
        self.declare_parameter("read_state", True)
        # 功能开关（控制）
        self.declare_parameter("enable_control", True)
        # TF
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_map_tf", True)
        # 移动控制
        self.declare_parameter("cmd_rate", 50.0)          # Hz，Move 指令发送频率
        self.declare_parameter("vel_timeout", 0.5)        # s，超时自动停车
        self.declare_parameter("max_linear_vel", 1.0)     # m/s
        self.declare_parameter("max_angular_vel", 1.5)    # rad/s
        self.declare_parameter("invert_x", False)         # 前后方向取反
        self.declare_parameter("invert_y", False)         # 左右方向取反
        self.declare_parameter("invert_z", False)         # 旋转方向取反
        # 2D 激光合成
        self.declare_parameter("scan_robot_centric", True)
        self.declare_parameter("scan_num_beams", 360)
        self.declare_parameter("scan_range_min", 0.15)
        self.declare_parameter("scan_range_max", 12.0)
        self.declare_parameter("scan_z_min", 0.05)
        self.declare_parameter("scan_z_max", 0.60)
        # 相机
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("camera_fx", 0.0)          # 0 = 不填内参
        self.declare_parameter("camera_fy", 0.0)
        self.declare_parameter("camera_cx", 0.0)
        self.declare_parameter("camera_cy", 0.0)

        p = self.get_parameter
        # 功能开关
        self.read_lidar = bool(p("read_lidar").value)
        self.read_scan = bool(p("read_scan").value)
        self.read_imu = bool(p("read_imu").value)
        self.read_camera = bool(p("read_camera").value)
        self.read_odom = bool(p("read_odom").value)
        self.read_state = bool(p("read_state").value)
        self.enable_control = bool(p("enable_control").value)
        # 命名空间 / 坐标系
        self.ns = p("robot_ns").value.strip("/") or "go2"
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.map_frame = p("map_frame").value
        self.imu_frame = p("imu_frame").value
        self.camera_frame = p("camera_frame").value
        self.lidar_frame = p("lidar_frame").value
        self.scan_frame = p("scan_frame").value
        # 移动控制
        self.vel_timeout = float(p("vel_timeout").value)
        self.max_lin = abs(float(p("max_linear_vel").value))
        self.max_ang = abs(float(p("max_angular_vel").value))
        self._sgn_x = -1.0 if bool(p("invert_x").value) else 1.0
        self._sgn_y = -1.0 if bool(p("invert_y").value) else 1.0
        self._sgn_z = -1.0 if bool(p("invert_z").value) else 1.0
        # 相机
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY),
                             int(p("jpeg_quality").value)]
        self._cam_fx = float(p("camera_fx").value)
        self._cam_fy = float(p("camera_fy").value)
        self._cam_cx = float(p("camera_cx").value)
        self._cam_cy = float(p("camera_cy").value)

        self.scanner = LaserScanner(
            robot_centric=bool(p("scan_robot_centric").value),
            num_beams=int(p("scan_num_beams").value),
            range_min=float(p("scan_range_min").value),
            range_max=float(p("scan_range_max").value),
            z_min=float(p("scan_z_min").value),
            z_max=float(p("scan_z_max").value))

        # ── 数据话题发布器 ───────────────────────────────────────────
        outputs = []          # (话题, 类型) 用于启动打印

        if self.read_lidar:
            self.pub_points = self.create_publisher(
                PointCloud2, f"/{self.ns}/lidar/points", 5)
            outputs.append((f"/{self.ns}/lidar/points", "sensor_msgs/PointCloud2"))
        if self.read_scan:
            self.pub_scan = self.create_publisher(
                LaserScan, f"/{self.ns}/scan", 5)
            outputs.append((f"/{self.ns}/scan", "sensor_msgs/LaserScan"))
        if self.read_imu:
            self.pub_imu = self.create_publisher(Imu, f"/{self.ns}/imu", 10)
            self.pub_imu_temp = self.create_publisher(
                Temperature, f"/{self.ns}/imu/temperature", 10)
            outputs.append((f"/{self.ns}/imu", "sensor_msgs/Imu"))
            outputs.append((f"/{self.ns}/imu/temperature",
                            "sensor_msgs/Temperature"))
        if self.read_odom:
            self.pub_odom = self.create_publisher(
                Odometry, f"/{self.ns}/odom", 10)
            outputs.append((f"/{self.ns}/odom", "nav_msgs/Odometry"))
            if bool(p("publish_tf").value):
                self.tf_broadcaster = TransformBroadcaster(self)
                outputs.append(("/tf", "odom→base_link"))
            else:
                self.tf_broadcaster = None
            if bool(p("publish_map_tf").value):
                self._static_tf = StaticTransformBroadcaster(self)
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.now()
                tf_msg.header.frame_id = self.map_frame
                tf_msg.child_frame_id = self.odom_frame
                tf_msg.transform.rotation.w = 1.0
                self._static_tf.sendTransform(tf_msg)
                outputs.append(("/tf_static", f"{self.map_frame}→{self.odom_frame} 静态"))
        else:
            self.tf_broadcaster = None
        if self.read_camera:
            self.pub_jpeg = self.create_publisher(
                CompressedImage, f"/{self.ns}/camera/image_compressed", 10)
            self.pub_raw = self.create_publisher(
                Image, f"/{self.ns}/camera/image_raw", 10)
            self.pub_info = self.create_publisher(
                CameraInfo, f"/{self.ns}/camera/camera_info", 10)
            outputs.append((f"/{self.ns}/camera/image_compressed",
                            "sensor_msgs/CompressedImage"))
            outputs.append((f"/{self.ns}/camera/image_raw", "sensor_msgs/Image"))
            outputs.append((f"/{self.ns}/camera/camera_info",
                            "sensor_msgs/CameraInfo"))
        if self.read_state:
            self.pub_joints = self.create_publisher(
                JointState, f"/{self.ns}/joint_states", 10)
            self.pub_battery = self.create_publisher(
                BatteryState, f"/{self.ns}/battery", 10)
            self.pub_vel = self.create_publisher(
                TwistStamped, f"/{self.ns}/sport/velocity", 10)
            self.pub_gait_type = self.create_publisher(
                Int32, f"/{self.ns}/sport/gait_type", 10)
            self.pub_sport_mode = self.create_publisher(
                Int32, f"/{self.ns}/sport/mode", 10)
            outputs.append((f"/{self.ns}/joint_states", "sensor_msgs/JointState"))
            outputs.append((f"/{self.ns}/battery", "sensor_msgs/BatteryState"))
            outputs.append((f"/{self.ns}/sport/velocity",
                            "geometry_msgs/TwistStamped"))
            outputs.append((f"/{self.ns}/sport/gait_type", "std_msgs/Int32"))
            outputs.append((f"/{self.ns}/sport/mode", "std_msgs/Int32"))

        self.pub_status = self.create_publisher(
            String, f"/{self.ns}/status", 10)
        outputs.append((f"/{self.ns}/status", "std_msgs/String"))

        # ── 控制话题 ─────────────────────────────────────────────────
        self._target = [0.0, 0.0, 0.0]
        self._last_cmd = 0.0
        self._moving = False
        if self.enable_control:
            self.pub_result = self.create_publisher(
                String, f"/{self.ns}/cmd/result", 10)
            self.create_subscription(Twist, f"/{self.ns}/cmd/vel",
                                     self._on_cmd_vel, 10)
            self.create_subscription(String, f"/{self.ns}/cmd/state",
                                     self._on_state_cmd, 10)
            self.create_subscription(String, f"/{self.ns}/cmd/gait",
                                     self._on_gait_cmd, 10)
            self.create_subscription(Int32, f"/{self.ns}/cmd/gait_id",
                                     self._on_gait_id, 10)
            self.create_subscription(String, f"/{self.ns}/cmd/mode",
                                     self._on_mode_cmd, 10)
            cmd_rate = max(10.0, float(p("cmd_rate").value))
            self.create_timer(1.0 / cmd_rate, self._vel_tick)

            # 指令串行执行线程（request 阻塞等待 SDK 应答，不占执行器线程）
            self._cmd_queue = queue.Queue()
            self._worker_stop = threading.Event()
            self._worker = threading.Thread(target=self._cmd_worker,
                                            name="go2-cmd", daemon=True)
            self._worker.start()
            outputs.append((f"/{self.ns}/cmd/vel", "geometry_msgs/Twist"))
            outputs.append((f"/{self.ns}/cmd/state, cmd/gait", "std_msgs/String"))
            outputs.append((f"/{self.ns}/cmd/gait_id", "std_msgs/Int32"))
            outputs.append((f"/{self.ns}/cmd/mode", "std_msgs/String"))
            outputs.append((f"/{self.ns}/cmd/result", "std_msgs/String(反馈)"))

        # ── WebRTC 链路（按功能开关订阅数据流）────────────────────────
        streams, lidar_streams = {}, {}
        if self.read_imu or self.read_state:
            streams[RT_SPORT_STATE] = self._on_sport_state
        if self.read_state:
            streams[RT_LOW_STATE] = self._on_low_state
        if self.read_odom or self.read_scan:
            streams[RT_ROBOT_POSE] = self._on_robot_pose
        if self.read_lidar or self.read_scan:
            lidar_streams[RT_VOXEL] = self._on_lidar
            lidar_streams[RT_VOXEL_COMPRESSED] = self._on_lidar

        self._pose = None            # 最新机器人位姿 (t, rot)，供 scan 合成
        self._pose_lock = threading.Lock()
        self._scan_warned = False

        self.source = WebRTCSource(
            ip=p("robot_ip").value,
            connection_method=p("connection_method").value,
            serial_number=p("serial_number").value,
            username=p("username").value,
            password=p("password").value,
            aes_key=p("aes_key").value,
            decoder=p("decoder").value,
            enable_video=self.read_camera,
            lidar_switch=bool(p("lidar_switch").value) and bool(lidar_streams),
            reconnect_delay=float(p("reconnect_delay").value),
            streams=streams, lidar_streams=lidar_streams)
        self.source.on_video = self._on_video
        self.source.on_status = self._on_status
        self._connected = False
        self._detail = "connecting"
        self._fail_logged = 0
        self.create_timer(1.0, self._publish_status)
        self.source.start()

        # ── 启动打印（一行接口确认）───────────────────────────────────
        items = ", ".join(f"{t} [{m}]" for t, m in outputs)
        self.get_logger().info(f"→ {items}")
        self.get_logger().info(
            f"正在连接 {p('robot_ip').value} "
            f"({p('connection_method').value}) …")

    # ══ 数据回调（WebRTC asyncio 线程）════════════════════════════════
    def now(self):
        return self.get_clock().now().to_msg()

    def _on_sport_state(self, payload):
        """rt/lf/sportmodestate：IMU + 速度 + 步态/模式（20Hz）。"""
        if not isinstance(payload, dict):
            return
        stamp = self.now()

        if self.read_imu:
            imu = payload.get("imu_state")
            if isinstance(imu, dict):
                out = Imu()
                out.header.stamp = stamp
                out.header.frame_id = self.imu_frame
                quat = fvec(imu, "quaternion", 4, 1.0)   # GO2 顺序 [w, x, y, z]
                out.orientation.w = quat[0]
                out.orientation.x = quat[1]
                out.orientation.y = quat[2]
                out.orientation.z = quat[3]
                gyro = fvec(imu, "gyroscope", 3)
                out.angular_velocity.x = float(gyro[0])
                out.angular_velocity.y = float(gyro[1])
                out.angular_velocity.z = float(gyro[2])
                acc = fvec(imu, "accelerometer", 3)
                out.linear_acceleration.x = float(acc[0])
                out.linear_acceleration.y = float(acc[1])
                out.linear_acceleration.z = float(acc[2])
                self.pub_imu.publish(out)

                temp = Temperature()
                temp.header.stamp = stamp
                temp.header.frame_id = self.imu_frame
                temp.temperature = fnum(imu, "temperature")
                self.pub_imu_temp.publish(temp)

        if self.read_state:
            vel = TwistStamped()
            vel.header.stamp = stamp
            vel.header.frame_id = self.odom_frame
            v = fvec(payload, "velocity", 3)
            vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z = v
            vel.twist.angular.z = fnum(payload, "yaw_speed")
            self.pub_vel.publish(vel)
            self.pub_gait_type.publish(Int32(data=inum(payload, "gait_type")))
            self.pub_sport_mode.publish(Int32(data=inum(payload, "mode")))

    def _on_low_state(self, payload):
        """rt/lf/lowstate：12 关节角 + 电池（20Hz）。"""
        if not isinstance(payload, dict):
            return
        stamp = self.now()

        motors = payload.get("motor_state")
        if isinstance(motors, list) and motors:
            n = len(JOINT_NAMES)
            entries = [m for m in motors[:n] if isinstance(m, dict)]
            joints = JointState()
            joints.header.stamp = stamp
            joints.header.frame_id = self.base_frame
            joints.name = list(JOINT_NAMES)
            pos = [fnum(m, "q") for m in entries]
            joints.position = (pos + [0.0] * n)[:n]
            self.pub_joints.publish(joints)

        bms = payload.get("bms_state")
        if isinstance(bms, dict):
            out = BatteryState()
            out.header.stamp = stamp
            out.header.frame_id = self.base_frame
            out.voltage = fnum(payload, "power_v")             # 主母线电压 [V]
            out.current = inum(bms, "current") / 1000.0        # mA -> A，负=放电
            out.percentage = inum(bms, "soc") / 100.0          # 0..1
            out.temperature = float(inum(payload, "temperature_ntc1"))
            out.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                if out.current < 0 else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
            out.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            out.present = True
            self.pub_battery.publish(out)

    def _on_robot_pose(self, payload):
        """rt/utlidar/robot_pose：雷达里程计位姿（18.7Hz）。"""
        if not isinstance(payload, dict):
            return
        pose = payload.get("pose")
        if not isinstance(pose, dict):
            return
        pos = pose.get("position") if isinstance(pose.get("position"), dict) else {}
        ori = pose.get("orientation") if isinstance(pose.get("orientation"), dict) else {}
        t = np.array([float(pos.get("x", 0.0)), float(pos.get("y", 0.0)),
                      float(pos.get("z", 0.0))])
        qw = float(ori.get("w", 1.0))
        qx = float(ori.get("x", 0.0))
        qy = float(ori.get("y", 0.0))
        qz = float(ori.get("z", 0.0))
        with self._pose_lock:
            self._pose = (t, quat_to_rot(qw, qx, qy, qz))

        if not self.read_odom:
            return
        stamp = self.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(pos.get("x", 0.0))
        odom.pose.pose.position.y = float(pos.get("y", 0.0))
        odom.pose.pose.position.z = float(pos.get("z", 0.0))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = odom.pose.pose.position.z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def _on_lidar(self, meta, points):
        """L2 雷达体素点云（~7.6Hz）：发布 PointCloud2 并合成 2D 激光。"""
        if points is None or not points.size:
            return
        stamp = self.now()

        if self.read_lidar:
            frame = meta.get("frame_id") if isinstance(meta, dict) else None
            self.pub_points.publish(
                xyz_to_pointcloud2(points, stamp, frame or self.lidar_frame))

        if self.read_scan:
            with self._pose_lock:
                pose = self._pose
            scan = self.scanner.build(points, pose, stamp, self.scan_frame)
            if scan is not None:
                self.pub_scan.publish(scan)
            elif not self._scan_warned:
                self._scan_warned = True
                self.get_logger().warning("等待机器人位姿，/scan 暂不输出")

    def _on_video(self, img):
        """相机帧（~14Hz）：JPEG 压缩 + 原始 bgr8 + 内参。"""
        if not self.read_camera:
            return
        stamp = self.now()
        height, width = img.shape[0], img.shape[1]

        ok, buf = cv2.imencode(".jpg", img, self._jpeg_params)
        if ok:
            jpeg = CompressedImage()
            jpeg.header.stamp = stamp
            jpeg.header.frame_id = self.camera_frame
            jpeg.format = "jpeg"
            jpeg.data = u8_array(buf)
            self.pub_jpeg.publish(jpeg)

        raw = Image()
        raw.header.stamp = stamp
        raw.header.frame_id = self.camera_frame
        raw.height = height
        raw.width = width
        raw.encoding = "bgr8"
        raw.is_bigendian = 0
        raw.step = width * 3
        raw.data = u8_array(img)
        self.pub_raw.publish(raw)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.camera_frame
        info.width = width
        info.height = height
        if self._cam_fx > 0.0 and self._cam_fy > 0.0:
            cx = self._cam_cx if self._cam_cx > 0.0 else width / 2.0
            cy = self._cam_cy if self._cam_cy > 0.0 else height / 2.0
            info.k = [self._cam_fx, 0.0, cx, 0.0, self._cam_fy, cy, 0.0, 0.0, 1.0]
            info.p = [self._cam_fx, 0.0, cx, 0.0, 0.0, self._cam_fy, cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
        self.pub_info.publish(info)

    # ══ 移动控制 ═════════════════════════════════════════════════════
    def _on_cmd_vel(self, msg: Twist):
        """geometry_msgs/Twist -> 速度目标（x 前后, y 左右, z 旋转）。"""
        self._target = [
            self._sgn_x * max(-self.max_lin, min(self.max_lin, float(msg.linear.x))),
            self._sgn_y * max(-self.max_lin, min(self.max_lin, float(msg.linear.y))),
            self._sgn_z * max(-self.max_ang, min(self.max_ang, float(msg.angular.z))),
        ]
        self._last_cmd = time.monotonic()

    def _vel_tick(self):
        """按 cmd_rate 节奏流式发送 Move；指令超时后自动停车。"""
        active = any(abs(v) > 1e-9 for v in self._target) and \
            (time.monotonic() - self._last_cmd) <= self.vel_timeout
        if active:
            self._send_move(self._target)
            self._moving = True
        elif self._moving:
            self._send_move([0.0, 0.0, 0.0])
            self._moving = False

    def _send_move(self, v):
        self.source.publish_noreply(RT_SPORT_MOD, MOVE_API,
                                    {"x": v[0], "y": v[1], "z": v[2]})

    def _stop_motion(self):
        self._target = [0.0, 0.0, 0.0]
        if self._moving:
            self._send_move([0.0, 0.0, 0.0])
            self._moving = False

    # ══ 状态 / 步态 / 模式指令 ═══════════════════════════════════════
    def _on_state_cmd(self, msg: String):
        name = msg.data.strip().lower()
        name = STATE_ALIASES.get(name, name)
        if name not in STATE_CMDS:
            self._publish_result(f"state:{msg.data}", False, -1,
                                 f"未知状态指令，可用: {', '.join(STATE_CMDS)}")
            return
        self._cmd_queue.put(("state", name))

    def _on_gait_cmd(self, msg: String):
        parts = [x for x in re.split(r"[\s:;,]+", msg.data.strip().lower()) if x]
        if not parts:
            return
        name = GAIT_ALIASES.get(parts[0], parts[0])
        enable = True
        if len(parts) > 1:
            enable = parts[1] not in ("off", "0", "false", "no", "n")
        if name not in GAITS:
            self._publish_result(f"gait:{msg.data}", False, -1,
                                 f"未知步态，可用: {', '.join(GAITS)}")
            return
        self._cmd_queue.put(("gait", (name, enable)))

    def _on_gait_id(self, msg: Int32):
        self._cmd_queue.put(("gait_id", int(msg.data)))

    def _on_mode_cmd(self, msg: String):
        name = msg.data.strip().lower()
        if name == "mfc":
            name = "mcf"
        if name not in ("normal", "ai", "mcf"):
            self._publish_result(f"mode:{msg.data}", False, -1,
                                 "未知运动模式，可用: normal, ai, mcf")
            return
        self._cmd_queue.put(("mode", name))

    def _cmd_worker(self):
        while not self._worker_stop.is_set():
            try:
                kind, arg = self._cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._run_command(kind, arg)
            except Exception as exc:
                self.get_logger().error(f"指令执行异常: {exc}")
                self._publish_result(f"{kind}:{arg}", False, -1, str(exc))

    def _run_command(self, kind, arg):
        if kind == "state":
            self._stop_motion()                     # 安全：先停车再切换状态
            api_id, label = STATE_CMDS[arg]
            ok, code, data = self.source.request(RT_SPORT_MOD, api_id, timeout=8.0)
            self._publish_result(f"state:{arg}", ok, code,
                                 f"{label}: {data}".strip())
            self._log_cmd(f"state {arg}: code={code} {data}", ok)
        elif kind == "gait":
            name, enable = arg
            api_id, label = GAITS[name]
            ok, code, data = self.source.request(
                RT_SPORT_MOD, api_id, {"data": bool(enable)}, timeout=6.0)
            self._publish_result(f"gait:{name}{' off' if not enable else ''}",
                                 ok, code, f"{label}: {data}".strip())
            self._log_cmd(f"gait {name}={'on' if enable else 'off'}: "
                          f"code={code} {data}", ok)
        elif kind == "gait_id":
            ok, code, data = self.source.request(
                RT_SPORT_MOD, SWITCH_GAIT_API, {"data": int(arg)}, timeout=6.0)
            self._publish_result(f"gait_id:{arg}", ok, code, data)
        elif kind == "mode":
            ok, code, data = self._switch_mode(arg)
            self._publish_result(f"mode:{arg}", ok, code, data)
            self._log_cmd(f"mode {arg}: {data}", ok)

    def _log_cmd(self, text, ok):
        """指令结果日志。注意：rclpy 要求同一调用点的 severity 固定，
        且带 throttle 时严禁在 info/warning 之间切换（会抛 ValueError）。"""
        if ok:
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)

    def _switch_mode(self, name):
        """运动模式切换（回读判据：固件返回码不稳定，以 1001 回读为准）。"""
        cur = self._query_motion_mode()
        if cur == name:
            return True, 0, f"已在 {name} 模式，无需切换"
        self.source.request(RT_SPORT_MOD, STOP_MOVE_API, timeout=4.0)  # 解锁
        last = ""
        for _attempt in range(3):
            ok, code, data = self.source.request(
                RT_MOTION_SWITCHER, 1002, {"name": name}, timeout=8.0)
            last = f"code={code} {data}".strip()
            time.sleep(1.0)
            cur = self._query_motion_mode()
            if cur == name:
                return True, code, f"已切换到 {cur}"
        return False, -1, (f"模式切换失败: {last}；当前模式: {cur or '未知'}；"
                           f"建议先 state:=damp 或 stand_down 后重试")

    def _query_motion_mode(self):
        ok, _code, data = self.source.request(RT_MOTION_SWITCHER, 1001,
                                              timeout=6.0)
        if ok:
            try:
                return str(json.loads(data).get("name", ""))
            except Exception:
                pass
        return ""

    def _publish_result(self, cmd, ok, code, detail):
        msg = String()
        msg.data = json.dumps({"cmd": cmd, "ok": bool(ok), "code": int(code),
                               "detail": str(detail)}, ensure_ascii=False)
        self.pub_result.publish(msg)

    # ══ 链路状态 ═════════════════════════════════════════════════════
    def _on_status(self, connected, detail):
        self._connected = connected
        self._detail = detail
        if connected:
            self.get_logger().info("✔ WebRTC 已连接，数据转发中")
            self._fail_logged = 0
        else:
            self._fail_logged += 1
            if self._fail_logged == 1 or self._fail_logged % 12 == 0:
                self.get_logger().warning(
                    f"连接断开/失败：{detail}"
                    f"（每 {self.source.reconnect_delay:.0f}s 自动重连）")

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({"connected": self._connected,
                               "detail": self._detail, "stamp": time.time()})
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._worker_stop.set()
        try:
            self.source.stop()
        finally:
            super().destroy_node()


def main(args=None):
    logging.basicConfig(level=logging.WARNING)
    rclpy.init(args=args)
    node = Go2Node()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass          # SIGTERM 时上下文已关闭，忽略重复 shutdown


if __name__ == "__main__":
    main()
