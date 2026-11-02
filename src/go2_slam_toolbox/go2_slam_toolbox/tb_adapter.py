"""slam_toolbox 适配层（精简版）—— 启动即建图，控制/保存用 rviz 面板。

本节点只做兼容转播（go2_nav / rviz 话题不变）：
  /slam/map        nav_msgs/OccupancyGrid  转播 slam_toolbox 的 /map（latched）
  /go2/slam/pose   geometry_msgs/PoseStamped  TF map->base_link 转换（20Hz）
  /go2/slam/status std_msgs/String  JSON（1Hz）

建图控制与地图保存走 slam_toolbox 原生方式：
  · rviz 面板 Panels → SlamToolboxPlugin（开始/暂停/保存/序列化）
  · 或原生服务 /slam_toolbox/{pause_new_measurements,save_map,serialize_map,...}

mode:=extend（继续建图）初始位姿：
  extend_wait_pose:=true（默认）—— 等待 rviz "2D Pose Estimate" 的 /initialpose
  （鼠标在旧图上点选机器人当前位姿），按该位姿（match_type=2）载入旧图续建，
  防止里程计/起点不匹配；
  extend_wait_pose:=false —— 启动立即按 map_start_x/y/yaw 载入（旧行为，
  match_type 由 extend_match_type 决定）。
"""

import json
import math
import threading
import time

from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

from slam_toolbox.srv import DeserializePoseGraph


class TbAdapter(Node):
    def __init__(self):
        super().__init__("tb_adapter")
        d = self.declare_parameter
        d("robot_ns", "go2")
        d("mode", "new")                 # new | extend | localization
        d("map_file", "")                # extend/localization：位姿图 stem
        d("map_frame", "map")
        d("base_frame", "base_link")
        d("extend_match_type", 1)        # 1=从首节点续建 2=给定位姿续建
        d("map_start_x", 0.0)            # extend(match_type=2) 初始位姿
        d("map_start_y", 0.0)
        d("map_start_yaw", 0.0)
        d("extend_wait_pose", True)      # true=等 rviz 鼠标点选初始位姿
        p = self.get_parameter
        self.ns = p("robot_ns").value.strip("/") or "go2"
        self.mode = str(p("mode").value).lower()
        self.map_file = str(p("map_file").value)
        self.map_frame = p("map_frame").value
        self.base_frame = p("base_frame").value
        self.extend_match_type = int(p("extend_match_type").value)
        self.extend_wait_pose = bool(p("extend_wait_pose").value)
        self.map_start = (float(p("map_start_x").value),
                          float(p("map_start_y").value),
                          float(p("map_start_yaw").value))

        self.cb = ReentrantCallbackGroup()
        self.cli_des = self.create_client(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map",
            callback_group=self.cb)

        self.pub_pose = self.create_publisher(PoseStamped, f"/{self.ns}/slam/pose", 10)
        self.pub_status = self.create_publisher(String, f"/{self.ns}/slam/status", 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_map = self.create_publisher(OccupancyGrid, "/slam/map", map_qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1,
                                 callback_group=self.cb)

        # 鼠标初始位姿（rviz "2D Pose Estimate" -> /initialpose）
        self._pose_event = threading.Event()
        self._initial_pose = self.map_start
        self._extend_loaded = False
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10,
            callback_group=self.cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose = None
        self.create_timer(1.0 / 20.0, self._relay_pose)
        self.create_timer(1.0, self._relay_status)

        threading.Thread(target=self._startup, daemon=True).start()
        self.get_logger().info(
            f"→ /slam/map、/{self.ns}/slam/pose；模式={self.mode}（slam_toolbox 后端，"
            f"建图自动开始；控制/保存用 rviz SlamToolboxPlugin 或 /slam_toolbox/* 服务）")
        if self.mode == "extend" and self.map_file and self.extend_wait_pose:
            self.get_logger().info(
                "extend：请在 rviz 工具栏点击 \"2D Pose Estimate\"（2D 位姿估计），"
                "在旧图上点选机器人当前所在位置并拖动箭头定朝向 —— "
                "收到后自动按该位姿载入旧图继续建图（防止里程计不匹配）")

    # ── extend 载入 ─────────────────────────────────────────────────
    def _on_initial_pose(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._initial_pose = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw)
        self.get_logger().info(
            f"收到鼠标初始位姿: x={self._initial_pose[0]:.2f} "
            f"y={self._initial_pose[1]:.2f} "
            f"yaw={math.degrees(self._initial_pose[2]):.1f}°")
        self._pose_event.set()

    def _startup(self):
        """extend 模式：载入旧位姿图继续建图。"""
        if self.mode != "extend" or not self.map_file:
            return
        if self.extend_wait_pose:
            # 等待 rviz 鼠标点选的初始位姿（不限时；未点选前不载入）
            self._pose_event.wait()
            match_type = 2          # START_AT_GIVEN_POSE：按鼠标给定位姿续建
        else:
            match_type = self.extend_match_type
        self._load_extend(self._initial_pose, match_type)

    def _load_extend(self, pose, match_type):
        if self._extend_loaded:
            return
        self._extend_loaded = True
        if not self.cli_des.wait_for_service(timeout_sec=20.0):
            self.get_logger().error("deserialize_map 服务不可用，extend 载入失败")
            return
        req = DeserializePoseGraph.Request()
        req.filename = self.map_file
        req.match_type = match_type
        req.initial_pose = Pose2D(x=pose[0], y=pose[1], theta=pose[2])
        fut = self.cli_des.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 30:
            time.sleep(0.1)
        self.get_logger().info(
            f"已载入旧图继续建图: {self.map_file} "
            f"(match_type={match_type}, 初始位姿={pose[0]:.2f},{pose[1]:.2f},"
            f"{pose[2]:.2f})")

    # ── 转播 ─────────────────────────────────────────────────────────
    def _on_map(self, msg):
        self.pub_map.publish(msg)

    def _relay_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        msg = PoseStamped()
        msg.header.stamp = self.now()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.orientation = t.transform.rotation
        self.pose = msg
        self.pub_pose.publish(msg)

    def _relay_status(self):
        msg = String()
        msg.data = json.dumps({
            "backend": "slam_toolbox",
            "mode": self.mode,
            "pose": None if self.pose is None else
            [round(self.pose.pose.position.x, 3),
             round(self.pose.pose.position.y, 3)],
        })
        self.pub_status.publish(msg)

    def now(self):
        return self.get_clock().now().to_msg()


def main(args=None):
    rclpy.init(args=args)
    node = TbAdapter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
