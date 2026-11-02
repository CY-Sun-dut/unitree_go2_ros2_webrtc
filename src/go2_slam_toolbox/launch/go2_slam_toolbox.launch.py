"""GO2 slam_toolbox 建图启动（用法与 go2_slam 相同）。

    ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=new
    ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=extend map_file:=<stem>
    ros2 launch go2_slam_toolbox go2_slam_toolbox.launch.py mode:=localization map_file:=<stem>

前置：go2 数据链路（publish_map_tf:=false —— map→odom 由 slam_toolbox 发布）。
map_file = save_map 产物的位姿图 stem（同名 .posegraph/.data）。
模式分支：new/extend → async_slam_toolbox_node（建图）；localization → localization 节点。

可视化：new/extend 自动启动 rviz（go2_slam.rviz：激光红点 0.03m、/slam/map 地图、
/go2/slam/pose 姿态、位姿图），localization 永不自动启动；rviz:=false 可全局关闭。

extend（继续建图）初始位姿：rviz 打开后显示旧图（/slam/old_map）作参照，
用工具栏 "2D Pose Estimate"（2D 位姿估计）在旧图上点选机器人当前所在位置并
拖动箭头定朝向 → tb_adapter 收到 /initialpose 后按该位姿载入旧图续建
（match_type=2），避免里程计/起点不匹配。不想用鼠标时：
  extend_wait_pose:=false map_start_x:=<x> map_start_y:=<y> map_start_yaw:=<yaw>
（旧行为，match_type 由 extend_match_type 决定）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("go2_slam_toolbox")
    arg = DeclareLaunchArgument
    ns = LaunchConfiguration("robot_ns")
    mode = LaunchConfiguration("mode")
    map_file = LaunchConfiguration("map_file")
    rviz = LaunchConfiguration("rviz")
    is_loc = PythonExpression(["'", mode, "' == 'localization'"])
    not_loc = PythonExpression(["'", mode, "' != 'localization'"])
    is_extend = PythonExpression(
        ["'", mode, "' == 'extend' and '", map_file, "' != ''"])
    # rviz：new/extend 自动启动；localization 永不自动启动
    use_rviz = PythonExpression(
        ["'", rviz, "' == 'true' and '", mode, "' != 'localization'"])

    common = {"scan_topic": "/go2/scan", "base_frame": "base_link",
              "odom_frame": "odom", "map_frame": "map"}

    return LaunchDescription([
        arg("robot_ns", default_value="go2"),
        arg("mode", default_value="new",
            description="new | extend | localization"),
        arg("map_file", default_value="",
            description="extend/localization：位姿图 stem（save_map 产物）"),
        arg("map_start_x", default_value="0.0"),
        arg("map_start_y", default_value="0.0"),
        arg("map_start_yaw", default_value="0.0"),
        arg("extend_wait_pose", default_value="true",
            description="extend：true=等 rviz 鼠标点选初始位姿后载入旧图；"
                        "false=直接用 map_start_* 载入"),
        arg("rviz", default_value="true",
            description="new/extend 自动启动 rviz（localization 永不自动启动）"),

        # new/extend：建图节点
        Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[os.path.join(pkg, "config", "mapper_params.yaml"), common],
            condition=IfCondition(not_loc),
        ),
        # localization：定位节点（加载位姿图）
        Node(
            package="slam_toolbox", executable="localization_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[os.path.join(pkg, "config", "mapper_params_localization.yaml"),
                        dict(common,
                             map_file_name=LaunchConfiguration("map_file"))],
            # 初始位姿用配置里的 map_start_pose: [x,y,yaw]（默认 0,0,0），
            # 或启动后用 rviz "2D Pose Estimate"(/initialpose) 修正
            condition=IfCondition(is_loc),
        ),

        # extend：旧图参考显示（栅格图，仅供鼠标点选初始位姿时参照）
        Node(
            package="nav2_map_server", executable="map_server",
            name="old_map_server", output="screen",
            parameters=[{
                "yaml_filename": ParameterValue(
                    [map_file, ".yaml"], value_type=str),
                "topic_name": "/slam/old_map",
                "frame_id": "map",
            }],
            condition=IfCondition(is_extend),
        ),
        # map_server 是 lifecycle 节点，需要管理器自动 configure+activate
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="old_map_lifecycle_manager", output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["old_map_server"],
            }],
            condition=IfCondition(is_extend),
        ),

        # 适配层：原包同名指令 + /slam/map + /go2/slam/pose
        Node(
            package="go2_slam_toolbox", executable="tb_adapter", name="tb_adapter",
            namespace=ns, output="screen",
            parameters=[{
                "robot_ns": ns,
                "mode": mode,
                "map_file": LaunchConfiguration("map_file"),
                "map_start_x": LaunchConfiguration("map_start_x"),
                "map_start_y": LaunchConfiguration("map_start_y"),
                "map_start_yaw": LaunchConfiguration("map_start_yaw"),
                "extend_wait_pose": LaunchConfiguration("extend_wait_pose"),
            }],
        ),

        # rviz（new/extend 自动启动；rviz:=false 关闭）
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(pkg, "rviz", "go2_slam.rviz")],
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
