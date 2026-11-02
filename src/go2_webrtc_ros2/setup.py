from setuptools import setup
from glob import glob
import os

package_name = "go2_webrtc_ros2"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name), glob("*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2-webrtc-ros",
    maintainer_email="user@todo.todo",
    description="Unitree GO2 single-node ROS2 interface: sensor data "
                "(lidar/scan/imu/camera/odom) and motion control, all "
                "standard message types, over one WebRTC connection.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "go2_node = go2_webrtc_ros2.go2_node:main",
        ],
    },
)
