from setuptools import setup
from glob import glob
import os

package_name = "go2_slam_toolbox"

setup(
    name=package_name,
    version="1.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name), glob("*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2-webrtc-ros",
    maintainer_email="user@todo.todo",
    description="slam_toolbox mapping wrapper for GO2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tb_adapter = go2_slam_toolbox.tb_adapter:main",
        ],
    },
)
