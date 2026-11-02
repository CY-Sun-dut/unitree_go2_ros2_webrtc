from setuptools import setup

package_name = "go2_webrtc_test"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2-webrtc-ros",
    maintainer_email="user@todo.todo",
    description="Test programs for go2_webrtc_ros2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "keyboard_teleop = go2_webrtc_test.keyboard_teleop:main",
            "cmd_console = go2_webrtc_test.cmd_console:main",
        ],
    },
)
