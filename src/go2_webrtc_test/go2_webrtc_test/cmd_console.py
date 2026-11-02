#!/usr/bin/env python3
"""GO2 指令功能测试控制台 —— 菜单式测试全部 6 类指令。

    ros2 run go2_webrtc_test cmd_console     # 独立终端

前置：go2_cmd/go2_all 的 bridge + 指令节点已启动。
每一项执行后显示 SDK 返回码（✔/✘ code=...），可逐条验证功能。
"""

import time

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, Int32, String
from std_srvs.srv import SetBool, Trigger

import rclpy
from rclpy.node import Node

# 复用主包的指令 id 表（单一数据源）
from go2_webrtc_ros2.state_cmd_node import STATE_ACTIONS
from go2_webrtc_ros2.action_cmd_node import ACTIONS

NS = "go2"

TRIGGER_SERVICES = (
    list(STATE_ACTIONS) + ["mode_normal", "mode_ai", "mode_mcf",
                           "get_motion_mode", "get_speed_level", "get_state",
                           "get_obstacle_avoid", "get_volume", "get_brightness"]
    + list(ACTIONS))
SETBOOL_SERVICES = ("enable", "obstacle_avoid", "continuous_gait",
                    "joystick", "pose_mode", "stand_out")


class Console(Node):
    def __init__(self):
        super().__init__("cmd_console")
        self.trig = {n: self.create_client(Trigger, f"/{NS}/cmd/{n}")
                     for n in TRIGGER_SERVICES}
        self.setb = {n: self.create_client(SetBool, f"/{NS}/cmd/{n}")
                     for n in SETBOOL_SERVICES}
        self.pub_vel = self.create_publisher(Twist, f"/{NS}/cmd/vel", 10)
        self.pub_avoid = self.create_publisher(Twist, f"/{NS}/cmd/vel_avoid", 10)
        self.pub_pose = self.create_publisher(Vector3, f"/{NS}/cmd/pose", 10)
        self.pub_height = self.create_publisher(Float64, f"/{NS}/cmd/body_height", 10)
        self.pub_foot = self.create_publisher(Float64, f"/{NS}/cmd/foot_raise_height", 10)
        self.pub_speed = self.create_publisher(Int32, f"/{NS}/cmd/speed_level", 10)
        self.pub_gait = self.create_publisher(Int32, f"/{NS}/cmd/gait", 10)
        self.pub_mode = self.create_publisher(String, f"/{NS}/cmd/motion_mode", 10)
        self.pub_volume = self.create_publisher(Int32, f"/{NS}/cmd/volume", 10)
        self.pub_bright = self.create_publisher(Int32, f"/{NS}/cmd/brightness", 10)
        self.pub_led = self.create_publisher(String, f"/{NS}/cmd/led", 10)
        self.pub_action = self.create_publisher(Int32, f"/{NS}/cmd/action_id", 10)

    # ── 执行工具 ───────────────────────────────────────────────────────
    def call_trigger(self, name):
        cli = self.trig.get(name)
        if cli is None or not cli.wait_for_service(timeout_sec=5.0):
            print(f"    ✘ 未找到服务 /{NS}/cmd/{name}"
                  f"（确认 go2_cmd/go2_all 已用最新构建启动：pkill -f go2_webrtc 后重新 launch）")
            return
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 15:
            rclpy.spin_once(self, timeout_sec=0.1)
        res = fut.result() if fut.done() else None
        if res is None:
            print(f"    ✘ {name}: 响应超时")
        else:
            print(f"    {'✔' if res.success else '✘'} {name}: {res.message}")

    def call_setbool(self, name, value):
        cli = self.setb.get(name)
        if cli is None or not cli.wait_for_service(timeout_sec=5.0):
            print(f"    ✘ 未找到服务 /{NS}/cmd/{name}"
                  f"（确认 go2_cmd/go2_all 已用最新构建启动：pkill -f go2_webrtc 后重新 launch）")
            return
        req = SetBool.Request()
        req.data = bool(value)
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 15:
            rclpy.spin_once(self, timeout_sec=0.1)
        res = fut.result() if fut.done() else None
        if res is None:
            print(f"    ✘ {name}: 响应超时")
        else:
            print(f"    {'✔' if res.success else '✘'} {name}: {res.message}")

    @staticmethod
    def ask_float(prompt, default=0.0):
        try:
            return float(input(f"    {prompt} [{default}]: ").strip() or default)
        except ValueError:
            return default

    @staticmethod
    def ask_int(prompt, default=0):
        try:
            return int(input(f"    {prompt} [{default}]: ").strip() or default)
        except ValueError:
            return default

    # ── 各类子菜单 ─────────────────────────────────────────────────────
    def menu_motion(self):
        print("  [1] 解锁 enable   [2] 锁定 disable   [3] 发送 vel   [4] 发送 vel_avoid")
        s = input("  选择> ").strip()
        if s == "1":
            self.call_setbool("enable", True)
        elif s == "2":
            self.call_setbool("enable", False)
        elif s in ("3", "4"):
            x = self.ask_float("x 前后(+前)")
            y = self.ask_float("y 左右(+左)")
            z = self.ask_float("z 旋转(+逆时针)")
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = x, y, z
            pub = self.pub_vel if s == "3" else self.pub_avoid
            print("    持续发送 1s 后自动停止帧...")
            for _ in range(50):
                pub.publish(msg)
                time.sleep(0.02)
            pub.publish(Twist())
            print("    ✔ 已发送")

    def menu_state(self):
        for i, (name, (_id, label)) in enumerate(STATE_ACTIONS.items(), 1):
            print(f"  [{i}] {label}")
        idx = self.ask_int("选择", 0)
        names = list(STATE_ACTIONS)
        if 1 <= idx <= len(names):
            self.call_trigger(names[idx - 1])

    def menu_adjust(self):
        print("  [1] 姿态 Euler   [2] 机身高度   [3] 抬腿高度   [4] 速度档"
              "   [5] 步态 id\n  [6] 避障开关   [7] 连续步态   [8] 遥控器"
              "   [9] 姿态模式Pose   [10] 倒立StandOut⚠   [11] 查询避障")
        s = input("  选择> ").strip()
        if s == "1":
            r = self.ask_float("roll")
            p = self.ask_float("pitch")
            y = self.ask_float("yaw")
            self.pub_pose.publish(Vector3(x=r, y=p, z=y))
            print("    ✔ 已发送 Euler")
        elif s == "2":
            self.pub_height.publish(Float64(data=self.ask_float("高度 0.05-0.45", 0.30)))
            print("    ✔ 已发送")
        elif s == "3":
            self.pub_foot.publish(Float64(data=self.ask_float("抬腿 ±0.10", 0.0)))
            print("    ✔ 已发送")
        elif s == "4":
            self.pub_speed.publish(Int32(data=max(1, min(3, self.ask_int("速度档 1-3", 1)))))
            print("    ✔ 已发送")
        elif s == "5":
            self.pub_gait.publish(Int32(data=self.ask_int("步态 id", 0)))
            print("    ✔ 已发送")
        elif s == "6":
            self.call_setbool("obstacle_avoid", input("    开? [y/N] ").strip() == "y")
        elif s == "7":
            self.call_setbool("continuous_gait", input("    开? [y/N] ").strip() == "y")
        elif s == "8":
            self.call_setbool("joystick", input("    开? [y/N] ").strip() == "y")
        elif s == "9":
            self.call_setbool("pose_mode", input("    开? [y/N] ").strip() == "y")
        elif s == "10":
            self.call_setbool("stand_out", input("    开? [y/N] ").strip() == "y")
        elif s == "11":
            self.call_trigger("get_obstacle_avoid")

    def menu_mode(self):
        print("  [1] normal   [2] ai   [3] mcf   [4] 输入模式名   [5] 查询模式"
              "   [6] 查询速度档   [7] 查询运动状态")
        s = input("  选择> ").strip()
        if s == "1":
            self.call_trigger("mode_normal")
        elif s == "2":
            self.call_trigger("mode_ai")
        elif s == "3":
            self.call_trigger("mode_mcf")
        elif s == "4":
            name = input("    模式名: ").strip()
            if name:
                self.pub_mode.publish(String(data=name))
                print("    ✔ 已发送（回读 get_motion_mode 确认）")
        elif s == "5":
            self.call_trigger("get_motion_mode")
        elif s == "6":
            self.call_trigger("get_speed_level")
        elif s == "7":
            self.call_trigger("get_state")

    def menu_action(self):
        names = list(ACTIONS)
        for i, (name, (_id, label)) in enumerate(ACTIONS.items(), 1):
            print(f"  [{i:2d}] {label:<22s} id={_id}")
        print(f"  [99] action_id 直通（输入任意 SPORT_CMD id）")
        idx = self.ask_int("选择", 0)
        if idx == 99:
            self.pub_action.publish(Int32(data=self.ask_int("SPORT_CMD id")))
            print("    ✔ 已发送")
        elif 1 <= idx <= len(names):
            self.call_trigger(names[idx - 1])

    def menu_vui(self):
        print("  [1] 音量   [2] 亮度   [3] 灯色   [4] 查询音量   [5] 查询亮度")
        s = input("  选择> ").strip()
        if s == "1":
            self.pub_volume.publish(Int32(data=max(0, min(10, self.ask_int("音量 0-10", 5)))))
            print("    ✔ 已发送")
        elif s == "2":
            self.pub_bright.publish(Int32(data=max(0, min(10, self.ask_int("亮度 0-10", 5)))))
            print("    ✔ 已发送")
        elif s == "3":
            print("    white/red/yellow/blue/green/cyan/purple")
            self.pub_led.publish(String(data=input("    颜色: ").strip()))
            print("    ✔ 已发送")
        elif s == "4":
            self.call_trigger("get_volume")
        elif s == "5":
            self.call_trigger("get_brightness")


def main():
    rclpy.init()
    node = Console()
    menus = {"1": node.menu_motion, "2": node.menu_state, "3": node.menu_adjust,
             "4": node.menu_mode, "5": node.menu_action, "6": node.menu_vui}
    print("""
╔══ GO2 指令功能测试控制台 ════════════════════════╗
║  [1] 移动类   [2] 状态转换   [3] 姿态/参数        ║
║  [4] 模式切换 [5] 动作类     [6] 灯光/音量        ║
║  [0] 退出                                        ║
╚═════════════════════════════════════════════════╝""")
    try:
        while True:
            s = input("\n主菜单> ").strip()
            if s == "0":
                break
            fn = menus.get(s)
            if fn:
                try:
                    fn()
                except Exception as exc:
                    print(f"    ✘ {exc}")
            else:
                print("  无效选择")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("退出")


if __name__ == "__main__":
    main()
