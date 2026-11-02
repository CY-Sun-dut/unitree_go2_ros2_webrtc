#!/usr/bin/env python3
"""GO2 移动键盘测试（精简版）——只测 解锁/锁定 + 移动。

    ros2 run go2_webrtc_ros2 keyboard_teleop     # 独立终端

  u 解锁(enable)     l 锁定(disable)      空格 急停      Ctrl+C 退出
  w/s 前进/后退      a/d 左移/右移        q/e 逆时针/顺时针旋转

流程：启动后先按 u 解锁，再按移动键；松开 0.5s 自动发停止帧。
界面：帮助区固定；底部两行原地刷新（状态 / 最新指令）。
"""

import shutil
import sys
import time
import unicodedata

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

from rclpy.node import Node

HELP = """
╔══ GO2 移动键盘测试 keyboard_teleop ════════════════════════╗
║  第一步: 按 u 解锁(enable)，然后按移动键                    ║
║                                                            ║
║   u 解锁(enable)      l 锁定(disable)                      ║
║   w/s 前进/后退       a/d 左移/右移     q/e 逆时针/顺时针   ║
║   1 站立  2 趴下  3 平衡站立  4 恢复站立                    ║
║   5 阻尼  6 坐下  7 坐起   0 停止(StopMove)                ║
║   空格 急停(停止帧)   Ctrl+C 退出(自动停止)                 ║
╚════════════════════════════════════════════════════════════╝
"""

KEY_LABELS = {"w": "前进", "s": "后退", "a": "左移", "d": "右移",
              "q": "逆时针", "e": "顺时针", "u": "解锁(enable)",
              "l": "锁定(disable)", " ": "急停"}
STATE_KEYS = {
    "1": ("stand_up", "站立"), "2": ("stand_down", "趴下"),
    "3": ("balance_stand", "平衡站立"), "4": ("recovery_stand", "恢复站立"),
    "5": ("damp", "阻尼"), "6": ("sit", "坐下"),
    "7": ("rise_sit", "坐姿站起"), "0": ("stop", "停止移动"),
}


def _cols():
    return shutil.get_terminal_size((100, 24)).columns


def _disp_width(text):
    return sum(2 if unicodedata.east_asian_width(c) in ("W", "F") else 1
               for c in text)


def _truncate(text, width):
    if width <= 2:
        return text
    out, w = [], 0
    for c in text:
        cw = 2 if unicodedata.east_asian_width(c) in ("W", "F") else 1
        if w + cw > width - 1:
            return "".join(out) + "…"
        out.append(c)
        w += cw
    return "".join(out)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("robot_ns", "go2")
        self.declare_parameter("speed", 0.15)         # m/s → 摇杆 0.45（点按速度，实测减半）
        self.declare_parameter("turn_speed", 0.25)    # rad/s（旋转速度减半）
        self.declare_parameter("key_timeout", 0.5)    # s
        p = self.get_parameter
        self.ns = p("robot_ns").value.strip("/") or "go2"
        self.speed = float(p("speed").value)
        self.turn = float(p("turn_speed").value)
        self.key_timeout = float(p("key_timeout").value)

        self.pub_vel = self.create_publisher(Twist, f"/{self.ns}/cmd/vel", 10)
        self.cli_enable = self.create_client(SetBool, f"/{self.ns}/cmd/enable")
        self.states = {name: self.create_client(Trigger, f"/{self.ns}/cmd/{name}")
                       for name, _ in STATE_KEYS.values()}

        self.target = [0.0, 0.0, 0.0]
        self.last_key = 0.0
        self.unlocked = False
        self.last_cmd = "先按 u 解锁，再按 w/s/a/d/q/e 移动"
        self.create_timer(0.2, self._refresh)         # 5Hz 状态行刷新
        self.create_timer(1.0 / 20.0, self._publish_vel)   # 20Hz 速度发布

    # ── 速度发布（重写回归修复：移动必须持续发 /go2/cmd/vel） ──────────
    def _publish_vel(self):
        moving = any(abs(v) > 1e-9 for v in self.target)
        if moving and time.time() - self.last_key > self.key_timeout:
            self.target = [0.0, 0.0, 0.0]             # 松键超时 → 停止
            self.pub_vel.publish(Twist())
            self.last_cmd = "松开按键 → 已停止"
            return
        if moving:
            msg = Twist()
            msg.linear.x = self.target[0]
            msg.linear.y = self.target[1]
            msg.angular.z = self.target[2]
            self.pub_vel.publish(msg)

    # ── TUI（帮助区锚定不动，底部两行原地刷新） ─────────────────────────
    def _refresh(self, new_cmd=None):
        if new_cmd is not None:
            self.last_cmd = new_cmd
        moving = any(abs(v) > 1e-9 for v in self.target)
        t = self.target
        status = (f"状态: {'已解锁' if self.unlocked else '未解锁'} | "
                  f"vx={t[0]:+.2f} vy={t[1]:+.2f} wz={t[2]:+.2f} | "
                  f"{'运动中' if moving else '停止'}")
        width = _cols()
        status = _truncate(status, width)
        cmd_line = _truncate(f"最新指令: {self.last_cmd}", width)
        sys.stdout.write(f"\033[u\r\033[K{status}\n\033[K{cmd_line}")
        sys.stdout.flush()

    def _ui_init(self):
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()
        print(HELP)
        sys.stdout.write("\033[s")
        sys.stdout.flush()
        self._refresh()

    # ── 按键 ──────────────────────────────────────────────────────────
    def handle_key(self, key: str):
        label = KEY_LABELS.get(key) or (
            STATE_KEYS[key][1] if key in STATE_KEYS else None)
        if label is None:
            return

        if key == "u":
            self._enable(True, "解锁(enable)")
            return
        if key == "l":
            self.target = [0.0, 0.0, 0.0]
            self.pub_vel.publish(Twist())
            self._enable(False, "锁定(disable)")
            return
        if key == " ":
            self.target = [0.0, 0.0, 0.0]
            self.pub_vel.publish(Twist())
            self._refresh("空格 → 急停（已发停止帧）")
            return
        if key in STATE_KEYS:
            name, cname = STATE_KEYS[key]
            self._call_state(name, f"{key} → {cname}({name})")
            return

        if not self.unlocked:
            self._refresh(f"{key} → {label}：请先按 u 解锁")
            return

        if key == "w":
            self.target[0] = self.speed
        elif key == "s":
            self.target[0] = -self.speed
        elif key == "a":
            self.target[1] = self.speed
        elif key == "d":
            self.target[1] = -self.speed
        elif key == "q":
            self.target[2] = self.turn
        elif key == "e":
            self.target[2] = -self.turn
        self.last_key = time.time()
        self._refresh(f"{key} → {label}")

    def _enable(self, value, label):
        if not self.cli_enable.wait_for_service(timeout_sec=2.0):
            self._refresh(f"{label} ✘ 未找到 /{self.ns}/cmd/enable "
                          f"(motion_cmd_node 未启动或版本旧，请重启)")
            return
        req = SetBool.Request()
        req.data = bool(value)
        future = self.cli_enable.call_async(req)

        def done(fut):
            try:
                res = fut.result()
                if value:
                    self.unlocked = res.success
                else:
                    self.unlocked = False
                mark = "✔" if res.success else "✘"
                self._refresh(f"{label} {mark} {res.message}")
            except Exception as exc:
                self._refresh(f"{label} ✘ {exc}")
        future.add_done_callback(done)

    def _call_state(self, name, label):
        client = self.states[name]
        if not client.wait_for_service(timeout_sec=2.0):
            self._refresh(f"{label} ✘ 未找到 /{self.ns}/cmd/{name} "
                          f"(motion_cmd_node 未启动或版本旧，请重启)")
            return
        future = client.call_async(Trigger.Request())

        def done(fut):
            try:
                res = fut.result()
                mark = "✔" if res.success else "✘"
                self._refresh(f"{label} {mark} {res.message}")
            except Exception as exc:
                self._refresh(f"{label} ✘ {exc}")
        future.add_done_callback(done)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = KeyboardTeleop()

    if not sys.stdin.isatty():
        print("keyboard_teleop 需要在真实终端中运行（stdin 不是 TTY）")
        node.destroy_node()
        rclpy.shutdown()
        return

    import select
    import termios
    import tty
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    node._ui_init()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            try:
                ch = sys.stdin.read(1)
            except OSError:
                ch = ""
            if ch == "\x03":
                raise KeyboardInterrupt
            if ch:
                node.handle_key(ch)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        try:
            node.pub_vel.publish(Twist())
            sys.stdout.write("\n[teleop] 退出，已发送停止帧\n")
        except Exception:
            sys.stdout.write("\n[teleop] 退出\n")
        sys.stdout.flush()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
