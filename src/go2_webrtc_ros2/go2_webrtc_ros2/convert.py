"""消息转换工具：JSON 字段取值、PointCloud2 构造、点云投影为 LaserScan。"""

import array
import math

import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud2, PointField


# ── JSON 字段取值 ─────────────────────────────────────────────────────
def fvec(data, key, length=None, default=0.0):
    """dict[key] -> float 列表（缺省补位/截断到 length）。"""
    value = data.get(key) if isinstance(data, dict) else None
    if not isinstance(value, (list, tuple)):
        value = []
    out = [float(v) for v in value]
    if length is not None:
        out = (out + [float(default)] * length)[:length]
    return out


def fnum(data, key, default=0.0):
    value = data.get(key) if isinstance(data, dict) else None
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def inum(data, key, default=0):
    value = data.get(key) if isinstance(data, dict) else None
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


# ── PointCloud2 ──────────────────────────────────────────────────────
def xyz_to_pointcloud2(points, stamp, frame_id):
    """(N,3) float 点集 -> sensor_msgs/PointCloud2（x/y/z float32）。"""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def u8_array(data):
    """bytes/ndarray -> array.array('B')（uint8[] 字段的高效赋值）。"""
    out = array.array("B")
    if isinstance(data, (bytes, bytearray, memoryview)):
        out.frombytes(bytes(data))
    else:
        out.frombytes(data.tobytes())
    return out


def quat_to_rot(qw, qx, qy, qz):
    """四元数 -> 3x3 旋转矩阵。"""
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


class LaserScanner:
    """把 3D 点云投影成 2D sensor_msgs/LaserScan。

    默认**机器人中心**语义：点云（odom 系）经机器人位姿换算到 base_link，
    每个角度取最近距离 —— 2D SLAM / Nav2 需要的正是这种扫描。
    """

    def __init__(self, *, angle_min=-math.pi, angle_max=math.pi, num_beams=360,
                 range_min=0.15, range_max=12.0, z_min=0.05, z_max=0.60,
                 robot_centric=True):
        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        self.num_beams = max(1, int(num_beams))
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.robot_centric = bool(robot_centric)
        self.angle_inc = (self.angle_max - self.angle_min) / self.num_beams

    def build(self, points, pose, stamp, frame_id):
        """points:(N,3) odom 系；pose:(t,rot) 或 None -> LaserScan | None。

        robot_centric 且位姿未知时返回 None（不输出错误的扫描）。
        """
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        ranges = np.full(self.num_beams, self.range_max, dtype=np.float32)

        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3) \
            if points is not None else np.empty((0, 3), dtype=np.float32)
        if pts.size:
            sel = pts[(pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)]
            if sel.size:
                if self.robot_centric:
                    if pose is None:
                        return None
                    t, rot = pose
                    xy = (sel[:, :3].astype(np.float64) - t) @ rot   # -> base 系
                else:
                    xy = sel[:, :2].astype(np.float64)
                x, y = xy[:, 0], xy[:, 1]
                r = np.hypot(x, y)
                theta = np.arctan2(y, x)
                bins = np.floor((theta - self.angle_min) / self.angle_inc).astype(np.int64)
                valid = (bins >= 0) & (bins < self.num_beams) & \
                        (r >= self.range_min) & (r <= self.range_max)
                if valid.any():
                    np.minimum.at(ranges, bins[valid], r[valid].astype(np.float32))

        scan.ranges = ranges.tolist()
        return scan
