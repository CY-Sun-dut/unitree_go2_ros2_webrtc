"""WebRTC 连接封装 —— ROS 节点与 GO2 之间的唯一数据链路。

GO2 只接受一个 WebRTC 对端，本类持有该连接（专用 asyncio 线程 + 断线自动重连）。
对上层提供三种能力：

  1. 数据订阅   : ``streams`` / ``lidar_streams`` 登记的回调在 asyncio 线程被调用
  2. 请求/应答  : ``request(rtc_topic, api_id, parameter)`` 线程安全，阻塞等待 SDK 回复
  3. 单向发送   : ``publish_noreply(rtc_topic, api_id, parameter)`` 高频指令（Move 等）

连接参数与 SDK 一致（LocalSTA / LocalAP / Remote），固件 ≥ 1.1.15 需要 aes_128_key。
"""

import asyncio
import json
import logging
import random
import threading
import time

import numpy as np

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection
from unitree_webrtc_connect.constants import WebRTCConnectionMethod

logger = logging.getLogger("go2_webrtc_ros2")

# 屏蔽 aiortc/av 的编解码刷屏（H264 "non-existing PPS" 等）
for _name in ("aiortc", "aiortc.codecs", "aiortc.codecs.h264", "pyee", "av"):
    logging.getLogger(_name).setLevel(logging.ERROR)
try:
    import av.logging
    av.logging.set_level(av.logging.FATAL)
except Exception:
    pass

_CONNECTION_METHODS = {
    "local_sta": WebRTCConnectionMethod.LocalSTA,
    "local_ap": WebRTCConnectionMethod.LocalAP,
    "remote": WebRTCConnectionMethod.Remote,
}


def _jsonable(obj, depth=0):
    """把 numpy/bytes 等不可 JSON 化的对象转成可打印的摘要（仅用于日志/元数据）。"""
    if depth > 16:
        return "..."
    if isinstance(obj, dict):
        return {str(k): _jsonable(v, depth + 1) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v, depth + 1) for v in obj]
    if isinstance(obj, np.ndarray):
        return {"__ndarray__": True, "shape": list(obj.shape),
                "dtype": str(obj.dtype), "size": int(obj.size)}
    if isinstance(obj, (bytes, bytearray, memoryview)):
        b = bytes(obj)
        return {"__bytes__": len(b), "head_hex": b[:16].hex()}
    if isinstance(obj, (int, float, str, bool)) or obj is None:
        return obj
    return repr(obj)


class WebRTCSource:
    """一条到机器狗的 WebRTC 连接（含自动重连）。"""

    def __init__(self, *, ip="192.168.123.161", connection_method="local_sta",
                 serial_number=None, username=None, password=None,
                 aes_key=None, decoder="native", enable_video=True,
                 lidar_switch=True, reconnect_delay=5.0,
                 streams=None, lidar_streams=None):
        # 连接参数
        self.ip = ip
        self.connection_method = connection_method
        self.serial_number = serial_number or None
        self.username = username or None
        self.password = password or None
        self.aes_key = aes_key or None
        self.decoder = decoder
        self.enable_video = enable_video
        self.lidar_switch = lidar_switch
        self.reconnect_delay = float(reconnect_delay)

        # 数据回调：rtc_topic -> fn(payload: dict)          （asyncio 线程调用）
        # 雷达回调：rtc_topic -> fn(meta: dict, points: np|None)
        self.streams = dict(streams or {})
        self.lidar_streams = dict(lidar_streams or {})
        self.on_video = None    # fn(bgr: np.ndarray)
        self.on_status = None   # fn(connected: bool, detail: str)

        self._loop = None
        self._thread = None
        self._stop = threading.Event()
        self._conn = None
        self._connected = False

    # ── 生命周期 ─────────────────────────────────────────────────────
    def start(self):
        self._thread = threading.Thread(target=self._thread_main,
                                        name="go2-webrtc", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        conn, loop = self._conn, self._loop
        if conn is not None and loop is not None and loop.is_running():
            asyncio.run_coroutine_threadsafe(self._safe_disconnect(conn), loop)
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    @property
    def connected(self):
        return self._connected

    def _thread_main(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._run())
        finally:
            try:
                self._loop.run_until_complete(self._loop.shutdown_asyncgens())
            except Exception:
                pass
            self._loop.close()

    # ── 异步主循环（断线重连）────────────────────────────────────────
    async def _run(self):
        while not self._stop.is_set():
            try:
                await self._connect_and_stream()
            except asyncio.CancelledError:
                return
            except Exception as exc:
                self._connected = False
                self._emit_status(False, f"{type(exc).__name__}: {exc}")
            if self._stop.is_set():
                return
            await asyncio.sleep(self.reconnect_delay)

    async def _safe_disconnect(self, conn):
        try:
            await conn.disconnect()
        except Exception:
            pass

    async def _connect_and_stream(self):
        method = _CONNECTION_METHODS[self.connection_method]
        kwargs = {}
        if self.connection_method == "remote":
            kwargs.update(serialNumber=self.serial_number,
                          username=self.username, password=self.password)
        elif self.serial_number and not self.ip:
            kwargs.update(serialNumber=self.serial_number)
        else:
            kwargs.update(ip=self.ip)
        if self.aes_key:
            kwargs["aes_128_key"] = self.aes_key

        conn = UnitreeWebRTCConnection(method, **kwargs)
        self._conn = conn
        try:
            await conn.connect()
        except Exception:
            self._conn = None          # 连接失败不留残留对象
            raise

        try:
            await conn.datachannel.disableTrafficSaving(True)   # 全速转发
        except Exception:
            pass
        conn.datachannel.set_decoder(self.decoder)

        if self.lidar_switch and (self.lidar_streams):
            try:
                conn.datachannel.pub_sub.publish_without_callback(
                    "rt/utlidar/switch", "on")                   # 打开 L2 雷达
            except Exception:
                pass

        for topic, cb in self.streams.items():
            conn.datachannel.pub_sub.subscribe(topic, self._make_callback(cb))
        for topic, cb in self.lidar_streams.items():
            conn.datachannel.pub_sub.subscribe(
                topic, self._make_lidar_callback(cb))

        if self.enable_video:
            conn.video.switchVideoChannel(True)
            conn.video.add_track_callback(self._video_track)

        self._connected = True
        self._emit_status(True, "connected")
        try:
            await self._watch_connection(conn)
        finally:
            self._connected = False
            self._conn = None
            await self._safe_disconnect(conn)

    async def _watch_connection(self, conn):
        grace = 0.0
        while not self._stop.is_set():
            await asyncio.sleep(1.0)
            pc = getattr(conn, "pc", None)
            state = getattr(pc, "connectionState", None)
            if state in ("failed", "closed"):
                raise ConnectionError(f"peer connection state: {state}")
            if state == "disconnected":
                grace += 1.0
                if grace >= 5.0:
                    raise ConnectionError("peer connection disconnected")
            else:
                grace = 0.0

    # ── SDK 回调 ─────────────────────────────────────────────────────
    def _make_callback(self, callback):
        def cb(message):
            try:
                payload = message.get("data") if isinstance(message, dict) else message
                callback(payload)
            except Exception:
                logger.exception("error in stream callback")
        return cb

    def _make_lidar_callback(self, callback):
        def cb(message):
            try:
                payload = message.get("data") if isinstance(message, dict) else message
                if not isinstance(payload, dict):
                    return
                decoded = payload.get("data")
                points, meta = None, dict(payload)
                if isinstance(decoded, dict):
                    pts = decoded.get("points")
                    if isinstance(pts, np.ndarray):
                        points = pts.astype(np.float32).reshape(-1, 3)
                    elif "positions" in decoded:
                        try:
                            points = np.asarray(decoded["positions"],
                                                dtype=np.float32).reshape(-1, 3)
                        except Exception:
                            points = None
                    meta["data"] = {k: _jsonable(v) for k, v in decoded.items()
                                    if k not in ("points", "positions", "uvs", "indices")}
                callback(meta, points)
            except Exception:
                logger.exception("error in lidar callback")
        return cb

    async def _video_track(self, track):
        try:
            while not self._stop.is_set():
                frame = await track.recv()
                if self.on_video is not None:
                    self.on_video(frame.to_ndarray(format="bgr24"))
        except Exception:
            pass          # 视频轨结束（机器人关闭推流）

    def _emit_status(self, connected, detail):
        if self.on_status is not None:
            try:
                self.on_status(connected, detail)
            except Exception:
                pass

    # ── 指令接口（任意 ROS 线程可调用）────────────────────────────────
    def request(self, rtc_topic, api_id, parameter=None, timeout=6.0):
        """请求/应答式指令 -> (ok, code, data_str)。"""
        conn, loop = self._conn, self._loop
        if conn is None or loop is None or not self._connected:
            return False, -1, "WebRTC 未连接"

        async def _req():
            options = {"api_id": int(api_id)}
            if parameter is not None:
                options["parameter"] = parameter
            return await conn.datachannel.pub_sub.publish_request_new(
                rtc_topic, options)

        try:
            fut = asyncio.run_coroutine_threadsafe(_req(), loop)
            response = fut.result(timeout=timeout)
        except Exception as exc:
            return False, -1, f"{type(exc).__name__}: {exc}"

        body = response.get("data", {}) if isinstance(response, dict) else {}
        code = body.get("header", {}).get("status", {}).get("code", -1)
        data = str(body.get("data", ""))
        return code == 0, code, data

    def publish_noreply(self, rtc_topic, api_id, parameter=None):
        """单向（无应答）指令，适用于 Move 等高频帧。"""
        conn, loop = self._conn, self._loop
        if conn is None or loop is None or not self._connected:
            return False
        payload = {
            "header": {
                "identity": {
                    "id": int(time.time() * 1000) % 2147483648
                           + random.randint(0, 1000),
                    "api_id": int(api_id),
                },
                "policy": {"priority": 0, "noreply": True},
            },
            "parameter": json.dumps(parameter) if parameter is not None else "",
            "binary": [],
        }
        loop.call_soon_threadsafe(
            conn.datachannel.pub_sub.publish_without_callback, rtc_topic, payload)
        return True
