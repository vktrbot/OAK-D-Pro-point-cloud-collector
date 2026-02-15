import os
import json
import time
from dataclasses import asdict, dataclass
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger

from .ply_writer import decode_rgb, save_ply_ascii, save_ply_binary_le


@dataclass
class SnapshotMeta:
    wall_time_iso: str
    ros_stamp_sec: int
    ros_stamp_nanosec: int
    frame_id: str
    point_count: int
    bbox_min: list
    bbox_max: list
    topic: str
    out_dir: str
    max_points: int
    voxel_size: float
    format: str
    binary: bool
    has_rgb: bool


def voxel_downsample(xyz: np.ndarray, rgb: np.ndarray | None, voxel: float):
    if voxel <= 0.0:
        return xyz, rgb
    q = np.floor(xyz / voxel).astype(np.int32)
    _, idx = np.unique(q, axis=0, return_index=True)
    idx.sort()
    xyz_ds = xyz[idx]
    rgb_ds = rgb[idx] if rgb is not None else None
    return xyz_ds, rgb_ds


def read_cloud_numpy(msg: PointCloud2, field_names: tuple[str, ...]):
    if hasattr(point_cloud2, "read_points_numpy"):
        return point_cloud2.read_points_numpy(msg, field_names=field_names, skip_nans=True)

    pts_list = list(point_cloud2.read_points(msg, field_names=field_names, skip_nans=True))
    if not pts_list:
        return None

    first = pts_list[0]
    if isinstance(first, (tuple, list, np.ndarray)):
        pts = np.asarray(pts_list)
        if pts.ndim == 1:
            pts = np.stack([np.asarray(p) for p in pts_list], axis=0)
        return pts

    return np.asarray(pts_list)


class SnapshotNode(Node):
    def __init__(self):
        super().__init__("snapshot")

        self.declare_parameter("topic", "/oak/points")
        self.declare_parameter("out_dir", os.path.expanduser("~/Desktop/OAKD/clouds"))
        self.declare_parameter("max_points", 200000)
        self.declare_parameter("format", "ply")
        self.declare_parameter("binary", True)
        self.declare_parameter("voxel_size", 0.0)

        self.topic = self.get_parameter("topic").value
        self.out_dir = os.path.expanduser(self.get_parameter("out_dir").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.format = str(self.get_parameter("format").value).lower()
        self.binary = bool(self.get_parameter("binary").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)

        os.makedirs(self.out_dir, exist_ok=True)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.last_xyz = None
        self.last_rgb = None
        self.last_frame = ""
        self.last_stamp = (0, 0)
        self.has_rgb = False

        self.create_subscription(PointCloud2, self.topic, self.cb, qos)

        self.srv = self.create_service(Trigger, "save_snapshot", self.on_trigger)

        self.get_logger().info(f"Subscribed to {self.topic}")
        self.get_logger().info(f"Service: /{self.get_name()}/save_snapshot")
        self.get_logger().info(f"Output: {self.out_dir}")

    def cb(self, msg: PointCloud2):
        fields = [f.name for f in msg.fields]
        has_rgb = "rgb" in fields
        has_rgba = "rgba" in fields
        has_sep = all(ch in fields for ch in ("r", "g", "b"))

        if has_rgb:
            read_fields = ("x", "y", "z", "rgb")
        elif has_rgba:
            read_fields = ("x", "y", "z", "rgba")
        elif has_sep:
            read_fields = ("x", "y", "z", "r", "g", "b")
        else:
            read_fields = ("x", "y", "z")

        data = read_cloud_numpy(msg, read_fields)
        if data is None:
            return

        if hasattr(data, "dtype") and getattr(data.dtype, "names", None):
            n = int(data.shape[0])
            if n == 0:
                return
            if self.max_points > 0 and n > self.max_points:
                idx = np.random.choice(n, size=self.max_points, replace=False)
                data = data[idx]

            xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float32)

            rgb = None
            if "rgb" in data.dtype.names:
                rgb = decode_rgb(data["rgb"])
            elif "rgba" in data.dtype.names:
                rgb = decode_rgb(data["rgba"])
            elif all(ch in data.dtype.names for ch in ("r", "g", "b")):
                rgb = np.stack([data["r"], data["g"], data["b"]], axis=1)
                rgb = np.clip(rgb, 0, 255).astype(np.uint8)

            self.last_xyz, self.last_rgb = voxel_downsample(xyz, rgb, self.voxel_size)
            self.has_rgb = rgb is not None
        else:
            pts = np.asarray(data)
            if pts.ndim != 2 or pts.shape[1] < 3:
                return

            n = int(pts.shape[0])
            if n == 0:
                return
            if self.max_points > 0 and n > self.max_points:
                idx = np.random.choice(n, size=self.max_points, replace=False)
                pts = pts[idx]

            xyz = pts[:, :3].astype(np.float32)
            rgb = None
            if pts.shape[1] >= 4 and (has_rgb or has_rgba):
                rgb = decode_rgb(pts[:, 3])
            elif pts.shape[1] >= 6 and has_sep:
                rgb = np.clip(pts[:, 3:6], 0, 255).astype(np.uint8)

            self.last_xyz, self.last_rgb = voxel_downsample(xyz, rgb, self.voxel_size)
            self.has_rgb = rgb is not None

        self.last_frame = msg.header.frame_id
        self.last_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def write_snapshot(self):
        if self.last_xyz is None:
            return None, "No cloud received yet"

        ts = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.out_dir, f"oak_points_{ts}")

        xyz = self.last_xyz
        rgb = self.last_rgb

        if xyz.shape[0] == 0:
            return None, "Empty cloud"

        bbox_min = xyz.min(axis=0).tolist()
        bbox_max = xyz.max(axis=0).tolist()

        wall_iso = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        meta = SnapshotMeta(
            wall_time_iso=wall_iso,
            ros_stamp_sec=int(self.last_stamp[0]),
            ros_stamp_nanosec=int(self.last_stamp[1]),
            frame_id=str(self.last_frame),
            point_count=int(xyz.shape[0]),
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            topic=str(self.topic),
            out_dir=str(self.out_dir),
            max_points=int(self.max_points),
            voxel_size=float(self.voxel_size),
            format=str(self.format),
            binary=bool(self.binary),
            has_rgb=bool(rgb is not None),
        )

        ply_path = base + ".ply"
        json_path = base + ".json"

        if self.format != "ply":
            return None, "Only format='ply' is supported in this version"

        if self.binary:
            save_ply_binary_le(ply_path, xyz, rgb)
        else:
            save_ply_ascii(ply_path, xyz, rgb)

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(asdict(meta), f, indent=2)

        return ply_path, f"Saved snapshot: {ply_path}"

    def on_trigger(self, request, response):
        path, msg = self.write_snapshot()
        response.success = path is not None
        response.message = msg
        return response


def main():
    rclpy.init()
    node = SnapshotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
