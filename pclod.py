import os
import sys
import time
import select
import termios
import tty
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def decode_rgb(packed_arr: np.ndarray) -> np.ndarray:
    packed_arr = np.asarray(packed_arr)
    ui = packed_arr.view(np.uint32) if packed_arr.dtype == np.float32 else packed_arr.astype(np.uint32)
    r = ((ui >> 16) & 255).astype(np.uint8)
    g = ((ui >> 8) & 255).astype(np.uint8)
    b = (ui & 255).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def save_ply_ascii(path: str, xyz: np.ndarray, rgb: np.ndarray | None):
    n = xyz.shape[0]
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if rgb is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        if rgb is None:
            np.savetxt(f, xyz, fmt="%.6f %.6f %.6f")
        else:
            data = np.concatenate([xyz, rgb.astype(np.float32)], axis=1)
            np.savetxt(f, data, fmt="%.6f %.6f %.6f %d %d %d")


def read_cloud_as_numpy(msg: PointCloud2, field_names: tuple[str, ...]):
    if hasattr(point_cloud2, "read_points_numpy"):
        arr = point_cloud2.read_points_numpy(msg, field_names=field_names, skip_nans=True)
        return arr

    pts_list = list(point_cloud2.read_points(msg, field_names=field_names, skip_nans=True))
    if len(pts_list) == 0:
        return None

    first = pts_list[0]
    if isinstance(first, (tuple, list, np.ndarray)):
        pts = np.asarray(pts_list)
        if pts.ndim == 1:
            pts = np.stack([np.asarray(p) for p in pts_list], axis=0)
        return pts

    pts = np.asarray(pts_list)
    return pts


class CloudSnapshot(Node):
    def __init__(self, topic: str, out_dir: str, max_points: int | None):
        super().__init__("pclod_snapshot")
        self.topic = topic
        self.out_dir = os.path.expanduser(out_dir)
        self.max_points = max_points

        os.makedirs(self.out_dir, exist_ok=True)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.last_xyz = None
        self.last_rgb = None
        self.last_frame = None
        self.last_stamp = None
        self.frames = 0

        self.create_subscription(PointCloud2, self.topic, self.cb, qos)
        self.get_logger().info(f"Subscribed to {self.topic}")
        self.get_logger().info("Press 's' to save a snapshot, 'q' to quit.")
        self.get_logger().info(f"Output directory: {self.out_dir}")

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

        data = read_cloud_as_numpy(msg, read_fields)
        if data is None:
            return

        if hasattr(data, "dtype") and getattr(data.dtype, "names", None):
            n = data.shape[0]
            if n == 0:
                return
            if self.max_points is not None and n > self.max_points:
                idx = np.random.choice(n, size=self.max_points, replace=False)
                data = data[idx]
                n = data.shape[0]

            xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float32)
            rgb = None

            if "rgb" in data.dtype.names:
                rgb = decode_rgb(data["rgb"])
            elif "rgba" in data.dtype.names:
                rgb = decode_rgb(data["rgba"])
            elif all(ch in data.dtype.names for ch in ("r", "g", "b")):
                rgb = np.stack([data["r"], data["g"], data["b"]], axis=1)
                rgb = np.clip(rgb, 0, 255).astype(np.uint8)

            self.last_xyz = xyz
            self.last_rgb = rgb
        else:
            pts = np.asarray(data)
            if pts.ndim != 2 or pts.shape[1] < 3:
                return

            n = pts.shape[0]
            if n == 0:
                return
            if self.max_points is not None and n > self.max_points:
                idx = np.random.choice(n, size=self.max_points, replace=False)
                pts = pts[idx]

            self.last_xyz = pts[:, :3].astype(np.float32)
            self.last_rgb = None

            if pts.shape[1] >= 4 and (has_rgb or has_rgba):
                self.last_rgb = decode_rgb(pts[:, 3])
            elif pts.shape[1] >= 6 and has_sep:
                self.last_rgb = np.clip(pts[:, 3:6], 0, 255).astype(np.uint8)

        self.last_frame = msg.header.frame_id
        self.last_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info(f"Clouds received: {self.frames}")

    def save_snapshot(self):
        if self.last_xyz is None:
            self.get_logger().warn("No cloud received yet. Is the topic publishing?")
            return

        ts = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(self.out_dir, f"oak_points_{ts}.ply")
        save_ply_ascii(out_path, self.last_xyz, self.last_rgb)

        n = self.last_xyz.shape[0]
        self.get_logger().info(
            f"Saved {n} points to {out_path} (frame='{self.last_frame}', stamp={self.last_stamp})"
        )


def get_key_nonblocking() -> str | None:
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/oak/points")
    ap.add_argument("--out-dir", default="~/Desktop/OAKD/clouds")
    ap.add_argument("--max-points", type=int, default=200000)
    args = ap.parse_args()
    max_points = None if args.max_points == 0 else args.max_points

    rclpy.init()
    node = CloudSnapshot(args.topic, args.out_dir, max_points)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            k = get_key_nonblocking()
            if not k:
                continue
            if k.lower() == "s":
                node.save_snapshot()
            elif k.lower() == "q":
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
