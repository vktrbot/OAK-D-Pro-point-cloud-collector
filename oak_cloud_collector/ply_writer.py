import numpy as np


def decode_rgb(packed_arr: np.ndarray) -> np.ndarray:
    packed_arr = np.asarray(packed_arr)
    ui = packed_arr.view(np.uint32) if packed_arr.dtype == np.float32 else packed_arr.astype(np.uint32)
    r = ((ui >> 16) & 255).astype(np.uint8)
    g = ((ui >> 8) & 255).astype(np.uint8)
    b = (ui & 255).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def save_ply_ascii(path: str, xyz: np.ndarray, rgb: np.ndarray | None):
    n = int(xyz.shape[0])
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


def save_ply_binary_le(path: str, xyz: np.ndarray, rgb: np.ndarray | None):
    n = int(xyz.shape[0])
    header = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if rgb is not None:
        header += [
            "property uchar red",
            "property uchar green",
            "property uchar blue",
        ]
    header.append("end_header\n")
    header_bytes = ("\n".join(header)).encode("ascii")

    with open(path, "wb") as f:
        f.write(header_bytes)
        if rgb is None:
            xyz.astype("<f4").tofile(f)
        else:
            xyz_f = xyz.astype("<f4")
            rgb_u = rgb.astype(np.uint8)
            inter = np.empty((n, 15), dtype=np.uint8)
            inter[:, 0:12] = xyz_f.view(np.uint8).reshape(n, 12)
            inter[:, 12:15] = rgb_u
            inter.tofile(f)
