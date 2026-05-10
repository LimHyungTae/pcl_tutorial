#!/usr/bin/env python3
"""One-shot helper: voxel-downsample a binary-XYZ PCD and rewrite it.

KISS-Matcher's KITTI00-to-07 sample ships a 22 MB / 4.5 MB submap pair —
fine for the algorithm, big to commit to the repo. This trims them to
roughly the same point density that the demo's voxelization step uses
anyway, so neither the algorithm result nor the visualization changes
meaningfully.

Used once during dataset onboarding. Not wired into the chapter
generation pipeline.
"""
import sys
from pathlib import Path

import numpy as np


def read_pcd(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        data = f.read()
    header_end = data.find(b"DATA")
    header = data[:header_end].decode("ascii", errors="replace")
    rest = data[header_end:]
    body_start = rest.find(b"\n") + 1
    encoding_line = rest[:body_start].decode("ascii", errors="replace")
    body = rest[body_start:]
    if "binary" not in encoding_line:
        raise SystemExit(f"only binary PCD supported, got: {encoding_line.strip()}")

    num_points = None
    for line in header.splitlines():
        if line.startswith("POINTS"):
            num_points = int(line.split()[1])
    if num_points is None:
        raise SystemExit("POINTS header field missing")

    arr = np.frombuffer(body, dtype=np.float32, count=num_points * 3).reshape(-1, 3).copy()
    return arr


def write_pcd_binary_xyz(path: Path, xyz: np.ndarray) -> None:
    n = len(xyz)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(xyz.astype(np.float32).tobytes())


def voxel_downsample(xyz: np.ndarray, leaf: float) -> np.ndarray:
    """Pick one representative per occupied voxel (the first encountered)."""
    keys = np.floor(xyz / leaf).astype(np.int64)
    flat = (
        (keys[:, 0] + (1 << 20)) * (1 << 42)
        + (keys[:, 1] + (1 << 20)) * (1 << 21)
        + (keys[:, 2] + (1 << 20))
    )
    _, idx = np.unique(flat, return_index=True)
    return xyz[np.sort(idx)]


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("usage: _voxel_downsample_pcd.py <in.pcd> <out.pcd> <leaf>")
        sys.exit(1)
    inp, outp, leaf = Path(sys.argv[1]), Path(sys.argv[2]), float(sys.argv[3])
    xyz = read_pcd(inp)
    print(f"  in : {len(xyz):>9,} pts ({inp.stat().st_size / 1024 / 1024:.1f} MB)")
    down = voxel_downsample(xyz, leaf)
    write_pcd_binary_xyz(outp, down)
    print(f"  out: {len(down):>9,} pts ({outp.stat().st_size / 1024 / 1024:.1f} MB)  [leaf={leaf}]")
