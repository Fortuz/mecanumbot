#!/usr/bin/env python3
"""
Stage the real robot's meshes into the Gazebo model directory.

`mecanumbot_description/meshes` holds the CAD the real robot is built from, and
that is what the Gazebo model should look like - but `base.stl` is 27 MB /
550k triangles, which is more than a simulator wants to reload on every spawn.
This script copies the small meshes verbatim and decimates the big ones, writing
the result to `models/mecanumbot/meshes/` where the SDF references them with
paths relative to the model directory.

Decimation is vertex clustering: snap every vertex to a grid, then drop the
triangles that collapse to a line or a point. It is shape-preserving in the way
that matters here - the two chassis posts either side of the lidar stay separate
objects, so the simulated scan keeps the real robot's blind slivers - and it
needs nothing but numpy.

Re-run after changing a source mesh:

    python3 src/mecanumbot/mecanumbot_sim/scripts/prepare_gazebo_meshes.py
"""

import os
import struct
import sys

import numpy as np

# Source mesh -> grid size in millimetres. None means copy without decimating;
# the mesh is already small enough that clustering would only lose detail.
MESHES = {
    "base.stl": 2.0,
    "wheel.stl": None,
    "head.stl": None,
    "arm_left.stl": None,
    "arm_right.stl": None,
    "lds.stl": 1.5,
}

SOURCE_SUBDIR = os.path.join("mecanumbot_description", "meshes")
TARGET_SUBDIR = os.path.join(
    "mecanumbot_sim", "models", "mecanumbot", "meshes"
)


def read_binary_stl(path):
    """Return the (n, 3, 3) vertex array of a binary STL."""
    with open(path, "rb") as handle:
        data = handle.read()

    if len(data) < 84:
        raise ValueError(f"{path} is too short to be a binary STL")

    count = struct.unpack("<I", data[80:84])[0]
    expected = 84 + count * 50
    if len(data) < expected:
        raise ValueError(
            f"{path} claims {count} triangles but is only {len(data)} bytes; "
            "ASCII STL is not supported"
        )

    records = np.frombuffer(data[84:expected], dtype=np.uint8).reshape(count, 50)
    return records[:, 12:48].copy().view("<f4").reshape(count, 3, 3)


def write_binary_stl(path, triangles):
    """Write an (n, 3, 3) vertex array as a binary STL with recomputed normals."""
    count = len(triangles)
    edge_a = triangles[:, 1] - triangles[:, 0]
    edge_b = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edge_a, edge_b)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)

    # Each 50-byte record is: normal (3 floats), 3 vertices (9 floats), 2 pad bytes.
    payload = np.empty((count, 12), dtype="<f4")
    payload[:, 0:3] = normals
    payload[:, 3:12] = triangles.reshape(count, 9)

    records = np.zeros((count, 50), dtype=np.uint8)
    records[:, :48] = payload.view(np.uint8).reshape(count, 48)

    with open(path, "wb") as handle:
        handle.write(b"mecanumbot_sim decimated mesh".ljust(80, b"\0"))
        handle.write(struct.pack("<I", count))
        handle.write(records.tobytes())


def decimate(triangles, grid_mm):
    """Cluster vertices onto a grid and drop the triangles that degenerate."""
    snapped = np.round(triangles / grid_mm) * grid_mm

    # A triangle survives only if its three snapped corners are still distinct.
    same_ab = np.all(snapped[:, 0] == snapped[:, 1], axis=1)
    same_bc = np.all(snapped[:, 1] == snapped[:, 2], axis=1)
    same_ca = np.all(snapped[:, 2] == snapped[:, 0], axis=1)
    keep = ~(same_ab | same_bc | same_ca)
    snapped = snapped[keep]

    # Drop duplicate faces produced by collapsing neighbouring detail.
    if len(snapped):
        flat = np.ascontiguousarray(snapped.reshape(len(snapped), 9))
        _, unique_index = np.unique(
            flat.view([("", flat.dtype)] * 9), return_index=True
        )
        snapped = snapped[np.sort(unique_index)]

    return snapped


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(here)
    src_root = os.path.dirname(package_root)

    source_dir = os.path.join(src_root, SOURCE_SUBDIR)
    target_dir = os.path.join(src_root, TARGET_SUBDIR)

    if not os.path.isdir(source_dir):
        print(f"Source mesh directory not found: {source_dir}", file=sys.stderr)
        return 1

    os.makedirs(target_dir, exist_ok=True)

    for name, grid_mm in MESHES.items():
        source = os.path.join(source_dir, name)
        target = os.path.join(target_dir, name)
        if not os.path.isfile(source):
            print(f"  skip {name}: not found in {source_dir}", file=sys.stderr)
            continue

        triangles = read_binary_stl(source)
        if grid_mm is None:
            write_binary_stl(target, triangles)
            print(f"  {name}: copied, {len(triangles)} triangles")
            continue

        reduced = decimate(triangles, grid_mm)
        write_binary_stl(target, reduced)
        print(
            f"  {name}: {len(triangles)} -> {len(reduced)} triangles "
            f"({100.0 * len(reduced) / max(len(triangles), 1):.1f}%, {grid_mm} mm grid)"
        )

    print(f"Wrote meshes to {target_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
