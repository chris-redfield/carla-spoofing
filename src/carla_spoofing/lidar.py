"""Point-cloud helpers for LiDAR-level cooperative perception spoofing.

A point cloud is an ``(N, 4)`` float32 array of ``x, y, z, intensity`` in the
CARLA world frame (the extra column is dropped/ignored if you only have XYZ).

The two attack primitives at the raw-sensor level are:
* ``inject_object_points`` -- synthesise a plausible cluster of returns for a
  *phantom* object (a ghost car/pedestrian that isn't there).
* ``carve_box``            -- delete the returns that fall inside a *real*
  object's bounding box, so it "disappears" from the shared cloud.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


def _yaw_matrix(yaw_deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def points_in_box_mask(points: np.ndarray, center: Vec3, dimensions: Vec3,
                       yaw_deg: float = 0.0, margin: float = 0.0) -> np.ndarray:
    """Boolean mask of points inside an oriented box (length,width,height)."""
    xyz = np.asarray(points, dtype=np.float64)[:, :3]
    rel = xyz - np.asarray(center, dtype=np.float64)
    # World->local: with world = R(yaw) @ local, local = R(yaw)^T @ rel, which
    # for row vectors is rel @ R(yaw).
    local = rel @ _yaw_matrix(yaw_deg)
    l, w, h = dimensions
    half = np.array([l / 2.0 + margin, w / 2.0 + margin, h / 2.0 + margin])
    return np.all(np.abs(local) <= half, axis=1)


def carve_box(points: np.ndarray, center: Vec3, dimensions: Vec3,
              yaw_deg: float = 0.0, margin: float = 0.15) -> np.ndarray:
    """Remove all points inside the (slightly enlarged) box -> object vanishes."""
    mask = points_in_box_mask(points, center, dimensions, yaw_deg, margin)
    return np.asarray(points)[~mask]


def inject_object_points(center: Vec3, dimensions: Vec3, yaw_deg: float = 0.0,
                         sensor_origin: Vec3 = (0.0, 0.0, 2.4),
                         spacing: float = 0.12, intensity: float = 0.9,
                         jitter: float = 0.01, seed: int = 0) -> np.ndarray:
    """Synthesise LiDAR returns on the *visible shell* of a phantom box.

    We sample points on the box faces, then keep only the faces oriented toward
    the sensor (a real LiDAR only sees the near/top surfaces). Returns an
    ``(M, 4)`` array in the world frame.
    """
    rng = np.random.default_rng(seed)
    l, w, h = dimensions
    R = _yaw_matrix(yaw_deg)
    faces = []

    def grid(u_len, v_len, spacing):
        nu = max(2, int(u_len / spacing))
        nv = max(2, int(v_len / spacing))
        u = np.linspace(-u_len / 2, u_len / 2, nu)
        v = np.linspace(-v_len / 2, v_len / 2, nv)
        uu, vv = np.meshgrid(u, v)
        return uu.ravel(), vv.ravel()

    # +/-X faces (front/back), +/-Y faces (sides), +Z face (top)
    a, b = grid(w, h, spacing)      # X faces span width(y) x height(z)
    faces.append(np.column_stack([np.full_like(a, +l / 2), a, b]))  # +X
    faces.append(np.column_stack([np.full_like(a, -l / 2), a, b]))  # -X
    a, b = grid(l, h, spacing)      # Y faces span length(x) x height(z)
    faces.append(np.column_stack([a, np.full_like(a, +w / 2), b]))  # +Y
    faces.append(np.column_stack([a, np.full_like(a, -w / 2), b]))  # -Y
    a, b = grid(l, w, spacing)      # top face span length(x) x width(y)
    faces.append(np.column_stack([a, b, np.full_like(a, +h / 2)]))  # +Z

    local = np.vstack(faces)
    world = local @ R.T + np.asarray(center, dtype=np.float64)

    # Keep only faces facing the sensor: normal . (sensor - point) > 0.
    # Compute per-point outward normal in world frame.
    normals_local = np.zeros_like(local)
    n = 0
    for pts, nrm in (
        (faces[0], (+1, 0, 0)), (faces[1], (-1, 0, 0)),
        (faces[2], (0, +1, 0)), (faces[3], (0, -1, 0)),
        (faces[4], (0, 0, +1)),
    ):
        normals_local[n:n + len(pts)] = nrm
        n += len(pts)
    normals_world = normals_local @ R.T
    to_sensor = np.asarray(sensor_origin, dtype=np.float64) - world
    facing = np.sum(normals_world * to_sensor, axis=1) > 0
    world = world[facing]

    world += rng.normal(0.0, jitter, world.shape)
    inten = np.full((world.shape[0], 1), intensity, dtype=np.float64)
    return np.hstack([world, inten]).astype(np.float32)


def add_points(cloud: np.ndarray, extra: np.ndarray) -> np.ndarray:
    """Concatenate two (N,4) clouds (pads/truncates extra to match width)."""
    cloud = np.asarray(cloud, dtype=np.float32)
    extra = np.asarray(extra, dtype=np.float32)
    if cloud.shape[1] != extra.shape[1]:
        width = cloud.shape[1]
        if extra.shape[1] > width:
            extra = extra[:, :width]
        else:
            pad = np.zeros((extra.shape[0], width - extra.shape[1]), dtype=np.float32)
            extra = np.hstack([extra, pad])
    return np.vstack([cloud, extra])


def save_pcd(points: np.ndarray, path: str) -> None:
    """Write an ASCII PCD (viewable in CloudCompare / Open3D) for inspection."""
    pts = np.asarray(points, dtype=np.float32)
    xyz = pts[:, :3]
    n = xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data\nVERSION 0.7\nFIELDS x y z\n"
        "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n"
    )
    with open(path, "w") as fh:
        fh.write(header)
        for x, y, z in xyz:
            fh.write(f"{x} {y} {z}\n")
