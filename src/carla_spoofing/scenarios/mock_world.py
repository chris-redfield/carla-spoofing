"""A tiny synthetic world so the full spoofing pipeline runs without CARLA.

It mirrors what the CARLA backend provides: object states (for CPMs), a LiDAR
point cloud, and a camera frame + projector -- all in the CARLA world frame.
This makes the attacks unit-testable and lets researchers reproduce results
with only numpy + Pillow (no GPU, no 6.85 GB binary).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
from PIL import Image

from ..perception import ObjectState, build_cpm_from_objects
from ..lidar import inject_object_points, add_points
from ..attacks.camera_injection import CameraProjector

Vec3 = Tuple[float, float, float]


@dataclass
class MockWorld:
    attacker_id: int = 101
    attacker_pos: Vec3 = (0.0, 0.0, 0.0)
    attacker_yaw: float = 0.0                 # attacker faces +X
    ego_id: int = 100
    ego_pos: Vec3 = (-8.0, 0.0, 0.0)          # victim behind the attacker
    objects: List[ObjectState] = field(default_factory=list)
    sensor_height: float = 2.4

    @classmethod
    def default_scene(cls) -> "MockWorld":
        """Attacker + victim on a road with two genuine vehicles ahead."""
        objs = [
            ObjectState(200, (25.0, 0.5, 0.0), velocity=(8.0, 0.0, 0.0),
                        yaw_deg=0.0, dimensions=(4.6, 2.0, 1.5), classification="car"),
            ObjectState(201, (40.0, -3.5, 0.0), velocity=(6.0, 0.0, 0.0),
                        yaw_deg=0.0, dimensions=(6.0, 2.4, 3.0), classification="truck"),
        ]
        return cls(objects=objs)

    # ---- object level ----
    def all_states(self) -> List[ObjectState]:
        return list(self.objects)

    def honest_cpm(self, generation_time: float = 0.0):
        return build_cpm_from_objects(
            station_id=self.attacker_id,
            reference_position=self.attacker_pos,
            objects=self.objects,
            generation_time=generation_time,
        )

    # ---- LiDAR ----
    def honest_pointcloud(self, seed: int = 0) -> np.ndarray:
        rng = np.random.default_rng(seed)
        # sparse ground plane + noise around the attacker
        gx = rng.uniform(-5, 60, 4000)
        gy = rng.uniform(-15, 15, 4000)
        gz = rng.normal(-1.6, 0.03, 4000)      # road ~1.6 m below sensor
        ground = np.column_stack([gx, gy, gz, np.full_like(gx, 0.2)]).astype(np.float32)
        cloud = ground
        origin = (self.attacker_pos[0], self.attacker_pos[1],
                  self.attacker_pos[2] + self.sensor_height)
        for o in self.objects:
            pts = inject_object_points(o.position, o.dimensions, o.yaw_deg,
                                       sensor_origin=origin, seed=o.object_id)
            cloud = add_points(cloud, pts)
        return cloud

    # ---- camera ----
    def camera(self, width: int = 800, height: int = 600, fov: float = 90.0):
        """Return (RGB frame as HxWx3 uint8, CameraProjector) from attacker POV."""
        # simple sky/road gradient background
        img = Image.new("RGB", (width, height))
        px = img.load()
        horizon = int(height * 0.5)
        for y in range(height):
            if y < horizon:
                t = y / max(1, horizon)
                col = (int(120 + 80 * t), int(160 + 60 * t), int(210 + 30 * t))
            else:
                t = (y - horizon) / max(1, height - horizon)
                g = int(70 - 30 * t)
                col = (g, g, g)
            for x in range(width):
                px[x, y] = col
        frame = np.asarray(img, dtype=np.uint8)
        cam_loc = (self.attacker_pos[0], self.attacker_pos[1],
                   self.attacker_pos[2] + self.sensor_height)
        projector = CameraProjector.from_pose(cam_loc, self.attacker_yaw,
                                              width, height, fov)
        return frame, projector
