"""A tiny synthetic world so the full spoofing pipeline runs without CARLA.

It mirrors the CARLA backend: a roster of actors (vehicles + pedestrians), from
which *every vehicle* can broadcast its own honest CPM, plus a LiDAR cloud and a
camera frame for the attacker's raw-data channel. One vehicle is the attacker.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
from PIL import Image

from ..perception import ObjectState, build_cpm_from_objects
from ..lidar import inject_object_points, add_points
from ..attacks.camera_injection import CameraProjector

Vec3 = Tuple[float, float, float]

# Which classifications broadcast CPMs (pedestrians don't; vehicles/RSUs do).
SENDER_CLASSES = {"car", "truck", "bus", "bicycle"}


@dataclass
class MockWorld:
    # Full roster of actors in the scene (vehicles AND pedestrians).
    objects: List[ObjectState] = field(default_factory=list)
    attacker_id: int = 101
    sensor_height: float = 2.4

    @classmethod
    def default_scene(cls) -> "MockWorld":
        """Attacker + victim + two more vehicles + a pedestrian on a road.

        IDs: 101 attacker (at origin, facing +X), 100 victim/ego behind it,
        200 a car ahead, 201 a truck ahead-right, 300 a pedestrian.
        """
        objs = [
            ObjectState(101, (0.0, 0.0, 0.0), velocity=(10.0, 0.0, 0.0),
                        yaw_deg=0.0, dimensions=(4.6, 2.0, 1.5), classification="car"),
            ObjectState(100, (-8.0, 0.0, 0.0), velocity=(10.0, 0.0, 0.0),
                        yaw_deg=0.0, dimensions=(4.6, 2.0, 1.5), classification="car"),
            ObjectState(200, (25.0, 0.5, 0.0), velocity=(8.0, 0.0, 0.0),
                        yaw_deg=0.0, dimensions=(4.6, 2.0, 1.5), classification="car"),
            ObjectState(201, (40.0, -3.5, 0.0), velocity=(6.0, 0.0, 0.0),
                        yaw_deg=0.0, dimensions=(6.0, 2.4, 3.0), classification="truck"),
            ObjectState(300, (18.0, 4.0, 0.0), velocity=(0.0, 1.2, 0.0),
                        yaw_deg=90.0, dimensions=(0.6, 0.6, 1.8), classification="pedestrian"),
        ]
        return cls(objects=objs, attacker_id=101)

    def _state(self, sid: int) -> ObjectState:
        for o in self.objects:
            if o.object_id == sid:
                return o
        raise KeyError(sid)

    def sender_ids(self) -> List[int]:
        """Vehicles that broadcast a CPM (attacker first)."""
        ids = [o.object_id for o in self.objects if o.classification in SENDER_CLASSES]
        ids.sort(key=lambda i: (i != self.attacker_id, i))  # attacker first
        return ids

    # ---- object level ----
    def honest_cpm_for(self, station_id: int, generation_time: float = 0.0):
        """The honest CPM that `station_id` would broadcast (perceives others)."""
        st = self._state(station_id)
        return build_cpm_from_objects(
            station_id=station_id,
            reference_position=st.position,
            objects=self.objects,
            generation_time=generation_time,
        )

    def honest_cpm(self, generation_time: float = 0.0):
        """Back-compat: the attacker's honest CPM."""
        return self.honest_cpm_for(self.attacker_id, generation_time)

    # ---- LiDAR (attacker's own sensor) ----
    def honest_pointcloud(self, seed: int = 0) -> np.ndarray:
        rng = np.random.default_rng(seed)
        gx = rng.uniform(-5, 60, 4000)
        gy = rng.uniform(-15, 15, 4000)
        gz = rng.normal(-1.6, 0.03, 4000)
        cloud = np.column_stack([gx, gy, gz, np.full_like(gx, 0.2)]).astype(np.float32)
        atk = self._state(self.attacker_id)
        origin = (atk.position[0], atk.position[1], atk.position[2] + self.sensor_height)
        for o in self.objects:
            if o.object_id == self.attacker_id:
                continue
            pts = inject_object_points(o.position, o.dimensions, o.yaw_deg,
                                       sensor_origin=origin, seed=o.object_id)
            cloud = add_points(cloud, pts)
        return cloud

    # ---- camera (attacker's own sensor) ----
    def camera(self, width: int = 800, height: int = 600, fov: float = 90.0):
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
        atk = self._state(self.attacker_id)
        cam_loc = (atk.position[0], atk.position[1], atk.position[2] + self.sensor_height)
        projector = CameraProjector.from_pose(cam_loc, atk.yaw_deg, width, height, fov)
        return frame, projector
