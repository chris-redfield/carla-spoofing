"""Fake-object injection: a phantom object that does not exist.

The attacker broadcasts a CPM containing a fabricated ``PerceivedObject`` (e.g.
a stopped car or a pedestrian right in front of a victim). Optionally it also
injects matching LiDAR returns, so a victim that fuses raw clouds -- not just
object lists -- is fooled too. A believable ghost forces emergency braking or
an evasive manoeuvre in the victim.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .base import Attack
from ..lidar import inject_object_points, add_points
from ..v2x.cpm import CollectivePerceptionMessage, PerceivedObject

Vec3 = Tuple[float, float, float]


@dataclass
class FakeObjectSpec:
    position: Vec3                       # world coords of the phantom
    object_id: int = 999001              # id the attacker assigns the ghost
    velocity: Vec3 = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0
    dimensions: Vec3 = (4.5, 2.0, 1.5)
    classification: str = "car"
    confidence: float = 0.92             # plausible, not suspiciously perfect
    inject_lidar: bool = True            # also fabricate raw returns
    sensor_origin: Vec3 = (0.0, 0.0, 2.4)  # attacker LiDAR height, for shell culling


class FakeObjectAttack(Attack):
    name = "fake_object"

    def __init__(self, spec: FakeObjectSpec):
        super().__init__()
        self.spec = spec

    def apply_cpm(self, cpm: CollectivePerceptionMessage
                  ) -> CollectivePerceptionMessage:
        out = cpm.copy()
        s = self.spec
        out.add_object(PerceivedObject(
            object_id=s.object_id,
            position=s.position,
            velocity=s.velocity,
            yaw_deg=s.yaw_deg,
            dimensions=s.dimensions,
            classification=s.classification,
            confidence=s.confidence,
        ))
        self.result.added_object_ids = [s.object_id]
        self.result.notes = (
            f"phantom {s.classification} at {tuple(round(v,2) for v in s.position)}"
        )
        return out

    def apply_pointcloud(self, points: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if points is None or not self.spec.inject_lidar:
            return points
        s = self.spec
        # sensor origin is relative to the attacker; place it at the attacker's
        # reference by offsetting to the phantom's neighbourhood via world coords.
        ghost = inject_object_points(
            center=s.position, dimensions=s.dimensions, yaw_deg=s.yaw_deg,
            sensor_origin=s.sensor_origin,
        )
        self.result.points_added = int(ghost.shape[0])
        return add_points(points, ghost)
