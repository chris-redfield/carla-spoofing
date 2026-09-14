"""Object-removal ("vanishing") attack.

The attacker broadcasts a CPM from which a *real* object has been deleted, and
carves the corresponding returns out of the shared LiDAR cloud. A victim that
relies on cooperative perception to see around an occlusion never learns the
object is there -- e.g. a pedestrian stepping out from behind a bus is erased.
This is the most safety-critical of the three attacks.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .base import Attack
from ..lidar import carve_box
from ..v2x.cpm import CollectivePerceptionMessage

Vec3 = Tuple[float, float, float]


@dataclass
class RemoveTarget:
    """Identify the victim object either by id or by nearest position."""
    object_id: Optional[int] = None
    position: Optional[Vec3] = None       # used if object_id is None
    dimensions: Vec3 = (4.5, 2.0, 1.5)    # box to carve from the point cloud
    yaw_deg: float = 0.0
    carve_lidar: bool = True


class RemoveObjectAttack(Attack):
    name = "remove_object"

    def __init__(self, target: RemoveTarget):
        super().__init__()
        self.target = target

    def _resolve_id(self, cpm: CollectivePerceptionMessage) -> Optional[int]:
        if self.target.object_id is not None:
            return self.target.object_id
        if self.target.position is None:
            return None
        # nearest perceived object to the given position
        best, best_d = None, float("inf")
        px, py, pz = self.target.position
        for o in cpm.perceived_objects:
            d = (o.position[0]-px)**2 + (o.position[1]-py)**2 + (o.position[2]-pz)**2
            if d < best_d:
                best, best_d = o.object_id, d
        return best

    def apply_cpm(self, cpm: CollectivePerceptionMessage
                  ) -> CollectivePerceptionMessage:
        out = cpm.copy()
        tid = self._resolve_id(out)
        # capture the object's pose for LiDAR carving before we drop it
        self._carve_center = None
        if tid is not None:
            for o in out.perceived_objects:
                if o.object_id == tid:
                    self._carve_center = o.position
                    self._carve_dims = o.dimensions
                    self._carve_yaw = o.yaw_deg
                    break
            if out.remove_object(tid):
                self.result.removed_object_ids = [tid]
                self.result.notes = f"erased object {tid}"
        else:
            self.result.notes = "no matching object to remove"
        return out

    def apply_pointcloud(self, points: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if points is None or not self.target.carve_lidar:
            return points
        center = getattr(self, "_carve_center", None) or self.target.position
        if center is None:
            return points
        dims = getattr(self, "_carve_dims", None) or self.target.dimensions
        yaw = getattr(self, "_carve_yaw", None)
        yaw = self.target.yaw_deg if yaw is None else yaw
        before = points.shape[0]
        out = carve_box(points, center, dims, yaw)
        self.result.points_removed = int(before - out.shape[0])
        return out
