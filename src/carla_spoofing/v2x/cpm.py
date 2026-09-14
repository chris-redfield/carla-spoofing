"""Collective Perception Message (CPM) model.

A pragmatic, JSON-serialisable encoding of the ETSI EN 302 637-5 information
model: a *station* (vehicle/RSU) reports the objects it perceives, so neighbours
can fuse them into their own world model ("cooperative / collective perception").

This is exactly the message an attacker forges: adding a non-existent object,
dropping a real one, or shifting/mis-classifying one. The struct is kept flat
and explicit so the OMNeT++ side and any analysis code can read it directly.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]


def _vec3(v: Sequence[float]) -> Vec3:
    return (float(v[0]), float(v[1]), float(v[2]))


@dataclass
class PerceivedObject:
    """One object as perceived and reported by a station.

    Positions/velocities are in the CARLA world frame (see config.py). A real
    CPM reports object state relative to the sender's reference position; we
    keep absolute world coordinates for clarity and note the sender pose on the
    message, from which relative coordinates can be derived if needed.
    """

    object_id: int
    position: Vec3                 # (x, y, z) metres, world frame
    velocity: Vec3 = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0           # heading about Z
    dimensions: Vec3 = (4.5, 2.0, 1.5)  # (length, width, height) metres
    classification: str = "unknown"     # car|truck|pedestrian|bicycle|unknown
    confidence: float = 1.0             # 0..1 detection confidence

    def __post_init__(self) -> None:
        self.position = _vec3(self.position)
        self.velocity = _vec3(self.velocity)
        self.dimensions = _vec3(self.dimensions)

    def to_dict(self) -> dict:
        d = asdict(self)
        for k in ("position", "velocity", "dimensions"):
            d[k] = list(d[k])
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "PerceivedObject":
        return cls(
            object_id=int(d["object_id"]),
            position=_vec3(d["position"]),
            velocity=_vec3(d.get("velocity", (0.0, 0.0, 0.0))),
            yaw_deg=float(d.get("yaw_deg", 0.0)),
            dimensions=_vec3(d.get("dimensions", (4.5, 2.0, 1.5))),
            classification=str(d.get("classification", "unknown")),
            confidence=float(d.get("confidence", 1.0)),
        )


@dataclass
class CollectivePerceptionMessage:
    """A CPM: who sent it, when, from where, and what objects it perceived."""

    station_id: int                       # sender's V2X station id (== CARLA actor id)
    generation_time: float                # seconds (sim clock)
    reference_position: Vec3              # sender's own (x, y, z)
    station_type: str = "vehicle"         # vehicle|rsu|drone
    perceived_objects: List[PerceivedObject] = field(default_factory=list)
    protocol_version: int = 2

    # ---- object-set helpers (used by the attacks) ----
    def add_object(self, obj: PerceivedObject) -> None:
        self.perceived_objects.append(obj)

    def remove_object(self, object_id: int) -> bool:
        n = len(self.perceived_objects)
        self.perceived_objects = [
            o for o in self.perceived_objects if o.object_id != object_id
        ]
        return len(self.perceived_objects) != n

    def object_ids(self) -> List[int]:
        return [o.object_id for o in self.perceived_objects]

    def copy(self) -> "CollectivePerceptionMessage":
        return CollectivePerceptionMessage.from_dict(self.to_dict())

    # ---- serialisation ----
    def to_dict(self) -> dict:
        return {
            "protocol_version": self.protocol_version,
            "station_id": self.station_id,
            "station_type": self.station_type,
            "generation_time": self.generation_time,
            "reference_position": list(_vec3(self.reference_position)),
            "perceived_objects": [o.to_dict() for o in self.perceived_objects],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CollectivePerceptionMessage":
        return cls(
            station_id=int(d["station_id"]),
            generation_time=float(d["generation_time"]),
            reference_position=_vec3(d["reference_position"]),
            station_type=str(d.get("station_type", "vehicle")),
            perceived_objects=[
                PerceivedObject.from_dict(o) for o in d.get("perceived_objects", [])
            ],
            protocol_version=int(d.get("protocol_version", 2)),
        )

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    @classmethod
    def from_json(cls, s: str) -> "CollectivePerceptionMessage":
        return cls.from_dict(json.loads(s))

    @staticmethod
    def now() -> float:
        return time.time()


def diff_cpms(honest: CollectivePerceptionMessage,
              spoofed: CollectivePerceptionMessage) -> dict:
    """Return the set difference of perceived object ids (for logging/eval)."""
    h = set(honest.object_ids())
    s = set(spoofed.object_ids())
    return {"added": sorted(s - h), "removed": sorted(h - s), "kept": sorted(h & s)}
