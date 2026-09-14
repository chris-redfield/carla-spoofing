"""Build an *honest* CPM (and optional point cloud) from a world.

Two backends:
* CARLA world  -> ``build_cpm_from_carla`` reads ``world.get_actors()`` and
  reports vehicles/walkers within range of the sender (the ego/attacker). This
  is the ground truth an honest station would broadcast.
* Generic/mock -> ``build_cpm_from_objects`` builds the same CPM from a list of
  plain object states, so the whole pipeline runs without CARLA.

An attacker starts from this honest CPM and then applies a spoofing attack.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .config import DEFAULT_PERCEPTION_RANGE, CPM_PROTOCOL_VERSION
from .v2x.cpm import CollectivePerceptionMessage, PerceivedObject

Vec3 = Tuple[float, float, float]


@dataclass
class ObjectState:
    """Backend-independent object state (what a perfect sensor would measure)."""
    object_id: int
    position: Vec3
    velocity: Vec3 = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0
    dimensions: Vec3 = (4.5, 2.0, 1.5)
    classification: str = "car"


def _dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def build_cpm_from_objects(station_id: int, reference_position: Vec3,
                           objects: Sequence[ObjectState],
                           generation_time: float,
                           perception_range: float = DEFAULT_PERCEPTION_RANGE,
                           station_type: str = "vehicle",
                           confidence: float = 0.95
                           ) -> CollectivePerceptionMessage:
    """Honest CPM: report every object within ``perception_range`` of sender."""
    cpm = CollectivePerceptionMessage(
        station_id=station_id,
        generation_time=generation_time,
        reference_position=reference_position,
        station_type=station_type,
        protocol_version=CPM_PROTOCOL_VERSION,
    )
    for obj in objects:
        if obj.object_id == station_id:
            continue  # don't report yourself
        if _dist(obj.position, reference_position) > perception_range:
            continue
        cpm.add_object(PerceivedObject(
            object_id=obj.object_id,
            position=obj.position,
            velocity=obj.velocity,
            yaw_deg=obj.yaw_deg,
            dimensions=obj.dimensions,
            classification=obj.classification,
            confidence=confidence,
        ))
    return cpm


# --------------------------------------------------------------------------- #
# CARLA backend (imported lazily so the library works without carla installed) #
# --------------------------------------------------------------------------- #
def _classify_carla(actor) -> str:
    tid = actor.type_id
    if tid.startswith("walker"):
        return "pedestrian"
    if "bike" in tid or "bicycle" in tid or "vespa" in tid or "yamaha" in tid \
            or "harley" in tid or "kawasaki" in tid:
        return "bicycle"
    if any(t in tid for t in ("truck", "carlacola", "cybertruck", "firetruck",
                              "ambulance", "sprinter", "bus")):
        return "truck"
    if tid.startswith("vehicle"):
        return "car"
    return "unknown"


def carla_actor_to_state(actor) -> ObjectState:
    """Convert a CARLA actor to a backend-independent ObjectState."""
    tf = actor.get_transform()
    loc, rot = tf.location, tf.rotation
    vel = actor.get_velocity()
    try:
        ext = actor.bounding_box.extent  # half-sizes
        dims = (2 * ext.x, 2 * ext.y, 2 * ext.z)
    except Exception:
        dims = (4.5, 2.0, 1.5)
    return ObjectState(
        object_id=actor.id,
        position=(loc.x, loc.y, loc.z),
        velocity=(vel.x, vel.y, vel.z),
        yaw_deg=rot.yaw,
        dimensions=dims,
        classification=_classify_carla(actor),
    )


def build_cpm_from_carla(world, sender_actor,
                         perception_range: float = DEFAULT_PERCEPTION_RANGE,
                         confidence: float = 0.95
                         ) -> CollectivePerceptionMessage:
    """Honest CPM from a live CARLA world, as seen by ``sender_actor``."""
    snapshot = world.get_snapshot()
    gen_time = snapshot.timestamp.elapsed_seconds
    sender_state = carla_actor_to_state(sender_actor)
    states: List[ObjectState] = []
    for actor in world.get_actors():
        tid = actor.type_id
        if not (tid.startswith("vehicle") or tid.startswith("walker")):
            continue
        states.append(carla_actor_to_state(actor))
    station_type = "drone" if "drone" in sender_actor.type_id or \
        "uav" in sender_actor.type_id else "vehicle"
    return build_cpm_from_objects(
        station_id=sender_actor.id,
        reference_position=sender_state.position,
        objects=states,
        generation_time=gen_time,
        perception_range=perception_range,
        station_type=station_type,
        confidence=confidence,
    )
