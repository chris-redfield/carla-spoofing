"""Attack base class and no-op attack."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..v2x.cpm import CollectivePerceptionMessage


@dataclass
class AttackResult:
    """Book-keeping about what an attack changed (for logging/evaluation).

    This is *ground-truth about the attack itself* and is NOT part of the wire
    payload the victim receives -- the whole point of a good spoof is that the
    victim can't tell. It's recorded out-of-band so researchers can score it.
    """
    name: str
    added_object_ids: list = field(default_factory=list)
    removed_object_ids: list = field(default_factory=list)
    points_added: int = 0
    points_removed: int = 0
    image_injections: list = field(default_factory=list)  # list of dicts
    # Identity spoofing: the station id the message CLAIMS vs who really sent it.
    # Both None for attacks that only edit the object list.
    impersonated_station_id: Optional[int] = None
    true_sender_id: Optional[int] = None
    notes: str = ""

    def to_dict(self) -> dict:
        return {
            "attack": self.name,
            "added_object_ids": self.added_object_ids,
            "removed_object_ids": self.removed_object_ids,
            "points_added": self.points_added,
            "points_removed": self.points_removed,
            "image_injections": self.image_injections,
            "impersonated_station_id": self.impersonated_station_id,
            "true_sender_id": self.true_sender_id,
            "notes": self.notes,
        }


class Attack:
    """Base class. Subclasses override the modality methods they affect."""

    name = "attack"

    def __init__(self):
        self.result = AttackResult(name=self.name)

    def apply_cpm(self, cpm: CollectivePerceptionMessage
                  ) -> CollectivePerceptionMessage:
        return cpm

    def apply_pointcloud(self, points: Optional[np.ndarray]) -> Optional[np.ndarray]:
        return points

    def apply_image(self, image, projector=None):
        return image


class NoAttack(Attack):
    """Identity attack (honest baseline)."""
    name = "none"
