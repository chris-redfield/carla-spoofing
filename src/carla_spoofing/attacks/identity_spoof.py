"""Identity spoofing: broadcast a CPM under someone else's station id.

Every other attack here edits *what* a message says. This one edits **who it
claims to be from** -- the drone transmits, but the message is stamped with the
victim infrastructure's ``station_id`` and reference position, so a receiver
files it under the Road Side Unit's entry.

That distinction is what makes suppression work at all. A receiver fuses CPMs
per station (see ``fusion.fuse_latest_by_station``): a message from a *new*
sender ADDS to the world view, so dropping an object from it changes nothing --
the genuine RSU still reports the object. A message that impersonates the RSU
REPLACES the RSU's entry, so anything the attacker omits genuinely vanishes.

The attack composes: wrap an inner attack (typically ``RemoveObjectAttack``) and
this re-stamps whatever the inner attack produced.

    IdentitySpoofAttack(ForgedIdentity(rsu_id, rsu_pose, "rsu"),
                        inner=RemoveObjectAttack(RemoveTarget(object_id=oncoming)))

Signed/certificated V2X (IEEE 1609.2, ETSI TS 103 097) is what stops this in a
deployed system; modelling that defence is future work -- see the detection
framework noted in the project memory.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .base import Attack
from ..v2x.cpm import CollectivePerceptionMessage

Vec3 = Tuple[float, float, float]


@dataclass
class ForgedIdentity:
    """The station the attacker pretends to be.

    ``reference_position`` should be the impersonated station's *real* pose, not
    the attacker's: a receiver that sanity-checks a known RSU against its
    registered location must see the expected coordinates. The attacker's true
    pose is recorded out-of-band in the attack result instead, where it is
    available as ground truth for direction-of-arrival style detection work.
    """

    station_id: int
    reference_position: Optional[Vec3] = None
    station_type: Optional[str] = "rsu"


class IdentitySpoofAttack(Attack):
    name = "identity_spoof"

    def __init__(self, identity: ForgedIdentity, inner: Optional[Attack] = None):
        self.identity = identity
        self.inner = inner
        # Composite name, e.g. "identity_spoof+remove_object", so logs and
        # run summaries say what the full attack chain was.
        if inner is not None and inner.name != "none":
            self.name = f"{IdentitySpoofAttack.name}+{inner.name}"
        super().__init__()

    def apply_cpm(self, cpm: CollectivePerceptionMessage
                  ) -> CollectivePerceptionMessage:
        out = self.inner.apply_cpm(cpm) if self.inner is not None else cpm.copy()
        if out is cpm:
            out = cpm.copy()                   # never mutate the honest message
        true_sender = cpm.station_id
        out.station_id = self.identity.station_id
        if self.identity.reference_position is not None:
            out.reference_position = self.identity.reference_position
        if self.identity.station_type is not None:
            out.station_type = self.identity.station_type

        if self.inner is not None:
            self.result.added_object_ids = list(self.inner.result.added_object_ids)
            self.result.removed_object_ids = list(self.inner.result.removed_object_ids)
            inner_note = self.inner.result.notes
        else:
            inner_note = ""
        self.result.impersonated_station_id = self.identity.station_id
        self.result.true_sender_id = true_sender
        note = (f"sender {true_sender} broadcast as station "
                f"{self.identity.station_id} ({self.identity.station_type})")
        self.result.notes = f"{note}; {inner_note}" if inner_note else note
        return out

    # Raw-sensor channels belong to the inner attack, if any.
    def apply_pointcloud(self, points):
        if self.inner is None:
            return points
        out = self.inner.apply_pointcloud(points)
        self.result.points_added = self.inner.result.points_added
        self.result.points_removed = self.inner.result.points_removed
        return out

    def apply_image(self, image, projector=None):
        if self.inner is None:
            return image
        out = self.inner.apply_image(image, projector)
        self.result.image_injections = list(self.inner.result.image_injections)
        return out
