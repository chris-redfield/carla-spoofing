"""Receiver-side fusion of incoming CPMs into one world view.

Why this matters for the attack
-------------------------------
A collective-perception receiver keeps **one entry per sending station**: a
station's newest CPM replaces whatever that station said before. That rule is
exactly what makes identity spoofing powerful. If the drone broadcast under its
own id, the victim would simply hold two views -- the RSU's (with the oncoming
car) and the drone's (without it) -- and the union would still contain the
hazard, so suppression would fail. By stamping the forged CPM with the **RSU's**
station id, the drone's message *supersedes* the genuine one instead of adding
to it, and the oncoming vehicle disappears from the fused view.

So ``fuse_latest_by_station`` is not incidental plumbing: it is the mechanism the
attack exploits, and it is modelled explicitly so the effect is visible in logs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Sequence, Tuple

from .v2x.cpm import CollectivePerceptionMessage, PerceivedObject

Vec3 = Tuple[float, float, float]


@dataclass
class FusedView:
    """The receiver's world model plus provenance for every object."""

    objects: List[PerceivedObject] = field(default_factory=list)
    source_station: Dict[int, int] = field(default_factory=dict)   # object_id -> station_id
    stations: Dict[int, float] = field(default_factory=dict)       # station_id -> generation_time
    superseded: List[int] = field(default_factory=list)            # station ids overwritten

    def object_ids(self) -> List[int]:
        return [o.object_id for o in self.objects]


def fuse_latest_by_station(cpms: Sequence[CollectivePerceptionMessage]) -> FusedView:
    """Keep each station's newest CPM, then union the objects it reports.

    Messages are processed in arrival order. A later message from a station id
    replaces an earlier one when its ``generation_time`` is not older -- ties go
    to the later arrival, which is the realistic outcome for a replay/overwrite
    attacker that transmits just after the station it impersonates.
    """
    latest: Dict[int, CollectivePerceptionMessage] = {}
    superseded: List[int] = []
    for cpm in cpms:
        prev = latest.get(cpm.station_id)
        if prev is not None:
            if cpm.generation_time < prev.generation_time:
                continue                      # stale, drop it
            superseded.append(cpm.station_id)
        latest[cpm.station_id] = cpm

    view = FusedView(superseded=superseded)
    by_id: Dict[int, PerceivedObject] = {}
    for sid, cpm in latest.items():
        view.stations[sid] = cpm.generation_time
        for o in cpm.perceived_objects:
            # Two stations may both see an object; keep the more confident report.
            known = by_id.get(o.object_id)
            if known is None or o.confidence > known.confidence:
                by_id[o.object_id] = o
                view.source_station[o.object_id] = sid
    view.objects = [by_id[k] for k in sorted(by_id)]
    return view


def is_occluded(observer: Vec3, target: Vec3, blocker: Vec3,
                blocker_dimensions: Vec3, margin_deg: float = 0.0) -> bool:
    """Angular-shadow test: does ``blocker`` hide ``target`` from ``observer``?

    A deliberately cheap stand-in for ray casting: the blocker hides the target
    when the target is farther away and falls inside the angular wedge the
    blocker subtends. Good enough to model the real premise of this scenario --
    the ego cannot see round the slow vehicle it wants to overtake, which is
    precisely why it must trust the infrastructure's message.
    """
    bx, by = blocker[0] - observer[0], blocker[1] - observer[1]
    tx, ty = target[0] - observer[0], target[1] - observer[1]
    d_blocker = math.hypot(bx, by)
    d_target = math.hypot(tx, ty)
    if d_blocker < 1e-3 or d_target <= d_blocker:
        return False                          # nearer than the blocker: visible
    # Half-width the blocker subtends, from its largest horizontal half-extent.
    half_extent = 0.5 * max(blocker_dimensions[0], blocker_dimensions[1])
    half_angle = math.atan2(half_extent, d_blocker) + math.radians(margin_deg)
    cos_sep = (bx * tx + by * ty) / (d_blocker * d_target)
    cos_sep = max(-1.0, min(1.0, cos_sep))
    return math.acos(cos_sep) <= half_angle


def visible_objects(observer: Vec3, objects: Sequence[PerceivedObject],
                    blockers: Sequence[PerceivedObject],
                    margin_deg: float = 0.0) -> List[PerceivedObject]:
    """Filter ``objects`` to those not hidden behind any of ``blockers``."""
    out = []
    for o in objects:
        hidden = any(
            b.object_id != o.object_id
            and is_occluded(observer, o.position, b.position, b.dimensions, margin_deg)
            for b in blockers
        )
        if not hidden:
            out.append(o)
    return out
