"""Receiver-side Vulnerable Road User (VRU) Crossing Warning decision logic.

Companion to ``do_not_pass_warning``: same shape of problem (ego pose +
perceived objects -> a binary safety decision), applied to a blind
intersection instead of an overtake. A pedestrian or cyclist approaching on the
cross street is a hazard the ego cannot see round -- see
``scenarios/intersection_vru_spoofing.py`` for the corner-building occlusion --
so it depends on a Road Side Unit's Collective Perception Message to know they
are there at all.

    (ego pose, perceived objects)  ->  GO | STOP

Hazard model
------------
A perceived pedestrian/cyclist is a crossing hazard when it is ahead of the ego
(on the approach to the intersection) and is either already inside the ego's
lane corridor, or closing on it from the side. Two arrival times are compared:

* ``ego_time_to_conflict_s``  -- how long until the ego reaches the crossing
  point, at its desired speed (not necessarily its current one -- a stopped
  ego considering whether to depart asks "if I go now, do I make it across
  before they do", not "how long until I coast there at 0 km/h").
* ``vru_time_to_lane_s``      -- how long until the VRU reaches the near edge
  of that corridor (zero if it is already inside it).

The decision is STOP when the two arrival times are within
``required_clearance_time_s`` of each other -- the classic gap-acceptance
criterion for a crossing conflict, the same shape of test
``do_not_pass_warning`` uses for a head-on passing conflict, just applied to a
lateral crossing instead of a longitudinal one.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .do_not_pass_warning import EgoState, ego_frame, relative_to_ego
from .v2x.cpm import PerceivedObject

Vec3 = Tuple[float, float, float]

GO = "GO"
STOP = "STOP"

# Classes that count as a vulnerable road user for this warning. Vehicles are
# somebody else's problem (do_not_pass_warning.VEHICLE_CLASSES).
VRU_CLASSES = frozenset({"pedestrian", "bicycle"})


def _along(vec: Vec3, unit: Tuple[float, float]) -> float:
    return vec[0] * unit[0] + vec[1] * unit[1]


@dataclass
class VRUCrossingThresholds:
    """Tunable decision thresholds (sane defaults for a two-lane town crossing)."""

    corridor_half_width_m: float = 2.0        # the ego's own lane, the VRU must cross it
    max_relevant_range_m: float = 60.0        # ignore VRUs further ahead than this
    required_clearance_time_s: float = 4.0    # minimum gap between the two arrival times
    min_speed_mps: float = 0.05               # below this, treat a velocity as zero


@dataclass
class VRUCrossingDecision:
    """The decision plus every number that produced it (so logs are auditable)."""

    decision: str
    reason: str
    blocking_object_id: Optional[int] = None
    ego_time_to_conflict_s: Optional[float] = None
    vru_time_to_lane_s: Optional[float] = None
    n_vru_considered: int = 0
    n_objects_considered: int = 0

    @property
    def is_stop(self) -> bool:
        return self.decision == STOP

    def to_dict(self) -> dict:
        def r(v, n=2):
            return None if v is None else (round(v, n) if math.isfinite(v) else None)
        return {
            "decision": self.decision,
            "reason": self.reason,
            "blocking_object_id": self.blocking_object_id,
            "ego_time_to_conflict_s": r(self.ego_time_to_conflict_s),
            "vru_time_to_lane_s": r(self.vru_time_to_lane_s),
            "n_vru_considered": self.n_vru_considered,
            "n_objects_considered": self.n_objects_considered,
        }


def evaluate_crossing(ego: EgoState, perceived_objects: Sequence[PerceivedObject],
                      thresholds: Optional[VRUCrossingThresholds] = None
                      ) -> VRUCrossingDecision:
    """Pure function: ego pose + perceived objects -> GO / STOP."""
    t = thresholds or VRUCrossingThresholds()
    fwd, right = ego_frame(ego.yaw_deg)
    # The desired speed, not the current one: a stopped ego deciding whether to
    # depart asks "would I clear the crossing if I went now", which is a
    # question about the speed it means to travel at, not its speed at a
    # standstill (see do_not_pass_warning.EgoState.reference_speed).
    ego_speed = max(t.min_speed_mps, ego.reference_speed)

    considered = 0
    hazards: List[Tuple[float, PerceivedObject, float, float]] = []  # (gap, obj, ego_ttc, vru_ttc)

    for o in perceived_objects:
        if o.object_id == ego.station_id:
            continue                      # never reason about yourself
        if o.classification not in VRU_CLASSES:
            continue
        s, d = relative_to_ego(ego, o.position)
        if s <= 0.0 or s > t.max_relevant_range_m:
            continue                      # behind the ego, or too far to matter yet
        considered += 1

        v_lat = _along(o.velocity, right)     # + moving right, - moving left, ego frame
        already_in_lane = abs(d) <= t.corridor_half_width_m
        moving_toward_lane = ((d > 0.0 and v_lat < -t.min_speed_mps)
                              or (d < 0.0 and v_lat > t.min_speed_mps))
        if not already_in_lane and not moving_toward_lane:
            continue                      # off to the side, not closing: no conflict

        vru_ttc = 0.0 if already_in_lane else \
            (abs(d) - t.corridor_half_width_m) / abs(v_lat)
        ego_ttc = s / ego_speed
        hazards.append((abs(ego_ttc - vru_ttc), o, ego_ttc, vru_ttc))

    if not hazards:
        return VRUCrossingDecision(
            decision=GO, reason="no pedestrian/cyclist near the crossing",
            n_objects_considered=considered)

    # Most critical hazard: whichever pair of arrival times is closest together.
    gap, obj, ego_ttc, vru_ttc = min(hazards, key=lambda h: h[0])
    if gap <= t.required_clearance_time_s:
        return VRUCrossingDecision(
            decision=STOP,
            reason=(f"{obj.classification} {obj.object_id} would be in the "
                   f"crossing within {gap:.1f}s of the ego (required "
                   f"{t.required_clearance_time_s:.0f}s clearance)"),
            blocking_object_id=obj.object_id,
            ego_time_to_conflict_s=ego_ttc, vru_time_to_lane_s=vru_ttc,
            n_vru_considered=len(hazards), n_objects_considered=considered)

    return VRUCrossingDecision(
        decision=GO,
        reason=f"nearest {obj.classification} {obj.object_id} clears with {gap:.1f}s to spare",
        ego_time_to_conflict_s=ego_ttc, vru_time_to_lane_s=vru_ttc,
        n_vru_considered=len(hazards), n_objects_considered=considered)
