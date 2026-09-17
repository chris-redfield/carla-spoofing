"""Receiver-side Do-Not-Pass Warning (DNPW) decision logic.

This is the first piece of *victim* logic in the toolkit. Everything else in
``carla_spoofing`` produces messages; this consumes them and makes the safety
decision an overtaking-assist application would make:

    (ego pose, perceived objects)  ->  PASS | DO_NOT_PASS

Semantics
---------
``decision`` answers **"is overtaking unsafe right now?"** -- it fires whenever a
vehicle travelling the other way is inside the passing sight distance, whether
or not the ego actually wants to pass. ``pass_intent`` separately reports
whether a slower lead vehicle gives the ego a *reason* to pass. They are kept
apart on purpose: the spoofing experiment measures the decision flip, and that
measurement must not silently depend on how the traffic manager happens to be
pacing the ego in a given run. An *unsafe pass* is the conjunction:
``pass_intent and decision == PASS`` while a hazard really exists in ground truth.

Hazard model
------------
A perceived object is an oncoming hazard when it is ahead of the ego, inside a
corridor around the ego's path, and heading roughly the opposite way. It blocks
the pass when either

* the longitudinal gap is below ``min_sight_distance_m`` (you can see it at all
  within cooperative-perception range -> do not pull out), or
* the time to meet, gap / closing speed, is below ``required_pass_time_s``
  (the classic passing-sight-distance criterion: you cannot complete the
  manoeuvre before the gap closes).

Thresholds are deliberately explicit and overridable -- see
:class:`DoNotPassThresholds` for the defaults and where they come from.
All geometry is in the CARLA world frame (see ``config.py``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .v2x.cpm import PerceivedObject

Vec3 = Tuple[float, float, float]

PASS = "PASS"
DO_NOT_PASS = "DO_NOT_PASS"

# Classes that count as a vehicle for lead / oncoming reasoning. Pedestrians are
# a hazard for other warnings, not for "may I use the opposing lane".
VEHICLE_CLASSES = frozenset({"car", "truck", "bus", "bicycle", "motorcycle"})


@dataclass
class EgoState:
    """The receiver's own pose -- what the DNPW application knows about itself."""

    station_id: int
    position: Vec3
    yaw_deg: float = 0.0
    velocity: Vec3 = (0.0, 0.0, 0.0)
    # Speed the ego *wants* to travel at. Pass intent is judged against this,
    # not against the current speed: a car already queued behind a slow lead is
    # travelling at the lead's speed, and comparing the two would conclude it has
    # no reason to overtake -- exactly backwards. None falls back to the current
    # speed, which is right for a one-off evaluation with no controller attached.
    desired_speed_mps: Optional[float] = None

    @property
    def speed(self) -> float:
        return math.sqrt(sum(c * c for c in self.velocity))

    @property
    def reference_speed(self) -> float:
        return self.speed if self.desired_speed_mps is None else self.desired_speed_mps


@dataclass
class DoNotPassThresholds:
    """Tunable decision thresholds (sane defaults for a two-lane town road).

    ``min_sight_distance_m`` is tied to the cooperative-perception range rather
    than to a road-design table: at 75 m (``DEFAULT_PERCEPTION_RANGE``) an
    oncoming vehicle that is reported at all is close enough to forbid the pass.
    ``required_pass_time_s`` is the usual ~10 s allowed for a passing manoeuvre
    on a two-lane rural road.
    """

    lane_half_width_m: float = 2.0          # ego's own lane, for spotting the lead
    oncoming_corridor_half_width_m: float = 6.0  # ego lane + opposing lane
    lead_range_m: float = 60.0              # how far ahead a lead still matters
    lead_slower_margin_mps: float = 0.5     # lead must be this much slower
    min_sight_distance_m: float = 70.0
    required_pass_time_s: float = 10.0
    opposed_heading_cos: float = -0.5       # >120 deg apart counts as oncoming
    min_speed_mps: float = 0.1              # below this, treat a velocity as zero


@dataclass
class DoNotPassDecision:
    """The decision plus every number that produced it (so logs are auditable)."""

    decision: str
    reason: str
    pass_intent: bool = False
    lead_object_id: Optional[int] = None
    blocking_object_id: Optional[int] = None
    gap_m: Optional[float] = None
    closing_speed_mps: Optional[float] = None
    time_to_meet_s: Optional[float] = None
    n_oncoming: int = 0
    n_objects_considered: int = 0

    @property
    def is_do_not_pass(self) -> bool:
        return self.decision == DO_NOT_PASS

    def to_dict(self) -> dict:
        def r(v, n=2):
            return None if v is None else (round(v, n) if math.isfinite(v) else None)
        return {
            "decision": self.decision,
            "reason": self.reason,
            "pass_intent": self.pass_intent,
            "lead_object_id": self.lead_object_id,
            "blocking_object_id": self.blocking_object_id,
            "gap_m": r(self.gap_m),
            "closing_speed_mps": r(self.closing_speed_mps),
            "time_to_meet_s": r(self.time_to_meet_s),
            "n_oncoming": self.n_oncoming,
            "n_objects_considered": self.n_objects_considered,
        }


# --------------------------------------------------------------------------- #
# Geometry helpers (CARLA frame: X forward, Y right, yaw about Z, degrees)     #
# --------------------------------------------------------------------------- #
def heading_unit(yaw_deg: float) -> Tuple[float, float]:
    """Planar forward unit vector for a yaw, matching CARLA's forward vector."""
    r = math.radians(yaw_deg)
    return (math.cos(r), math.sin(r))


def ego_frame(yaw_deg: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """(forward, right) unit vectors. Y is right-handed *in CARLA's left-handed
    world*, so 'right' is the forward vector rotated +90 deg about Z."""
    fx, fy = heading_unit(yaw_deg)
    return (fx, fy), (-fy, fx)


def relative_to_ego(ego: EgoState, position: Vec3) -> Tuple[float, float]:
    """(longitudinal, lateral) offset of ``position`` in the ego's frame."""
    fwd, right = ego_frame(ego.yaw_deg)
    dx = position[0] - ego.position[0]
    dy = position[1] - ego.position[1]
    return dx * fwd[0] + dy * fwd[1], dx * right[0] + dy * right[1]


def _along(vec: Vec3, unit: Tuple[float, float]) -> float:
    return vec[0] * unit[0] + vec[1] * unit[1]


# --------------------------------------------------------------------------- #
# The decision                                                                 #
# --------------------------------------------------------------------------- #
def evaluate_do_not_pass(ego: EgoState,
                         perceived_objects: Sequence[PerceivedObject],
                         thresholds: Optional[DoNotPassThresholds] = None
                         ) -> DoNotPassDecision:
    """Pure function: ego pose + perceived objects -> PASS / DO_NOT_PASS."""
    t = thresholds or DoNotPassThresholds()
    fwd, right = ego_frame(ego.yaw_deg)
    ego_along = max(0.0, _along(ego.velocity, fwd))

    lead: Optional[Tuple[float, PerceivedObject, float]] = None  # (gap, obj, speed)
    hazards: List[Tuple[float, float, float, PerceivedObject]] = []  # (ttm, gap, closing, obj)
    considered = 0

    for o in perceived_objects:
        if o.object_id == ego.station_id:
            continue                      # never reason about yourself
        if o.classification not in VEHICLE_CLASSES:
            continue
        considered += 1
        s, d = relative_to_ego(ego, o.position)
        if s <= 0.0:
            continue                      # behind the ego
        obj_fwd = heading_unit(o.yaw_deg)
        align = fwd[0] * obj_fwd[0] + fwd[1] * obj_fwd[1]

        # --- same-direction vehicle in our lane: the reason to overtake ---
        if align > 0.5 and abs(d) <= t.lane_half_width_m and s <= t.lead_range_m:
            speed = math.sqrt(sum(c * c for c in o.velocity))
            if lead is None or s < lead[0]:
                lead = (s, o, speed)
            continue

        # --- opposing-direction vehicle in the corridor: the hazard ---
        if align < t.opposed_heading_cos and abs(d) <= t.oncoming_corridor_half_width_m:
            approach = max(0.0, -_along(o.velocity, fwd))
            closing = ego_along + approach
            ttm = s / closing if closing > t.min_speed_mps else math.inf
            hazards.append((ttm, s, closing, o))

    pass_intent = False
    lead_id = None
    if lead is not None:
        lead_id = lead[1].object_id
        pass_intent = lead[2] <= max(0.0, ego.reference_speed - t.lead_slower_margin_mps)

    if not hazards:
        return DoNotPassDecision(
            decision=PASS, reason="no oncoming vehicle within perception range",
            pass_intent=pass_intent, lead_object_id=lead_id,
            n_objects_considered=considered)

    # Most critical hazard: soonest meeting, then shortest gap.
    ttm, gap, closing, obj = min(hazards, key=lambda h: (h[0], h[1]))
    too_close = gap <= t.min_sight_distance_m
    too_soon = ttm <= t.required_pass_time_s
    if too_close or too_soon:
        why = []
        if too_close:
            why.append(f"gap {gap:.1f} m <= {t.min_sight_distance_m:.0f} m")
        if too_soon:
            why.append(f"time-to-meet {ttm:.1f} s <= {t.required_pass_time_s:.0f} s")
        return DoNotPassDecision(
            decision=DO_NOT_PASS,
            reason=f"oncoming {obj.classification} {obj.object_id}: " + "; ".join(why),
            pass_intent=pass_intent, lead_object_id=lead_id,
            blocking_object_id=obj.object_id, gap_m=gap,
            closing_speed_mps=closing, time_to_meet_s=ttm,
            n_oncoming=len(hazards), n_objects_considered=considered)

    return DoNotPassDecision(
        decision=PASS,
        reason=f"nearest oncoming {obj.object_id} is {gap:.1f} m away, clear",
        pass_intent=pass_intent, lead_object_id=lead_id,
        gap_m=gap, closing_speed_mps=closing, time_to_meet_s=ttm,
        n_oncoming=len(hazards), n_objects_considered=considered)


def compare_decisions(honest: DoNotPassDecision, spoofed: DoNotPassDecision) -> dict:
    """Summarise an honest-vs-spoofed pair (the headline result of the run)."""
    flipped = honest.decision != spoofed.decision
    return {
        "flipped": flipped,
        # The attack we care about: a real hazard suppressed into a clear road.
        "unsafe_pass_enabled": (honest.decision == DO_NOT_PASS
                                and spoofed.decision == PASS),
        "honest_decision": honest.decision,
        "spoofed_decision": spoofed.decision,
        "hidden_object_id": (honest.blocking_object_id
                             if honest.blocking_object_id != spoofed.blocking_object_id
                             else None),
    }
