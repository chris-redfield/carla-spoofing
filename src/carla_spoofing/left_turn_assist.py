"""Receiver-side Left Turn Assist decision logic.

The second piece of *victim* logic in the toolkit, and the sibling of
``do_not_pass_warning``:

    (ego pose, perceived objects)  ->  TURN | DO_NOT_TURN

The manoeuvre modelled is a **permissive left turn**: the ego has a green ball
(not a protected arrow), so it is legally entitled to enter the intersection and
the entire safety question is whether the gap in oncoming through-traffic is big
enough to cross. Nothing here is about running a red light. That distinction is
the point of the use case -- under attack the ego does not break a rule, it
correctly executes a turn on a world model that has been falsified.

Semantics
---------
``decision`` answers **"is turning left across the opposing lane unsafe right
now?"**. ``turn_intent`` separately reports whether the ego is actually at the
junction wanting to turn. As in the do-not-pass application they are kept apart
so the measured decision flip does not silently depend on where the ego happens
to be in a given run. An *unsafe turn* is the conjunction: ``turn_intent and
decision == TURN`` while a conflicting vehicle really exists in ground truth.

Hazard model
------------
The **conflict point** is where the ego's turn path crosses another stream of
traffic. A left turn has two such streams -- the oncoming through-lane it cuts
across, and the cross street it turns into -- and one rule covers both: a
perceived object conflicts when it is *not* travelling the ego's way, is still
approaching the conflict point, and will pass close enough to it to matter.

Which stream provides the hazard is a property of the scene, not of this module,
and it decides what can hide it. Oncoming traffic is nearly head-on down the same
road, so only another vehicle can occlude it. Cross-street traffic arrives from
the side, which is what a building on the junction corner hides. It blocks the
turn when either

* its **time to arrival** at the conflict point is below ``critical_gap_s`` --
  the classic accepted-gap criterion for a permitted left turn, or
* it is already within ``min_gap_m`` of the conflict point, which catches a
  slow-moving or stopped vehicle that time-to-arrival would score as harmless.

Why time-to-arrival and not time-to-collision: the ego is typically stationary
or crawling when it makes this decision, so a closing-speed formulation
degenerates. What matters is how long the ego has before the conflict point is
occupied, measured against how long the ego needs to clear it
(``clearance_time_s``); ``critical_gap_s`` must exceed that with margin.

Thresholds are deliberately explicit and overridable -- see
:class:`LeftTurnThresholds`. All geometry is in the CARLA world frame.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .geometry import (
    VEHICLE_CLASSES, EgoState, Vec3, along, ego_frame, heading_unit,
    relative_to_ego, speed_of,
)
from .v2x.cpm import PerceivedObject

TURN = "TURN"
DO_NOT_TURN = "DO_NOT_TURN"


@dataclass
class LeftTurnThresholds:
    """Tunable decision thresholds for a permissive left turn in town.

    ``critical_gap_s`` is the accepted-gap figure: the Highway Capacity Manual
    puts the critical headway for a permitted left turn against opposing through
    traffic at ~4.1 s, and a *warning* application wants margin on top of the
    value at which a median driver would commit. 5.5 s is that, and it sits
    comfortably above ``clearance_time_s``.

    ``min_gap_m`` exists because time-to-arrival is blind to a stopped car: a
    vehicle 5 m from the conflict point at 0 m/s scores an infinite gap while
    plainly blocking the turn.
    """

    critical_gap_s: float = 5.5
    clearance_time_s: float = 4.0        # how long the ego needs to cross
    min_gap_m: float = 12.0              # too close regardless of speed
    conflict_ahead_m: float = 8.0        # stop line -> conflict point, when not given
    approach_distance_m: float = 30.0    # within this, the ego is deciding
    conflict_corridor_half_width_m: float = 5.0   # must actually cross our path
    # A left turn conflicts with everything that is NOT going our way: oncoming
    # through-traffic, and traffic on the cross street the turn puts us into.
    # One rule covers both -- anything within 60 deg of our own heading is
    # travelling with us and cannot be crossed.
    same_direction_cos: float = 0.5
    min_speed_mps: float = 0.1           # below this, treat a velocity as zero


@dataclass
class LeftTurnDecision:
    """The decision plus every number that produced it (so logs are auditable)."""

    decision: str
    reason: str
    turn_intent: bool = False
    conflicting_object_id: Optional[int] = None
    gap_m: Optional[float] = None                 # hazard -> conflict point
    approach_speed_mps: Optional[float] = None
    time_to_arrival_s: Optional[float] = None
    ego_to_conflict_m: Optional[float] = None
    n_conflicting: int = 0
    n_objects_considered: int = 0

    @property
    def is_do_not_turn(self) -> bool:
        return self.decision == DO_NOT_TURN

    def to_dict(self) -> dict:
        def r(v, n=2):
            return None if v is None else (round(v, n) if math.isfinite(v) else None)
        return {
            "decision": self.decision,
            "reason": self.reason,
            "turn_intent": self.turn_intent,
            "conflicting_object_id": self.conflicting_object_id,
            "gap_m": r(self.gap_m),
            "approach_speed_mps": r(self.approach_speed_mps),
            "time_to_arrival_s": r(self.time_to_arrival_s),
            "ego_to_conflict_m": r(self.ego_to_conflict_m),
            "n_conflicting": self.n_conflicting,
            "n_objects_considered": self.n_objects_considered,
        }


def default_conflict_point(ego: EgoState, ahead_m: float) -> Vec3:
    """Where the ego's turn path crosses the opposing lane, if nobody says.

    A fallback only. The scenario knows the junction geometry and should pass a
    fixed world point instead: this estimate travels with the ego, so it stops
    being meaningful the moment the ego starts turning.
    """
    fwd, _ = ego_frame(ego.yaw_deg)
    return (ego.position[0] + fwd[0] * ahead_m,
            ego.position[1] + fwd[1] * ahead_m,
            ego.position[2])


def evaluate_left_turn(ego: EgoState,
                       perceived_objects: Sequence[PerceivedObject],
                       conflict_point: Optional[Vec3] = None,
                       thresholds: Optional[LeftTurnThresholds] = None
                       ) -> LeftTurnDecision:
    """Pure function: ego pose + perceived objects -> TURN / DO_NOT_TURN."""
    t = thresholds or LeftTurnThresholds()
    fwd, _ = ego_frame(ego.yaw_deg)
    cp = conflict_point if conflict_point is not None \
        else default_conflict_point(ego, t.conflict_ahead_m)

    # How far the ego still is from the point it has to cross. Signed along the
    # ego's heading so a conflict point already behind it reads as negative and
    # the ego stops claiming intent once it is through the junction.
    ego_s, _ = relative_to_ego(ego, cp)
    turn_intent = 0.0 <= ego_s <= t.approach_distance_m

    hazards: List[Tuple[float, float, float, PerceivedObject]] = []
    considered = 0

    for o in perceived_objects:
        if o.object_id == ego.station_id:
            continue                       # never reason about yourself
        if o.classification not in VEHICLE_CLASSES:
            continue
        considered += 1

        obj_fwd = heading_unit(o.yaw_deg)
        align = fwd[0] * obj_fwd[0] + fwd[1] * obj_fwd[1]
        if align >= t.same_direction_cos:
            continue                       # travelling with us: nothing to cross

        # Where the object is relative to the conflict point, in ITS frame:
        # how far it still has to travel, and how far it will miss us by.
        dx = cp[0] - o.position[0]
        dy = cp[1] - o.position[1]
        approach_dist = dx * obj_fwd[0] + dy * obj_fwd[1]
        lateral_miss = abs(-dx * obj_fwd[1] + dy * obj_fwd[0])

        if approach_dist < 0.0:
            continue                       # already through the junction
        if lateral_miss > t.conflict_corridor_half_width_m:
            continue                       # its path does not cross ours

        speed = speed_of(o.velocity)
        tta = approach_dist / speed if speed > t.min_speed_mps else math.inf
        hazards.append((tta, approach_dist, speed, o))

    if not hazards:
        return LeftTurnDecision(
            decision=TURN,
            reason="no conflicting vehicle approaching the conflict point",
            turn_intent=turn_intent, ego_to_conflict_m=ego_s,
            n_objects_considered=considered)

    # Most critical hazard: soonest arrival, then shortest distance.
    tta, gap, speed, obj = min(hazards, key=lambda h: (h[0], h[1]))
    too_soon = tta <= t.critical_gap_s
    too_close = gap <= t.min_gap_m
    if too_soon or too_close:
        why = []
        if too_soon:
            why.append(f"arrives in {tta:.1f} s <= {t.critical_gap_s:.1f} s "
                       f"(need {t.clearance_time_s:.1f} s to cross)")
        if too_close:
            why.append(f"only {gap:.1f} m <= {t.min_gap_m:.0f} m from the "
                       f"conflict point")
        return LeftTurnDecision(
            decision=DO_NOT_TURN,
            reason=f"oncoming {obj.classification} {obj.object_id}: " + "; ".join(why),
            turn_intent=turn_intent, conflicting_object_id=obj.object_id,
            gap_m=gap, approach_speed_mps=speed, time_to_arrival_s=tta,
            ego_to_conflict_m=ego_s, n_conflicting=len(hazards),
            n_objects_considered=considered)

    return LeftTurnDecision(
        decision=TURN,
        reason=(f"nearest oncoming {obj.object_id} arrives in {tta:.1f} s, "
                f"gap is adequate"),
        turn_intent=turn_intent, gap_m=gap, approach_speed_mps=speed,
        time_to_arrival_s=tta, ego_to_conflict_m=ego_s,
        n_conflicting=len(hazards), n_objects_considered=considered)


def compare_decisions(honest: LeftTurnDecision,
                      spoofed: LeftTurnDecision) -> dict:
    """Summarise an honest-vs-spoofed pair (the headline result of the run)."""
    return {
        "flipped": honest.decision != spoofed.decision,
        # The attack we care about: a real hazard suppressed into a clear gap.
        "unsafe_turn_enabled": (honest.decision == DO_NOT_TURN
                                and spoofed.decision == TURN),
        "honest_decision": honest.decision,
        "spoofed_decision": spoofed.decision,
        "hidden_object_id": (honest.conflicting_object_id
                             if honest.conflicting_object_id
                             != spoofed.conflicting_object_id else None),
    }
