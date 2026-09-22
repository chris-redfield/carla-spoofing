"""Simple vehicle controllers -- deliberately minimal, but *actually closed-loop*.

The point of this module is causality. An earlier version of this experiment
(CarlaNetpp, use case 1) staged the crash: at t = 7.8 s the script called
``apply_control(throttle=1, steer=-1)`` and the car swerved into the opposing
lane regardless of what any message said. That demonstrates the *consequence* of
an unsafe pass, but not that the spoofed message *caused* it -- the same crash
happens with the attack switched off.

Here the ego has no script. It lane-follows, and it pulls out only when its
Do-Not-Pass Warning says the road is clear (see ``do_not_pass_warning``). Feed it
honest messages and it stays put; feed it the forged ones and it overtakes. The
collision, if it happens, is an emergent outcome of the attack.

The controllers are intentionally crude -- pure pursuit on the lane centreline
plus a proportional speed controller. They are enough to make the causal chain
real, and small enough to stay auditable. Anything fancier (a proper MPC, CARLA's
``BehaviorAgent``) would hide the mechanism behind a black box.

Backend independence: controllers consume a :class:`LaneReference`, so the same
controller drives a CARLA actor (waypoint-based reference) or a kinematic point
mass in the CARLA-free mock (straight-line reference).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .do_not_pass_warning import (
    DO_NOT_PASS, PASS, DoNotPassDecision, EgoState, heading_unit, relative_to_ego,
)
from .left_turn_assist import DO_NOT_TURN, TURN, LeftTurnDecision
from .v2x.cpm import PerceivedObject

Vec3 = Tuple[float, float, float]

KMH = 1.0 / 3.6


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


@dataclass
class VehicleCommand:
    """Backend-neutral actuation (maps 1:1 onto ``carla.VehicleControl``)."""

    throttle: float = 0.0
    steer: float = 0.0
    brake: float = 0.0

    def to_carla(self):
        import carla
        return carla.VehicleControl(throttle=float(self.throttle),
                                    steer=float(self.steer),
                                    brake=float(self.brake))


# --------------------------------------------------------------------------- #
# Lane reference: "where is the centreline, this far ahead of me?"             #
# --------------------------------------------------------------------------- #
class LaneReference:
    """Supplies a target point ahead on the lane, optionally offset sideways.

    ``lateral_offset`` is signed in the CARLA convention: positive is to the
    right of travel, so an overtake into the opposing lane uses a negative one.
    """

    def point_ahead(self, position: Vec3, yaw_deg: float, distance: float,
                    lateral_offset: float = 0.0) -> Vec3:
        raise NotImplementedError


@dataclass
class StraightLaneReference(LaneReference):
    """A perfectly straight lane through ``origin`` with heading ``yaw_deg``.

    Used by the mock backend and the tests, where a road graph would be overkill.
    """

    origin: Vec3 = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0

    def point_ahead(self, position: Vec3, yaw_deg: float, distance: float,
                    lateral_offset: float = 0.0) -> Vec3:
        fx, fy = heading_unit(self.yaw_deg)
        rx, ry = -fy, fx
        # Project ourselves onto the lane, then advance along it.
        dx, dy = position[0] - self.origin[0], position[1] - self.origin[1]
        s = dx * fx + dy * fy + distance
        x = self.origin[0] + fx * s + rx * lateral_offset
        y = self.origin[1] + fy * s + ry * lateral_offset
        return (x, y, position[2])


@dataclass
class PolylineReference(LaneReference):
    """A fixed path given as a list of points -- used for the turn through a junction.

    A junction has no single lane to follow: the ego leaves one lane, crosses
    open carriageway, and joins another running 90 degrees away. Rather than
    fight ``get_waypoint``, which snaps to whichever lane is nearest and would
    hand back the *opposing* lane halfway round the corner, the turn is baked
    into an explicit path once at setup and simply followed.

    ``point_ahead`` projects the ego onto the polyline, walks forward by the
    lookahead distance and returns that point, which is exactly what pure
    pursuit wants. ``progress`` exposes how far along the path the ego is, so
    the controller can tell "committed" from "still at the line".
    """

    points: List[Vec3] = field(default_factory=list)

    def __post_init__(self) -> None:
        self._cum = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self._cum.append(self._cum[-1] + math.dist(a[:2], b[:2]))

    @property
    def length(self) -> float:
        return self._cum[-1] if self._cum else 0.0

    def _nearest_index(self, position: Vec3) -> int:
        best, best_d = 0, float("inf")
        for i, p in enumerate(self.points):
            d = math.dist(p[:2], position[:2])
            if d < best_d:
                best, best_d = i, d
        return best

    def progress(self, position: Vec3) -> float:
        """Arc length from the start of the path to the ego's nearest point."""
        if not self.points:
            return 0.0
        return self._cum[self._nearest_index(position)]

    def remaining(self, position: Vec3) -> float:
        return max(0.0, self.length - self.progress(position))

    def point_ahead(self, position: Vec3, yaw_deg: float, distance: float,
                    lateral_offset: float = 0.0) -> Vec3:
        if not self.points:
            return StraightLaneReference(position, yaw_deg).point_ahead(
                position, yaw_deg, distance, lateral_offset)
        target_s = self.progress(position) + distance
        # Past the end: extrapolate along the final heading so the controller
        # keeps a sane reference while it drives out of the junction.
        if target_s >= self.length and len(self.points) >= 2:
            a, b = self.points[-2], self.points[-1]
            hx, hy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(hx, hy) or 1.0
            over = target_s - self.length
            return (b[0] + hx / n * over, b[1] + hy / n * over, b[2])
        i = max(1, next(k for k, s in enumerate(self._cum) if s >= target_s))
        a, b = self.points[i - 1], self.points[i]
        seg = self._cum[i] - self._cum[i - 1]
        f = 0.0 if seg <= 1e-9 else (target_s - self._cum[i - 1]) / seg
        px = a[0] + (b[0] - a[0]) * f
        py = a[1] + (b[1] - a[1]) * f
        pz = a[2] + (b[2] - a[2]) * f
        if lateral_offset:
            hx, hy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(hx, hy) or 1.0
            px += (-hy / n) * lateral_offset
            py += (hx / n) * lateral_offset
        return (px, py, pz)


def straight_points(start: Vec3, end: Vec3, step_m: float = 1.5) -> List[Vec3]:
    """Sample a straight line between two points, ``start`` excluded of nothing."""
    d = math.dist(start[:2], end[:2])
    n = max(1, int(round(d / step_m)))
    out = []
    for k in range(n + 1):
        t = k / n
        out.append((start[0] + (end[0] - start[0]) * t,
                    start[1] + (end[1] - start[1]) * t,
                    start[2] + (end[2] - start[2]) * t))
    return out


def approach_then_turn(ego_position: Vec3, entry: Vec3, entry_yaw_deg: float,
                       exit_: Vec3, exit_yaw_deg: float,
                       tangent_scale: float = 0.5) -> List[Vec3]:
    """Drive straight into the junction, THEN turn: the path a driver actually takes.

    An arc that starts where the car is standing begins bending while it is
    still short of the junction, and the car describes a long diagonal across
    the corner -- which is how the ego came to clip a traffic-light pole on the
    island between the two roads. A car stopped at the line does not turn from
    there; it pulls forward into the junction and turns from inside it.

    So the path is a straight run from the ego's pose up to ``entry`` (a point
    inside the junction, on the approach heading), and only then the arc round
    to the exit. The straight part also gives pure pursuit a sane target during
    the first moments of the manoeuvre, when a curve starting under the front
    axle gives it nothing useful to aim at.
    """
    lead_in = straight_points(ego_position, entry)
    arc = hermite_turn_path(entry, entry_yaw_deg, exit_, exit_yaw_deg,
                            tangent_scale=tangent_scale)
    return lead_in[:-1] + arc          # drop the duplicated join point


def hermite_turn_path(entry: Vec3, entry_yaw_deg: float,
                      exit_: Vec3, exit_yaw_deg: float,
                      samples: int = 24, tangent_scale: float = 0.6) -> List[Vec3]:
    """Smooth path from one pose to another -- the ego's track through the junction.

    A cubic Hermite spline, not a quarter-circle arc: real junction arms are not
    reliably at 90 degrees to each other, and the entry and exit lane centres are
    usually offset rather than meeting at a corner point. Hermite takes both
    poses and their headings and produces a curve that leaves along the entry
    heading and arrives along the exit heading whatever the angle between them.

    ``tangent_scale`` sets how far the curve runs straight before it bends --
    0.6 of the endpoint separation gives a turn that looks driven rather than
    geometric.
    """
    ex, ey = heading_unit(entry_yaw_deg)
    xx, xy = heading_unit(exit_yaw_deg)
    span = math.dist(entry[:2], exit_[:2]) * tangent_scale
    m0 = (ex * span, ey * span)
    m1 = (xx * span, xy * span)
    pts: List[Vec3] = []
    for k in range(samples + 1):
        t = k / samples
        t2, t3 = t * t, t * t * t
        h00 = 2 * t3 - 3 * t2 + 1
        h10 = t3 - 2 * t2 + t
        h01 = -2 * t3 + 3 * t2
        h11 = t3 - t2
        x = h00 * entry[0] + h10 * m0[0] + h01 * exit_[0] + h11 * m1[0]
        y = h00 * entry[1] + h10 * m0[1] + h01 * exit_[1] + h11 * m1[1]
        z = entry[2] + (exit_[2] - entry[2]) * t
        pts.append((x, y, z))
    return pts


def straightest(waypoints, heading: Tuple[float, float]):
    """The waypoint whose heading best matches ``heading``.

    CARLA's ``Waypoint.next()`` and ``.previous()`` return a *list* wherever the
    road graph branches -- every exit of a junction, every lane a road splits
    into. Indexing ``[0]`` picks whichever one happens to come first, which is
    arbitrary and silently wrong: it sent a vehicle that was supposed to cross a
    junction off down the right-hand exit, and walking backwards it put a
    vehicle's spawn on an entirely different road 64 m from where it belonged.

    Preferring the straightest continuation is the fix in both directions: going
    forward it means "carry on through the junction", going back it means "stay
    on this road".
    """
    best, best_score = None, -2.0
    for wp in waypoints:
        h = heading_unit(wp.transform.rotation.yaw)
        score = h[0] * heading[0] + h[1] * heading[1]
        if score > best_score:
            best, best_score = wp, score
    return best


def walk_back(waypoint, distance_m: float, step_m: float = 2.0):
    """Walk ``distance_m`` back along the road, staying on the SAME road.

    ``previous(80.0)`` in one hop is not the same thing: it branches at every
    junction on the way and returns an arbitrary member of the result. Stepping
    back a couple of metres at a time and taking the straightest branch each
    time keeps the walk on the approach the caller meant.

    Returns as far back as it got, which may be short of ``distance_m`` if the
    road runs out -- better a shorter approach than one on the wrong street.
    """
    forward = heading_unit(waypoint.transform.rotation.yaw)
    current, travelled = waypoint, 0.0
    while travelled < distance_m:
        hop = min(step_m, distance_m - travelled)
        candidates = current.previous(hop)
        if not candidates:
            break
        nxt = straightest(candidates, forward)
        if nxt is None:
            break
        current, travelled = nxt, travelled + hop
    return current, travelled


class CarlaLaneReference(LaneReference):
    """Follows the CARLA road graph via waypoints on the ego's current lane.

    An overtake is expressed as a lateral offset from the ego's *own* lane rather
    than by hopping onto the opposing lane's waypoints: the opposing lane runs
    the other way, so its waypoint headings would fight the controller. Offsetting
    the centreline gives the same trajectory with far less lane-graph handling.

    That alone is not enough, though. ``get_waypoint`` snaps to the *nearest*
    lane, so the moment an overtake actually puts the car across the line it
    starts resolving to the opposing lane -- whose heading is reversed. The
    reference then inverts mid-manoeuvre and the controller drives off the road.
    So the caller's intended heading is used to reject a lane running the wrong
    way and hop back to the one that runs with us.
    """

    def __init__(self, carla_map):
        self._map = carla_map

    @staticmethod
    def _runs_with(waypoint, forward) -> bool:
        wx, wy = heading_unit(waypoint.transform.rotation.yaw)
        return forward[0] * wx + forward[1] * wy > 0.0

    def point_ahead(self, position: Vec3, yaw_deg: float, distance: float,
                    lateral_offset: float = 0.0) -> Vec3:
        import carla
        loc = carla.Location(x=position[0], y=position[1], z=position[2])
        wp = self._map.get_waypoint(loc, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        if wp is None:
            return StraightLaneReference(position, yaw_deg).point_ahead(
                position, yaw_deg, distance, lateral_offset)

        # Straddling or crossing the centre line resolves to the oncoming lane.
        # Step back to the neighbouring lane that travels our way, so the
        # heading reference stays continuous through the whole overtake.
        forward = heading_unit(yaw_deg)
        if not self._runs_with(wp, forward):
            for neighbour in (wp.get_left_lane(), wp.get_right_lane()):
                if (neighbour is not None
                        and neighbour.lane_type == carla.LaneType.Driving
                        and self._runs_with(neighbour, forward)):
                    wp = neighbour
                    break
        # At a junction, next() returns EVERY branch -- left, straight, right --
        # and taking [0] picks an arbitrary one. A car meant to cross the
        # junction then turns off it, which is exactly what sent the crossing
        # vehicle away to the right instead of through. Keep going straight:
        # prefer the branch whose heading best matches where we are pointing.
        nxt = wp.next(max(0.5, distance))
        target_wp = straightest(nxt, forward) if nxt else wp
        tf = target_wp.transform
        fx, fy = heading_unit(tf.rotation.yaw)
        rx, ry = -fy, fx
        return (tf.location.x + rx * lateral_offset,
                tf.location.y + ry * lateral_offset,
                tf.location.z)


# --------------------------------------------------------------------------- #
# Base controller: pure pursuit + proportional speed control                   #
# --------------------------------------------------------------------------- #
@dataclass
class ControllerGains:
    lookahead_gain: float = 1.2        # seconds of travel to look ahead
    lookahead_min_m: float = 5.0
    lookahead_max_m: float = 18.0
    steer_gain: float = 1.1            # rad of heading error -> steer units
    throttle_gain: float = 0.45        # per m/s of speed error
    brake_gain: float = 0.35
    brake_deadband_mps: float = 0.6    # coast rather than brake inside this band


class BaseController:
    def __init__(self, lane: LaneReference,
                 gains: Optional[ControllerGains] = None):
        self.lane = lane
        self.g = gains or ControllerGains()
        # Last pure-pursuit target and its bearing, kept purely so a trace can
        # record them. Steering is only ever as good as this point, and without
        # it in the log a bad turn is indistinguishable from a bad controller.
        self.last_target: Optional[Vec3] = None
        self.last_bearing_deg: Optional[float] = None

    def _steer(self, ego: EgoState, lateral_offset: float) -> float:
        speed = ego.speed
        lookahead = _clamp(self.g.lookahead_gain * speed,
                           self.g.lookahead_min_m, self.g.lookahead_max_m)
        target = self.lane.point_ahead(ego.position, ego.yaw_deg, lookahead,
                                       lateral_offset)
        self.last_target = target
        s, d = relative_to_ego(ego, target)
        self.last_bearing_deg = math.degrees(math.atan2(d, max(s, 0.5)))
        # Pure pursuit: steer proportional to the bearing of the target point.
        # d > 0 is to our right, and CARLA's positive steer is right, so the
        # sign carries through unchanged.
        alpha = math.atan2(d, max(s, 0.5))
        return _clamp(self.g.steer_gain * alpha, -1.0, 1.0)

    def _speed(self, ego: EgoState, target_speed: float) -> Tuple[float, float]:
        err = target_speed - ego.speed
        if err > 0:
            return _clamp(self.g.throttle_gain * err, 0.0, 1.0), 0.0
        if err < -self.g.brake_deadband_mps:
            return 0.0, _clamp(-self.g.brake_gain * err, 0.0, 1.0)
        return 0.0, 0.0


class ConstantSpeedController(BaseController):
    """Lane-follow at a fixed speed. Drives the lead and the oncoming vehicle.

    These two get no evasive behaviour on purpose. CARLA's traffic manager has
    built-in collision avoidance, which would act as an oracle that quietly
    rescues the victim from the attack and hide the very outcome we measure.
    A car that simply holds its lane and speed is both simpler and more honest.
    """

    def __init__(self, lane: LaneReference, target_speed_mps: float,
                 lateral_offset: float = 0.0,
                 gains: Optional[ControllerGains] = None):
        super().__init__(lane, gains)
        self.target_speed = target_speed_mps
        self.lateral_offset = lateral_offset

    def step(self, ego: EgoState, dt: float = 0.05) -> VehicleCommand:
        throttle, brake = self._speed(ego, self.target_speed)
        return VehicleCommand(throttle, self._steer(ego, self.lateral_offset), brake)


# --------------------------------------------------------------------------- #
# The ego: an overtake state machine gated by the Do-Not-Pass Warning          #
# --------------------------------------------------------------------------- #
FOLLOW = "FOLLOW"
PASSING = "PASSING"
RETURNING = "RETURNING"


@dataclass
class OvertakeParams:
    cruise_speed_mps: float = 30.0 * KMH
    pass_speed_mps: float = 45.0 * KMH
    follow_gap_m: float = 20.0        # start gap-keeping on the lead here
    target_gap_m: float = 10.0        # steady-state following distance
    gap_gain: float = 0.5             # m/s of speed correction per metre of gap error
    pass_trigger_gap_m: float = 25.0  # close enough that overtaking is tempting
    lane_width_m: float = 3.5         # lateral offset used for the pass
    lateral_rate_mps: float = 1.2     # how fast the offset is ramped
    clear_of_lane_frac: float = 0.6   # fraction of the offset that counts as "out"
    return_clearance_m: float = 10.0  # get this far past the lead before returning
    abort_offset_m: float = 1.0       # past this much offset, the pass is committed
    pass_timeout_s: float = 20.0      # give up and tuck back in if the pass stalls


class DoNotPassController(BaseController):
    """Ego controller whose *only* reason to overtake is the DNPW decision.

    ``step`` takes the decision the receiver just computed from the messages it
    got. Nothing else can start a pass -- there is no timer, no waypoint script,
    no scripted steering input. That is what makes the experiment causal: swap
    the honest message stream for the spoofed one and the behaviour changes,
    with no other difference between the two runs.
    """

    def __init__(self, lane: LaneReference,
                 params: Optional[OvertakeParams] = None,
                 gains: Optional[ControllerGains] = None):
        super().__init__(lane, gains)
        self.p = params or OvertakeParams()
        self.state = FOLLOW
        self.lateral_offset = 0.0
        self.pass_started_at: Optional[float] = None
        # The vehicle being overtaken, latched when the pass starts. It must be
        # tracked by id rather than re-read from each decision: once the ego
        # swings into the opposing lane the lead is no longer inside the ego's
        # lane corridor, so the warning stops reporting it as the lead. Reading
        # that absence as "cleared it" makes the ego cut back in on top of the
        # very car it is passing.
        self.pass_lead_id: Optional[int] = None
        self.history: List[Tuple[float, str]] = []

    # -- helpers ----------------------------------------------------------- #
    def _lead(self, ego: EgoState, decision: DoNotPassDecision,
              objects: Sequence[PerceivedObject]
              ) -> Tuple[Optional[float], Optional[float]]:
        """(gap, speed) of the vehicle being followed or overtaken, if visible."""
        lead_id = self.pass_lead_id if self.pass_lead_id is not None \
            else decision.lead_object_id
        if lead_id is None:
            return None, None
        for o in objects:
            if o.object_id == lead_id:
                s, _ = relative_to_ego(ego, o.position)
                speed = math.sqrt(sum(c * c for c in o.velocity))
                return s, speed
        return None, None

    def _ramp(self, target_offset: float, dt: float) -> None:
        step = self.p.lateral_rate_mps * dt
        delta = target_offset - self.lateral_offset
        if abs(delta) <= step:
            self.lateral_offset = target_offset
        else:
            self.lateral_offset += math.copysign(step, delta)

    def _follow_speed(self, gap: Optional[float], lead_speed: Optional[float]) -> float:
        """Speed that keeps a sane gap on the lead, capped at cruise.

        Used in FOLLOW, and also during the first part of a pass while the ego is
        still inside its own lane -- without that, the ego accelerates to passing
        speed while still directly behind the lead and simply rear-ends it.
        """
        if gap is None or lead_speed is None or gap > self.p.follow_gap_m:
            return self.p.cruise_speed_mps
        target = lead_speed + self.p.gap_gain * (gap - self.p.target_gap_m)
        return _clamp(target, 0.0, self.p.cruise_speed_mps)

    def _enter(self, state: str, sim_time: float) -> None:
        if state != self.state:
            self.state = state
            self.history.append((sim_time, state))

    # -- the loop ---------------------------------------------------------- #
    def step(self, ego: EgoState, decision: DoNotPassDecision,
             objects: Sequence[PerceivedObject], dt: float = 0.05,
             sim_time: float = 0.0) -> VehicleCommand:
        gap, lead_speed = self._lead(ego, decision, objects)
        # The opposing lane is to the left, which is negative in CARLA's frame.
        pass_offset = -self.p.lane_width_m

        if self.state == FOLLOW:
            target_speed = self._follow_speed(gap, lead_speed)
            self._ramp(0.0, dt)
            # The one and only trigger for an overtake.
            if (decision.decision == PASS and decision.pass_intent
                    and gap is not None and gap <= self.p.pass_trigger_gap_m):
                self._enter(PASSING, sim_time)
                self.pass_started_at = sim_time
                self.pass_lead_id = decision.lead_object_id

        elif self.state == PASSING:
            target_speed = self.p.pass_speed_mps
            # A warning that arrives before we have committed still aborts the pass.
            if (decision.decision == DO_NOT_PASS
                    and abs(self.lateral_offset) < self.p.abort_offset_m):
                self._enter(FOLLOW, sim_time)          # warned before committing
                self.pass_lead_id = None
                self._ramp(0.0, dt)
                target_speed = self._follow_speed(gap, lead_speed)
            else:
                self._ramp(pass_offset, dt)
                # Only accelerate once genuinely clear of the lead's lane.
                if abs(self.lateral_offset) < self.p.clear_of_lane_frac * self.p.lane_width_m:
                    target_speed = self._follow_speed(gap, lead_speed)
                stalled = (self.pass_started_at is not None
                           and sim_time - self.pass_started_at > self.p.pass_timeout_s)
                # Return only on a *positive* clearance reading. A missing gap
                # means the lead is out of view, not that it has been passed.
                if (gap is not None and gap <= -self.p.return_clearance_m) or stalled:
                    self._enter(RETURNING, sim_time)

        else:  # RETURNING
            target_speed = self.p.cruise_speed_mps
            self._ramp(0.0, dt)
            if abs(self.lateral_offset) < 0.2:
                self._enter(FOLLOW, sim_time)
                self.pass_lead_id = None

        throttle, brake = self._speed(ego, target_speed)
        return VehicleCommand(throttle, self._steer(ego, self.lateral_offset), brake)


# --------------------------------------------------------------------------- #
# The ego: a left-turn state machine gated by the Left Turn Assist decision    #
# --------------------------------------------------------------------------- #
APPROACH = "APPROACH"
WAITING = "WAITING"
TURNING = "TURNING"
CLEARED = "CLEARED"


@dataclass
class LeftTurnParams:
    cruise_speed_mps: float = 30.0 * KMH
    turn_speed_mps: float = 18.0 * KMH     # you slow down to turn across traffic
    stop_decel_mps2: float = 2.0           # comfortable approach to the line
    creep_speed_mps: float = 1.0           # holding speed at the line
    hold_tolerance_m: float = 2.0          # "at the line" band
    # Once this far along the turn path the manoeuvre is committed: the ego is
    # out in the junction, where stopping is more dangerous than continuing.
    # Its real-world analogue is the point of no return every driver knows, and
    # it is what makes a late-arriving true warning useless -- the same emergent
    # detail the do-not-pass run shows when the ego finally sees the car it was
    # told was not there.
    commit_distance_m: float = 3.0
    # How close to the line the ego may be when it commits. It does NOT have to
    # have stopped: if the junction reads as clear on the approach, a real
    # driver flows through without ever halting, and requiring a full stop hid
    # that difference. Under attack the ego now simply never stops -- a sharper
    # demonstration than stopping and then going.
    #
    # Measured in the mock: 2.5-9 m all give the wanted split (honest stops and
    # waits, spoofed flows through). At 14 m it breaks -- from that far out the
    # junction still reads clear in BOTH runs, so the honest ego commits too and
    # then has to abort. 8 m sits in the middle of the working range.
    commit_within_m: float = 8.0


class LeftTurnController(BaseController):
    """Ego controller whose *only* reason to turn is the Left Turn Assist decision.

    Same contract as :class:`DoNotPassController`: no timer, no scripted steering,
    no waypoint schedule. The ego drives up to the line, holds while the warning
    says DO_NOT_TURN, and commits the moment it says TURN. Feed it honest
    messages and it waits for a real gap; feed it the forged ones and it turns
    into the path of a car it has been told is not there.

    This is deliberately *not* how the CarlaNetpp reference scene worked. There
    the turn was ``apply_control(throttle=1, steer=-0.15)`` on a ``sleep`` timer
    and happened identically with the attack switched off -- see
    ``causal-not-choreographed`` in the project notes.
    """

    # A junction turn is a much tighter path than a lane change, and the default
    # gains are tuned for the latter. Looking 18 m ahead on a ~20 m arc puts the
    # target point almost at the exit, so pure pursuit aims straight across the
    # junction and cuts the corner instead of following the curve. These gains
    # keep the target on the arc.
    TURN_GAINS = ControllerGains(lookahead_gain=1.0, lookahead_min_m=4.0,
                                 lookahead_max_m=9.0, steer_gain=1.3)

    def __init__(self, approach: LaneReference, turn_path: PolylineReference,
                 hold_point: Vec3,
                 params: Optional[LeftTurnParams] = None,
                 gains: Optional[ControllerGains] = None,
                 turn_path_factory=None,
                 exit_lane: Optional[LaneReference] = None):
        super().__init__(approach, gains or LeftTurnController.TURN_GAINS)
        self.approach_lane = approach
        self.turn_path = turn_path
        # Built from the ego's ACTUAL pose at the moment it commits, when given.
        # A path precomputed from the stop line does not start where the ego
        # ends up -- it halts within a tolerance band, a metre or two short and
        # a few centimetres off the centreline -- so the first pure-pursuit
        # target sits off to one side and the controller answers with full lock.
        # That instantaneous slam to the stop is what a heavy vehicle turns into
        # a visible lurch. Re-deriving the arc from the real pose makes the
        # first target lie straight ahead, and the turn begins smoothly.
        self.turn_path_factory = turn_path_factory
        # Where to steer once through the junction. The turn path ends; the road
        # does not, and extrapolating off the end of a polyline is not driving.
        self.exit_lane = exit_lane
        self.hold_point = hold_point
        self.p = params or LeftTurnParams()
        self.state = APPROACH
        self.turn_started_at: Optional[float] = None
        self.committed = False
        self.history: List[Tuple[float, str]] = []

    def _enter(self, state: str, sim_time: float) -> None:
        if state != self.state:
            self.state = state
            self.history.append((sim_time, state))

    def _distance_to_hold(self, ego: EgoState) -> float:
        """Signed distance to the stop line, positive while still short of it."""
        s, _ = relative_to_ego(ego, self.hold_point)
        return s

    def _approach_speed(self, ego: EgoState, stopping: bool) -> float:
        """Cruise, but bleed off smoothly so we arrive at the line at a crawl."""
        if not stopping:
            return self.p.cruise_speed_mps
        d = max(0.0, self._distance_to_hold(ego) - self.p.hold_tolerance_m)
        # v = sqrt(2 a d): the fastest speed from which we can still stop in d.
        allowed = math.sqrt(2.0 * self.p.stop_decel_mps2 * d)
        return _clamp(min(self.p.cruise_speed_mps, allowed),
                      0.0, self.p.cruise_speed_mps)

    def step(self, ego: EgoState, decision: LeftTurnDecision,
             objects: Sequence[PerceivedObject], dt: float = 0.05,
             sim_time: float = 0.0) -> VehicleCommand:
        clear = decision.decision == TURN and decision.turn_intent

        if self.state in (APPROACH, WAITING):
            at_line = self._distance_to_hold(ego) <= self.p.hold_tolerance_m
            if at_line and self.state == APPROACH:
                self._enter(WAITING, sim_time)
            # The turn may be commenced from anywhere inside `commit_within_m`
            # of the line -- not only from a standstill at it. A driver who can
            # see the junction is clear flows through; one who cannot stops and
            # waits. Both runs still make the decision over the same stretch of
            # road, so the comparison remains about beliefs rather than
            # geometry, but the honest and attacked behaviours now differ in
            # the way they should: waiting versus not even slowing.
            near_line = self._distance_to_hold(ego) <= self.p.commit_within_m
            if clear and (self.state == WAITING or near_line):
                if self.turn_path_factory is not None:
                    self.turn_path = self.turn_path_factory(ego)
                self._enter(TURNING, sim_time)
                self.turn_started_at = sim_time
                self.lane = self.turn_path
                target = self.p.turn_speed_mps
            else:
                # Hold at the line. Creep rather than fully stop while still
                # approaching, so the ego is moving when a gap appears.
                target = (0.0 if self.state == WAITING
                          else self._approach_speed(ego, stopping=True))
                self.lane = self.approach_lane

        elif self.state == TURNING:
            target = self.p.turn_speed_mps
            travelled = self.turn_path.progress(ego.position)
            if travelled >= self.p.commit_distance_m:
                self.committed = True
            # A warning that arrives before we are committed still aborts.
            if decision.decision == DO_NOT_TURN and not self.committed:
                self._enter(WAITING, sim_time)
                self.turn_started_at = None
                self.lane = self.approach_lane
                target = 0.0
            elif self.turn_path.remaining(ego.position) <= 0.5:
                self._enter(CLEARED, sim_time)

        else:  # CLEARED -- through the junction, drive out along the exit road
            target = self.p.cruise_speed_mps
            self.lane = self.exit_lane if self.exit_lane is not None else self.turn_path

        throttle, brake = self._speed(ego, target)
        # A commanded zero really means stop, not coast: without this the ego
        # rolls through the line on residual speed while "waiting".
        if target <= 1e-6 and ego.speed > 0.1:
            throttle, brake = 0.0, 1.0
        return VehicleCommand(throttle, self._steer(ego, 0.0), brake)


# --------------------------------------------------------------------------- #
# Mock plant: a kinematic bicycle, so the loop closes without CARLA            #
# --------------------------------------------------------------------------- #
@dataclass
class KinematicVehicle:
    """Minimal bicycle model -- enough to close the loop in mock mode and tests."""

    object_id: int
    position: Vec3 = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0
    speed: float = 0.0
    classification: str = "car"
    dimensions: Vec3 = (4.6, 2.0, 1.5)
    wheelbase_m: float = 2.8
    max_steer_deg: float = 35.0
    max_accel_mps2: float = 3.0
    max_decel_mps2: float = 6.0
    drag: float = 0.02

    def state(self, station_id: Optional[int] = None) -> EgoState:
        fx, fy = heading_unit(self.yaw_deg)
        return EgoState(station_id=self.object_id if station_id is None else station_id,
                        position=self.position, yaw_deg=self.yaw_deg,
                        velocity=(fx * self.speed, fy * self.speed, 0.0))

    def step(self, cmd: VehicleCommand, dt: float) -> None:
        accel = (cmd.throttle * self.max_accel_mps2
                 - cmd.brake * self.max_decel_mps2
                 - self.drag * self.speed)
        self.speed = max(0.0, self.speed + accel * dt)
        steer_rad = math.radians(_clamp(cmd.steer, -1.0, 1.0) * self.max_steer_deg)
        yaw_rate = (self.speed / self.wheelbase_m) * math.tan(steer_rad)
        self.yaw_deg += math.degrees(yaw_rate) * dt
        fx, fy = heading_unit(self.yaw_deg)
        self.position = (self.position[0] + fx * self.speed * dt,
                         self.position[1] + fy * self.speed * dt,
                         self.position[2])
