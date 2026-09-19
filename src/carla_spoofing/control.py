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
from .v2x.cpm import PerceivedObject
from .vru_warning import GO, STOP, VRUCrossingDecision

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
        nxt = wp.next(max(0.5, distance))
        target_wp = nxt[0] if nxt else wp
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

    def _steer(self, ego: EgoState, lateral_offset: float) -> float:
        speed = ego.speed
        lookahead = _clamp(self.g.lookahead_gain * speed,
                           self.g.lookahead_min_m, self.g.lookahead_max_m)
        target = self.lane.point_ahead(ego.position, ego.yaw_deg, lookahead,
                                       lateral_offset)
        s, d = relative_to_ego(ego, target)
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


# --------------------------------------------------------------------------- #
# The ego at a blind intersection, gated by the VRU Crossing Warning           #
# --------------------------------------------------------------------------- #
APPROACH = "APPROACH"
STOPPED = "STOPPED"
CROSSING = "CROSSING"


@dataclass
class CrossingParams:
    cruise_speed_mps: float = 30.0 * KMH
    stop_distance_m: float = 3.0      # how far short of the conflict point to halt
    brake_lead_m: float = 20.0        # start shedding speed this far from the stop line
    creep_speed_mps: float = 4.0 * KMH  # below this, close enough to latch a full stop


class IntersectionApproachController(BaseController):
    """Ego controller whose *only* reason to enter the intersection is the VRU
    Crossing Warning. Mirrors :class:`DoNotPassController`: no timer or scripted
    input decides whether the ego crosses, only the decision computed from
    received messages -- so swapping the honest message stream for the spoofed
    one is the only thing that can change what happens next.

    ``conflict_point_m`` is the distance, measured along the ego's heading at
    the moment this controller is built, from the ego's start to the point
    where the VRUs' path crosses the ego's lane.
    """

    def __init__(self, lane: LaneReference, conflict_point_m: float,
                 params: Optional[CrossingParams] = None,
                 gains: Optional[ControllerGains] = None):
        super().__init__(lane, gains)
        self.p = params or CrossingParams()
        self.conflict_point_m = conflict_point_m
        self._origin: Optional[Vec3] = None
        self._origin_fwd: Optional[Tuple[float, float]] = None
        self.state = APPROACH
        self.stopped_at: Optional[float] = None
        self.history: List[Tuple[float, str]] = []
        # Latched once braking starts inside brake_lead_m for a STOP decision;
        # a single noisy GO reading (e.g. a VRU's reported velocity jittering
        # for one tick) must not undo it, or the ego re-accelerates right where
        # it needs to be slowing down, with no runway left to recover before
        # committing to CROSSING. Only a full stop or crossing the stop line
        # clears it.
        self._braking_committed = False

    def _enter(self, state: str, sim_time: float) -> None:
        if state != self.state:
            self.state = state
            self.history.append((sim_time, state))

    def distance_to_stop_line(self, ego: EgoState) -> float:
        """Metres still to travel before the stop line (negative once past it)."""
        if self._origin is None:
            self._origin = ego.position
            self._origin_fwd = heading_unit(ego.yaw_deg)
        dx = ego.position[0] - self._origin[0]
        dy = ego.position[1] - self._origin[1]
        travelled = dx * self._origin_fwd[0] + dy * self._origin_fwd[1]
        return (self.conflict_point_m - self.p.stop_distance_m) - travelled

    def step(self, ego: EgoState, decision: VRUCrossingDecision, dt: float = 0.05,
             sim_time: float = 0.0) -> VehicleCommand:
        gap = self.distance_to_stop_line(ego)

        if self.state == CROSSING or gap <= 0.0:
            # Past the point of no return: committed, no scripted braking
            # mid-intersection -- a real ADAS can't undo entering the crossing.
            self._enter(CROSSING, sim_time)
            target_speed = self.p.cruise_speed_mps
        elif self.state == STOPPED:
            if decision.decision == STOP:
                return VehicleCommand(0.0, self._steer(ego, 0.0), 1.0)
            # Only a stopped ego, actually re-evaluating whether to depart,
            # may release the brake on a GO.
            self._braking_committed = False
            self._enter(APPROACH, sim_time)
            target_speed = self.p.cruise_speed_mps
        elif gap <= self.p.brake_lead_m and (decision.decision == STOP
                                             or self._braking_committed):
            self._braking_committed = True
            if ego.speed <= self.p.creep_speed_mps:
                # Already at rest in the braking zone: latch the brake fully
                # rather than routing zero through the proportional speed
                # controller below, whose coast deadband would otherwise let
                # the last bit of residual speed idle the car through the
                # line -- a real ADAS holds the brake, it doesn't drift
                # forward at walking pace while it waits for the road to clear.
                self._enter(STOPPED, sim_time)
                if self.stopped_at is None:
                    self.stopped_at = sim_time
                return VehicleCommand(0.0, self._steer(ego, 0.0), 1.0)
            # In the braking zone: ramp down proportionally to the remaining gap.
            target_speed = _clamp(gap / max(self.p.brake_lead_m, 1e-3)
                                  * self.p.cruise_speed_mps,
                                  0.0, self.p.cruise_speed_mps)
            self._enter(APPROACH, sim_time)
        else:
            # Still well short of the line, or genuinely never told to brake
            # inside the zone: keep approaching at cruise speed.
            target_speed = self.p.cruise_speed_mps
            self._enter(APPROACH, sim_time)

        throttle, brake = self._speed(ego, target_speed)
        return VehicleCommand(throttle, self._steer(ego, 0.0), brake)
