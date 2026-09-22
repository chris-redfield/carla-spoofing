"""A minimal stand-in for the slice of the CARLA API the scenarios use.

Why this exists: every failure in the left-turn scenario so far has been in the
``--mode carla`` path, and the test suite could not see any of them because that
path needs a simulator. Three of them were trivial -- a missing import, a call to
a renamed function, a ``None`` compared with an int -- and each cost a full
round-trip through the GPU container to discover.

This fakes just enough road graph to exercise scene derivation: a square
crossroads with four arms, spawn points on them, waypoints that walk forwards
and backwards, and a junction that reports its movements the way CARLA does
(one ``(entry, exit)`` pair per turning movement, with unnormalised yaws).

It is deliberately NOT a simulator. It proves the geometry code runs, picks the
right arms and produces a coherent scene; it says nothing about whether a car
drives well, which only the real thing can.
"""
from __future__ import annotations

import math


def _unit(yaw_deg):
    r = math.radians(yaw_deg)
    return math.cos(r), math.sin(r)


class Location:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class Rotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location or Location()
        self.rotation = rotation or Rotation()


class LaneType:
    Driving = "Driving"


class CityObjectLabel:
    Buildings = "Buildings"


class TrafficLightState:
    Green = "Green"


class VehicleControl:
    def __init__(self, throttle=0.0, steer=0.0, brake=0.0):
        self.throttle, self.steer, self.brake = throttle, steer, brake


class Waypoint:
    """A point on a straight arm of the crossroads.

    ``s`` is distance from the junction centre, positive meaning "still
    approaching". Walking forward decreases it; at s <= 0 we are in the junction.
    """

    def __init__(self, world, arm_yaw, s, lane_id=1, road_id=None):
        self.world = world
        self.arm_yaw = arm_yaw
        self.s = s
        self.lane_id = lane_id
        self.road_id = road_id if road_id is not None else int(arm_yaw) % 360

    @property
    def is_junction(self):
        return self.s <= 0.0

    @property
    def transform(self):
        fx, fy = _unit(self.arm_yaw)
        # s metres back along the arm from the junction centre.
        return Transform(Location(-fx * self.s, -fy * self.s, 0.0),
                         Rotation(yaw=self.arm_yaw))

    def next(self, d):
        nxt = Waypoint(self.world, self.arm_yaw, self.s - d, self.lane_id)
        if nxt.s > 0:
            return [nxt]
        # Inside the junction: every exit becomes reachable, plus carrying on.
        return [Waypoint(self.world, y, nxt.s, self.lane_id)
                for y in self.world.arm_yaws]

    def previous(self, d):
        back = Waypoint(self.world, self.arm_yaw, self.s + d, self.lane_id)
        # Offer a tempting wrong branch first, as CARLA may well do.
        wrong = Waypoint(self.world, self.arm_yaw + 90.0, self.s + d,
                         self.lane_id)
        return [wrong, back]

    def get_junction(self):
        return self.world.junction


class Junction:
    def __init__(self, world):
        self.world = world
        self.id = 42

    def get_waypoints(self, _lane_type):
        """One (entry, exit) pair per turning movement, as CARLA does.

        Includes an unnormalised yaw, which a live survey really did return.
        """
        pairs = []
        for entry_yaw in self.world.arm_yaws:
            for turn in (-90.0, 0.0, 90.0):
                exit_yaw = entry_yaw + turn
                ey = entry_yaw - 360.0 if turn == 90.0 else entry_yaw
                pairs.append((Waypoint(self.world, ey, 6.0),
                              Waypoint(self.world, exit_yaw, -6.0)))
        return pairs


class Map:
    def __init__(self, world):
        self.world = world
        self.name = "Carla/Maps/Town10HD"

    def get_spawn_points(self):
        return self.world.spawn_points

    def get_waypoint(self, location, project_to_road=True, lane_type=None):
        best, best_d = None, float("inf")
        for yaw in self.world.arm_yaws:
            fx, fy = _unit(yaw)
            s = -(location.x * fx + location.y * fy)
            if s <= 0:
                continue
            px, py = -fx * s, -fy * s
            d = math.dist((px, py), (location.x, location.y))
            if d < best_d:
                best, best_d = Waypoint(self.world, yaw, s), d
        return best


class World:
    """A square crossroads: arms at 0, 90, 180, 270 degrees."""

    def __init__(self, arm_yaws=(0.0, 90.0, 180.0, 270.0)):
        self.arm_yaws = list(arm_yaws)
        self.junction = Junction(self)
        self._map = Map(self)
        # Ego on the 90-degree arm (approaching heading +Y); hazard on the
        # 180-degree arm, which crosses from the ego's left.
        self.spawn_points = []
        for yaw, s in ((90.0, 16.0), (90.0, 42.0), (180.0, 50.0), (0.0, 30.0)):
            fx, fy = _unit(yaw)
            self.spawn_points.append(
                Transform(Location(-fx * s, -fy * s, 0.6), Rotation(yaw=yaw)))

    def get_map(self):
        return self._map

    def get_actors(self):
        return []

    def get_level_bbs(self, _label):
        return []


def make_module():
    """A module-like object with the ``carla.*`` names the scenario touches."""
    import types
    m = types.ModuleType("carla")
    for name in ("Location", "Rotation", "Transform", "LaneType",
                 "CityObjectLabel", "TrafficLightState", "VehicleControl"):
        setattr(m, name, globals()[name])
    return m


# --------------------------------------------------------------------------- #
# A straight two-way road, for the do-not-pass scenario                        #
# --------------------------------------------------------------------------- #
class LaneWaypoint:
    """A point on a straight two-way road running along +X.

    ``lane_id`` +1 travels +X, -1 travels -X, offset sideways in Y. Enough to
    exercise ``CarlaLaneReference``, whose whole job is refusing to follow the
    opposing lane when an overtake puts the car across the centre line.
    """

    LANE_WIDTH = 3.5

    def __init__(self, world, s, lane_id=1, branch=False):
        self.world = world
        self.s = s
        self.lane_id = lane_id
        self.road_id = 1
        self.lane_type = LaneType.Driving
        self.is_junction = False
        self._branch = branch

    @property
    def transform(self):
        yaw = 0.0 if self.lane_id > 0 else 180.0
        y = -self.LANE_WIDTH / 2 if self.lane_id > 0 else self.LANE_WIDTH / 2
        return Transform(Location(self.s, y, 0.0), Rotation(yaw=yaw))

    def _step(self, d, sign):
        ahead = LaneWaypoint(self.world, self.s + sign * d * (1 if self.lane_id > 0 else -1),
                             self.lane_id)
        if not self.world.branching:
            return [ahead]
        # A side road listed FIRST, as CARLA may well do.
        return [_BranchWaypoint(self.world, self.s, self.lane_id), ahead]

    def next(self, d):
        return self._step(d, +1)

    def previous(self, d):
        return self._step(d, -1)

    def get_left_lane(self):
        return LaneWaypoint(self.world, self.s, -self.lane_id)

    def get_right_lane(self):
        return None


class _BranchWaypoint(LaneWaypoint):
    @property
    def transform(self):
        return Transform(Location(self.s, 0.0, 0.0), Rotation(yaw=90.0))


class StraightRoadMap:
    def __init__(self, world):
        self.world = world
        self.name = "Carla/Maps/Town01"

    def get_waypoint(self, location, project_to_road=True, lane_type=None):
        # Snap to whichever lane is nearer in Y -- exactly the behaviour that
        # makes an overtaking car resolve to the opposing lane mid-manoeuvre.
        lane_id = 1 if location.y < 0 else -1
        return LaneWaypoint(self.world, location.x, lane_id)


class StraightRoadWorld:
    def __init__(self, branching=False):
        self.branching = branching
        self._map = StraightRoadMap(self)

    def get_map(self):
        return self._map

    def get_actors(self):
        return []
