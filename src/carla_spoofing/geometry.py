"""Shared planar geometry and the receiver's own-pose type.

Extracted from ``do_not_pass_warning`` when a second receiver application (
``left_turn_assist``) needed the same primitives. Both are *sibling* V2X safety
applications, so neither should have to import the other just to get a dot
product; this module is the neutral ground they share.

``do_not_pass_warning`` re-exports everything here, so existing imports from
that module keep working unchanged.

Frame conventions (CARLA): X forward, Y **right**, Z up, yaw about Z in degrees.
CARLA's world is left-handed, which is why ``ego_frame`` builds "right" by
rotating the forward vector by +90 deg rather than -90.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]

# Classes that count as a vehicle for right-of-way reasoning. Pedestrians are a
# hazard for other warnings (VRU alerts), not for "may I use that piece of road".
VEHICLE_CLASSES = frozenset({"car", "truck", "bus", "bicycle", "motorcycle"})


@dataclass
class EgoState:
    """The receiver's own pose -- what a V2X safety application knows about itself."""

    station_id: int
    position: Vec3
    yaw_deg: float = 0.0
    velocity: Vec3 = (0.0, 0.0, 0.0)
    # Speed the ego *wants* to travel at. Manoeuvre intent is judged against
    # this, not against the current speed: a car already queued behind a slow
    # lead (or stopped at a junction waiting for a gap) is travelling slowly,
    # and comparing the two would conclude it has no reason to manoeuvre --
    # exactly backwards. None falls back to the current speed, which is right
    # for a one-off evaluation with no controller attached.
    desired_speed_mps: Optional[float] = None

    @property
    def speed(self) -> float:
        return math.sqrt(sum(c * c for c in self.velocity))

    @property
    def reference_speed(self) -> float:
        return self.speed if self.desired_speed_mps is None else self.desired_speed_mps


def heading_unit(yaw_deg: float) -> Tuple[float, float]:
    """Planar forward unit vector for a yaw, matching CARLA's forward vector."""
    r = math.radians(yaw_deg)
    return (math.cos(r), math.sin(r))


def ego_frame(yaw_deg: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """(forward, right) unit vectors for a heading."""
    fx, fy = heading_unit(yaw_deg)
    return (fx, fy), (-fy, fx)


def relative_to_ego(ego: EgoState, position: Vec3) -> Tuple[float, float]:
    """(longitudinal, lateral) offset of ``position`` in the ego's frame."""
    fwd, right = ego_frame(ego.yaw_deg)
    dx = position[0] - ego.position[0]
    dy = position[1] - ego.position[1]
    return dx * fwd[0] + dy * fwd[1], dx * right[0] + dy * right[1]


def along(vec: Vec3, unit: Tuple[float, float]) -> float:
    """Component of a planar vector along a unit direction."""
    return vec[0] * unit[0] + vec[1] * unit[1]


def speed_of(velocity: Vec3) -> float:
    return math.sqrt(sum(c * c for c in velocity))
