"""Place the CarlaAir / AirSim drone at a pose in the CARLA world frame.

CarlaAir's drone is an AirSim multirotor that *also* shows up as a CARLA actor,
so it has two poses that have to be reconciled:

* AirSim flies it in its own **NED** frame (Z points down, origin wherever AirSim
  put it),
* CARLA reports it in the world frame (Z points up).

The constant offset between the two is measured once by comparing the same drone
in both frames -- the approach CarlaAir itself uses in
``examples/air_ground_sync.py::calibrate_offset``.

Why a scenario needs this at all: the container entrypoint takes the drone off to
a ~3 m hover **once**, at startup. Loading a different map re-creates the drone on
the ground at that map's default spot, so any scenario that switches maps has to
take off again and put the drone where the scene actually is.

Placement is a **teleport, not a flight**. A drone seen cruising across town to
its post is a distraction in a recording -- the attacker is supposed to be
*already* hovering roadside when the scene starts. ``simSetVehiclePose`` moves it
with no transit; flying is kept only as a fallback for when the teleport does not
take, since AirSim's physics can reject a pose change mid-flight.

Everything here is best effort. The drone's physical pose is *cosmetic* for the
message-level attack -- a forged CPM claims the impersonated station's position,
not the attacker's -- so a drone that refuses to move must never fail a run.
Every entry point returns a status string and swallows its errors.
"""
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]

DEFAULT_AIRSIM_PORT = 41451
ARRIVAL_TOLERANCE_M = 4.0
SETTLE_S = 2.0

# The frame offset is measured ONCE per process and reused.
#
# It has to be, because the drone's CARLA actor does not track where AirSim
# actually flies it -- the actor keeps reporting its spawn pose. Re-deriving the
# offset after the drone has moved therefore subtracts a current AirSim position
# from a stale CARLA one, and the result is wrong by exactly the distance the
# drone has travelled. That is what put it 113.8 m off on a second run.
_OFFSET_CACHE: dict = {}


def reset_offset_cache() -> None:
    """Forget the cached frame offset (e.g. after a world reload)."""
    _OFFSET_CACHE.clear()


def carla_to_airsim_offset(world, client, key: str = "default") -> Vec3:
    """(ox, oy, oz) with ``airsim = (cx + ox, cy + oy, -cz + oz)``.

    Mirrors CarlaAir's own calibration: read the drone's pose in both frames and
    take the difference. Only valid while the drone is still where it was
    created, so the first result is cached for the rest of the process -- see
    ``_OFFSET_CACHE``.
    """
    if key in _OFFSET_CACHE:
        return _OFFSET_CACHE[key]
    offset = (0.0, 0.0, 0.0)
    for actor in world.get_actors():
        if "drone" in actor.type_id.lower():
            cl = actor.get_location()
            ap = client.getMultirotorState().kinematics_estimated.position
            offset = (ap.x_val - cl.x, ap.y_val - cl.y, ap.z_val - (-cl.z))
            break
    _OFFSET_CACHE[key] = offset
    return offset


def _distance_to(client, target_ned: Vec3) -> float:
    p = client.getMultirotorState().kinematics_estimated.position
    return math.dist((p.x_val, p.y_val, p.z_val), target_ned)


def place_at(world, host: str, target: Vec3, yaw_deg: float = 0.0,
             port: int = DEFAULT_AIRSIM_PORT, timeout_s: float = 30.0) -> str:
    """Put the drone AT ``target`` (CARLA world coords), without a visible transit.

    Returns a human-readable status for the run log. Never raises.

    Call this *before* putting the world into synchronous mode: AirSim's flight
    controller needs the simulation stepping freely to settle into its hover.
    """
    try:
        import airsim
        client = airsim.MultirotorClient(ip=host, port=port)
        client.confirmConnection()
    except Exception as exc:                       # noqa: BLE001 - cosmetic only
        return f"drone not placed: cannot reach AirSim at {host}:{port} ({exc})"

    pose_str = tuple(round(v, 1) for v in target)
    try:
        client.enableApiControl(True)
        client.armDisarm(True)

        # Sample the frame offset while the drone is still where AirSim put it
        # (cached, so a second run does not re-derive it from a stale pose).
        ox, oy, oz = carla_to_airsim_offset(world, client, key=f"{host}:{port}")
        target_ned = (target[0] + ox, target[1] + oy, -target[2] + oz)

        # Teleport. Only yaw is applied: a multirotor levels itself out, so
        # forcing pitch/roll just makes the controller fight the pose.
        # How far it has to go. Zero means the configured spawn pose already put
        # it in the right place and nothing visible happens here at all.
        moved = _distance_to(client, target_ned)

        pose = airsim.Pose(airsim.Vector3r(*target_ned),
                           airsim.to_quaternion(0.0, 0.0, math.radians(yaw_deg)))
        client.simSetVehiclePose(pose, True)
        time.sleep(SETTLE_S)

        # Hold station. Without this SimpleFlight drops the drone out of the sky
        # the moment physics resumes.
        client.moveToPositionAsync(*target_ned, 10.0)
        deadline = time.time() + timeout_s
        gap = _distance_to(client, target_ned)
        while gap > ARRIVAL_TOLERANCE_M and time.time() < deadline:
            time.sleep(0.5)
            gap = _distance_to(client, target_ned)
        client.hoverAsync()

        # The offset is reported because it is what the AirSim settings.json
        # spawn pose has to be expressed in: if it is ~zero the configured spawn
        # lands the drone correctly with no teleport at all.
        off_str = f"offset=({ox:.1f}, {oy:.1f}, {oz:.1f})"
        if gap <= ARRIVAL_TOLERANCE_M:
            if moved <= ARRIVAL_TOLERANCE_M:
                return f"drone already at {pose_str}, hovering [{off_str}]"
            return (f"drone placed at {pose_str}, hovering "
                    f"(moved {moved:.1f} m) [{off_str}]")
        return (f"drone teleport to {pose_str} did not hold "
                f"({gap:.1f} m off); hovering where it settled [{off_str}]")
    except Exception as exc:                       # noqa: BLE001 - cosmetic only
        return f"drone not placed: {exc}"
