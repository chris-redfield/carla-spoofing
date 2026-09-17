"""Do-Not-Pass Warning spoofing: a drone impersonating an RSU causes an unsafe overtake.

Whitepaper use case 1b (``White_paper___B5GCyberTestV2X.pdf``, Fig. 2): *"a
collision resulting from a spoofer drone sending false information masquerading
as a Road Side Unit (RSU)."*

The scene (two-lane road, one lane each way)
--------------------------------------------
* **lead**      -- a slow vehicle ahead of the ego, in the ego's lane. It is also
                   the occluder: the ego cannot see round it.
* **ego (Rx)**  -- follows the lead, wants to overtake, and asks its Do-Not-Pass
                   Warning whether the opposing lane is clear.
* **oncoming**  -- a vehicle approaching in the opposing lane. Real, and on a
                   collision course with anyone who pulls out.
* **RSU**       -- a static roadside station that sees the whole segment and
                   broadcasts a CPM of it. The ego depends on it, because its own
                   line of sight is blocked by the lead.
* **drone**     -- the attacker, hovering at the roadside.

The attack
----------
The drone broadcasts a CPM stamped with the **RSU's station id** and the RSU's
reference position, listing everything the RSU would report *except* the oncoming
vehicle. Because a receiver keeps one entry per station, the forgery replaces the
genuine RSU report rather than adding to it (see ``fusion``), so the hazard
vanishes from the ego's fused world model. Its warning flips DO_NOT_PASS -> PASS
and it overtakes into the oncoming vehicle.

Closed loop, not choreography
-----------------------------
The ego is driven by :class:`~carla_spoofing.control.DoNotPassController`, whose
only trigger for an overtake is the DNPW decision computed from received
messages. There is no timed steering input anywhere in this file. Run it with
``--run honest`` and the ego never pulls out; run ``--run spoofed`` and it does,
with nothing else changed between the two. That is the causal claim the earlier
scripted version of this experiment could not make.

Every round also records the *counterfactual* decision -- what the ego would have
concluded from the other message stream -- so a single run's CSV shows the flip
directly. Counterfactual messages are analysis only and are never transmitted.

Examples
--------
    # No simulator needed: kinematic mock of the same scene.
    carla-do-not-pass --mode mock --run both

    # Live sim, honest baseline then the attack. Start the sim however you
    # like (`make up`); this loads Town01 and clears stray traffic itself.
    carla-do-not-pass --mode carla --host carla-sim --run both
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..attacks import (
    ForgedIdentity, IdentitySpoofAttack, RemoveObjectAttack, RemoveTarget,
)
from ..control import (
    KMH, CarlaLaneReference, ConstantSpeedController, DoNotPassController,
    KinematicVehicle, OvertakeParams, StraightLaneReference, VehicleCommand,
)
from ..drone import place_at as place_drone_at
from ..do_not_pass_warning import (
    DO_NOT_PASS, PASS, DoNotPassDecision, DoNotPassThresholds, EgoState,
    evaluate_do_not_pass, heading_unit, relative_to_ego,
)
from ..fusion import fuse_latest_by_station, visible_objects
from ..perception import ObjectState, build_cpm_from_objects, carla_states_from_snapshot
from ..report import DecisionWriter, ReportWriter
from ..v2x.cpm import CollectivePerceptionMessage
from ..v2x.packet import FileSink, NullSink, Packet, UdpSink

Vec3 = Tuple[float, float, float]

# --------------------------------------------------------------------------- #
# Town01 scene geometry.                                                       #
#                                                                              #
# Map, weather and the drone's roadside hover pose are reused verbatim from the #
# earlier experiment this work continues:                                      #
#   ~/proj/CarlaNetpp/cooperative_perception_dev/use_case_1/main.py (Nov 2024)  #
# so the recorded scene is the same one. What is NOT reused is that script's    #
# timed steering input, which staged the collision -- see the module docstring. #
# --------------------------------------------------------------------------- #
MAP_NAME = "Town01"
WEATHER = "CloudyNoon"

DRONE_LOCATION: Vec3 = (392.791443, 107.608482, 14.855516)
DRONE_ROTATION = (-27.212101, -89.532944, -0.025725)      # pitch, yaw, roll
# The original pose faced the drone down one direction of the road. This scene
# plays out the other way, so the drone is turned to look along it. Kept as an
# offset rather than edited into DRONE_ROTATION, so the surveyed pose above stays
# verbatim from the CarlaNetpp run.
DRONE_VIEW_YAW_DEG = DRONE_ROTATION[1] + 180.0            # -89.53 -> 90.47
SPECTATOR_LOCATION: Vec3 = (377.332733, 125.954506, 35.793575)
SPECTATOR_ROTATION = (-54.568348, -0.546539, -0.018311)

# The RSU the drone impersonates: the same roadside spot at pole height, so the
# drone is literally hovering above the station whose identity it steals.
RSU_LOCATION: Vec3 = (392.791443, 107.608482, 6.0)
RSU_STATION_ID = 9001        # infrastructure ids live above the CARLA actor range
VIRTUAL_DRONE_STATION_ID = 9002   # used when no drone actor exists in the world

# Spawn-point indices from the original run. They positioned cars for a traffic
# manager to drive into formation over several seconds; a closed-loop run needs a
# defined starting formation instead, so placement is derived from the road graph
# around the same segment by default. --use-spawn-points forces the originals.
SPAWN_EGO, SPAWN_LEAD, SPAWN_ONCOMING = 181, 177, 163
ALT_SAME_LANE = (183, 219)
ALT_OPPOSING_LANE = (65,)

# Mock-backend ids (no CARLA actors involved).
MOCK_EGO_ID, MOCK_LEAD_ID, MOCK_ONCOMING_ID = 101, 102, 103

# Switching maps at runtime is a LAST RESORT in this build, not a normal step.
# The server has to tear down a level while a second client (the container's
# auto_traffic.py) still owns actors and the AirSim plugin sits in the same
# engine, and the call intermittently never returns -- observed completing twice
# and hanging twice on identical inputs. The sim therefore starts on MAP_NAME by
# default (see docker-compose.yml) so this path is normally never taken, and the
# attempt is time-boxed so a hang costs a bounded wait and a clear message
# instead of an indefinite freeze.
MAP_LOAD_TIMEOUT_S = 120.0
MAP_SETTLE_S = 5.0

HONEST, SPOOFED = "honest", "spoofed"


@dataclass
class ScenarioConfig:
    """Everything tunable about the scene, the messages and the manoeuvre."""

    run: str = SPOOFED                  # honest | spoofed
    duration_s: float = 60.0
    tick_s: float = 0.05                # control period; also the sim fixed delta
    message_rate_hz: float = 1.0        # CPM rate (ETSI CPS allows 1-10 Hz)

    # Perception ranges per station. The RSU deliberately outranges the ego:
    # seeing further than its receivers is the reason infrastructure is useful.
    rsu_range_m: float = 160.0
    drone_range_m: float = 160.0
    ego_range_m: float = 75.0

    # Initial formation, measured along the road from the RSU anchor.
    ego_back_m: float = 55.0
    lead_ahead_m: float = 30.0
    oncoming_ahead_m: float = 110.0
    lead_speed_kmh: float = 18.0
    oncoming_speed_kmh: float = 35.0

    use_spawn_points: bool = False
    clean_vehicles: bool = True
    out_dir: str = "out/do_not_pass"
    rsu_position: Vec3 = RSU_LOCATION
    drone_position: Vec3 = DRONE_LOCATION

    def message_period_ticks(self) -> int:
        return max(1, int(round((1.0 / self.message_rate_hz) / self.tick_s)))


# --------------------------------------------------------------------------- #
# Message construction and fusion (shared by both backends)                    #
# --------------------------------------------------------------------------- #
@dataclass
class RoundMessages:
    rsu: CollectivePerceptionMessage
    drone_honest: CollectivePerceptionMessage
    forged: CollectivePerceptionMessage
    ego: CollectivePerceptionMessage
    honest_objects: List
    spoofed_objects: List


def ego_self_cpm(cfg: ScenarioConfig, states: Sequence[ObjectState], ego_id: int,
                 gen_time: float) -> CollectivePerceptionMessage:
    """What the ego's own sensors see -- with the lead vehicle blocking the view.

    Without this occlusion step the scenario would be vacuous: an ego that can
    see the oncoming car itself has no reason to trust the RSU, and suppressing
    the RSU's report would change nothing. Being unable to see round the vehicle
    you intend to overtake is the whole premise of the use case.
    """
    ego = next(s for s in states if s.object_id == ego_id)
    cpm = build_cpm_from_objects(
        station_id=ego_id, reference_position=ego.position, objects=states,
        generation_time=gen_time, perception_range=cfg.ego_range_m,
        station_type="vehicle")
    cpm.perceived_objects = visible_objects(
        ego.position, cpm.perceived_objects, cpm.perceived_objects)
    return cpm


def build_round(cfg: ScenarioConfig, states: Sequence[ObjectState], ego_id: int,
                drone_station_id: int, gen_time: float,
                attack: IdentitySpoofAttack) -> RoundMessages:
    """Build this round's genuine and forged messages, and both fused views."""
    rsu = build_cpm_from_objects(
        station_id=RSU_STATION_ID, reference_position=cfg.rsu_position,
        objects=states, generation_time=gen_time,
        perception_range=cfg.rsu_range_m, station_type="rsu")
    drone_honest = build_cpm_from_objects(
        station_id=drone_station_id, reference_position=cfg.drone_position,
        objects=states, generation_time=gen_time,
        perception_range=cfg.drone_range_m, station_type="drone")
    forged = attack.apply_cpm(drone_honest)
    ego = ego_self_cpm(cfg, states, ego_id, gen_time)

    honest_view = fuse_latest_by_station([ego, rsu])
    # Arrival order matters: the forgery carries the RSU's station id and arrives
    # after the genuine report, so it supersedes rather than supplements it.
    spoofed_view = fuse_latest_by_station([ego, rsu, forged])
    return RoundMessages(rsu, drone_honest, forged, ego,
                         honest_view.objects, spoofed_view.objects)


def make_attack(oncoming_id: int, cfg: ScenarioConfig) -> IdentitySpoofAttack:
    """The drone: erase the oncoming vehicle, then sign it as the RSU."""
    return IdentitySpoofAttack(
        ForgedIdentity(station_id=RSU_STATION_ID,
                       reference_position=cfg.rsu_position,
                       station_type="rsu"),
        inner=RemoveObjectAttack(RemoveTarget(object_id=oncoming_id,
                                              carve_lidar=False)))


def _ego_state(states: Sequence[ObjectState], ego_id: int,
               desired_speed: Optional[float] = None) -> EgoState:
    s = next(o for o in states if o.object_id == ego_id)
    return EgoState(station_id=ego_id, position=s.position, yaw_deg=s.yaw_deg,
                    velocity=s.velocity, desired_speed_mps=desired_speed)


def truth_decision(states: Sequence[ObjectState], ego: EgoState,
                   gen_time: float) -> DoNotPassDecision:
    """The decision an *omniscient* receiver would make: no range limit, no
    occlusion, no messages. This is the yardstick the experiment is scored
    against -- a pass is unsafe when ground truth says DO_NOT_PASS, whatever the
    ego was told. It is analysis only and never transmitted.
    """
    omniscient = build_cpm_from_objects(
        station_id=ego.station_id, reference_position=ego.position,
        objects=states, generation_time=gen_time,
        perception_range=float("inf"), station_type="vehicle")
    return evaluate_do_not_pass(ego, omniscient.perceived_objects)


def _boxes_overlap(a: ObjectState, b: ObjectState) -> bool:
    """Crude oriented-box overlap in a's frame -- enough to flag a mock collision."""
    ego = EgoState(a.object_id, a.position, a.yaw_deg, a.velocity)
    s, d = relative_to_ego(ego, b.position)
    return (abs(s) <= 0.5 * (a.dimensions[0] + b.dimensions[0])
            and abs(d) <= 0.5 * (a.dimensions[1] + b.dimensions[1]))


# --------------------------------------------------------------------------- #
# Outcome book-keeping                                                         #
# --------------------------------------------------------------------------- #
@dataclass
class Outcome:
    run: str
    mode: str
    overtake_started_s: Optional[float] = None
    pass_committed_s: Optional[float] = None   # past the point of no return
    overtake_was_unsafe: bool = False       # ground truth said DO_NOT_PASS
    truth_at_overtake: str = ""
    max_lateral_offset_m: float = 0.0
    collision: Optional[dict] = None
    n_decisions: int = 0
    n_flips: int = 0
    first_flip_s: Optional[float] = None
    ego_state_changes: List = field(default_factory=list)
    attack: dict = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "run": self.run, "mode": self.mode,
            "overtake_started_s": self.overtake_started_s,
            "overtake_attempted": self.overtake_started_s is not None,
            "pass_committed_s": self.pass_committed_s,
            "overtake_committed": self.pass_committed_s is not None,
            # A pull-out the ego aborted when the next message arrived is a
            # success of the warning, not a failure -- counted, not scored.
            "aborted_passes": sum(
                1 for i in range(1, len(self.ego_state_changes))
                if self.ego_state_changes[i][1] == "FOLLOW"
                and self.ego_state_changes[i - 1][1] == "PASSING"),
            "overtake_was_unsafe": self.overtake_was_unsafe,
            "truth_at_overtake": self.truth_at_overtake,
            "max_lateral_offset_m": round(self.max_lateral_offset_m, 2),
            "collision": self.collision,
            "collided": self.collision is not None,
            "n_decisions": self.n_decisions, "n_flips": self.n_flips,
            "first_flip_s": self.first_flip_s,
            "ego_state_changes": self.ego_state_changes,
            "attack": self.attack, "notes": self.notes,
        }


def _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time, ego_id,
               ego_speed, ego_state_name, msgs: RoundMessages,
               acted: DoNotPassDecision, counter: DoNotPassDecision,
               drone_station_id: int, outcome: Outcome,
               truth: Optional[DoNotPassDecision] = None) -> None:
    hidden = sorted(set(msgs.rsu.object_ids()) - set(msgs.forged.object_ids()))
    dec_writer.add(run=cfg.run, frame=seq, sim_time=sim_time, ego_id=ego_id,
                   ego_speed=ego_speed, ego_state=ego_state_name,
                   acted=acted, counterfactual=counter, truth=truth,
                   hidden_object_id=hidden[0] if hidden else None)
    outcome.n_decisions += 1
    if acted.decision != counter.decision:
        outcome.n_flips += 1
        if outcome.first_flip_s is None:
            outcome.first_flip_s = round(sim_time, 2)

    # Genuine traffic is always on the air; the forgery only in the attack run.
    for cpm, sender, stype, atk in ((msgs.rsu, RSU_STATION_ID, "rsu", False),
                                    (msgs.ego, ego_id, "vehicle", False)):
        sink.send(Packet.cpm(cpm, seq=seq))
        msg_writer.add(frame=seq, sim_time=sim_time, sender_id=sender,
                       sender_type=stype, is_attacker=atk,
                       honest=cpm, broadcast=cpm)
    if cfg.run == SPOOFED:
        sink.send(Packet.cpm(msgs.forged, seq=seq))
        msg_writer.add(frame=seq, sim_time=sim_time, sender_id=drone_station_id,
                       sender_type="drone", is_attacker=True,
                       honest=msgs.drone_honest, broadcast=msgs.forged)


# --------------------------------------------------------------------------- #
# Mock backend: the same closed loop, on a kinematic straight road             #
# --------------------------------------------------------------------------- #
def run_mock(cfg: ScenarioConfig, sink, msg_writer, dec_writer) -> Outcome:
    outcome = Outcome(run=cfg.run, mode="mock")
    lane_y, opp_y = 0.0, -3.5
    cfg.rsu_position = (cfg.ego_back_m, -8.0, 6.0)
    cfg.drone_position = (cfg.ego_back_m, -10.0, 15.0)

    ego_v = KinematicVehicle(MOCK_EGO_ID, position=(0.0, lane_y, 0.0), yaw_deg=0.0,
                             speed=30.0 * KMH)
    lead_v = KinematicVehicle(MOCK_LEAD_ID, position=(cfg.lead_ahead_m, lane_y, 0.0),
                              yaw_deg=0.0, speed=cfg.lead_speed_kmh * KMH)
    onc_v = KinematicVehicle(MOCK_ONCOMING_ID,
                             position=(cfg.oncoming_ahead_m, opp_y, 0.0),
                             yaw_deg=180.0, speed=cfg.oncoming_speed_kmh * KMH)

    ego_ctrl = DoNotPassController(StraightLaneReference((0.0, lane_y, 0.0), 0.0))
    lead_ctrl = ConstantSpeedController(StraightLaneReference((0.0, lane_y, 0.0), 0.0),
                                        cfg.lead_speed_kmh * KMH)
    onc_ctrl = ConstantSpeedController(StraightLaneReference((0.0, opp_y, 0.0), 180.0),
                                       cfg.oncoming_speed_kmh * KMH)

    attack = make_attack(MOCK_ONCOMING_ID, cfg)
    # Safe default before any message has been received.
    decision = DoNotPassDecision(DO_NOT_PASS, "no cooperative message received yet")
    acted_objects: List = []
    period = cfg.message_period_ticks()
    n_ticks = int(cfg.duration_s / cfg.tick_s)
    seq = 0

    for tick in range(n_ticks):
        sim_time = tick * cfg.tick_s
        states = [
            ObjectState(v.object_id, v.position,
                        velocity=(heading_unit(v.yaw_deg)[0] * v.speed,
                                  heading_unit(v.yaw_deg)[1] * v.speed, 0.0),
                        yaw_deg=v.yaw_deg, dimensions=v.dimensions,
                        classification=v.classification)
            for v in (ego_v, lead_v, onc_v)
        ]
        if tick % period == 0:
            msgs = build_round(cfg, states, MOCK_EGO_ID, VIRTUAL_DRONE_STATION_ID,
                               sim_time, attack)
            ego_st = _ego_state(states, MOCK_EGO_ID, ego_ctrl.p.cruise_speed_mps)
            d_honest = evaluate_do_not_pass(ego_st, msgs.honest_objects)
            d_spoofed = evaluate_do_not_pass(ego_st, msgs.spoofed_objects)
            d_truth = truth_decision(states, ego_st, sim_time)
            if cfg.run == SPOOFED:
                decision, counter, acted_objects = d_spoofed, d_honest, msgs.spoofed_objects
            else:
                decision, counter, acted_objects = d_honest, d_spoofed, msgs.honest_objects
            _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time,
                       MOCK_EGO_ID, ego_v.speed, ego_ctrl.state, msgs,
                       decision, counter, VIRTUAL_DRONE_STATION_ID, outcome,
                       truth=d_truth)
            seq += 1

        ego_st = _ego_state(states, MOCK_EGO_ID, ego_ctrl.p.cruise_speed_mps)
        ego_v.step(ego_ctrl.step(ego_st, decision, acted_objects, cfg.tick_s, sim_time),
                   cfg.tick_s)
        lead_v.step(lead_ctrl.step(lead_v.state(), cfg.tick_s), cfg.tick_s)
        onc_v.step(onc_ctrl.step(onc_v.state(), cfg.tick_s), cfg.tick_s)

        outcome.max_lateral_offset_m = max(outcome.max_lateral_offset_m,
                                           abs(ego_ctrl.lateral_offset))
        if outcome.overtake_started_s is None and ego_ctrl.state == "PASSING":
            outcome.overtake_started_s = round(sim_time, 2)
        # Score the pass at the point of no return, not at the first twitch of
        # the wheel. Messages arrive at 1 Hz, so the ego can pull out on second-
        # old information and tuck straight back in when the next CPM corrects
        # it -- latching "unsafe" at PASSING entry would score that correct
        # abort as a full unsafe overtake.
        if (outcome.pass_committed_s is None and ego_ctrl.state == "PASSING"
                and abs(ego_ctrl.lateral_offset) >= ego_ctrl.p.abort_offset_m):
            outcome.pass_committed_s = round(sim_time, 2)
            t_now = truth_decision(states, ego_st, sim_time)
            outcome.truth_at_overtake = t_now.decision
            outcome.overtake_was_unsafe = t_now.decision == DO_NOT_PASS
        if outcome.collision is None:
            ego_os = states[0]
            for other in states[1:]:
                if _boxes_overlap(ego_os, other):
                    outcome.collision = {
                        "sim_time": round(sim_time, 2), "with_object_id": other.object_id,
                        "classification": other.classification,
                        "ego_speed_mps": round(ego_v.speed, 2),
                        "lateral_offset_m": round(ego_ctrl.lateral_offset, 2),
                    }
                    break
            if outcome.collision is not None:
                break

    outcome.ego_state_changes = [[round(t, 2), s] for t, s in ego_ctrl.history]
    outcome.attack = attack.result.to_dict()
    return outcome


# --------------------------------------------------------------------------- #
# CARLA backend                                                                #
# --------------------------------------------------------------------------- #
def _derive_transforms(carla, world, cfg):
    """Place the three cars off the road graph, anchored at the RSU/drone spot.

    Version-safe: unlike spawn-point indices, waypoints are derived from the map
    itself, so the formation survives a CARLA version bump.
    """
    cmap = world.get_map()
    anchor = carla.Location(x=cfg.rsu_position[0], y=cfg.rsu_position[1], z=0.5)
    wp = cmap.get_waypoint(anchor, project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    if wp is None:
        raise SystemExit(f"No drivable lane near {cfg.rsu_position} on {MAP_NAME}.")
    back = wp.previous(cfg.ego_back_m)
    ego_wp = back[0] if back else wp
    lead_list = ego_wp.next(cfg.lead_ahead_m)
    onc_same = ego_wp.next(cfg.oncoming_ahead_m)
    if not lead_list or not onc_same:
        raise SystemExit("Road segment too short for the configured formation; "
                         "lower --oncoming-ahead.")
    lead_wp = lead_list[0]
    opp = onc_same[0].get_left_lane()
    if opp is None or opp.lane_type != carla.LaneType.Driving:
        opp = onc_same[0].get_right_lane()
    if opp is None or opp.lane_type != carla.LaneType.Driving:
        raise SystemExit("No opposing lane next to the segment; this scenario "
                         "needs an undivided two-way road.")

    def lift(t):
        return carla.Transform(
            carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + 0.3),
            t.rotation)

    return (lift(ego_wp.transform), lift(lead_wp.transform), lift(opp.transform),
            "derived from the road graph around the original drone pose")


def _spawn_point_transforms(carla, world, cfg):
    points = world.get_map().get_spawn_points()
    idx = (SPAWN_EGO, SPAWN_LEAD, SPAWN_ONCOMING)
    if max(idx) >= len(points):
        raise SystemExit(f"--use-spawn-points: {MAP_NAME} in this build has only "
                         f"{len(points)} spawn points, need index {max(idx)}.")
    return (points[idx[0]], points[idx[1]], points[idx[2]],
            f"original CarlaNetpp spawn points {idx}")


def _ensure_map(client, map_name: str, rpc_timeout: float):
    """Put the simulator on ``map_name`` if it is not already there.

    Time-boxed: if the reload has not landed within ``MAP_LOAD_TIMEOUT_S`` we
    stop waiting and say how to avoid the reload altogether, rather than hanging.
    Returns (world, previous_map_name_or_None).
    """
    current = client.get_world().get_map().name.split("/")[-1]
    if current == map_name:
        return client.get_world(), None

    print(f"[dnp] simulator is on {current}, this scene needs {map_name}.")
    print(f"[dnp] reloading the map (up to {MAP_LOAD_TIMEOUT_S:.0f}s; the window "
          f"freezes while the level streams). This call is unreliable in this "
          f"build -- starting the sim with MAP={map_name} avoids it entirely.")

    deadline = time.monotonic() + MAP_LOAD_TIMEOUT_S
    client.set_timeout(MAP_LOAD_TIMEOUT_S * 0.6)
    world = None
    try:
        world = client.load_world(map_name)
    except RuntimeError as exc:
        print(f"[dnp] load_world did not return ({exc}); checking whether the "
              f"simulator got there anyway ...")
        while time.monotonic() < deadline:
            time.sleep(3.0)
            try:
                candidate = client.get_world()
                if candidate.get_map().name.split("/")[-1] == map_name:
                    world = candidate
                    break
            except RuntimeError:
                continue          # still unreachable; keep waiting
    if world is None:
        raise SystemExit(
            f"Map reload to {map_name} did not complete within "
            f"{MAP_LOAD_TIMEOUT_S:.0f}s. This is a known failure of this build, "
            f"not something the scenario can retry around.\n"
            f"Start the simulator on the right map instead:\n"
            f"    make down && MAP={map_name} make up\n"
            f"(the compose default is already {map_name}; you only need this if "
            f"you overrode it)")
    time.sleep(MAP_SETTLE_S)      # let the fresh level settle before spawning
    client.set_timeout(rpc_timeout)
    return world, current


def _find_drone(world):
    for a in world.get_actors():
        if "drone" in a.type_id.lower():
            return a
    return None


def run_carla(cfg: ScenarioConfig, sink, msg_writer, dec_writer, args) -> Outcome:
    try:
        import carla
    except ImportError as e:
        print("ERROR: `carla` module not importable. Run inside the CarlaAir "
              "conda env / Docker image, with the sim reachable.", file=sys.stderr)
        raise SystemExit(2) from e

    outcome = Outcome(run=cfg.run, mode="carla")
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    # The scenario prepares its own world, so it does not matter how the sim was
    # started: whatever map is up, we switch to the one this scene is defined on.
    world, previous = _ensure_map(client, MAP_NAME, args.timeout)
    if previous is not None:
        # A reload drops every actor, the AirSim drone included; the attacker
        # then falls back to a virtual station at the original hover pose.
        outcome.notes.append(f"loaded {MAP_NAME} (sim was on {previous})")
    world.set_weather(getattr(carla.WeatherParameters, WEATHER))

    existing = [a for a in world.get_actors().filter("vehicle.*")
                if "drone" not in a.type_id.lower()]
    if existing:
        if cfg.clean_vehicles:
            print(f"[dnp] removing {len(existing)} pre-existing vehicles ...")
            for a in existing:
                try:
                    a.destroy()
                except Exception:
                    pass
        else:
            outcome.notes.append(
                f"--keep-vehicles: {len(existing)} other vehicles left in the "
                "world; they may disturb the scene.")
            print("[dnp] WARNING: " + outcome.notes[-1])

    # Put the drone in the scene before synchronous mode: AirSim's controller
    # needs the sim stepping freely to actually fly there. Cosmetic for the
    # attack itself, but the whitepaper scene has the drone hovering roadside.
    if not args.no_fly_drone:
        status = place_drone_at(world, args.host, cfg.drone_position,
                                yaw_deg=DRONE_VIEW_YAW_DEG,
                                timeout_s=args.drone_timeout)
        print(f"[dnp] {status}")
        outcome.notes.append(status)

    original_settings = world.get_settings()
    spawned, collision_sensor = [], None
    collisions: List[dict] = []
    ego_ctrl = attack = None
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = cfg.tick_s
        world.apply_settings(settings)

        if cfg.use_spawn_points:
            ego_tf, lead_tf, onc_tf, placement = _spawn_point_transforms(carla, world, cfg)
        else:
            ego_tf, lead_tf, onc_tf, placement = _derive_transforms(carla, world, cfg)
        outcome.notes.append(f"placement: {placement}")
        print(f"[dnp] placement: {placement}")

        bl = world.get_blueprint_library()
        def bp(name, fallback="vehicle.*"):
            found = bl.filter(name)
            return found[0] if found else bl.filter(fallback)[0]

        # Same vehicle models as the original run, for a recognisable scene.
        ego = world.try_spawn_actor(bp("vehicle.tesla.cybertruck"), ego_tf)
        lead = world.try_spawn_actor(bp("vehicle.citroen.c3"), lead_tf)
        onc = world.try_spawn_actor(bp("vehicle.tesla.model3"), onc_tf)
        if not all((ego, lead, onc)):
            for a in (ego, lead, onc):
                if a:
                    a.destroy()
            raise SystemExit("Could not spawn all three vehicles (blocked spawn "
                             "points?). Try --clean-vehicles.")
        spawned = [ego, lead, onc]
        print(f"[dnp] ego={ego.id} lead={lead.id} oncoming={onc.id}")

        drone = _find_drone(world)
        if drone is not None:
            drone_station_id = drone.id
            print(f"[dnp] drone actor {drone.id} ({drone.type_id}) is the attacker")
        else:
            drone_station_id = VIRTUAL_DRONE_STATION_ID
            outcome.notes.append(
                "no drone actor in the world; the attacker is a virtual station "
                "at the original drone hover pose")
            print("[dnp] " + outcome.notes[-1])

        cs_bp = bl.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(cs_bp, carla.Transform(), attach_to=ego)
        collision_sensor.listen(lambda e: collisions.append({
            "with_actor_id": e.other_actor.id,
            "with_type_id": e.other_actor.type_id,
        }))

        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(*SPECTATOR_LOCATION),
            carla.Rotation(*SPECTATOR_ROTATION)))

        # In synchronous mode a snapshot reflects the LAST tick, so freshly
        # spawned actors are absent until the world steps. Without this the very
        # first iteration cannot find the ego and the whole run aborts.
        for _ in range(2):
            world.tick()

        lane = CarlaLaneReference(world.get_map())
        ego_ctrl = DoNotPassController(lane)
        lead_ctrl = ConstantSpeedController(lane, cfg.lead_speed_kmh * KMH)
        onc_ctrl = ConstantSpeedController(lane, cfg.oncoming_speed_kmh * KMH)
        attack = make_attack(onc.id, cfg)

        # Start the scene at speed. Spawned actors are at rest, and from a
        # standstill the ego is still accelerating when the oncoming vehicle
        # arrives -- the encounter is over before the formation the scenario
        # describes ever exists.
        for actor, speed in ((ego, ego_ctrl.p.cruise_speed_mps),
                             (lead, cfg.lead_speed_kmh * KMH),
                             (onc, cfg.oncoming_speed_kmh * KMH)):
            fwd = actor.get_transform().get_forward_vector()
            actor.set_target_velocity(
                carla.Vector3D(fwd.x * speed, fwd.y * speed, 0.0))
        world.tick()

        decision = DoNotPassDecision(DO_NOT_PASS, "no cooperative message received yet")
        acted_objects: List = []
        period = cfg.message_period_ticks()
        n_ticks = int(cfg.duration_s / cfg.tick_s)
        seq = 0

        wall_deadline = time.monotonic() + args.wall_timeout
        # CARLA's elapsed_seconds never resets, so a second run would otherwise
        # start its log at ~60 s. Every time recorded here is relative to the
        # first tick of THIS run.
        clock_origin = None
        # Set once a collision happens, so the loop keeps stepping for a moment
        # instead of cutting the instant the cars touch -- otherwise the crash is
        # the last frame anyone sees.
        stop_at: Optional[float] = None
        for tick in range(n_ticks):
            if time.monotonic() > wall_deadline:
                outcome.notes.append(
                    f"wall-clock budget of {args.wall_timeout:.0f}s exhausted "
                    f"after {tick} of {n_ticks} ticks")
                print("[dnp] " + outcome.notes[-1])
                break
            snap = world.get_snapshot()
            if clock_origin is None:
                clock_origin = snap.timestamp.elapsed_seconds
            sim_time = snap.timestamp.elapsed_seconds - clock_origin
            states = carla_states_from_snapshot(world, snap)
            if not any(s.object_id == ego.id for s in states):
                outcome.notes.append("ego actor vanished from the snapshot")
                break

            if tick % period == 0:
                msgs = build_round(cfg, states, ego.id, drone_station_id,
                                   sim_time, attack)
                ego_st = _ego_state(states, ego.id, ego_ctrl.p.cruise_speed_mps)
                d_honest = evaluate_do_not_pass(ego_st, msgs.honest_objects)
                d_spoofed = evaluate_do_not_pass(ego_st, msgs.spoofed_objects)
                d_truth = truth_decision(states, ego_st, sim_time)
                if cfg.run == SPOOFED:
                    decision, counter = d_spoofed, d_honest
                    acted_objects = msgs.spoofed_objects
                else:
                    decision, counter = d_honest, d_spoofed
                    acted_objects = msgs.honest_objects
                _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time,
                           ego.id, ego_st.speed, ego_ctrl.state, msgs,
                           decision, counter, drone_station_id, outcome,
                           truth=d_truth)
                seq += 1

            ego_st = _ego_state(states, ego.id, ego_ctrl.p.cruise_speed_mps)
            ego.apply_control(
                ego_ctrl.step(ego_st, decision, acted_objects,
                              cfg.tick_s, sim_time).to_carla())
            for actor, ctrl in ((lead, lead_ctrl), (onc, onc_ctrl)):
                st = next((s for s in states if s.object_id == actor.id), None)
                if st is None:
                    continue
                actor.apply_control(ctrl.step(
                    EgoState(actor.id, st.position, st.yaw_deg, st.velocity),
                    cfg.tick_s).to_carla())

            outcome.max_lateral_offset_m = max(outcome.max_lateral_offset_m,
                                               abs(ego_ctrl.lateral_offset))
            if outcome.overtake_started_s is None and ego_ctrl.state == "PASSING":
                outcome.overtake_started_s = round(sim_time, 2)
            if (outcome.pass_committed_s is None and ego_ctrl.state == "PASSING"
                    and abs(ego_ctrl.lateral_offset) >= ego_ctrl.p.abort_offset_m):
                outcome.pass_committed_s = round(sim_time, 2)
                t_now = truth_decision(states, ego_st, sim_time)
                outcome.truth_at_overtake = t_now.decision
                outcome.overtake_was_unsafe = t_now.decision == DO_NOT_PASS

            world.tick()

            if collisions and outcome.collision is None:
                ev = collisions[0]
                outcome.collision = {
                    "sim_time": round(sim_time, 2),
                    "with_object_id": ev["with_actor_id"],
                    "classification": ev["with_type_id"],
                    "ego_speed_mps": round(ego_st.speed, 2),
                    "lateral_offset_m": round(ego_ctrl.lateral_offset, 2),
                }
                print(f"[dnp] collision at {sim_time:.2f}s with "
                      f"{ev['with_type_id']} ({ev['with_actor_id']})")
                stop_at = sim_time + args.crash_hold
            if stop_at is not None and sim_time >= stop_at:
                break

    except KeyboardInterrupt:
        # Ctrl-C during a blocking RPC only lands once that call returns, so say
        # something immediately and fall through to cleanup rather than dying
        # mid-run and leaving actors and synchronous mode behind.
        outcome.notes.append("interrupted by the user")
        print("\n[dnp] interrupted — cleaning up ...")
    except RuntimeError as exc:
        # A wedged or stopped simulator surfaces as an RPC timeout. Bail out on
        # the first one instead of re-timing-out once per remaining tick.
        outcome.notes.append(f"simulator stopped responding: {exc}")
        print("[dnp] " + outcome.notes[-1])
    finally:
        if ego_ctrl is not None:
            outcome.ego_state_changes = [[round(t, 2), s] for t, s in ego_ctrl.history]
        if attack is not None:
            outcome.attack = attack.result.to_dict()
        # Cleanup must not inherit the run's deadline: against a dead server each
        # destroy would otherwise block for the full RPC timeout, several times over.
        try:
            client.set_timeout(3.0)
        except Exception:
            pass
        if collision_sensor is not None:
            try:
                collision_sensor.stop()
                collision_sensor.destroy()
            except Exception:
                pass
        for a in spawned:
            try:
                a.destroy()
            except Exception:
                pass
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass
    return outcome


# --------------------------------------------------------------------------- #
# CLI                                                                          #
# --------------------------------------------------------------------------- #
def _make_sink(args, out_dir):
    if args.sink == "file":
        return FileSink(os.path.join(out_dir, "v2x_packets.jsonl"))
    if args.sink == "udp":
        return UdpSink(args.udp_host, args.udp_port)
    return NullSink()


def _one_run(args, run_name: str) -> dict:
    cfg = ScenarioConfig(
        run=run_name, duration_s=args.duration, tick_s=args.tick,
        message_rate_hz=args.rate, ego_back_m=args.ego_back,
        lead_ahead_m=args.lead_ahead, oncoming_ahead_m=args.oncoming_ahead,
        lead_speed_kmh=args.lead_speed, oncoming_speed_kmh=args.oncoming_speed,
        use_spawn_points=args.use_spawn_points,
        clean_vehicles=not args.keep_vehicles,
        out_dir=os.path.join(args.out, run_name))
    os.makedirs(cfg.out_dir, exist_ok=True)
    sink = _make_sink(args, cfg.out_dir)
    msg_writer = ReportWriter(cfg.out_dir)
    dec_writer = DecisionWriter(cfg.out_dir)
    try:
        if args.mode == "mock":
            outcome = run_mock(cfg, sink, msg_writer, dec_writer)
        else:
            outcome = run_carla(cfg, sink, msg_writer, dec_writer, args)
    finally:
        sink.close()
        msg_writer.close()
        dec_writer.close()

    d = outcome.to_dict()
    d["out_dir"] = cfg.out_dir
    d["messages"] = msg_writer.n_messages
    with open(os.path.join(cfg.out_dir, "run_summary.json"), "w") as fh:
        json.dump(d, fh, indent=2)
    return d


def _verdict(results: Dict[str, dict]) -> dict:
    """The headline comparison: did the attack, and only the attack, cause it?"""
    h, s = results.get(HONEST), results.get(SPOOFED)
    if not (h and s):
        return {}
    return {
        "honest_overtake": h["overtake_attempted"],
        "spoofed_overtake": s["overtake_attempted"],
        # An overtake on its own is not a failure -- the honest ego is expected to
        # pass once the road really is clear. The failure is passing while an
        # oncoming vehicle is genuinely there.
        "honest_aborted_passes": h["aborted_passes"],
        "spoofed_aborted_passes": s["aborted_passes"],
        "honest_unsafe_overtake": h["overtake_was_unsafe"],
        "spoofed_unsafe_overtake": s["overtake_was_unsafe"],
        "honest_collision": h["collided"],
        "spoofed_collision": s["collided"],
        "attack_caused_unsafe_overtake": (s["overtake_was_unsafe"]
                                          and not h["overtake_was_unsafe"]),
        "attack_caused_collision": s["collided"] and not h["collided"],
    }


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Do-Not-Pass Warning spoofing: drone impersonates an RSU "
                    "to suppress an oncoming vehicle and trigger an unsafe pass.")
    p.add_argument("--mode", choices=["mock", "carla"], default="mock")
    p.add_argument("--run", choices=[HONEST, SPOOFED, "both"], default="both",
                   help="honest baseline, the attack, or both for the comparison")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=15.0,
                   help="CARLA RPC timeout in seconds for ordinary calls (map "
                        "loading gets its own, much longer, deadline). Also caps "
                        "how long a Ctrl-C waits on an in-flight call.")
    p.add_argument("--crash-hold", type=float, default=3.0,
                   help="seconds to keep simulating after a collision, so the "
                        "crash is visible rather than the last frame")
    p.add_argument("--linger", type=float, default=6.0,
                   help="seconds to hold the finished scene before destroying "
                        "the vehicles (0 = tear down immediately)")
    p.add_argument("--wall-timeout", type=float, default=600.0,
                   help="hard wall-clock budget for one run; stops a wedged "
                        "simulator from hanging the scenario indefinitely")
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--tick", type=float, default=0.05,
                   help="control period and sim fixed delta, seconds")
    p.add_argument("--rate", type=float, default=1.0, help="CPM rate in Hz")
    p.add_argument("--ego-back", type=float, default=55.0,
                   help="ego start distance behind the RSU anchor, metres")
    p.add_argument("--lead-ahead", type=float, default=30.0)
    p.add_argument("--oncoming-ahead", type=float, default=110.0)
    p.add_argument("--lead-speed", type=float, default=18.0, help="km/h")
    p.add_argument("--oncoming-speed", type=float, default=35.0, help="km/h")
    p.add_argument("--use-spawn-points", action="store_true",
                   help=f"use the original CarlaNetpp spawn points "
                        f"{(SPAWN_EGO, SPAWN_LEAD, SPAWN_ONCOMING)} instead of "
                        f"deriving the formation from the road graph")
    p.add_argument("--no-fly-drone", action="store_true",
                   help="leave the AirSim drone wherever it is (default: place it "
                        "at the roadside fake-RSU pose before the scene starts)")
    p.add_argument("--drone-timeout", type=float, default=30.0,
                   help="seconds to let the drone settle into its hover")
    p.add_argument("--keep-vehicles", action="store_true",
                   help="leave pre-existing vehicles in place (default: clear them, "
                        "since auto-spawned traffic disturbs the scene)")
    p.add_argument("--sink", choices=["file", "udp", "null"], default="file")
    p.add_argument("--out", default="out/do_not_pass")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=47000)
    args = p.parse_args(argv)

    runs = [HONEST, SPOOFED] if args.run == "both" else [args.run]
    results = {r: _one_run(args, r) for r in runs}
    summary = {"mode": args.mode, "map": MAP_NAME if args.mode == "carla" else "mock",
               "runs": results, "verdict": _verdict(results)}
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "comparison.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print(json.dumps(summary, indent=2))

    for name, r in results.items():
        print(f"\n[{name}] overtake={r['overtake_attempted']} "
              f"collision={r['collided']} flips={r['n_flips']}/{r['n_decisions']} "
              f"-> {r['out_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
