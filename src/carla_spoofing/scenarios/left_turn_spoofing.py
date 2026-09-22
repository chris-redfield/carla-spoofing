"""Left Turn Assist spoofing: a drone impersonating an RSU causes a junction collision.

Whitepaper use case 3b (Fig. 12): *"a spoofer drone posing as an RSU and
manipulating communication, leading to a collision at the crossover. The
inability of the Rx vehicle to receive precise information accentuates the need
for a secure communication framework."*

The scene
---------
An ego vehicle waits at a signalised crossroads to make a **permissive left
turn**. It has a green ball, so it is entitled to go: the only question is
whether the junction is clear. A building on the corner hides the cross street,
so the ego cannot answer that question with its own sensors and has to trust
what infrastructure tells it -- which is precisely the dependency the attack
exploits.

  * **RSU**      -- honest infrastructure overlooking the junction, station 9001.
  * **ego**      -- waiting to turn left, sees only what the corner allows.
  * **crossing** -- a vehicle approaching on the cross street, hidden.
  * **drone**    -- the attacker, hovering at the roadside.

The drone broadcasts a CPM stamped with the **RSU's station id**, containing
everything the RSU honestly reported *except* the crossing vehicle. Because a
receiver keeps one entry per station, the forgery supersedes the genuine report
rather than adding to it, and the hazard disappears from the ego's world model.
The Left Turn Assist logic then says TURN, the ego commits, and the two meet in
the middle of the junction.

Why this is not the reference scene
-----------------------------------
The CarlaNetpp original (``use_case_3/main.py``) staged this: a subprocess fed
``steer=-0.15 -> -0.10 -> -0.05`` on ``sleep`` timers at a fixed wall-clock
moment, and the "good" variant ran the identical turn four seconds later. The
turn happened whether or not anything was attacking. Here the ego has a real
controller (``control.LeftTurnController``) whose only trigger is the receiver's
decision, so the honest and spoofed runs differ in **nothing but the message
stream**. Map, weather, vehicle models, drone pose and spectator framing are
reused from that scene; the manoeuvre is not.

Run it::

    python -m carla_spoofing.scenarios.left_turn_spoofing --mode mock --run both
    python -m carla_spoofing.scenarios.left_turn_spoofing --mode carla --run both
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..attacks.identity_spoof import ForgedIdentity, IdentitySpoofAttack
from ..attacks.remove_object import RemoveObjectAttack, RemoveTarget
from ..control import (
    TURNING, WAITING, CarlaLaneReference, ConstantSpeedController,
    KinematicVehicle, KMH, LeftTurnController, LeftTurnParams,
    PolylineReference, StraightLaneReference, approach_then_turn,
    hermite_turn_path, straightest, walk_back,
)
from ..drone import place_at as place_drone_at
from ..fusion import (
    StaticOccluder, fuse_latest_by_station, occluders_near, visible_objects,
)
from ..geometry import EgoState, heading_unit
from ..left_turn_assist import (
    DO_NOT_TURN, TURN, LeftTurnDecision, LeftTurnThresholds, evaluate_left_turn,
)
from ..report import LeftTurnDecisionWriter, ReportWriter, TrajectoryWriter
from ..perception import build_cpm_from_objects
from ..v2x.cpm import CollectivePerceptionMessage, PerceivedObject
from ..v2x.packet import FileSink, NullSink, Packet, UdpSink

Vec3 = Tuple[float, float, float]

# --------------------------------------------------------------------------- #
# Town10 scene geometry.                                                       #
#                                                                              #
# Map, weather, drone hover pose and spectator framing are reused from the     #
# earlier experiment this work continues:                                      #
#   ~/proj/CarlaNetpp/cooperative_perception_dev/use_case_3/main.py            #
# What is NOT reused is that script's timed steering input -- see the module   #
# docstring.                                                                   #
# --------------------------------------------------------------------------- #
MAP_NAME = "Town10HD"
WEATHER = "CloudyNoon"

# Surveyed drone hover pose from the reference run.
DRONE_LOCATION: Vec3 = (-43.010139, 22.077925, 12.224535)
DRONE_ROTATION = (-16.181599, 89.799080, 0.000033)        # pitch, yaw, roll
SPECTATOR_LOCATION: Vec3 = (-62.067219, 3.026420, 27.673388)
SPECTATOR_ROTATION = (-50.634418, 55.887512, 0.000040)

# The RSU the drone impersonates: the same roadside spot at pole height, so the
# drone is literally hovering above the station whose identity it steals.
RSU_LOCATION: Vec3 = (-43.010139, 22.077925, 6.0)
RSU_STATION_ID = 9001         # infrastructure ids live above the CARLA actor range
VIRTUAL_DRONE_STATION_ID = 9002    # used when no drone actor exists in the world

# The junction is found from the road graph near this point rather than by spawn
# index. The reference used CARLA 0.9.13 indices 126/33/145, which are provenance
# only: index numbering drifts between CARLA versions, which is exactly why the
# equivalent flag was removed from the do-not-pass scenario.
# The scene is built from the reference run's SURVEYED SPAWN POINTS, and the
# junction is whatever lies ahead of the ego's. This reverses the stance taken
# in do_not_pass (where spawn indices were deleted as version-fragile): here
# they are the only thing that reliably put the cars on the right roads, and
# `--survey` verifies them against the live map in seconds. Index drift shows up
# as an obviously wrong survey, not as a subtly wrong run.
REFERENCE_SPAWN_POINTS = (126, 33, 145)      # ego, hazard, spare
JUNCTION_ANCHOR: Vec3 = (-46.0, 16.0, 0.0)   # provenance only; not used to place

# Mock-backend ids (no CARLA actors involved).
MOCK_EGO_ID, MOCK_CROSSING_ID, MOCK_DECOY_ID = 201, 202, 203

# Switching maps at runtime is OFF by default here -- see _ensure_map. The sim
# must be started on this map: `make up MAP=Town10HD`.
MAP_LOAD_TIMEOUT_S = 120.0
MAP_SETTLE_S = 5.0

HONEST, SPOOFED = "honest", "spoofed"


@dataclass
class ScenarioConfig:
    """Everything tunable about the scene, the messages and the manoeuvre."""

    run: str = SPOOFED
    duration_s: float = 60.0
    tick_s: float = 0.05
    message_rate_hz: float = 1.0

    # Perception ranges per station. The RSU deliberately outranges the ego:
    # seeing further than its receivers is the reason infrastructure is useful.
    rsu_range_m: float = 160.0
    drone_range_m: float = 160.0
    ego_range_m: float = 75.0

    # Initial formation, measured along the road from the junction.
    # Spawn distances. Vehicles start well back and DRIVE to the junction --
    # never spawned in the middle of the scene. A car that appears where the
    # action is looks staged, gives the controller no settling distance, and
    # hides exactly the approach behaviour the scenario is meant to show.
    ego_back_m: float = 80.0          # ego start, back along its approach arm
    # Where the hazard starts. None = derive it from how long the ego takes to
    # reach the line, so the two meet. Hand-tuned distances do not survive a
    # change to the approach length: lengthening the ego's run-up to 80 m made
    # the hazard arrive and leave before the ego had even decided, and both runs
    # then turned at the same moment with nothing to suppress.
    crossing_back_m: Optional[float] = None
    hazard_lead_s: float = 2.0        # hazard arrives this long AFTER the ego
    hazard_min_speed_kmh: float = 15.0
    hazard_max_speed_kmh: float = 55.0
    crossing_speed_kmh: float = 35.0
    stop_line_m: float = 6.0          # stop line, back from the junction centre
    # How far INTO the junction the ego drives before it starts turning. A car
    # stopped at the line does not pivot from there -- it pulls forward and
    # turns from inside. Starting the arc at the line made the ego cut the
    # corner and clip a traffic-light pole on the island between the two roads.
    # Measured against the pole the ego kept clipping, at (-37.4, 17.4) on the
    # live map: advance 5 / tangent 0.35 left 2.11 m of clearance -- about the
    # cybertruck's own width, hence the hits and near misses. 8 / 0.60 gives
    # 4.22 m, with the lane discipline on the approach unchanged (0.02 m) and
    # the exit heading slightly better. Both dials widen the turn monotonically;
    # these are as wide as they can go before the ego overshoots into the far
    # side of the junction.
    turn_entry_advance_m: float = 8.0
    exit_run_out_m: float = 6.0       # how far the turn path runs past the exit
    # Tangent length as a fraction of the turn's endpoint separation. Small,
    # because a junction turn is a tight arc: at 0.6 over a 46 m span the curve
    # bent 27 deg within 4 m and the controller answered with full lock.
    turn_tangent_scale: float = 0.60

    clean_vehicles: bool = True
    freeze_lights_green: bool = True
    out_dir: str = "out/left_turn"
    rsu_position: Vec3 = RSU_LOCATION
    drone_position: Vec3 = DRONE_LOCATION
    # Drone framing. The surveyed reference pose is a close-up stills position,
    # far too near the junction to watch the scene from -- see _derive_drone_pose.
    drone_back_m: float = 28.0        # back down the ego's approach arm
    drone_side_m: float = 6.0         # to the right of that arm, off the road
    drone_height_m: float = 20.0      # high enough to see over the corner

    def message_period_ticks(self) -> int:
        return max(1, int(round((1.0 / self.message_rate_hz) / self.tick_s)))

    def ego_arrival_s(self, approach_m: float) -> float:
        """Roughly when the ego reaches the stop line, from ``approach_m`` back."""
        cruise = LeftTurnParams().cruise_speed_mps
        t = max(0.0, approach_m - self.stop_line_m) / cruise
        return t + cruise / LeftTurnParams().stop_decel_mps2 * 0.5

    def hazard_speed_for(self, hazard_m: float, ego_approach_m: float) -> float:
        """Speed (km/h) that puts the hazard at the junction just after the ego.

        Both spawns are surveyed and therefore fixed, so the one free variable
        left for timing is how fast the hazard drives. Solve for it rather than
        nudging positions: positions are facts, speed is a dial.

        Clamped to something a car would plausibly do -- if the spawn is so
        close or so far that the required speed is absurd, that is a scene
        problem to be reported, not papered over with a 5 km/h crawl.
        """
        want_s = self.ego_arrival_s(ego_approach_m) + self.hazard_lead_s
        if want_s <= 0.1:
            return self.crossing_speed_kmh
        kmh = (hazard_m / want_s) / KMH
        return max(self.hazard_min_speed_kmh,
                   min(self.hazard_max_speed_kmh, kmh))

    def hazard_start_distance(self) -> float:
        """How far back the crossing vehicle starts, so it arrives on cue.

        The scenario only works if the hazard reaches the conflict point while
        the ego is deciding. Arrive too early and it has gone by; too late and
        the honest ego turns before it matters. Either way both runs behave
        identically and the run proves nothing.

        So the distance is derived rather than guessed: time for the ego to
        cover its approach and stop, plus ``hazard_lead_s``, times the hazard's
        speed. The lead is well inside ``critical_gap_s``, so an honest ego must
        wait and a deceived one pulls straight out in front of it.
        """
        if self.crossing_back_m is not None:
            return self.crossing_back_m
        cruise = LeftTurnParams().cruise_speed_mps
        approach_s = max(0.0, self.ego_back_m - self.stop_line_m) / cruise
        approach_s += cruise / LeftTurnParams().stop_decel_mps2 * 0.5   # braking
        speed = self.crossing_speed_kmh * KMH
        return speed * (approach_s + self.hazard_lead_s)


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


def ego_self_cpm(cfg: ScenarioConfig, states: Sequence[PerceivedObject],
                 ego_id: int, gen_time: float,
                 occluders: Sequence[StaticOccluder]) -> CollectivePerceptionMessage:
    """What the ego's own sensors see -- with the corner building in the way.

    Without this occlusion step the scenario would be vacuous: an ego that can
    see the crossing vehicle itself has no reason to trust the RSU, and
    suppressing the RSU's report would change nothing. Not being able to see
    past the corner is the whole premise of the use case.
    """
    ego = next(s for s in states if s.object_id == ego_id)
    cpm = build_cpm_from_objects(
        station_id=ego_id, reference_position=ego.position, objects=states,
        generation_time=gen_time, perception_range=cfg.ego_range_m,
        station_type="vehicle")
    cpm.perceived_objects = visible_objects(
        ego.position, cpm.perceived_objects, cpm.perceived_objects,
        static_occluders=occluders)
    return cpm


def build_round(cfg: ScenarioConfig, states: Sequence[PerceivedObject], ego_id: int,
                drone_station_id: int, gen_time: float, attack: IdentitySpoofAttack,
                occluders: Sequence[StaticOccluder]) -> RoundMessages:
    """One messaging round: honest reports, the forgery, and both fused views."""
    rsu = build_cpm_from_objects(
        station_id=RSU_STATION_ID, reference_position=cfg.rsu_position,
        objects=states, generation_time=gen_time,
        perception_range=cfg.rsu_range_m, station_type="rsu")
    drone_honest = build_cpm_from_objects(
        station_id=drone_station_id, reference_position=cfg.drone_position,
        objects=states, generation_time=gen_time,
        perception_range=cfg.drone_range_m, station_type="drone")
    forged = attack.apply_cpm(drone_honest)
    ego = ego_self_cpm(cfg, states, ego_id, gen_time, occluders)

    honest_view = fuse_latest_by_station([rsu, ego])
    spoofed_view = fuse_latest_by_station([rsu, forged, ego])
    return RoundMessages(rsu, drone_honest, forged, ego,
                         honest_view.objects, spoofed_view.objects)


def make_attack(crossing_id: int, cfg: ScenarioConfig) -> IdentitySpoofAttack:
    """The drone: erase the crossing vehicle, then sign it as the RSU."""
    return IdentitySpoofAttack(
        ForgedIdentity(station_id=RSU_STATION_ID,
                       reference_position=cfg.rsu_position,
                       station_type="rsu"),
        inner=RemoveObjectAttack(RemoveTarget(object_id=crossing_id,
                                              carve_lidar=False)))


def _ego_state(states: Sequence[PerceivedObject], ego_id: int,
               desired_speed: Optional[float] = None) -> EgoState:
    s = next(o for o in states if o.object_id == ego_id)
    return EgoState(station_id=ego_id, position=s.position, yaw_deg=s.yaw_deg,
                    velocity=s.velocity, desired_speed_mps=desired_speed)


def _occlusion_warning(ego: EgoState, truth_objects: Sequence[PerceivedObject],
                       ego_view: Sequence[PerceivedObject],
                       hazard_id: int) -> Optional[str]:
    """Flag a scene where the ego can see the hazard unaided.

    The same class of trap as the drone-range guard in the do-not-pass scenario,
    and the reason that guard exists. If the corner does not actually hide the
    crossing vehicle, the ego's own CPM still contains it, the fused view still
    contains it whatever the attacker deletes, and the run proves nothing --
    while very possibly still ending in a collision for unrelated reasons. The
    scenario has to say so rather than let the verdict flags imply a result.
    """
    if any(o.object_id == hazard_id for o in ego_view):
        return (f"WARNING: the ego can see the crossing vehicle {hazard_id} with "
                f"its own sensors, so the corner is not occluding it. The "
                f"suppressed RSU report is then irrelevant to the decision and "
                f"any collision is NOT evidence of the attack. Check the "
                f"building geometry or move the ego's start back.")
    return None


# --------------------------------------------------------------------------- #
# Outcome bookkeeping                                                          #
# --------------------------------------------------------------------------- #
def _trace_tick(trace, cfg, frame, sim_time, ego_ctrl, ego_state, command,
                decision, others) -> None:
    """Record one control tick for every actor.

    The ego row carries the controller's internals -- state, raw actuation, and
    crucially the pure-pursuit **target point** it was steering at. A vehicle
    that turns the wrong way is nearly always chasing a target in the wrong
    place, and comparing ``target_x/target_y`` against the path is the only way
    to tell that from a genuinely broken controller.
    """
    progress = remaining = None
    if ego_ctrl.state == TURNING or ego_ctrl.committed:
        progress = ego_ctrl.turn_path.progress(ego_state.position)
        remaining = ego_ctrl.turn_path.remaining(ego_state.position)
    trace.add(run=cfg.run, frame=frame, sim_time=sim_time,
              actor_id=ego_state.station_id, role="ego",
              position=ego_state.position, yaw_deg=ego_state.yaw_deg,
              speed=ego_state.speed, ctrl_state=ego_ctrl.state,
              command=command, target=ego_ctrl.last_target,
              target_bearing_deg=ego_ctrl.last_bearing_deg,
              path_progress=progress, path_remaining=remaining,
              dist_to_hold=ego_ctrl._distance_to_hold(ego_state),
              decision=decision.decision)
    for actor_id, role, obj in others:
        pos = obj.position
        yaw = obj.yaw_deg
        speed = getattr(obj, "speed", None)
        if speed is None:
            v = obj.velocity
            speed = math.sqrt(sum(c * c for c in v))
        trace.add(run=cfg.run, frame=frame, sim_time=sim_time,
                  actor_id=actor_id, role=role, position=pos, yaw_deg=yaw,
                  speed=speed)


@dataclass
class Outcome:
    run: str
    mode: str
    turn_started_s: Optional[float] = None
    turn_committed_s: Optional[float] = None
    turn_was_unsafe: bool = False           # ground truth said DO_NOT_TURN
    truth_at_turn: str = ""
    collision: Optional[dict] = None           # vehicle vs vehicle only
    static_collision: Optional[dict] = None    # scenery: a driving fault
    n_decisions: int = 0
    n_flips: int = 0
    first_flip_s: Optional[float] = None
    ego_state_changes: List = field(default_factory=list)
    attack: dict = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "run": self.run, "mode": self.mode,
            "turn_started_s": self.turn_started_s,
            "turn_attempted": self.turn_started_s is not None,
            "turn_committed_s": self.turn_committed_s,
            "turn_committed": self.turn_committed_s is not None,
            # A turn the ego started and aborted when the next message arrived is
            # a success of the warning, not a failure -- counted, not scored.
            "aborted_turns": sum(
                1 for i in range(1, len(self.ego_state_changes))
                if self.ego_state_changes[i][1] == WAITING
                and self.ego_state_changes[i - 1][1] == TURNING),
            "turn_was_unsafe": self.turn_was_unsafe,
            "truth_at_turn": self.truth_at_turn,
            "collision": self.collision,
            "collided": self.collision is not None,
            "static_collision": self.static_collision,
            "hit_scenery": self.static_collision is not None,
            "n_decisions": self.n_decisions, "n_flips": self.n_flips,
            "first_flip_s": self.first_flip_s,
            "ego_state_changes": self.ego_state_changes,
            "attack": self.attack, "notes": self.notes,
        }


def _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time, ego_id,
               ego_speed, ego_state_name, msgs: RoundMessages,
               acted: LeftTurnDecision, counter: LeftTurnDecision,
               drone_station_id: int, outcome: Outcome,
               truth: Optional[LeftTurnDecision] = None) -> None:
    """Write one messaging round to every output channel."""
    hidden = (counter.conflicting_object_id
              if counter.conflicting_object_id != acted.conflicting_object_id
              else None)
    dec_writer.add(run=cfg.run, frame=seq, sim_time=sim_time, ego_id=ego_id,
                   ego_speed=ego_speed, ego_state=ego_state_name,
                   acted=acted, counterfactual=counter,
                   hidden_object_id=hidden, truth=truth)
    # Genuine traffic is always on the air; the forgery only in the attack run,
    # so the honest baseline is a world in which the attacker simply is not
    # transmitting -- not one where it transmits something harmless.
    for cpm, sender, stype in ((msgs.rsu, RSU_STATION_ID, "rsu"),
                               (msgs.ego, ego_id, "vehicle")):
        sink.send(Packet.cpm(cpm, seq=seq))
        msg_writer.add(frame=seq, sim_time=sim_time, sender_id=sender,
                       sender_type=stype, is_attacker=False,
                       honest=cpm, broadcast=cpm)
    if cfg.run == SPOOFED:
        sink.send(Packet.cpm(msgs.forged, seq=seq))
        msg_writer.add(frame=seq, sim_time=sim_time, sender_id=drone_station_id,
                       sender_type="drone", is_attacker=True,
                       honest=msgs.drone_honest, broadcast=msgs.forged)

    outcome.n_decisions += 1
    if counter.decision != acted.decision:
        outcome.n_flips += 1
        if outcome.first_flip_s is None:
            outcome.first_flip_s = round(sim_time, 2)


# --------------------------------------------------------------------------- #
# Mock backend: the same closed loop, on a kinematic crossroads                #
# --------------------------------------------------------------------------- #
def mock_scene(cfg: ScenarioConfig):
    """A crossroads with no CARLA: ego heading +x, hazard crossing from the left.

    CARLA frame conventions throughout (X forward, Y **right**), so the ego's
    left turn is toward -Y and the cross street runs along Y. The corner
    building sits in the quadrant between the ego and the hazard's approach,
    which is the only place it can be and still do any work.
    """
    junction = (0.0, 0.0, 0.0)
    ego_start = (-cfg.ego_back_m, 0.0, 0.0)
    stop_line = (-cfg.stop_line_m, 0.0, 0.0)
    # Exit arm: the ego turns left, i.e. toward -Y, joining the cross street.
    exit_point = (0.0, -cfg.stop_line_m - 6.0, 0.0)
    turn_path = PolylineReference(
        points=hermite_turn_path(stop_line, 0.0, exit_point, -90.0))

    # The hazard comes DOWN the cross street from -Y toward +Y, i.e. from the
    # ego's left, and crosses the ego's turn path inside the junction.
    crossing_start = (0.0, -cfg.hazard_start_distance(), 0.0)
    conflict_point = (0.0, -3.5, 0.0)

    # Corner building: between the ego on its approach and the hazard's arm.
    # Offset into the quadrant so it blocks the diagonal sightline without
    # sitting on either carriageway.
    building = StaticOccluder(center=(-12.0, -14.0, 0.0), extent=(9.0, 9.0, 10.0),
                              label="corner_building")
    return junction, ego_start, stop_line, turn_path, crossing_start, \
        conflict_point, [building]


def run_mock(cfg: ScenarioConfig, sink, msg_writer, dec_writer,
             trace=None) -> Outcome:
    outcome = Outcome(run=cfg.run, mode="mock")
    (junction, ego_start, stop_line, turn_path, crossing_start,
     conflict_point, occluders) = mock_scene(cfg)

    cfg.rsu_position = (8.0, 8.0, 6.0)
    cfg.drone_position = (8.0, 8.0, 15.0)

    ego_v = KinematicVehicle(MOCK_EGO_ID, position=ego_start, yaw_deg=0.0,
                             speed=LeftTurnParams().cruise_speed_mps)
    # yaw +90 in CARLA's frame points toward +Y: the hazard drives up the cross
    # street from the ego's left, across the ego's turn path.
    crossing_v = KinematicVehicle(MOCK_CROSSING_ID, position=crossing_start,
                                  yaw_deg=90.0,
                                  speed=cfg.crossing_speed_kmh * KMH)

    ego_ctrl = LeftTurnController(
        approach=StraightLaneReference(ego_start, 0.0),
        turn_path=turn_path, hold_point=stop_line)
    crossing_ctrl = ConstantSpeedController(
        StraightLaneReference(crossing_start, 90.0), cfg.crossing_speed_kmh * KMH)

    attack = make_attack(MOCK_CROSSING_ID, cfg)
    thresholds = LeftTurnThresholds()
    acted = counter = LeftTurnDecision(
        decision=DO_NOT_TURN, reason="no message received yet")

    vehicles = [ego_v, crossing_v]
    steps = int(cfg.duration_s / cfg.tick_s)
    period = cfg.message_period_ticks()
    seq = 0
    warned = False
    last_truth: Optional[LeftTurnDecision] = None

    for i in range(steps):
        sim_time = i * cfg.tick_s
        states = [
            PerceivedObject(object_id=v.object_id,
                            position=v.position,
                            velocity=(heading_unit(v.yaw_deg)[0] * v.speed,
                                      heading_unit(v.yaw_deg)[1] * v.speed, 0.0),
                            yaw_deg=v.yaw_deg, dimensions=v.dimensions,
                            classification=v.classification)
            for v in vehicles
        ]

        if i % period == 0:
            msgs = build_round(cfg, states, MOCK_EGO_ID, VIRTUAL_DRONE_STATION_ID,
                               sim_time, attack, occluders)
            ego_s = _ego_state(states, MOCK_EGO_ID,
                               LeftTurnParams().cruise_speed_mps)
            honest = evaluate_left_turn(ego_s, msgs.honest_objects,
                                        conflict_point, thresholds)
            spoofed = evaluate_left_turn(ego_s, msgs.spoofed_objects,
                                         conflict_point, thresholds)
            truth = evaluate_left_turn(ego_s, states, conflict_point, thresholds)
            acted, counter = ((spoofed, honest) if cfg.run == SPOOFED
                              else (honest, spoofed))

            if not warned:
                w = _occlusion_warning(ego_s, states, msgs.ego.perceived_objects,
                                       MOCK_CROSSING_ID)
                if w:
                    print("[lta] " + w)
                    outcome.notes.append(w)
                warned = True

            _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time,
                       MOCK_EGO_ID, ego_v.speed, ego_ctrl.state, msgs, acted,
                       counter, VIRTUAL_DRONE_STATION_ID, outcome, truth)
            seq += 1
            last_truth = truth

        ego_now = _ego_state(states, MOCK_EGO_ID,
                             LeftTurnParams().cruise_speed_mps)
        ego_cmd = ego_ctrl.step(ego_now, acted, states, cfg.tick_s, sim_time)
        if trace is not None:
            _trace_tick(trace, cfg, i, sim_time, ego_ctrl, ego_now, ego_cmd,
                        acted, [(MOCK_CROSSING_ID, "crossing", crossing_v)])
        ego_v.step(ego_cmd, cfg.tick_s)
        crossing_v.step(crossing_ctrl.step(crossing_v.state(), cfg.tick_s),
                        cfg.tick_s)

        # Score the turn against ground truth at the instant the ego commits to
        # it, which happens inside ego_ctrl.step() -- i.e. AFTER the message
        # block above. Capturing it there would read the state from the previous
        # tick and never fire, which is what made a run report a collision with
        # `turn_was_unsafe` false.
        if ego_ctrl.state == TURNING and outcome.turn_started_s is None:
            outcome.turn_started_s = round(sim_time, 2)
            if last_truth is not None:
                outcome.truth_at_turn = last_truth.decision
                outcome.turn_was_unsafe = last_truth.decision == DO_NOT_TURN
        if ego_ctrl.committed and outcome.turn_committed_s is None:
            outcome.turn_committed_s = round(sim_time, 2)

        if outcome.collision is None and _mock_collision(ego_v, crossing_v):
            outcome.collision = {
                "with_actor_id": crossing_v.object_id,
                "with_type_id": "mock.crossing",
                "sim_time": round(sim_time, 2),
                "closing_speed_mps": round(ego_v.speed + crossing_v.speed, 2),
            }
            print(f"[lta] COLLISION at {sim_time:.2f}s in the junction")
            break

    outcome.ego_state_changes = [[round(t, 2), s] for t, s in ego_ctrl.history]
    outcome.attack = attack.result.to_dict()
    return outcome


def _mock_collision(a: KinematicVehicle, b: KinematicVehicle) -> bool:
    """Crude overlap test between two boxes, treated as circles.

    Good enough for the mock: the question it has to answer is "did these two
    end up in the same piece of junction", not what the impact looked like.
    """
    r = 0.5 * (max(a.dimensions[0], a.dimensions[1])
               + max(b.dimensions[0], b.dimensions[1])) * 0.5
    return math.dist(a.position[:2], b.position[:2]) <= r


# --------------------------------------------------------------------------- #
# CARLA backend                                                                #
# --------------------------------------------------------------------------- #
def walk_to_junction(carla, start_wp, max_m: float = 200.0, step_m: float = 2.0):
    """Drive forward along the road from ``start_wp`` until a junction.

    Returns ``(last_wp_before, junction_wp, junction, travelled_m, path)``.

    This replaced an earlier version that walked *backwards* from a surveyed
    anchor to find the ego's spawn. That direction was the source of a long run
    of failures: walking back branches at every junction it crosses and lands on
    a road the caller never meant, after which the "stop line" is a distant
    point projected onto the car's heading and the whole scene is nonsense.

    Walking **forward from a known spawn** has neither problem. The spawn is a
    surveyed fact, and the first junction ahead of it is the junction the car
    will actually arrive at -- which is the only one the scenario cares about.
    ``straightest`` keeps the walk on this road rather than turning off it.
    """
    probe, travelled = start_wp, 0.0
    path = [probe]
    before = probe
    while travelled < max_m:
        forward = heading_unit(probe.transform.rotation.yaw)
        nxt = probe.next(step_m)
        if not nxt:
            break
        chosen = straightest(nxt, forward)
        if chosen is None:
            break
        travelled += step_m
        if chosen.is_junction:
            return before, chosen, chosen.get_junction(), travelled, path
        before = chosen
        probe = chosen
        path.append(probe)
    return before, None, None, travelled, path


def pick_left_exit(ego_yaw_deg: float,
                   movements: Sequence[Tuple[float, float]]) -> Optional[int]:
    """Index of the movement that enters along our heading and exits LEFT.

    ``movements`` is one ``(entry_yaw, exit_yaw)`` per turning movement through
    the junction, as ``carla.Junction.get_waypoints`` reports them.

    Split out as a pure function because getting it wrong is what sent the ego
    across the junction on the first live run: the original picked the
    most-leftward *exit* over all movements without checking that the movement
    **started on the ego's own arm**, so it could return the exit of a turn
    beginning on a different approach entirely. Entry filtering comes first.
    """
    fwd = heading_unit(ego_yaw_deg)
    want = heading_unit(ego_yaw_deg - 90.0)     # left, in CARLA's left-handed frame
    best_i, best = None, 0.5                    # require a real match, not "least bad"
    for i, (entry_yaw, exit_yaw) in enumerate(movements):
        e = heading_unit(entry_yaw)
        if e[0] * fwd[0] + e[1] * fwd[1] <= 0.8:
            continue                            # not our approach arm
        h = heading_unit(exit_yaw)
        score = h[0] * want[0] + h[1] * want[1]
        if score > best:
            best, best_i = score, i
    return best_i


def pick_crossing_arm(ego_yaw_deg: float, arm_yaws: Sequence[float],
                      max_parallel: float = 0.45) -> Optional[int]:
    """Index of the arm running across us from the LEFT.

    Perpendicular enough to our heading that its traffic crosses our turn path,
    and travelling left-to-right across us, which is what "approaching from the
    ego's left" means once expressed as a heading.

    ``max_parallel`` accepts 63-117 degrees rather than a strict 70-110, because
    real junctions are skewed. It must stay **below** the decision logic's
    ``same_direction_cos`` (0.5): an arm this function accepted but
    ``evaluate_left_turn`` classed as "travelling with us" would give a hazard
    that can never conflict -- a scene that runs perfectly and proves nothing.
    """
    fwd = heading_unit(ego_yaw_deg)
    best_i, best = None, 0.0
    for i, yaw in enumerate(arm_yaws):
        h = heading_unit(yaw)
        parallel = abs(h[0] * fwd[0] + h[1] * fwd[1])
        if parallel > max_parallel:
            continue                            # too close to parallel with us
        # 2-D cross product: positive when the arm runs left-to-right across us.
        side = fwd[0] * h[1] - fwd[1] * h[0]
        score = 1.0 - parallel
        if side > 0 and score > best:
            best, best_i = score, i
    return best_i


def pick_oncoming_arm(ego_yaw_deg: float,
                      arm_yaws: Sequence[float]) -> Optional[int]:
    """Index of the arm carrying traffic straight at us -- the classic LTA hazard.

    The fallback when the junction has no arm entering from the left. A left
    turn crosses two streams, and this is the other one: the opposing through
    lane the ego cuts across. It is what Left Turn Assist is canonically about.

    Note the consequence for the scene, which the caller must handle: oncoming
    traffic is nearly head-on down the ego's own road, so a corner **building**
    cannot hide it. Only another vehicle can.
    """
    fwd = heading_unit(ego_yaw_deg)
    best_i, best = None, 0.5
    for i, yaw in enumerate(arm_yaws):
        h = heading_unit(yaw)
        opposed = -(h[0] * fwd[0] + h[1] * fwd[1])    # 1.0 = exactly head-on
        if opposed > best:
            best, best_i = opposed, i
    return best_i


def describe_arms(ego_yaw_deg: float, arm_yaws: Sequence[float]) -> List[str]:
    """Human-readable bearing of each junction arm, for diagnostics.

    When arm selection fails the only useful thing to print is what was actually
    on offer. Guessing at a junction's shape from an error message wastes a run;
    this turns it into one glance.
    """
    out = []
    for i, yaw in enumerate(arm_yaws):
        rel = (yaw - ego_yaw_deg + 180.0) % 360.0 - 180.0
        if abs(rel) < 30:
            kind = "same way as us"
        elif abs(rel) > 150:
            kind = "oncoming (head-on)"
        elif rel > 0:
            kind = "crosses from our LEFT"
        else:
            kind = "crosses from our RIGHT"
        out.append(f"arm {i}: yaw {yaw:7.1f} ({rel:+7.1f} rel) -- {kind}")
    return out


def spawns_reaching(carla, cmap, points, target_xy, radius_m: float = 25.0,
                    max_m: float = 160.0):
    """Every spawn point whose road leads to the junction at ``target_xy``.

    The reference scene's hazard index (33) is CARLA 0.9.13 numbering; on the
    0.9.16 build it lands 154 m away at a different junction. Rather than hunt
    for a replacement index by hand -- which would go stale again on the next
    version -- ask the map which spawns actually feed *this* junction.

    Returns ``[(index, distance_m, approach_yaw)]`` sorted by distance, so a
    caller can pick one with a sensible run-up rather than the first that fits.
    """
    found = []
    for idx, tf in enumerate(points):
        wp = cmap.get_waypoint(tf.location, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
        if wp is None:
            continue
        before, jwp, _j, travelled, _p = walk_to_junction(
            carla, wp, max_m=max_m)
        if jwp is None:
            continue
        jl = jwp.transform.location
        if math.dist((jl.x, jl.y), target_xy) <= radius_m:
            found.append((idx, travelled, before.transform.rotation.yaw))
    found.sort(key=lambda r: r[1])
    return found


def classify_approach(ego_yaw_deg: float, yaw: float) -> str:
    """'left' | 'right' | 'oncoming' | 'same' for an approach into our junction."""
    rel = (yaw - ego_yaw_deg + 180.0) % 360.0 - 180.0
    if abs(rel) < 30:
        return "same"
    if abs(rel) > 150:
        return "oncoming"
    return "left" if rel > 0 else "right"


def choose_hazard_spawn(carla, cmap, points, junction_xy, junction_yaw,
                        min_run_up_m: float = 35.0,
                        prefer: Sequence[str] = ("left", "oncoming")):
    """Pick a spawn that feeds our junction from a conflicting direction.

    Prefers a road crossing from the left (what a corner building can hide),
    falling back to the oncoming stream. Among the candidates it takes the one
    with the longest run-up, because a car needs road to get up to speed and to
    be visible arriving -- and because a short approach is what makes a scene
    look staged.

    Returns ``(index, distance_m, approach_yaw, kind)`` or ``None``.
    """
    candidates = spawns_reaching(carla, cmap, points, junction_xy)
    for kind in prefer:
        matching = [c for c in candidates
                    if classify_approach(junction_yaw, c[2]) == kind]
        long_enough = [c for c in matching if c[1] >= min_run_up_m]
        pool = long_enough or matching
        if pool:
            idx, dist, yaw = max(pool, key=lambda r: r[1])
            return idx, dist, yaw, kind
    return None


def choose_ego_spawn(carla, cmap, points, junction_xy, junction_yaw):
    """The spawn on OUR OWN approach with the longest run-up into this junction.

    The reference scene's ego index (126) resolves to only 16 m of approach on
    this build -- the car is effectively spawned at the stop line, which looks
    staged, gives the controller no settling distance and hides the approach
    behaviour the scenario exists to show. Other spawns sit further back on the
    same road into the same junction; this finds the furthest.

    Returns ``(index, distance_m)`` or ``None``.
    """
    same = [c for c in spawns_reaching(carla, cmap, points, junction_xy)
            if classify_approach(junction_yaw, c[2]) == "same"]
    if not same:
        return None
    idx, dist, _yaw = max(same, key=lambda r: r[1])
    return idx, dist


def run_survey(cfg: ScenarioConfig, args) -> int:
    """Print the scene's real geometry and change nothing. Piece 1 of 4.

    Everything that has gone wrong with this scenario went wrong because the
    geometry was inferred and never checked: a spawn on the wrong road, a stop
    line that was a distant point projected onto a heading, a "left" exit that
    was a right turn. Each cost a full run to discover.

    This connects, resolves the surveyed spawn points, walks forward to the
    junction each car will actually reach, and prints what it found. No actors
    are spawned, nothing is driven, no messages are sent. Run it first; if these
    numbers are wrong, nothing built on them can be right.
    """
    import carla

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world, _ = _ensure_map(client, MAP_NAME, args.timeout, args.allow_map_reload)
    cmap = world.get_map()
    points = cmap.get_spawn_points()

    print(f"\n=== {MAP_NAME}: {len(points)} spawn points ===")
    out = {"map": MAP_NAME, "n_spawn_points": len(points), "roles": {}}

    roles = (("ego", args.ego_spawn if args.ego_spawn is not None
              else REFERENCE_SPAWN_POINTS[0]),
             ("hazard", args.hazard_spawn))
    for role, idx in roles:
        if idx is None:
            print(f"\n[{role}] spawn: chosen from the map (see the listing below)")
            continue
        if idx >= len(points):
            print(f"\n[{role}] spawn index {idx} is out of range "
                  f"(map has {len(points)})")
            continue
        tf = points[idx]
        wp = cmap.get_waypoint(tf.location, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
        print(f"\n[{role}] spawn point {idx}: "
              f"({tf.location.x:.1f}, {tf.location.y:.1f}, {tf.location.z:.1f}) "
              f"yaw {tf.rotation.yaw:.1f}")
        info = {"index": idx,
                "location": [tf.location.x, tf.location.y, tf.location.z],
                "yaw": tf.rotation.yaw}

        before, jwp, junction, travelled, _path = walk_to_junction(carla, wp)
        if jwp is None:
            print(f"  no junction within {travelled:.0f} m ahead -- this spawn "
                  f"does not lead to an intersection")
            info["junction"] = None
        else:
            jl = jwp.transform.location
            approach_yaw = before.transform.rotation.yaw
            print(f"  junction {travelled:.0f} m ahead at "
                  f"({jl.x:.1f}, {jl.y:.1f}); approach heading there "
                  f"{approach_yaw:.1f} deg (spawn heading {tf.rotation.yaw:.1f})")
            info["junction"] = {
                "distance_m": travelled,
                "location": [jl.x, jl.y, jl.z],
                "approach_yaw": approach_yaw,
                "id": junction.id,
            }
            if role == "ego":
                entries = junction.get_waypoints(carla.LaneType.Driving)
                movements = [(en.transform.rotation.yaw,
                              ex.transform.rotation.yaw) for en, ex in entries]
                seen, arm_yaws = set(), []
                for en, _ex in entries:
                    key = (round(en.transform.rotation.yaw, 0),)
                    if key in seen:
                        continue
                    seen.add(key)
                    arm_yaws.append(en.transform.rotation.yaw)
                print(f"  {len(entries)} movements, {len(arm_yaws)} distinct "
                      f"arm headings:")
                for line in describe_arms(approach_yaw, arm_yaws):
                    print(f"    {line}")
                i = pick_left_exit(approach_yaw, movements)
                if i is None:
                    print("    !! NO LEFT-TURN EXIT from this approach")
                else:
                    ex = entries[i][1].transform
                    print(f"    left exit -> ({ex.location.x:.1f}, "
                          f"{ex.location.y:.1f}) yaw {ex.rotation.yaw:.1f}")
                    info["left_exit"] = [ex.location.x, ex.location.y,
                                         ex.rotation.yaw]
                j = pick_crossing_arm(approach_yaw, arm_yaws)
                k = pick_oncoming_arm(approach_yaw, arm_yaws)
                print(f"    crossing arm from the left: "
                      f"{'arm %d' % j if j is not None else 'NONE'}; "
                      f"oncoming arm: "
                      f"{'arm %d' % k if k is not None else 'NONE'}")
                info["arm_yaws"] = arm_yaws
        out["roles"][role] = info

    # Which spawns actually feed the ego's junction? This is the question the
    # stale reference index could not answer.
    e0 = out["roles"].get("ego", {}).get("junction")
    if e0:
        jxy = (e0["location"][0], e0["location"][1])
        jyaw = e0["approach_yaw"]
        feeders = spawns_reaching(carla, cmap, points, jxy)
        print(f"\n=== spawn points feeding the ego's junction "
              f"({len(feeders)} found) ===")
        for idx, dist, yaw in feeders:
            kind = classify_approach(jyaw, yaw)
            where = ("on OUR approach" if kind == "same"
                     else f"crosses from our {kind.upper()}")
            mark = " <- reference ego spawn" if idx == REFERENCE_SPAWN_POINTS[0] else ""
            print(f"  spawn {idx:3d}: {dist:5.1f} m of run-up, "
                  f"approach yaw {yaw:7.1f} -- {where}{mark}")
        out["feeders"] = [{"index": i, "run_up_m": d, "approach_yaw": y,
                           "kind": classify_approach(jyaw, y)}
                          for i, d, y in feeders]
        ego_pick = choose_ego_spawn(carla, cmap, points, jxy, jyaw)
        if ego_pick:
            print(f"\n  --> auto-selected EGO spawn {ego_pick[0]} "
                  f"({ego_pick[1]:.0f} m run-up, vs "
                  f"{REFERENCE_SPAWN_POINTS[0]}'s "
                  f"{out['roles'].get('ego', {}).get('junction', {}).get('distance_m', 0):.0f} m)")
            out["auto_ego_spawn"] = {"index": ego_pick[0],
                                     "run_up_m": ego_pick[1]}
        pick = choose_hazard_spawn(carla, cmap, points, jxy, jyaw)
        if pick:
            idx, dist, yaw, kind = pick
            print(f"\n  --> auto-selected hazard spawn {idx} "
                  f"({dist:.0f} m run-up, {kind})")
            out["auto_hazard_spawn"] = {"index": idx, "run_up_m": dist,
                                        "kind": kind}
        else:
            print("\n  --> NO usable hazard spawn feeds this junction")

    # Do the two cars actually meet?
    e = out["roles"].get("ego", {}).get("junction")
    h = out["roles"].get("hazard", {}).get("junction")
    if e and h:
        same = math.dist(e["location"][:2], h["location"][:2]) < 25.0
        print(f"\n=== do they meet? ===")
        print(f"  ego reaches its junction after    {e['distance_m']:.0f} m")
        print(f"  hazard reaches its junction after {h['distance_m']:.0f} m")
        print(f"  the two junctions are "
              f"{math.dist(e['location'][:2], h['location'][:2]):.0f} m apart "
              f"-- {'SAME junction, good' if same else 'DIFFERENT junctions, BAD'}")
        out["same_junction"] = same

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "survey.json")
    with open(path, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"\nwritten to {path}\n")
    return 0


def _derive_transforms(carla, world, cfg, args=None):
    """Place the vehicles from the SURVEYED spawn points, then walk forward.

    The ego starts at the reference scene's spawn point and drives to the first
    junction ahead of it. Nothing is derived by walking backwards any more --
    that is what put the ego on the wrong street five runs in a row, after which
    the stop line was a distant point projected onto its heading and the "left"
    exit was a right turn.

    A spawn point is a surveyed fact. The junction ahead of it is the junction
    the car will actually arrive at. Everything else hangs off those two.
    """
    cmap = world.get_map()
    points = cmap.get_spawn_points()
    ego_idx = getattr(args, "ego_spawn", None)
    if ego_idx is None:
        # Start from the reference spawn to locate the junction, then move back
        # to the furthest spawn on the same approach. Same road, same junction,
        # more run-up -- the car drives to the line instead of starting on it.
        # Seed from the reference spawn when the map has it -- that is what
        # identifies *which* junction this scene is about. A build with fewer
        # spawn points falls back to the first one rather than crashing on the
        # index, which is how this path should have behaved all along.
        seed_idx = (REFERENCE_SPAWN_POINTS[0]
                    if REFERENCE_SPAWN_POINTS[0] < len(points) else 0)
        seed = cmap.get_waypoint(points[seed_idx].location,
                                 project_to_road=True,
                                 lane_type=carla.LaneType.Driving)
        sb, sj, _sjn, _st, _sp = walk_to_junction(carla, seed)
        if sj is not None:
            jl = sj.transform.location
            better = choose_ego_spawn(carla, cmap, points, (jl.x, jl.y),
                                      sb.transform.rotation.yaw)
            if better:
                ego_idx, run_up = better
                print(f"[lta] ego spawn {ego_idx} chosen from the map "
                      f"({run_up:.0f} m of approach into the same junction; "
                      f"reference spawn {REFERENCE_SPAWN_POINTS[0]} has only "
                      f"16 m)")
        if ego_idx is None:
            ego_idx = REFERENCE_SPAWN_POINTS[0]
    # The hazard index is resolved later, from the junction we have not found
    # yet, so it is legitimately None here -- only range-check what is set.
    hazard_idx = getattr(args, "hazard_spawn", None)
    for name, idx in (("ego", ego_idx), ("hazard", hazard_idx)):
        if idx is not None and idx >= len(points):
            raise SystemExit(
                f"--{name}-spawn {idx} is out of range: {MAP_NAME} in this "
                f"build has {len(points)} spawn points. Run "
                f"`make left-turn-survey` to see the scene.")

    ego_wp = cmap.get_waypoint(points[ego_idx].location, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
    before_junction, junction_wp, junction, ego_walked, _path = \
        walk_to_junction(carla, ego_wp)
    if junction_wp is None:
        raise SystemExit(
            f"No junction ahead of ego spawn point {ego_idx} within "
            f"{ego_walked:.0f} m. Run `make left-turn-survey` to see where "
            f"that spawn actually leads.")

    # The stop line, a few metres short of the junction, on the road the ego is
    # genuinely driving down.
    stop_wp, _ = walk_back(before_junction, cfg.stop_line_m)

    # Every junction decision uses the heading where the ego MEETS the junction,
    # not where it spawns. Those are the same only on a dead-straight approach.
    junction_yaw = before_junction.transform.rotation.yaw
    ego_fwd = heading_unit(junction_yaw)
    entries = junction.get_waypoints(carla.LaneType.Driving)

    # `get_waypoints` returns one (entry, exit) pair per *turning movement*
    # through the junction -- every arm's left, straight and right. The choice of
    # which pair is ours is the part that broke on the first live run, so it
    # lives in pick_left_exit / pick_crossing_arm where it can be unit-tested.
    movements = [(en.transform.rotation.yaw, ex.transform.rotation.yaw)
                 for en, ex in entries]
    i = pick_left_exit(junction_yaw, movements)
    if i is None:
        raise SystemExit(
            f"No left-turn movement found at this junction: none of its "
            f"{len(entries)} movements both enter along the ego's heading "
            f"({junction_yaw:.0f} deg) and exit to the left. "
            f"The anchor may have landed on an arm that cannot turn left; "
            f"run `make left-turn-survey` to see this junction.")
    exit_wp = entries[i][1]

    # The hazard's arm, deduplicated by road/lane since several movements share
    # one entry waypoint.
    seen, arms = set(), []
    for en, _ex in entries:
        key = (en.road_id, en.lane_id)
        if key in seen:
            continue
        seen.add(key)
        arms.append(en)
    arm_yaws = [a.transform.rotation.yaw for a in arms]
    j = pick_crossing_arm(junction_yaw, arm_yaws)
    hazard_kind = "crossing from the left"
    if j is None:
        # No left arm. A left turn crosses two streams, so fall back to the
        # other one: the opposing through lane. That is the canonical Left Turn
        # Assist conflict anyway -- but it changes what can hide the hazard, so
        # say so rather than quietly producing a scene whose occluder does
        # nothing (see the occlusion note in the module docstring).
        j = pick_oncoming_arm(junction_yaw, arm_yaws)
        hazard_kind = "oncoming through-traffic"
        if j is not None:
            note = ("no arm enters this junction from the ego's left, so the "
                    "hazard is ONCOMING through-traffic instead. A corner "
                    "building cannot hide a car that is head-on down your own "
                    "road -- expect the occlusion warning unless another "
                    "vehicle is doing the hiding.")
            print("[lta] " + note)
    if j is None:
        raise SystemExit(
            "No usable conflicting arm at this junction. Its arms are:\n  "
            + "\n  ".join(describe_arms(junction_yaw, arm_yaws))
            + f"\n\nThe ego meets the junction heading {junction_yaw:.0f} deg. "
              f"This scenario needs either an arm crossing from the left or an "
              f"oncoming one; try a different --ego-spawn (see "
              f"`make left-turn-survey`).")
    crossing_wp = arms[j]
    print(f"[lta] hazard arm: {hazard_kind} "
          f"(yaw {arm_yaws[j]:.0f} deg, {len(arms)} arms at this junction)")
    for line in describe_arms(junction_yaw, arm_yaws):
        print(f"[lta]   {line}")

    # The hazard also starts from its SURVEYED spawn point, so its position is
    # a fact rather than a walk. That fixes where it is, which means the timing
    # has to come from somewhere else -- so it comes from its speed. Solving for
    # the speed that puts it at the junction just after the ego arrives keeps
    # both spawns honest and makes the encounter reproducible.
    # The reference hazard index is 0.9.13 numbering and lands at a different
    # junction on this build, so the hazard spawn is chosen FROM THE MAP: a
    # spawn whose road actually feeds this junction, from a conflicting
    # direction, with the longest run-up available. An explicit --hazard-spawn
    # overrides, which is what the survey's listing is for.
    jl0 = junction_wp.transform.location
    if args is not None and getattr(args, "hazard_spawn", None) is not None:
        chosen_idx, hazard_kind_hint = hazard_idx, None
    else:
        pick = choose_hazard_spawn(carla, cmap, points, (jl0.x, jl0.y),
                                   junction_yaw)
        if pick is None:
            raise SystemExit(
                "No spawn point feeds this junction from a conflicting "
                "direction. Run `make left-turn-survey` to see what does feed "
                "it, then pass --hazard-spawn explicitly.")
        chosen_idx, _dist, _yaw, hazard_kind_hint = pick
        print(f"[lta] hazard spawn {chosen_idx} chosen from the map "
              f"({_dist:.0f} m run-up, approaches from the "
              f"{hazard_kind_hint})")
    hazard_idx = chosen_idx
    crossing_start_wp = cmap.get_waypoint(
        points[hazard_idx].location, project_to_road=True,
        lane_type=carla.LaneType.Driving)
    _hb, hazard_jwp, _hj, crossing_walked, _hp = walk_to_junction(
        carla, crossing_start_wp)
    if hazard_jwp is None:
        raise SystemExit(
            f"No junction ahead of hazard spawn point {hazard_idx}. Run "
            f"`make left-turn-survey`.")
    gap_m = math.dist(
        (hazard_jwp.transform.location.x, hazard_jwp.transform.location.y),
        (junction_wp.transform.location.x, junction_wp.transform.location.y))
    if gap_m > 25.0:
        raise SystemExit(
            f"Ego spawn {ego_idx} and hazard spawn {hazard_idx} lead to "
            f"DIFFERENT junctions, {gap_m:.0f} m apart. They can never meet. "
            f"Run `make left-turn-survey` and pick spawn points that share a "
            f"junction.")

    hazard_speed_kmh = cfg.hazard_speed_for(crossing_walked, ego_walked)

    def lift(t):
        return carla.Transform(
            carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + 0.3),
            t.rotation)

    # Conflict point: where the ego's turn path crosses the hazard's arm. The
    # junction-entry waypoint of that arm is the best cheap stand-in.
    conflict_point = (crossing_wp.transform.location.x,
                      crossing_wp.transform.location.y,
                      crossing_wp.transform.location.z)

    # The point inside the junction the ego turns from: the junction boundary,
    # advanced along the approach heading so the car is properly in the box.
    _jfx, _jfy = heading_unit(junction_yaw)
    turn_entry = (junction_wp.transform.location.x + _jfx * cfg.turn_entry_advance_m,
                  junction_wp.transform.location.y + _jfy * cfg.turn_entry_advance_m,
                  junction_wp.transform.location.z)

    # Run the path a little PAST the junction's exit waypoint. That waypoint sits
    # on the junction boundary, so a path ending there leaves the ego steering at
    # a point it is already on the moment it arrives -- and it drifts. Extending
    # along the exit lane gives the controller somewhere to aim while it
    # straightens up and drives out.
    exit_beyond = exit_wp.next(cfg.exit_run_out_m)
    exit_end = exit_beyond[0] if exit_beyond else exit_wp
    turn_points = approach_then_turn(
        (stop_wp.transform.location.x, stop_wp.transform.location.y,
         stop_wp.transform.location.z),
        turn_entry, junction_yaw,
        (exit_end.transform.location.x, exit_end.transform.location.y,
         exit_end.transform.location.z),
        exit_end.transform.rotation.yaw,
        tangent_scale=cfg.turn_tangent_scale)

    return {
        "ego_tf": lift(ego_wp.transform),
        "crossing_tf": lift(crossing_start_wp.transform),
        "stop_point": (stop_wp.transform.location.x, stop_wp.transform.location.y,
                       stop_wp.transform.location.z),
        "turn_points": turn_points,
        "conflict_point": conflict_point,
        "junction_center": (junction_wp.transform.location.x,
                            junction_wp.transform.location.y,
                            junction_wp.transform.location.z),
        "ego_yaw_deg": junction_yaw,
        "turn_entry_point": turn_entry,
        "exit_yaw_deg": exit_end.transform.rotation.yaw,
        "exit_point": (exit_end.transform.location.x,
                       exit_end.transform.location.y,
                       exit_end.transform.location.z),
        "note": (f"ego from spawn point {ego_idx}, {ego_walked:.0f} m to the "
                 f"junction; hazard from spawn point {hazard_idx}, "
                 f"{crossing_walked:.0f} m to the same junction "
                 f"({gap_m:.0f} m apart) at {hazard_speed_kmh:.0f} km/h "
                 f"(solved so it arrives {cfg.hazard_lead_s:.0f}s after the ego)"),
        "hazard_kind": hazard_kind,
        "hazard_speed_kmh": hazard_speed_kmh,
        "hazard_junction_gap_m": gap_m,
        "ego_walked_m": ego_walked,
        "crossing_walked_m": crossing_walked,
    }


def _derive_drone_pose(cfg: ScenarioConfig, junction_center: Vec3,
                       ego_yaw_deg: float) -> Tuple[Vec3, float, str]:
    """Where the drone hovers: back down the ego's approach, looking at the junction.

    The surveyed pose from the reference scene is NOT reused as a position. That
    run was taking close-up stills, so the drone sits almost on top of the
    junction -- from there it sees the roofs of the cars it is supposed to be
    watching and nothing of the scene. Here it is pulled back along the ego's
    approach arm and lifted, so the crossroads, both vehicles and the whole
    manoeuvre are laid out in front of it.

    Exactly the same correction the do-not-pass scenario needed, for exactly the
    same reason -- see ``--drone-back`` there.

    The attack does not care: a forged CPM claims the *impersonated* station's
    position (the RSU's), never the attacker's, so the drone's own pose is
    cosmetic. What it must not do is fall outside its own perception range.
    """
    fwd = heading_unit(ego_yaw_deg)
    right = (-fwd[1], fwd[0])
    pos = (junction_center[0] - fwd[0] * cfg.drone_back_m
           + right[0] * cfg.drone_side_m,
           junction_center[1] - fwd[1] * cfg.drone_back_m
           + right[1] * cfg.drone_side_m,
           cfg.drone_height_m)
    # Face the junction: the ego's heading is by construction the way to look.
    note = (f"drone hover {cfg.drone_back_m:.0f} m back down the ego's approach, "
            f"{abs(cfg.drone_side_m):.0f} m to the "
            f"{'right' if cfg.drone_side_m >= 0 else 'left'}, at "
            f"{cfg.drone_height_m:.0f} m, facing {ego_yaw_deg:.0f} deg")
    reach = math.dist(pos[:2], junction_center[:2]) + cfg.hazard_start_distance()
    if reach > cfg.drone_range_m:
        note += (f" -- WARNING: the crossing vehicle starts ~{reach:.0f} m from "
                 f"the drone, beyond its {cfg.drone_range_m:.0f} m perception "
                 f"range, so early forged messages have nothing to suppress and "
                 f"any collision is a sensor-range artifact. Reduce --drone-back.")
    return pos, ego_yaw_deg, note


def _corner_buildings(carla, world, center: Vec3, radius_m: float = 45.0):
    """Building boxes near the junction, as static occluders.

    Queried once at setup: a Town10 level returns hundreds of boxes and only the
    handful on these corners can matter. Best effort -- ``get_level_bbs`` is not
    present on every build, and a missing occluder must not fail a run, it must
    show up in the occlusion warning instead.
    """
    try:
        bbs = world.get_level_bbs(carla.CityObjectLabel.Buildings)
    except Exception as exc:                       # noqa: BLE001
        print(f"[lta] could not query building geometry ({exc}); "
              f"the ego will see the junction unoccluded")
        return []
    boxes = [StaticOccluder.from_carla_bb(bb) for bb in bbs]
    near = occluders_near(boxes, center, radius_m)
    print(f"[lta] {len(near)} building box(es) within {radius_m:.0f} m of the junction")
    return near


def _freeze_lights_green(carla, world, center: Vec3, radius_m: float = 60.0) -> str:
    """Hold every light at the junction green for the whole run.

    The manoeuvre is a *permissive* left turn: the ego is entitled to enter the
    junction and the only question is the gap. Leaving Town10's cycle running
    would let a red phase stop the hazard mid-run and make the scene
    non-reproducible for reasons that have nothing to do with the attack.
    """
    n = 0
    for tl in world.get_actors().filter("traffic.traffic_light*"):
        loc = tl.get_location()
        if math.dist((loc.x, loc.y), (center[0], center[1])) > radius_m:
            continue
        try:
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)
            n += 1
        except Exception:                          # noqa: BLE001
            pass
    return f"froze {n} traffic light(s) green at the junction (permissive left turn)"


def _find_drone(world):
    for a in world.get_actors():
        if "drone" in a.type_id.lower():
            return a
    return None


def _same_map(current: str, wanted: str) -> bool:
    """Is the simulator on the map we want, allowing for build-specific suffixes?

    CarlaAir ships the junction level as ``Town10HD``; the CarlaNetpp reference
    asked for ``Town10HD_Opt``, and other builds carry other suffixes. They are
    the same level, so compare on the stem rather than demanding an exact match
    and triggering a pointless -- and in this build, dangerous -- reload.
    """
    c = current.split("/")[-1].replace("_Opt", "")
    w = wanted.split("/")[-1].replace("_Opt", "")
    return c == w


def _ensure_map(client, map_name: str, rpc_timeout: float, allow_reload: bool):
    """Check the simulator is on ``map_name``. By default, do NOT reload it.

    Reloading is off by default because it is genuinely destructive here: the
    call has to tear down a level while a second client (``auto_traffic.py``)
    still owns actors and the AirSim plugin lives in the same UE4 process, and it
    has been observed completing, hanging indefinitely, and segfaulting on
    identical inputs. Restarting the user's simulator out from under them as a
    side effect of running a scenario is worse than stopping with instructions.

    ``--allow-map-reload`` opts back in for anyone who wants to risk it.
    """
    current = client.get_world().get_map().name
    if _same_map(current, map_name):
        return client.get_world(), None

    short = current.split("/")[-1]
    if not allow_reload:
        raise SystemExit(
            f"The simulator is running {short}, but this scene is at a "
            f"{map_name} crossroads.\n\n"
            f"Start it on the right map (in the terminal running the sim):\n"
            f"    make down\n"
            f"    make up MAP={map_name}\n\n"
            f"then run this again. The scenario deliberately does NOT reload the "
            f"map for you: that call is unreliable in this CarlaAir build and "
            f"restarting your simulator as a side effect is worse than stopping "
            f"here. Use --allow-map-reload to try it anyway.")

    print(f"[lta] simulator is on {short}, this scene needs {map_name}.")
    print(f"[lta] reloading the map (up to {MAP_LOAD_TIMEOUT_S:.0f}s; the window "
          f"freezes while the level streams). This call is unreliable in this "
          f"build -- starting the sim with MAP={map_name} avoids it entirely.")

    deadline = time.monotonic() + MAP_LOAD_TIMEOUT_S
    client.set_timeout(MAP_LOAD_TIMEOUT_S * 0.6)
    world = None
    try:
        world = client.load_world(map_name)
    except RuntimeError as exc:
        print(f"[lta] load_world did not return ({exc}); checking whether the "
              f"simulator got there anyway ...")
        while time.monotonic() < deadline:
            time.sleep(3.0)
            try:
                candidate = client.get_world()
                if _same_map(candidate.get_map().name, map_name):
                    world = candidate
                    break
            except RuntimeError:
                continue
    if world is None:
        raise SystemExit(
            f"Map reload to {map_name} did not complete within "
            f"{MAP_LOAD_TIMEOUT_S:.0f}s. This is a known failure of this build, "
            f"not something the scenario can retry around.\n"
            f"Start the simulator on the right map instead:\n"
            f"    make down && MAP={map_name} make up")
    time.sleep(MAP_SETTLE_S)
    client.set_timeout(rpc_timeout)
    return world, current


def run_carla(cfg: ScenarioConfig, sink, msg_writer, dec_writer, args,
              trace=None) -> Outcome:
    import carla

    outcome = Outcome(run=cfg.run, mode="carla")
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world, previous_map = _ensure_map(client, MAP_NAME, args.timeout,
                                      args.allow_map_reload)
    if previous_map:
        outcome.notes.append(f"reloaded map from {previous_map} to {MAP_NAME}")

    try:
        world.set_weather(getattr(carla.WeatherParameters, WEATHER))
    except Exception:                              # noqa: BLE001
        pass

    existing = [a for a in world.get_actors().filter("vehicle.*")]
    if existing:
        if cfg.clean_vehicles:
            print(f"[lta] removing {len(existing)} pre-existing vehicles ...")
            for a in existing:
                try:
                    a.destroy()
                except Exception:
                    pass
        else:
            outcome.notes.append(
                f"--keep-vehicles: {len(existing)} other vehicles left in the "
                "world; they may disturb the scene.")
            print("[lta] WARNING: " + outcome.notes[-1])

    scene = _derive_transforms(carla, world, cfg, args)
    print(f"[lta] placement: {scene['note']}")
    outcome.notes.append(f"placement: {scene['note']}")
    # Approach lengths are whatever the chosen spawn points give; the hazard's
    # speed is solved to match. Only a genuinely tiny run-up is worth flagging.
    for who, got in (("ego", scene["ego_walked_m"]),
                     ("hazard", scene["crossing_walked_m"])):
        if got < 15.0:
            note = (f"WARNING: the {who} has only {got:.0f} m of approach -- it "
                    f"starts almost on top of the junction. Run "
                    f"`make left-turn-survey` for spawns with a longer run-up.")
            print("[lta] " + note)
            outcome.notes.append(note)

    if cfg.freeze_lights_green:
        note = _freeze_lights_green(carla, world, scene["junction_center"])
        print(f"[lta] {note}")
        outcome.notes.append(note)

    occluders = _corner_buildings(carla, world, scene["junction_center"])
    outcome.notes.append(f"{len(occluders)} building occluder(s) near the junction")

    # Put the drone in the scene before synchronous mode: AirSim's controller
    # needs the sim stepping freely to settle into its hover.
    # Derived even when the drone is not flown: the pose is also the reference
    # position of the CPMs the attacker transmits.
    cfg.drone_position, drone_yaw, hover_note = _derive_drone_pose(
        cfg, scene["junction_center"], scene["ego_yaw_deg"])
    print(f"[lta] {hover_note}")
    outcome.notes.append(hover_note)

    if not args.no_fly_drone:
        status = place_drone_at(world, args.host, cfg.drone_position,
                                yaw_deg=drone_yaw,
                                timeout_s=args.drone_timeout)
        print(f"[lta] {status}")
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

        bl = world.get_blueprint_library()

        def bp(name, fallback="vehicle.*"):
            found = bl.filter(name)
            return found[0] if found else bl.filter(fallback)[0]

        # Same vehicle models as the reference run, for a recognisable scene.
        ego = world.try_spawn_actor(bp("vehicle.tesla.cybertruck"), scene["ego_tf"])
        crossing = world.try_spawn_actor(bp("vehicle.audi.tt"), scene["crossing_tf"])
        if not all((ego, crossing)):
            for a in (ego, crossing):
                if a:
                    a.destroy()
            raise SystemExit("Could not spawn both vehicles (blocked spawn "
                             "points?). Try --clean-vehicles.")
        spawned = [ego, crossing]
        print(f"[lta] ego={ego.id} crossing={crossing.id}")

        drone = _find_drone(world)
        if drone is not None:
            drone_station_id = drone.id
            print(f"[lta] drone actor {drone.id} ({drone.type_id}) is the attacker")
        else:
            drone_station_id = VIRTUAL_DRONE_STATION_ID
            outcome.notes.append(
                "no drone actor in the world; the attacker is a virtual station "
                "at the surveyed drone hover pose")
            print("[lta] " + outcome.notes[-1])

        cs_bp = bl.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(cs_bp, carla.Transform(), attach_to=ego)
        collision_sensor.listen(lambda e: collisions.append({
            "with_actor_id": e.other_actor.id,
            "with_type_id": e.other_actor.type_id,
            # Only a vehicle-vs-vehicle impact is the outcome this scenario
            # measures. Clipping a pole or a wall is the ego driving badly --
            # it must be reported loudly, but it must NOT be scored as the
            # collision the attack caused. The honest run hit a traffic light
            # on the corner, which would otherwise have read as a crash.
            "is_vehicle": e.other_actor.type_id.startswith("vehicle."),
        }))

        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(*SPECTATOR_LOCATION),
            carla.Rotation(*SPECTATOR_ROTATION)))

        # Both vehicles follow the ROAD GRAPH, not a straight line from their
        # spawn pose. A StraightLaneReference only ever looked adequate because
        # the ego started 15 m from the line; over a realistic approach, and on
        # the crossing car's curved arm, it drives the vehicle off the tangent
        # and out of the scene. That is exactly what happened on the first live
        # run -- the crossing car left down a 118 deg diagonal and stalled 50 m
        # from the junction, so nothing ever conflicted with anything.
        road = CarlaLaneReference(world.get_map())
        turn_path = PolylineReference(points=scene["turn_points"])

        def _turn_from(ego_state):
            """Straight into the junction from wherever it stopped, then the arc."""
            return PolylineReference(points=approach_then_turn(
                ego_state.position, scene["turn_entry_point"],
                scene["ego_yaw_deg"], scene["exit_point"],
                scene["exit_yaw_deg"],
                tangent_scale=cfg.turn_tangent_scale))

        ego_ctrl = LeftTurnController(
            approach=road, turn_path=turn_path,
            hold_point=scene["stop_point"],
            turn_path_factory=_turn_from, exit_lane=road)
        crossing_ctrl = ConstantSpeedController(
            road, scene["hazard_speed_kmh"] * KMH)

        attack = make_attack(crossing.id, cfg)
        thresholds = LeftTurnThresholds()
        conflict_point = scene["conflict_point"]
        acted = counter = LeftTurnDecision(
            decision=DO_NOT_TURN, reason="no message received yet")

        steps = int(cfg.duration_s / cfg.tick_s)
        period = cfg.message_period_ticks()
        seq = 0
        warned = False
        last_truth: Optional[LeftTurnDecision] = None
        wall_deadline = time.monotonic() + args.wall_timeout
        crash_until: Optional[float] = None

        for i in range(steps):
            if time.monotonic() > wall_deadline:
                outcome.notes.append(
                    f"wall-clock budget of {args.wall_timeout:.0f}s exhausted")
                print("[lta] " + outcome.notes[-1])
                break
            world.tick()
            sim_time = i * cfg.tick_s
            states = _carla_states(world, spawned)

            if i % period == 0:
                msgs = build_round(cfg, states, ego.id, drone_station_id,
                                   sim_time, attack, occluders)
                ego_s = _ego_state(states, ego.id,
                                   LeftTurnParams().cruise_speed_mps)
                honest = evaluate_left_turn(ego_s, msgs.honest_objects,
                                            conflict_point, thresholds)
                spoofed = evaluate_left_turn(ego_s, msgs.spoofed_objects,
                                             conflict_point, thresholds)
                truth = evaluate_left_turn(ego_s, states, conflict_point, thresholds)
                acted, counter = ((spoofed, honest) if cfg.run == SPOOFED
                                  else (honest, spoofed))

                if not warned:
                    w = _occlusion_warning(ego_s, states,
                                           msgs.ego.perceived_objects, crossing.id)
                    if w:
                        print("[lta] " + w)
                        outcome.notes.append(w)
                    warned = True

                _log_round(cfg, dec_writer, msg_writer, sink, seq, sim_time,
                           ego.id, ego_s.speed, ego_ctrl.state, msgs, acted,
                           counter, drone_station_id, outcome, truth)
                seq += 1

                last_truth = truth

            ego_state = _ego_state(states, ego.id,
                                   LeftTurnParams().cruise_speed_mps)
            ego_cmd = ego_ctrl.step(ego_state, acted, states, cfg.tick_s,
                                    sim_time)
            ego.apply_control(ego_cmd.to_carla())
            crossing.apply_control(
                crossing_ctrl.step(_ego_state(states, crossing.id),
                                   cfg.tick_s).to_carla())
            if trace is not None:
                others = [(o.object_id, "crossing", o) for o in states
                          if o.object_id != ego.id]
                _trace_tick(trace, cfg, i, sim_time, ego_ctrl, ego_state,
                            ego_cmd, acted, others)

            # See the note in run_mock: the commit happens inside step(), so
            # ground truth must be sampled here, not in the message block.
            if ego_ctrl.state == TURNING and outcome.turn_started_s is None:
                outcome.turn_started_s = round(sim_time, 2)
                if last_truth is not None:
                    outcome.truth_at_turn = last_truth.decision
                    outcome.turn_was_unsafe = last_truth.decision == DO_NOT_TURN
            if ego_ctrl.committed and outcome.turn_committed_s is None:
                outcome.turn_committed_s = round(sim_time, 2)

            hits = [c for c in collisions if c["is_vehicle"]]
            scenery = [c for c in collisions if not c["is_vehicle"]]
            if scenery and outcome.static_collision is None:
                outcome.static_collision = dict(scenery[0])
                outcome.static_collision["sim_time"] = round(sim_time, 2)
                print(f"[lta] the ego hit scenery "
                      f"({outcome.static_collision['with_type_id']}) at "
                      f"{sim_time:.2f}s -- that is a driving fault, not the "
                      f"attack")
            if hits and outcome.collision is None:
                outcome.collision = dict(hits[0])
                outcome.collision["sim_time"] = round(sim_time, 2)
                outcome.collision["ego_speed_mps"] = round(ego_state.speed, 2)
                print(f"[lta] COLLISION at {sim_time:.2f}s with "
                      f"{outcome.collision['with_type_id']}")
                crash_until = sim_time + args.crash_hold
            # Keep simulating briefly past the impact so it is not the last frame.
            if crash_until is not None and sim_time >= crash_until:
                break

        if args.linger > 0:
            print(f"[lta] holding the scene for {args.linger:.0f}s ...")
            hold_until = time.monotonic() + args.linger
            while time.monotonic() < hold_until:
                world.tick()

    finally:
        try:
            if collision_sensor is not None:
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

    if ego_ctrl is not None:
        outcome.ego_state_changes = [[round(t, 2), s] for t, s in ego_ctrl.history]
    if attack is not None:
        outcome.attack = attack.result.to_dict()
    return outcome


def _carla_states(world, actors) -> List[PerceivedObject]:
    """Read the honest ground truth for the scene's actors from one snapshot."""
    out = []
    for a in actors:
        tf = a.get_transform()
        v = a.get_velocity()
        bb = a.bounding_box.extent
        out.append(PerceivedObject(
            object_id=a.id,
            position=(tf.location.x, tf.location.y, tf.location.z),
            velocity=(v.x, v.y, v.z),
            yaw_deg=tf.rotation.yaw,
            dimensions=(bb.x * 2.0, bb.y * 2.0, bb.z * 2.0),
            classification="car"))
    return out


# --------------------------------------------------------------------------- #
# Entry point                                                                  #
# --------------------------------------------------------------------------- #
def _make_sink(args, out_dir: str):
    if args.sink == "file":
        return FileSink(os.path.join(out_dir, "v2x_packets.jsonl"))
    if args.sink == "udp":
        return UdpSink(args.udp_host, args.udp_port)
    return NullSink()


def _one_run(args, run_name: str) -> dict:
    cfg = ScenarioConfig(
        run=run_name, duration_s=args.duration, tick_s=args.tick,
        message_rate_hz=args.rate, ego_back_m=args.ego_back,
        crossing_back_m=args.crossing_back,
        crossing_speed_kmh=args.crossing_speed,
        stop_line_m=args.stop_line,
        drone_back_m=args.drone_back, drone_side_m=args.drone_side,
        drone_height_m=args.drone_height,
        clean_vehicles=not args.keep_vehicles,
        freeze_lights_green=not args.live_lights,
        out_dir=os.path.join(args.out, run_name))
    os.makedirs(cfg.out_dir, exist_ok=True)
    sink = _make_sink(args, cfg.out_dir)
    msg_writer = ReportWriter(cfg.out_dir)
    dec_writer = LeftTurnDecisionWriter(cfg.out_dir)
    trace = None if args.no_trace else TrajectoryWriter(cfg.out_dir)
    try:
        if args.mode == "mock":
            outcome = run_mock(cfg, sink, msg_writer, dec_writer, trace)
        else:
            outcome = run_carla(cfg, sink, msg_writer, dec_writer, args, trace)
    finally:
        sink.close()
        msg_writer.close()
        dec_writer.close()
        if trace is not None:
            trace.close()

    d = outcome.to_dict()
    d["out_dir"] = cfg.out_dir
    d["messages"] = msg_writer.n_messages
    if trace is not None:
        d["trajectory_rows"] = trace.n_rows
    with open(os.path.join(cfg.out_dir, "run_summary.json"), "w") as fh:
        json.dump(d, fh, indent=2)
    return d


def _evidence_problems(results: Dict[str, dict]) -> List[str]:
    """Reasons this run proves nothing, even if it ended in a collision.

    Added after a live run that looked like a success and was not: both runs
    turned at exactly the same instant, every decision row read TURN/TURN/TURN,
    the hazard was never once counted as a conflict, and the spoofed run
    collided purely because two cars happened to occupy the same space. The
    verdict flags said ``attack_caused_collision: true``.

    The earlier occlusion guard could not see this -- it only asks whether the
    ego can *see* the hazard, not whether the hazard was ever *relevant*. These
    checks ask the question that actually matters: did the attack change what
    the ego believed, and was there ever anything real to hide?
    """
    problems = []
    s = results.get(SPOOFED, {})
    h = results.get(HONEST, {})

    for name, r in (("spoofed", s), ("honest", h)):
        if r and r.get("hit_scenery"):
            hit = r["static_collision"]
            problems.append(
                f"the {name} ego hit scenery ({hit['with_type_id']}) at "
                f"{hit['sim_time']}s -- it is not driving the junction "
                f"correctly, so nothing about this run is trustworthy")
    for name, r in (("spoofed", s), ("honest", h)):
        if r and not r.get("turn_attempted"):
            problems.append(
                f"the {name} ego NEVER TURNED -- it never left the approach. "
                f"Its trajectory.csv will show where it actually went; a common "
                f"cause is a spawn that landed on the wrong road, so the car "
                f"drives to a stop line that is not there and waits out the run")
    if s.get("n_flips", 0) == 0:
        problems.append(
            "the spoofed run produced ZERO decision flips: the forged messages "
            "never changed what the ego concluded, so nothing it did can be "
            "attributed to the attack")
    if not s.get("turn_was_unsafe", False) and s.get("collided"):
        problems.append(
            "the spoofed run collided but ground truth said the turn was SAFE "
            "when it committed -- the collision is not the attack's doing")
    if (h.get("turn_started_s") is not None
            and h.get("turn_started_s") == s.get("turn_started_s")):
        problems.append(
            f"both runs committed at the same instant "
            f"({s.get('turn_started_s')}s), so the message stream changed "
            f"nothing about the manoeuvre")
    if not s.get("attack", {}).get("removed_object_ids"):
        problems.append(
            "the forgery deleted nothing: an impersonated CPM supersedes the "
            "RSU's whether or not it was edited, so any effect here is not "
            "suppression")
    return problems


def _verdict(results: Dict[str, dict]) -> dict:
    """The headline comparison: did the attack, and only the attack, cause it?"""
    h, s = results.get(HONEST), results.get(SPOOFED)
    if not (h and s):
        return {}
    return {
        "honest_turn": h["turn_attempted"],
        "spoofed_turn": s["turn_attempted"],
        # Turning is not itself a failure -- the honest ego is expected to turn
        # once the junction really is clear. The failure is turning while a
        # conflicting vehicle is genuinely approaching.
        "honest_aborted_turns": h["aborted_turns"],
        "spoofed_aborted_turns": s["aborted_turns"],
        "honest_unsafe_turn": h["turn_was_unsafe"],
        "spoofed_unsafe_turn": s["turn_was_unsafe"],
        "honest_collision": h["collided"],
        "spoofed_collision": s["collided"],
        "attack_caused_unsafe_turn": (s["turn_was_unsafe"]
                                      and not h["turn_was_unsafe"]),
        "attack_caused_collision": s["collided"] and not h["collided"],
    }


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Left Turn Assist spoofing: a drone impersonates an RSU to "
                    "suppress a crossing vehicle and trigger an unsafe left turn.")
    p.add_argument("--mode", choices=["mock", "carla"], default="mock")
    p.add_argument("--run", choices=[HONEST, SPOOFED, "both"], default="both",
                   help="honest baseline, the attack, or both for the comparison")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=15.0,
                   help="CARLA RPC timeout in seconds for ordinary calls")
    p.add_argument("--crash-hold", type=float, default=3.0,
                   help="seconds to keep simulating after a collision, so the "
                        "crash is visible rather than the last frame")
    p.add_argument("--linger", type=float, default=7.0,
                   help="seconds to hold the finished scene before destroying "
                        "the vehicles (0 = tear down at once)")
    p.add_argument("--wall-timeout", type=float, default=600.0,
                   help="hard wall-clock budget for one run")
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--tick", type=float, default=0.05,
                   help="control period and sim fixed delta, seconds")
    p.add_argument("--rate", type=float, default=1.0, help="CPM rate in Hz")
    p.add_argument("--ego-back", type=float, default=80.0,
                   help="ego start distance back along its approach arm, metres. "
                        "Big on purpose: the ego drives to the junction, it is "
                        "never spawned next to it")
    p.add_argument("--crossing-back", type=float, default=None,
                   help="hazard start distance back along the cross street, "
                        "metres. Default: derived so it reaches the junction "
                        "just after the ego does, whatever the approach length")
    p.add_argument("--stop-line", type=float, default=6.0,
                   help="stop line distance back from the junction, metres")
    p.add_argument("--crossing-speed", type=float, default=35.0, help="km/h")
    p.add_argument("--drone-back", type=float, default=28.0,
                   help="drone hover distance back down the ego's approach, "
                        "metres (default %(default)s). The surveyed pose from "
                        "the reference scene sits on top of the junction, which "
                        "is unusable as a camera")
    p.add_argument("--drone-side", type=float, default=6.0,
                   help="drone offset to the right of the approach, metres")
    p.add_argument("--drone-height", type=float, default=20.0,
                   help="drone hover altitude, metres")
    p.add_argument("--survey", action="store_true",
                   help="print the scene's real geometry and exit, changing "
                        "nothing: the surveyed spawn points, the junction each "
                        "car actually reaches, that junction's arms, and which "
                        "exit is the left turn. Run this FIRST when anything "
                        "looks wrong -- every failure so far has been geometry "
                        "that was inferred and never checked")
    p.add_argument("--ego-spawn", type=int, default=None,
                   help="CARLA spawn point index for the ego. Default: the "
                        f"spawn with the longest approach into the reference "
                        f"junction. Pass {REFERENCE_SPAWN_POINTS[0]} for the "
                        f"reference scene's own spawn, which on this build sits "
                        f"only 16 m from the line")
    p.add_argument("--hazard-spawn", type=int, default=None,
                   help="CARLA spawn point index for the conflicting vehicle. "
                        "Default: chosen from the map -- a spawn that actually "
                        "feeds the ego's junction from a conflicting direction. "
                        "The reference scene's index (33) is 0.9.13 numbering "
                        "and lands 154 m away on this build")
    p.add_argument("--allow-map-reload", action="store_true",
                   help="let the scenario call load_world() if the simulator is "
                        "on the wrong map (default: stop and say how to start it "
                        "correctly; the reload is unreliable in this build and "
                        "restarts the simulator as a side effect)")
    p.add_argument("--live-lights", action="store_true",
                   help="let Town10's traffic light cycle run instead of freezing "
                        "the junction green (default: freeze, so a red phase "
                        "cannot stop the hazard for reasons unrelated to the attack)")
    p.add_argument("--no-fly-drone", action="store_true",
                   help="leave the AirSim drone wherever it is (default: place it "
                        "at the roadside fake-RSU pose before the scene starts)")
    p.add_argument("--drone-timeout", type=float, default=30.0,
                   help="seconds to let the drone settle into its hover")
    p.add_argument("--keep-vehicles", action="store_true",
                   help="leave pre-existing vehicles in place (default: clear them)")
    p.add_argument("--no-trace", action="store_true",
                   help="skip trajectory.csv (the per-tick pose, control and "
                        "pure-pursuit target log). It is on by default: it is "
                        "the only view of what the controller actually did")
    p.add_argument("--sink", choices=["file", "udp", "null"], default="file")
    p.add_argument("--out", default="out/left_turn")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=47000)
    args = p.parse_args(argv)

    if args.survey:
        if args.mode != "carla":
            raise SystemExit("--survey needs a live simulator: "
                             "run it with --mode carla")
        return run_survey(ScenarioConfig(), args)

    runs = [HONEST, SPOOFED] if args.run == "both" else [args.run]
    results = {r: _one_run(args, r) for r in runs}
    problems = _evidence_problems(results)
    summary = {"mode": args.mode, "map": MAP_NAME if args.mode == "carla" else "mock",
               "runs": results, "verdict": _verdict(results),
               "evidence_valid": not problems, "evidence_problems": problems}
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "comparison.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print(json.dumps(summary, indent=2))

    for name, r in results.items():
        print(f"\n[{name}] turn={r['turn_attempted']} "
              f"collision={r['collided']} flips={r['n_flips']}/{r['n_decisions']} "
              f"-> {r['out_dir']}")

    if problems:
        print("\n" + "=" * 72)
        print("THIS RUN IS NOT EVIDENCE OF THE ATTACK, whatever the verdict says:")
        for prob in problems:
            print(f"  - {prob}")
        print("Check out/left_turn/<run>/trajectory.csv to see what actually "
              "moved, and left_turn_decisions.csv to see what the ego believed.")
        print("=" * 72)
    return 0 if not problems else 2


if __name__ == "__main__":
    raise SystemExit(main())
