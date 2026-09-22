"""Left Turn Assist spoofing: decision logic, occlusion, and the closed loop."""
import json
import math

import pytest

from carla_spoofing.attacks.identity_spoof import ForgedIdentity, IdentitySpoofAttack
from carla_spoofing.attacks.remove_object import RemoveObjectAttack, RemoveTarget
from carla_spoofing.control import PolylineReference, hermite_turn_path
from carla_spoofing.fusion import (
    StaticOccluder, fuse_latest_by_station, occluders_near, segment_intersects_box,
    visible_objects,
)
from carla_spoofing.geometry import EgoState
from carla_spoofing.left_turn_assist import (
    DO_NOT_TURN, TURN, LeftTurnThresholds, evaluate_left_turn,
)
from carla_spoofing.perception import build_cpm_from_objects
from carla_spoofing.v2x.cpm import PerceivedObject

EGO, CROSSING = 201, 202
RSU = 9001
# Ego at the stop line heading +X; conflict point where its left turn crosses
# the cross street. The hazard comes up that street from -Y (the ego's left).
STOP_LINE = (-6.0, 0.0, 0.0)
CONFLICT = (0.0, -3.5, 0.0)


def _ego(speed=0.0):
    return EgoState(station_id=EGO, position=STOP_LINE, yaw_deg=0.0,
                    velocity=(speed, 0.0, 0.0), desired_speed_mps=8.3)


def _crossing(distance_m, speed_mps=9.7):
    """Hazard ``distance_m`` short of the conflict point, closing at ``speed``."""
    return PerceivedObject(object_id=CROSSING,
                           position=(CONFLICT[0], CONFLICT[1] - distance_m, 0.0),
                           velocity=(0.0, speed_mps, 0.0), yaw_deg=90.0,
                           classification="car")


# --------------------------------------------------------------------------- #
# The decision                                                                 #
# --------------------------------------------------------------------------- #
def test_close_fast_vehicle_forbids_the_turn():
    d = evaluate_left_turn(_ego(), [_crossing(30.0)], CONFLICT)
    assert d.decision == DO_NOT_TURN
    assert d.conflicting_object_id == CROSSING
    assert d.time_to_arrival_s == pytest.approx(30.0 / 9.7, rel=1e-3)


def test_distant_vehicle_allows_the_turn():
    d = evaluate_left_turn(_ego(), [_crossing(120.0)], CONFLICT)
    assert d.decision == TURN
    assert d.turn_intent is True


def test_empty_junction_allows_the_turn():
    d = evaluate_left_turn(_ego(), [], CONFLICT)
    assert d.decision == TURN
    assert d.n_conflicting == 0


def test_stopped_vehicle_on_the_conflict_point_still_forbids_it():
    """Time-to-arrival is infinite for a stopped car; distance must catch it.

    Without ``min_gap_m`` a vehicle sitting across the junction mouth scores an
    infinite gap and the ego turns straight into it.
    """
    stalled = PerceivedObject(object_id=CROSSING,
                              position=(CONFLICT[0], CONFLICT[1] - 4.0, 0.0),
                              velocity=(0.0, 0.0, 0.0), yaw_deg=90.0,
                              classification="car")
    d = evaluate_left_turn(_ego(), [stalled], CONFLICT)
    assert d.decision == DO_NOT_TURN
    assert math.isinf(d.time_to_arrival_s)


def test_vehicle_travelling_our_way_is_not_a_conflict():
    """A car alongside us going the same way crosses nothing."""
    same_way = PerceivedObject(object_id=CROSSING, position=(10.0, 3.5, 0.0),
                               velocity=(8.0, 0.0, 0.0), yaw_deg=0.0,
                               classification="car")
    d = evaluate_left_turn(_ego(), [same_way], CONFLICT)
    assert d.decision == TURN
    assert d.n_conflicting == 0


def test_vehicle_already_through_the_junction_is_not_a_conflict():
    gone = PerceivedObject(object_id=CROSSING,
                           position=(CONFLICT[0], CONFLICT[1] + 25.0, 0.0),
                           velocity=(0.0, 9.7, 0.0), yaw_deg=90.0,
                           classification="car")
    d = evaluate_left_turn(_ego(), [gone], CONFLICT)
    assert d.decision == TURN


def test_pedestrian_is_not_a_vehicle_conflict():
    walker = PerceivedObject(object_id=CROSSING,
                             position=(CONFLICT[0], CONFLICT[1] - 8.0, 0.0),
                             velocity=(0.0, 1.4, 0.0), yaw_deg=90.0,
                             classification="pedestrian")
    d = evaluate_left_turn(_ego(), [walker], CONFLICT)
    assert d.decision == TURN


def test_turn_intent_is_false_once_past_the_junction():
    past = EgoState(station_id=EGO, position=(30.0, 0.0, 0.0), yaw_deg=0.0,
                    velocity=(8.0, 0.0, 0.0))
    d = evaluate_left_turn(past, [_crossing(20.0)], CONFLICT)
    assert d.turn_intent is False


# --------------------------------------------------------------------------- #
# The attack                                                                   #
# --------------------------------------------------------------------------- #
def _rsu_cpm(states):
    return build_cpm_from_objects(RSU, (8.0, 8.0, 6.0), states, 0.0,
                                  perception_range=160.0, station_type="rsu")


def test_suppression_flips_the_decision():
    states = [_crossing(30.0)]
    rsu = _rsu_cpm(states)
    attack = IdentitySpoofAttack(
        ForgedIdentity(RSU, (8.0, 8.0, 6.0), "rsu"),
        inner=RemoveObjectAttack(RemoveTarget(object_id=CROSSING, carve_lidar=False)))
    forged = attack.apply_cpm(rsu)

    honest = evaluate_left_turn(_ego(), fuse_latest_by_station([rsu]).objects, CONFLICT)
    spoofed = evaluate_left_turn(
        _ego(), fuse_latest_by_station([rsu, forged]).objects, CONFLICT)
    assert honest.decision == DO_NOT_TURN
    assert spoofed.decision == TURN


def test_forging_under_the_drones_own_id_does_not_work():
    """The load-bearing detail: the forgery must SUPERSEDE the RSU's report.

    A message from a new station is unioned with the RSU's, so deleting an
    object from it changes nothing -- the genuine report still carries the
    hazard. Impersonation is what makes suppression possible, not a flourish.
    """
    states = [_crossing(30.0)]
    rsu = _rsu_cpm(states)
    naive = RemoveObjectAttack(
        RemoveTarget(object_id=CROSSING, carve_lidar=False)).apply_cpm(
            build_cpm_from_objects(9002, (8.0, 8.0, 15.0), states, 0.0,
                                   perception_range=160.0, station_type="drone"))
    fused = fuse_latest_by_station([rsu, naive])
    assert evaluate_left_turn(_ego(), fused.objects, CONFLICT).decision == DO_NOT_TURN


# --------------------------------------------------------------------------- #
# Occlusion                                                                    #
# --------------------------------------------------------------------------- #
def test_segment_box_intersection_basics():
    box = dict(center=(0.0, 0.0, 0.0), extent=(5.0, 5.0, 10.0))
    assert segment_intersects_box((-20, 0, 0), (20, 0, 0), **box)      # through
    assert not segment_intersects_box((-20, 20, 0), (20, 20, 0), **box)  # past
    assert not segment_intersects_box((-20, 0, 0), (-10, 0, 0), **box)  # stops short
    assert segment_intersects_box((0, 0, 0), (1, 0, 0), **box)          # inside


def test_rotating_a_box_changes_what_it_blocks():
    thin = dict(center=(0.0, 0.0, 0.0), extent=(10.0, 0.5, 10.0))
    # Long axis along X: a sightline down Y at x=8 is blocked.
    assert segment_intersects_box((8, -20, 0), (8, 20, 0), **thin)
    # Rotated 90 deg the long axis is along Y, and that same line misses.
    assert not segment_intersects_box((8, -20, 0), (8, 20, 0), yaw_deg=90.0, **thin)


def test_corner_building_is_load_bearing():
    """Remove the building and the ego sees the hazard itself -- scenario dead.

    This is the left-turn equivalent of the do-not-pass occlusion test. If the
    corner does not hide the cross street, suppressing the RSU's report changes
    nothing the ego believes, and a collision would prove nothing about the
    attack.
    """
    hazard = _crossing(26.0)
    building = StaticOccluder(center=(-12.0, -14.0, 0.0), extent=(9.0, 9.0, 10.0))

    hidden = visible_objects(STOP_LINE, [hazard], [], static_occluders=[building])
    seen = visible_objects(STOP_LINE, [hazard], [], static_occluders=[])
    assert hidden == []
    assert [o.object_id for o in seen] == [CROSSING]


def test_occluders_near_filters_by_distance():
    boxes = [StaticOccluder(center=(0.0, 0.0, 0.0), extent=(1.0, 1.0, 1.0)),
             StaticOccluder(center=(500.0, 0.0, 0.0), extent=(1.0, 1.0, 1.0))]
    assert len(occluders_near(boxes, (0.0, 0.0, 0.0), 50.0)) == 1


# --------------------------------------------------------------------------- #
# The turn path                                                                #
# --------------------------------------------------------------------------- #
def test_turn_path_leaves_and_arrives_on_the_right_headings():
    pts = hermite_turn_path((0, 0, 0), 0.0, (12, -12, 0), -90.0)
    ref = PolylineReference(points=pts)
    assert pts[0] == (0, 0, 0)
    assert pts[-1] == pytest.approx((12, -12, 0))
    # Leaves along +X ...
    assert pts[1][0] > pts[0][0] and abs(pts[1][1]) < 0.5
    # ... and arrives heading -Y (a left turn in CARLA's left-handed frame).
    assert pts[-1][1] < pts[-2][1] and abs(pts[-1][0] - pts[-2][0]) < 0.5
    assert ref.length > 12.0
    assert ref.remaining((0, 0, 0)) == pytest.approx(ref.length)


def test_turn_path_extrapolates_past_the_end():
    """The controller must keep a sane reference while it drives out."""
    ref = PolylineReference(points=hermite_turn_path((0, 0, 0), 0.0, (12, -12, 0), -90.0))
    beyond = ref.point_ahead((12, -12, 0), -90.0, 10.0)
    assert beyond[1] < -12.0


# --------------------------------------------------------------------------- #
# End to end                                                                   #
# --------------------------------------------------------------------------- #
def test_attack_causes_an_unsafe_turn(tmp_path):
    """Identical scene, identical controller, only the messages differ.

    The claim asserted here is the **unsafe turn**, not a collision. The ego
    pulling out across a vehicle that is genuinely there is the attack landing;
    whether that ends in contact depends on geometry the attacker does not
    control. The mock ego currently recovers -- its own sensors acquire the
    hazard once it clears the corner and it stops in the junction -- which is
    honest emergent behaviour, not a failure, and must not be tuned away.
    """
    from carla_spoofing.scenarios.left_turn_spoofing import main
    assert main(["--mode", "mock", "--run", "both", "--sink", "null",
                 "--out", str(tmp_path)]) == 0
    c = json.loads((tmp_path / "comparison.json").read_text())
    verdict = c["verdict"]
    assert verdict["attack_caused_unsafe_turn"] is True
    assert verdict["honest_unsafe_turn"] is False
    assert verdict["honest_collision"] is False
    # The honest ego is EXPECTED to turn -- once the junction is really clear.
    assert verdict["honest_turn"] is True
    # ... and the run must certify itself as actual evidence.
    assert c["evidence_valid"] is True, c["evidence_problems"]


def test_a_run_that_proves_nothing_says_so(tmp_path):
    """The guard that the first live run needed and did not have.

    A run where the forged messages never changed the ego's mind is not
    evidence, however it ended. Reproduced by starting the hazard so far back
    that it never reaches the junction -- the live failure mode exactly.
    """
    from carla_spoofing.scenarios.left_turn_spoofing import main
    rc = main(["--mode", "mock", "--run", "both", "--sink", "null",
               "--crossing-back", "600", "--out", str(tmp_path)])
    c = json.loads((tmp_path / "comparison.json").read_text())
    assert c["evidence_valid"] is False
    assert any("ZERO decision flips" in p for p in c["evidence_problems"])
    assert rc != 0, "a hollow run must not exit 0"


def test_both_runs_commit_from_the_same_place(tmp_path):
    """The runs must differ in beliefs, not in geometry.

    Both egos reach the stop line at the same moment; only how long they wait
    there differs. If the spoofed ego set off from further back the comparison
    would confound position with belief.
    """
    from carla_spoofing.scenarios.left_turn_spoofing import main
    assert main(["--mode", "mock", "--run", "both", "--sink", "null",
                 "--out", str(tmp_path)]) == 0
    c = json.loads((tmp_path / "comparison.json").read_text())

    def waiting_at(run):
        return next(t for t, s in c["runs"][run]["ego_state_changes"] if s == "WAITING")

    assert waiting_at("honest") == pytest.approx(waiting_at("spoofed"))
    # ... and the attack makes it go sooner.
    assert c["runs"]["spoofed"]["turn_started_s"] < c["runs"]["honest"]["turn_started_s"]
    # The spoofed ego commits the moment it reaches the line; the honest one
    # waits there for a real gap.
    assert c["runs"]["spoofed"]["turn_started_s"] == pytest.approx(
        waiting_at("spoofed"))


def test_attack_actually_removed_something(tmp_path):
    """Guard against a vacuous forgery -- the lesson from the do-not-pass runs.

    A collision is not evidence on its own: an impersonated CPM supersedes the
    RSU's whether or not the attacker edited it, so the run must also show the
    hazard really was deleted.
    """
    from carla_spoofing.scenarios.left_turn_spoofing import main
    assert main(["--mode", "mock", "--run", "spoofed", "--sink", "null",
                 "--out", str(tmp_path)]) == 0
    summary = json.loads((tmp_path / "spoofed" / "run_summary.json").read_text())
    assert summary["attack"]["removed_object_ids"], "forgery deleted nothing"


# --------------------------------------------------------------------------- #
# Junction arm selection — the bug that sent the ego across the junction       #
# --------------------------------------------------------------------------- #
def _four_way(ego_yaw=0.0):
    """Movements through a square crossroads, as CARLA would report them.

    Arms at ego_yaw + 0 / 90 / 180 / 270; each arm has a left, straight and
    right movement. In CARLA's left-handed frame, left of a heading is -90.
    """
    out = []
    for arm in (0.0, 90.0, 180.0, 270.0):
        entry = ego_yaw + arm
        for turn in (-90.0, 0.0, 90.0):          # left, straight, right
            out.append((entry, entry + turn))
    return out


@pytest.mark.parametrize("ego_yaw", [0.0, 90.0, -90.0, 180.0, 37.0, -143.0])
def test_left_exit_comes_from_our_own_arm(ego_yaw):
    """The regression: it must enter on OUR arm *and* exit left.

    The first live run picked the most-leftward exit across every movement
    without checking whose arm it started on, so the ego steered at a point on
    someone else's turn and drove across the junction.
    """
    from carla_spoofing.scenarios.left_turn_spoofing import pick_left_exit
    movements = _four_way(ego_yaw)
    i = pick_left_exit(ego_yaw, movements)
    assert i is not None
    entry_yaw, exit_yaw = movements[i]
    assert math.cos(math.radians(entry_yaw - ego_yaw)) > 0.8, "wrong arm"
    assert math.cos(math.radians(exit_yaw - (ego_yaw - 90.0))) > 0.9, "not a left turn"


def test_no_left_exit_is_reported_rather_than_guessed():
    """A T-junction arm with no left turn must return None, not a bad guess."""
    from carla_spoofing.scenarios.left_turn_spoofing import pick_left_exit
    # Our arm can only go straight or right.
    assert pick_left_exit(0.0, [(0.0, 0.0), (0.0, 90.0), (90.0, 0.0)]) is None


def test_crossing_arm_comes_from_the_left():
    from carla_spoofing.scenarios.left_turn_spoofing import pick_crossing_arm
    # Ego heading 0 (+X). +Y is the ego's RIGHT, so an arm whose traffic heads
    # +Y (yaw 90) is coming FROM the left. yaw 270 comes from the right.
    assert pick_crossing_arm(0.0, [0.0, 90.0, 180.0, 270.0]) == 1
    # Parallel arms are never crossing arms.
    assert pick_crossing_arm(0.0, [0.0, 180.0]) is None


@pytest.mark.parametrize("ego_yaw", [0.0, 45.0, 90.0, -120.0])
def test_crossing_arm_is_perpendicular_to_us(ego_yaw):
    from carla_spoofing.scenarios.left_turn_spoofing import pick_crossing_arm
    arms = [ego_yaw, ego_yaw + 90.0, ego_yaw + 180.0, ego_yaw + 270.0]
    j = pick_crossing_arm(ego_yaw, arms)
    assert j is not None
    assert abs(math.cos(math.radians(arms[j] - ego_yaw))) < 0.35


# --------------------------------------------------------------------------- #
# Trajectory trace                                                             #
# --------------------------------------------------------------------------- #
def test_trace_shows_a_real_left_turn(tmp_path):
    """The trace has to be good enough to debug a wrong turn from.

    Asserts what a human would check by eye: the ego's heading sweeps left, its
    pure-pursuit target advances along the turn path rather than sticking, and
    the path progress increases monotonically.
    """
    import csv
    from carla_spoofing.scenarios.left_turn_spoofing import main
    assert main(["--mode", "mock", "--run", "spoofed", "--sink", "null",
                 "--out", str(tmp_path)]) == 0
    rows = [r for r in csv.DictReader(
        (tmp_path / "spoofed" / "trajectory.csv").open())
        if r["role"] == "ego" and r["ctrl_state"] == "TURNING"]
    assert len(rows) > 10

    yaws = [float(r["yaw_deg"]) for r in rows]
    assert yaws[-1] < yaws[0] - 30.0, "heading did not sweep left"

    # The target must move along the path, not sit still.
    targets = {(r["target_x"], r["target_y"]) for r in rows}
    assert len(targets) > 5, "pure-pursuit target is stuck"

    progress = [float(r["path_progress_m"]) for r in rows if r["path_progress_m"]]
    assert progress == sorted(progress), "path progress went backwards"
    assert progress[-1] > progress[0]


def test_trace_can_be_switched_off(tmp_path):
    from carla_spoofing.scenarios.left_turn_spoofing import main
    assert main(["--mode", "mock", "--run", "spoofed", "--sink", "null",
                 "--no-trace", "--out", str(tmp_path)]) == 0
    assert not (tmp_path / "spoofed" / "trajectory.csv").exists()


# --------------------------------------------------------------------------- #
# Road-graph branch picking — the bug that lost the cybertruck                 #
# --------------------------------------------------------------------------- #
class _FakeWaypoint:
    """Minimal stand-in for carla.Waypoint: a heading and a predecessor list."""

    def __init__(self, yaw, prev=None):
        self.yaw = yaw
        self._prev = prev or []

    @property
    def transform(self):
        return type("T", (), {"rotation": type("R", (), {"yaw": self.yaw})()})()

    def previous(self, _d):
        return list(self._prev)


def test_straightest_prefers_carrying_on():
    from carla_spoofing.control import straightest
    from carla_spoofing.geometry import heading_unit
    branches = [_FakeWaypoint(90.0), _FakeWaypoint(2.0), _FakeWaypoint(-90.0)]
    assert straightest(branches, heading_unit(0.0)).yaw == 2.0
    assert straightest(branches, heading_unit(90.0)).yaw == 90.0
    assert straightest([], heading_unit(0.0)) is None


def test_walk_back_stays_on_the_same_road():
    """At a branch, keep going straight back rather than taking an arbitrary one.

    The regression: `previous(80.0)` in a single hop branches at every junction
    it crosses and returns an arbitrary member of the result. That put the ego
    64 m away on a perpendicular street, where it drove to a stop line that did
    not exist and waited out the entire run.
    """
    from carla_spoofing.control import walk_back
    # A straight road heading 0 deg, with a side street joining at every step.
    chain = _FakeWaypoint(0.0)
    for _ in range(50):
        chain = _FakeWaypoint(0.0, prev=[_FakeWaypoint(90.0), chain])
    # Note the tempting wrong branch is listed FIRST, as CARLA may well do.
    end, travelled = walk_back(chain, 20.0)
    assert travelled == pytest.approx(20.0)
    assert end.yaw == pytest.approx(0.0), "walked onto the side street"


def test_walk_back_reports_a_short_walk_rather_than_lying():
    from carla_spoofing.control import walk_back
    dead_end = _FakeWaypoint(0.0, prev=[])
    end, travelled = walk_back(dead_end, 50.0)
    assert travelled == 0.0
    assert end is dead_end


def test_a_run_where_the_ego_never_turns_is_flagged():
    from carla_spoofing.scenarios.left_turn_spoofing import _evidence_problems
    stuck = {"spoofed": {"turn_attempted": False, "n_flips": 3,
                         "attack": {"removed_object_ids": [1]}},
             "honest": {"turn_attempted": True, "turn_started_s": 9.0}}
    problems = _evidence_problems(stuck)
    assert any("NEVER TURNED" in p for p in problems)


def test_crossing_arm_threshold_stays_below_the_decision_threshold():
    """An arm the scene accepts must be one the decision logic can conflict with.

    If `pick_crossing_arm` accepted an arm at, say, 55 deg while
    `evaluate_left_turn` treats anything within 60 deg as "travelling our way",
    the hazard would be placed and driven and never once counted as a conflict:
    a scene that runs perfectly and proves nothing. That is precisely the hollow
    run this scenario has already produced once.
    """
    from carla_spoofing.scenarios.left_turn_spoofing import pick_crossing_arm
    from carla_spoofing.left_turn_assist import LeftTurnThresholds
    import inspect
    sig = inspect.signature(pick_crossing_arm)
    assert sig.parameters["max_parallel"].default < \
        LeftTurnThresholds().same_direction_cos


def test_skewed_junction_arm_is_still_accepted():
    from carla_spoofing.scenarios.left_turn_spoofing import pick_crossing_arm
    # A 65-degree skew crossing from the left: real junctions are not square.
    assert pick_crossing_arm(0.0, [0.0, 65.0, 180.0]) == 1
    # But something almost parallel is still rejected.
    assert pick_crossing_arm(0.0, [0.0, 25.0, 180.0]) is None


def test_oncoming_fallback_when_no_left_arm():
    """A T-junction with no left arm falls back to the oncoming stream."""
    from carla_spoofing.scenarios.left_turn_spoofing import (
        pick_crossing_arm, pick_oncoming_arm)
    arms = [0.0, 180.0, 270.0]          # ours, oncoming, crossing from the right
    assert pick_crossing_arm(0.0, arms) is None
    assert pick_oncoming_arm(0.0, arms) == 1


def test_arm_description_names_the_bearings():
    from carla_spoofing.scenarios.left_turn_spoofing import describe_arms
    lines = describe_arms(0.0, [0.0, 90.0, 180.0, 270.0])
    assert "same way as us" in lines[0]
    assert "LEFT" in lines[1]
    assert "oncoming" in lines[2]
    assert "RIGHT" in lines[3]
    # And it works off a non-zero ego heading, which is the case that matters.
    rotated = describe_arms(37.0, [37.0, 127.0, 217.0, 307.0])
    assert "same way as us" in rotated[0] and "LEFT" in rotated[1]


def test_classify_approach_matches_the_arm_description():
    """The two must agree, or the survey prints one thing and the code does another."""
    from carla_spoofing.scenarios.left_turn_spoofing import (
        classify_approach, describe_arms)
    for ego_yaw in (0.0, 89.8, -143.0):
        arms = [ego_yaw, ego_yaw + 90.0, ego_yaw + 180.0, ego_yaw - 90.0]
        kinds = [classify_approach(ego_yaw, y) for y in arms]
        assert kinds == ["same", "left", "oncoming", "right"]
        lines = describe_arms(ego_yaw, arms)
        assert "LEFT" in lines[1] and "RIGHT" in lines[3]


def test_classify_approach_handles_unnormalised_angles():
    """CARLA hands back yaws like -450.2; they must not read as a new direction.

    Seen in a live survey: an arm reported yaw -450.2, which is -90.2.
    """
    from carla_spoofing.scenarios.left_turn_spoofing import classify_approach
    assert classify_approach(89.8, -450.2) == classify_approach(89.8, -90.2)
    assert classify_approach(89.8, -450.2) == "oncoming"


def test_choose_hazard_spawn_prefers_left_then_oncoming_and_longest_run_up():
    """Selection policy, exercised without a simulator via a stub."""
    from carla_spoofing.scenarios.left_turn_spoofing import choose_hazard_spawn
    import carla_spoofing.scenarios.left_turn_spoofing as lt

    # (index, run_up_m, approach_yaw) against an ego approach of 90 deg.
    table = [(1, 20.0, 180.0),    # left, short
             (2, 60.0, 180.0),    # left, long   <- want this
             (3, 90.0, -90.0),    # oncoming, longest overall
             (4, 80.0, 0.0)]      # from the right: never a left-turn conflict
    original = lt.spawns_reaching
    lt.spawns_reaching = lambda *a, **k: table
    try:
        idx, dist, yaw, kind = choose_hazard_spawn(
            None, None, None, (0.0, 0.0), 90.0)
        assert (idx, kind) == (2, "left")
        # With no left-hand road at all, fall back to oncoming, not the right.
        lt.spawns_reaching = lambda *a, **k: [t for t in table if t[0] in (3, 4)]
        idx, _d, _y, kind = choose_hazard_spawn(None, None, None, (0.0, 0.0), 90.0)
        assert (idx, kind) == (3, "oncoming")
        # Nothing conflicting at all -> None, rather than a silly choice.
        lt.spawns_reaching = lambda *a, **k: [(4, 80.0, 0.0)]
        assert choose_hazard_spawn(None, None, None, (0.0, 0.0), 90.0) is None
    finally:
        lt.spawns_reaching = original


def test_choose_ego_spawn_takes_the_longest_approach_on_our_own_road():
    """The reference ego spawn is 16 m from the line on this build.

    Other spawns sit further back on the same road into the same junction. The
    car should start from one of those and drive in, not appear at the stop
    line -- a complaint the user made twice.
    """
    import carla_spoofing.scenarios.left_turn_spoofing as lt
    table = [(126, 16.0, 89.8),    # the reference spawn
             (151, 42.0, 89.8),    # same approach, much further back  <- want
             (46, 50.0, 180.2),    # a LEFT-crossing road: not ours
             (106, 2.0, 180.2)]
    original = lt.spawns_reaching
    lt.spawns_reaching = lambda *a, **k: table
    try:
        assert lt.choose_ego_spawn(None, None, None, (0.0, 0.0), 89.8) == (151, 42.0)
        # The ego and the hazard must never be handed the same road.
        ego = lt.choose_ego_spawn(None, None, None, (0.0, 0.0), 89.8)
        haz = lt.choose_hazard_spawn(None, None, None, (0.0, 0.0), 89.8)
        assert ego[0] != haz[0]
        assert haz[0] == 46 and haz[3] == "left"
        # No road on our own approach -> None rather than a wrong-road guess.
        lt.spawns_reaching = lambda *a, **k: [(46, 50.0, 180.2)]
        assert lt.choose_ego_spawn(None, None, None, (0.0, 0.0), 89.8) is None
    finally:
        lt.spawns_reaching = original
