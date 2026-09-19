import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from carla_spoofing.attacks import (
    ForgedIdentity, IdentitySpoofAttack, RemoveObjectsAttack, RemoveTarget)
from carla_spoofing.do_not_pass_warning import EgoState
from carla_spoofing.fusion import fuse_latest_by_station
from carla_spoofing.perception import ObjectState, build_cpm_from_objects
from carla_spoofing.vru_warning import GO, STOP, evaluate_crossing

EGO, CYCLIST, PEDESTRIAN = 111, 112, 113


def _scene(cyclist_y=9.0, pedestrian_y=10.5):
    """Ego at the origin heading +X toward a crossing at x=45; the cyclist and
    the pedestrian close in on it from the side street (heading -Y)."""
    return [
        ObjectState(EGO, (0.0, 0.0, 0.0), velocity=(8.33, 0.0, 0.0), yaw_deg=0.0),
        ObjectState(CYCLIST, (45.0, cyclist_y, 0.0), velocity=(0.0, -3.89, 0.0),
                    yaw_deg=-90.0, classification="bicycle"),
        ObjectState(PEDESTRIAN, (45.0, pedestrian_y, 0.0), velocity=(0.0, -1.25, 0.0),
                    yaw_deg=-90.0, classification="pedestrian"),
    ]


def _ego(desired=8.33):
    return EgoState(EGO, (0.0, 0.0, 0.0), yaw_deg=0.0, velocity=(8.33, 0.0, 0.0),
                    desired_speed_mps=desired)


def _objects(states, station_id=EGO, position=(0.0, 0.0, 0.0), rng=float("inf")):
    return build_cpm_from_objects(station_id, position, states, 0.0,
                                  perception_range=rng).perceived_objects


def test_cyclist_and_pedestrian_forbid_the_crossing():
    d = evaluate_crossing(_ego(), _objects(_scene()))
    assert d.decision == STOP
    assert d.blocking_object_id in (CYCLIST, PEDESTRIAN)


def test_clear_crossing_allows_going():
    states = [s for s in _scene() if s.object_id not in (CYCLIST, PEDESTRIAN)]
    d = evaluate_crossing(_ego(), _objects(states))
    assert d.decision == GO


def test_vru_already_past_is_not_a_hazard():
    """A VRU well clear of the ego's lane on the far side is no longer a conflict."""
    states = _scene(cyclist_y=-20.0, pedestrian_y=-20.0)
    assert evaluate_crossing(_ego(), _objects(states)).decision == GO


def test_vru_moving_away_off_to_the_side_is_not_a_hazard():
    """Stationary/receding VRUs off in the side street are not a conflict either."""
    states = _scene()
    states[1] = ObjectState(CYCLIST, (45.0, 9.0, 0.0), velocity=(0.0, 3.89, 0.0),
                            yaw_deg=90.0, classification="bicycle")   # moving away
    states[2] = ObjectState(PEDESTRIAN, (45.0, 10.5, 0.0), velocity=(0.0, 1.25, 0.0),
                            yaw_deg=90.0, classification="pedestrian")  # moving away
    assert evaluate_crossing(_ego(), _objects(states)).decision == GO


def test_vehicle_on_the_cross_street_is_not_a_vru_hazard():
    """This warning is about VRUs; a car in the same spot is somebody else's problem."""
    states = _scene()
    states[1] = ObjectState(CYCLIST, (45.0, 9.0, 0.0), velocity=(0.0, -3.89, 0.0),
                            yaw_deg=-90.0, classification="car")
    d = evaluate_crossing(_ego(), _objects([s for s in states if s.object_id != PEDESTRIAN]))
    assert d.decision == GO


def test_removing_both_targets_suppresses_the_hazard():
    """The crux of the two-victim attack: one forged message erases both VRUs."""
    states = _scene()
    rsu = build_cpm_from_objects(9101, (51.0, 4.0, 6.0), states, 0.0,
                                 perception_range=160.0, station_type="rsu")
    drone = build_cpm_from_objects(9102, (41.0, 6.0, 15.0), states, 0.0,
                                   perception_range=160.0, station_type="drone")
    attack = IdentitySpoofAttack(
        ForgedIdentity(9101, (51.0, 4.0, 6.0), "rsu"),
        inner=RemoveObjectsAttack([
            RemoveTarget(object_id=CYCLIST, carve_lidar=False),
            RemoveTarget(object_id=PEDESTRIAN, carve_lidar=False),
        ]))
    forged = attack.apply_cpm(drone)

    assert CYCLIST in rsu.object_ids() and PEDESTRIAN in rsu.object_ids()
    assert CYCLIST not in forged.object_ids() and PEDESTRIAN not in forged.object_ids()
    assert set(attack.result.removed_object_ids) == {CYCLIST, PEDESTRIAN}

    spoofed = fuse_latest_by_station([rsu, forged])
    assert CYCLIST not in spoofed.object_ids() and PEDESTRIAN not in spoofed.object_ids()


def test_suppression_flips_the_decision():
    states = _scene()
    rsu = build_cpm_from_objects(9101, (51.0, 4.0, 6.0), states, 0.0,
                                 perception_range=160.0, station_type="rsu")
    attack = IdentitySpoofAttack(
        ForgedIdentity(9101, (51.0, 4.0, 6.0), "rsu"),
        inner=RemoveObjectsAttack([
            RemoveTarget(object_id=CYCLIST, carve_lidar=False),
            RemoveTarget(object_id=PEDESTRIAN, carve_lidar=False),
        ]))
    forged = attack.apply_cpm(rsu)
    honest = evaluate_crossing(_ego(), fuse_latest_by_station([rsu]).objects)
    spoofed = evaluate_crossing(_ego(), fuse_latest_by_station([rsu, forged]).objects)
    assert honest.decision == STOP
    assert spoofed.decision == GO


def test_closed_loop_mock_only_crashes_under_attack(tmp_path):
    """End to end: identical scene, identical controller, only the messages differ."""
    from carla_spoofing.scenarios.intersection_vru_spoofing import main
    assert main(["--mode", "mock", "--run", "both", "--sink", "null",
                 "--out", str(tmp_path)]) == 0
    import json
    verdict = json.loads((tmp_path / "comparison.json").read_text())["verdict"]
    assert verdict["attack_caused_collision"] is True
    assert verdict["honest_collision"] is False
    assert verdict["honest_stopped"] is True
    assert verdict["spoofed_stopped"] is False
