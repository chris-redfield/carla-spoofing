import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from carla_spoofing.attacks import (
    ForgedIdentity, IdentitySpoofAttack, RemoveObjectAttack, RemoveTarget)
from carla_spoofing.do_not_pass_warning import (
    DO_NOT_PASS, PASS, EgoState, evaluate_do_not_pass)
from carla_spoofing.fusion import fuse_latest_by_station, is_occluded, visible_objects
from carla_spoofing.perception import ObjectState, build_cpm_from_objects
from carla_spoofing.v2x.cpm import PerceivedObject

EGO, LEAD, ONCOMING = 101, 102, 103


def _scene(oncoming_x=110.0):
    """Ego at the origin heading +X, slow lead ahead, oncoming in the other lane."""
    return [
        ObjectState(EGO, (0.0, 0.0, 0.0), velocity=(8.3, 0.0, 0.0), yaw_deg=0.0),
        ObjectState(LEAD, (30.0, 0.0, 0.0), velocity=(5.0, 0.0, 0.0), yaw_deg=0.0),
        ObjectState(ONCOMING, (oncoming_x, -3.5, 0.0), velocity=(-9.7, 0.0, 0.0),
                    yaw_deg=180.0),
    ]


def _ego(desired=8.3):
    return EgoState(EGO, (0.0, 0.0, 0.0), yaw_deg=0.0, velocity=(8.3, 0.0, 0.0),
                    desired_speed_mps=desired)


def _objects(states, station_id=EGO, position=(0.0, 0.0, 0.0), rng=float("inf")):
    return build_cpm_from_objects(station_id, position, states, 0.0,
                                  perception_range=rng).perceived_objects


def test_oncoming_vehicle_forbids_the_pass():
    d = evaluate_do_not_pass(_ego(), _objects(_scene()))
    assert d.decision == DO_NOT_PASS
    assert d.blocking_object_id == ONCOMING
    assert d.pass_intent                      # the slow lead is a reason to pass


def test_clear_road_allows_the_pass():
    states = [s for s in _scene() if s.object_id != ONCOMING]
    d = evaluate_do_not_pass(_ego(), _objects(states))
    assert d.decision == PASS
    assert d.lead_object_id == LEAD


def test_vehicle_behind_is_not_a_hazard():
    states = _scene(oncoming_x=-40.0)         # already gone past
    assert evaluate_do_not_pass(_ego(), _objects(states)).decision == PASS


def test_pass_intent_uses_desired_speed_not_current_speed():
    """A car queued behind a slow lead still wants to overtake."""
    states = _scene()
    states[0] = ObjectState(EGO, (0.0, 0.0, 0.0), velocity=(5.0, 0.0, 0.0), yaw_deg=0.0)
    queued = EgoState(EGO, (0.0, 0.0, 0.0), yaw_deg=0.0, velocity=(5.0, 0.0, 0.0),
                      desired_speed_mps=8.3)
    assert evaluate_do_not_pass(queued, _objects(states)).pass_intent
    # Without a desired speed it compares against its own (matched) speed and
    # concludes, wrongly for a controller, that it has no reason to pass.
    matched = EgoState(EGO, (0.0, 0.0, 0.0), yaw_deg=0.0, velocity=(5.0, 0.0, 0.0))
    assert not evaluate_do_not_pass(matched, _objects(states)).pass_intent


def test_lead_occludes_the_oncoming_vehicle_from_the_ego():
    states = _scene(oncoming_x=60.0)
    # Oncoming directly behind the lead from the ego's viewpoint.
    states[2] = ObjectState(ONCOMING, (60.0, 0.2, 0.0), velocity=(-9.7, 0.0, 0.0),
                            yaw_deg=180.0)
    objs = _objects(states)
    seen = visible_objects((0.0, 0.0, 0.0), objs, objs)
    assert ONCOMING not in [o.object_id for o in seen]
    assert LEAD in [o.object_id for o in seen]


def test_impersonated_cpm_supersedes_the_genuine_one():
    """The crux of the attack: same station id replaces, a new one would only add."""
    states = _scene()
    rsu = build_cpm_from_objects(9001, (55.0, -8.0, 6.0), states, 0.0,
                                 perception_range=160.0, station_type="rsu")
    drone = build_cpm_from_objects(9002, (55.0, -10.0, 15.0), states, 0.0,
                                   perception_range=160.0, station_type="drone")
    attack = IdentitySpoofAttack(
        ForgedIdentity(9001, (55.0, -8.0, 6.0), "rsu"),
        inner=RemoveObjectAttack(RemoveTarget(object_id=ONCOMING, carve_lidar=False)))
    forged = attack.apply_cpm(drone)

    assert ONCOMING in rsu.object_ids()
    assert ONCOMING not in forged.object_ids()
    assert attack.result.impersonated_station_id == 9001
    assert attack.result.true_sender_id == 9002

    # Impersonating the RSU: the hazard disappears from the fused view.
    spoofed = fuse_latest_by_station([rsu, forged])
    assert ONCOMING not in spoofed.object_ids()

    # Broadcasting honestly under its own id: the RSU's report still stands, so
    # the same object removal achieves nothing.
    naive = forged.copy()
    naive.station_id = 9002
    assert ONCOMING in fuse_latest_by_station([rsu, naive]).object_ids()


def test_suppression_flips_the_decision():
    states = _scene()
    rsu = build_cpm_from_objects(9001, (55.0, -8.0, 6.0), states, 0.0,
                                 perception_range=160.0, station_type="rsu")
    attack = IdentitySpoofAttack(
        ForgedIdentity(9001, (55.0, -8.0, 6.0), "rsu"),
        inner=RemoveObjectAttack(RemoveTarget(object_id=ONCOMING, carve_lidar=False)))
    forged = attack.apply_cpm(rsu)
    honest = evaluate_do_not_pass(_ego(), fuse_latest_by_station([rsu]).objects)
    spoofed = evaluate_do_not_pass(_ego(), fuse_latest_by_station([rsu, forged]).objects)
    assert honest.decision == DO_NOT_PASS
    assert spoofed.decision == PASS


def test_closed_loop_mock_only_crashes_under_attack(tmp_path):
    """End to end: identical scene, identical controller, only the messages differ."""
    from carla_spoofing.scenarios.do_not_pass_spoofing import main
    assert main(["--mode", "mock", "--run", "both", "--sink", "null",
                 "--out", str(tmp_path)]) == 0
    import json
    verdict = json.loads((tmp_path / "comparison.json").read_text())["verdict"]
    assert verdict["attack_caused_unsafe_overtake"] is True
    assert verdict["attack_caused_collision"] is True
    assert verdict["honest_unsafe_overtake"] is False
    assert verdict["honest_collision"] is False


def test_drone_range_warning_fires_only_when_the_forgery_is_vacuous():
    """A drone backed off past its own perception range makes the run a lie.

    It still ends in a collision -- an impersonated message replaces the RSU's
    either way -- but nothing was actually suppressed, so the crash is a sensor
    range artifact rather than the attack. The run has to say so.
    """
    from carla_spoofing.scenarios.do_not_pass_spoofing import (
        ScenarioConfig, _drone_range_warning)

    assert _drone_range_warning(ScenarioConfig()) is None          # default is safe
    assert _drone_range_warning(ScenarioConfig(drone_back_m=49)) is None   # 110+49 < 160
    assert _drone_range_warning(ScenarioConfig(drone_back_m=51)) is not None
    assert "160" in _drone_range_warning(ScenarioConfig(drone_back_m=80))


def test_moving_the_drone_back_does_not_disarm_the_attack(tmp_path):
    """The camera move is cosmetic: the verdict must be unchanged."""
    from carla_spoofing.scenarios.do_not_pass_spoofing import main
    import json
    assert main(["--mode", "mock", "--run", "both", "--sink", "null",
                 "--drone-back", "40", "--out", str(tmp_path)]) == 0
    verdict = json.loads((tmp_path / "comparison.json").read_text())["verdict"]
    assert verdict["attack_caused_collision"] is True
    assert verdict["honest_collision"] is False
