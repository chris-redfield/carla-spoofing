"""The guard must refuse runs that prove nothing -- including ones that look good.

A guard that never rejects anything is decoration. Each test here builds a run
pair that a verdict would happily call a successful attack, and asserts the guard
refuses it. The first one is the live left-turn run that caused the guard to be
written in the first place.
"""
import pytest

from carla_spoofing.evidence import (HONEST, SPOOFED, Evidence, ManoeuvreSpec,
                                     evidence_problems, is_road_user, report)
from carla_spoofing.scenarios.do_not_pass_spoofing import SPEC as DNP_SPEC
from carla_spoofing.scenarios.intersection_vru_spoofing import SPEC as VRU_SPEC
from carla_spoofing.scenarios.left_turn_spoofing import SPEC as LTA_SPEC


def _run(**over):
    """A run summary that passes every check, so each test breaks exactly one."""
    base = {
        "turn_attempted": True, "turn_started_s": 6.0, "turn_was_unsafe": True,
        "overtake_attempted": True, "overtake_started_s": 6.0,
        "overtake_was_unsafe": True,
        "ego_stopped": True, "ego_crossed": True, "crossed_s": 6.0,
        "crossing_was_unsafe": True,
        "collided": False, "hit_scenery": False, "static_collision": None,
        "n_flips": 3, "n_decisions": 60,
        "attack": {"removed_object_ids": [52]},
    }
    base.update(over)
    return base


def _pair(honest=None, spoofed=None):
    return {HONEST: _run(**(honest or {})), SPOOFED: _run(**(spoofed or {}))}


ALL_SPECS = [LTA_SPEC, DNP_SPEC, VRU_SPEC]


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_a_clean_pair_is_evidence(spec):
    # The honest run differs in commit time, as a real baseline would.
    ev = evidence_problems(_pair(honest={"turn_started_s": 11.0,
                                         "overtake_started_s": 11.0,
                                         "crossed_s": 19.0}), spec)
    assert ev.valid, ev.problems


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_zero_flips_in_the_spoofed_run_is_refused(spec):
    """The no-op forgery: behaviour changed, but not because of the messages."""
    ev = evidence_problems(_pair(honest={"turn_started_s": 11.0,
                                         "overtake_started_s": 11.0,
                                         "crossed_s": 19.0},
                                 spoofed={"n_flips": 0}), spec)
    assert not ev.valid
    assert any("ZERO decision flips" in p for p in ev.problems)


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_a_forgery_that_deleted_nothing_is_refused(spec):
    """Impersonation alone supersedes the RSU, so an effect is not suppression."""
    ev = evidence_problems(_pair(honest={"turn_started_s": 11.0,
                                         "overtake_started_s": 11.0,
                                         "crossed_s": 19.0},
                                 spoofed={"attack": {"removed_object_ids": []}}),
                           spec)
    assert not ev.valid
    assert any("deleted nothing" in p for p in ev.problems)


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_identical_commit_instants_are_refused(spec):
    """The incident this guard was written for: both runs committed together."""
    ev = evidence_problems(_pair(), spec)      # same commit time in both
    assert not ev.valid
    assert any("same instant" in p for p in ev.problems)


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_a_coincidental_collision_is_refused(spec):
    """Collided while ground truth said the manoeuvre was safe."""
    unsafe = {"turn_was_unsafe": False, "overtake_was_unsafe": False,
              "crossing_was_unsafe": False}
    ev = evidence_problems(_pair(honest={"turn_started_s": 11.0,
                                         "overtake_started_s": 11.0,
                                         "crossed_s": 19.0},
                                 spoofed={"collided": True, **unsafe}), spec)
    assert not ev.valid
    assert any("not the attack's doing" in p for p in ev.problems)


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_hitting_scenery_voids_the_run(spec):
    ev = evidence_problems(
        _pair(honest={"turn_started_s": 11.0, "overtake_started_s": 11.0,
                      "crossed_s": 19.0},
              spoofed={"hit_scenery": True,
                       "static_collision": {"with_type_id": "static.prop.streetsign04",
                                            "sim_time": 3.2}}), spec)
    assert not ev.valid
    assert any("hit scenery" in p for p in ev.problems)


def test_an_ego_that_never_turned_voids_the_left_turn():
    ev = evidence_problems(_pair(honest={"turn_started_s": 11.0},
                                 spoofed={"turn_attempted": False}), LTA_SPEC)
    assert any("NEVER TURNED" in p for p in ev.problems)


def test_an_ego_that_never_overtook_voids_do_not_pass():
    ev = evidence_problems(_pair(honest={"overtake_started_s": 11.0},
                                 spoofed={"overtake_attempted": False}), DNP_SPEC)
    assert any("NEVER OVERTOOK" in p for p in ev.problems)


def test_a_vru_baseline_that_never_stopped_is_not_a_baseline():
    """VRU is asymmetric: the honest ego must stop, the spoofed must cross."""
    ev = evidence_problems(_pair(honest={"ego_stopped": False, "crossed_s": 19.0}),
                           VRU_SPEC)
    assert any("NEVER STOPPED" in p for p in ev.problems)


def test_a_vru_spoofed_run_that_never_crossed_is_refused():
    ev = evidence_problems(_pair(honest={"crossed_s": 19.0},
                                 spoofed={"ego_crossed": False}), VRU_SPEC)
    assert any("NEVER CROSSED" in p for p in ev.problems)


@pytest.mark.parametrize("spec", ALL_SPECS)
@pytest.mark.parametrize("present", [HONEST, SPOOFED])
def test_a_single_run_is_incomplete_not_invalid(spec, present):
    """The earlier guard invented two problems for every --run honest call.

    A missing spoofed run has no flips and removed no objects, so the no-op
    checks both fired and the run was declared invalid. One run is an incomplete
    experiment, not a failed one, and it must not fail the make target.
    """
    ev = evidence_problems({present: _run()}, spec)
    assert not ev.comparable
    assert ev.problems == []
    assert ev.valid is False                      # cannot claim validity
    assert ev.to_summary()["evidence_valid"] is None   # ...but does not claim failure
    assert present in ev.note
    assert report(ev, spec, printer=lambda *a: None) == 0


@pytest.mark.parametrize("spec", ALL_SPECS)
def test_problems_make_the_process_fail(spec):
    """The guard is only load-bearing because the command exits non-zero."""
    ev = evidence_problems(_pair(), spec)         # identical commit instants
    assert report(ev, spec, printer=lambda *a: None) == 2
    assert report(Evidence(comparable=True), spec, printer=lambda *a: None) == 0


def test_scenery_is_not_a_road_user():
    assert is_road_user("vehicle.tesla.model3")
    assert is_road_user("walker.pedestrian.0001")
    assert is_road_user("vehicle.bh.crossbike")          # the cyclist
    assert not is_road_user("static.prop.streetsign04")  # the RSU's own pole
    assert not is_road_user("static.prop.colacan")
