"""Regression net for the do-not-pass scenario after the left-turn work.

The left-turn scenario needed changes to machinery do-not-pass also uses --
``CarlaLaneReference``, the geometry helpers, the linger teardown. Do-not-pass
is the one scenario verified working in live CARLA, so those shared edits need
holding in place by something other than "it looked fine last time".

The existing ``test_do_not_pass.py`` covers the decision logic and the mock
loop well, but nothing exercised the CARLA path. That gap is what let three
trivial errors reach the simulator in the left-turn work, so it is closed here
with the same fake road graph.
"""
import math

import pytest

import fake_carla
from carla_spoofing.control import CarlaLaneReference


def _ref(branching=False):
    return CarlaLaneReference(fake_carla.StraightRoadWorld(branching).get_map())


# --------------------------------------------------------------------------- #
# CarlaLaneReference -- shared with the left-turn scenario, and edited by it    #
# --------------------------------------------------------------------------- #
def test_lane_following_is_unchanged_on_a_straight_road(monkeypatch):
    """The `straightest` change must be a no-op where there are no branches.

    Lane following previously took ``wp.next(d)[0]``. That was changed to pick
    the straightest branch, which fixed a car turning off at a junction -- but
    do-not-pass runs on a straight road, and its behaviour there must be
    identical.
    """
    monkeypatch.setitem(__import__("sys").modules, "carla",
                        fake_carla.make_module())
    ref = _ref(branching=False)
    target = ref.point_ahead((10.0, -1.75, 0.0), 0.0, 8.0)
    assert target[0] == pytest.approx(18.0)
    assert target[1] == pytest.approx(-1.75)


def test_lane_following_no_longer_turns_off_at_a_branch(monkeypatch):
    """With a side road offered first, carry straight on.

    The improvement the left-turn work brought back here: indexing ``[0]`` took
    whichever branch CARLA happened to list first.
    """
    monkeypatch.setitem(__import__("sys").modules, "carla",
                        fake_carla.make_module())
    ref = _ref(branching=True)
    target = ref.point_ahead((10.0, -1.75, 0.0), 0.0, 8.0)
    assert target[0] == pytest.approx(18.0), "took the side road"


def test_overtaking_does_not_flip_onto_the_opposing_lane(monkeypatch):
    """The do-not-pass-specific guard, which must survive the shared edits.

    ``get_waypoint`` snaps to the nearest lane, so the moment an overtake puts
    the car across the centre line it resolves to the opposing lane -- whose
    heading is reversed. Without the guard the reference inverts mid-manoeuvre
    and the controller drives off the road.
    """
    monkeypatch.setitem(__import__("sys").modules, "carla",
                        fake_carla.make_module())
    ref = _ref()
    # Ego has crossed the centre line (y > 0) but is still travelling +X.
    target = ref.point_ahead((10.0, 1.0, 0.0), 0.0, 8.0)
    assert target[0] > 10.0, "reference inverted onto the opposing lane"


def test_lateral_offset_goes_left_for_an_overtake(monkeypatch):
    """Negative offset must put the target to the LEFT of travel."""
    monkeypatch.setitem(__import__("sys").modules, "carla",
                        fake_carla.make_module())
    ref = _ref()
    straight = ref.point_ahead((10.0, -1.75, 0.0), 0.0, 8.0)
    offset = ref.point_ahead((10.0, -1.75, 0.0), 0.0, 8.0, lateral_offset=-3.5)
    assert offset[1] < straight[1], "overtake offset went the wrong way"
    assert offset[1] - straight[1] == pytest.approx(-3.5, abs=0.01)


# --------------------------------------------------------------------------- #
# The public surface the geometry extraction moved                             #
# --------------------------------------------------------------------------- #
def test_do_not_pass_warning_still_exports_everything_it_used_to():
    """`geometry.py` was extracted out of this module; imports must still work.

    ``control.py``, the scenario and the existing tests all import these names
    from here. The extraction was meant to be invisible.
    """
    import carla_spoofing.do_not_pass_warning as dnp
    for name in ("PASS", "DO_NOT_PASS", "EgoState", "VEHICLE_CLASSES", "Vec3",
                 "heading_unit", "ego_frame", "relative_to_ego",
                 "DoNotPassDecision", "DoNotPassThresholds",
                 "evaluate_do_not_pass", "compare_decisions"):
        assert hasattr(dnp, name), f"do_not_pass_warning lost {name}"


def test_geometry_helpers_agree_across_both_modules():
    """The two receiver applications must share one frame convention.

    If these ever diverge, one scenario's idea of "left" silently stops
    matching the other's.
    """
    from carla_spoofing import geometry, do_not_pass_warning, left_turn_assist
    assert do_not_pass_warning.heading_unit is geometry.heading_unit
    assert do_not_pass_warning.EgoState is geometry.EgoState
    assert left_turn_assist.heading_unit is geometry.heading_unit
    # Left of a heading is yaw - 90 in CARLA's left-handed frame, for both.
    fwd = geometry.heading_unit(0.0)
    left = geometry.heading_unit(-90.0)
    _f, right = geometry.ego_frame(0.0)
    assert left[0] * right[0] + left[1] * right[1] == pytest.approx(-1.0)
    assert fwd == pytest.approx((1.0, 0.0))


# --------------------------------------------------------------------------- #
# Teardown                                                                     #
# --------------------------------------------------------------------------- #
def test_both_scenarios_brake_before_lingering():
    """A vehicle keeps its last control; the hold loop only ticks.

    Without an explicit stop, a run that ended normally left the ego holding
    cruise throttle and it drove on uncontrolled for the whole hold. Asserted
    for both scenarios because the bug was shared and only showed in one.
    """
    import inspect
    from carla_spoofing.scenarios import do_not_pass_spoofing, left_turn_spoofing
    for mod in (do_not_pass_spoofing, left_turn_spoofing):
        src = inspect.getsource(mod.run_carla)
        hold = src.index("args.linger > 0")
        window = src[hold:hold + 1200]
        assert "brake=1.0" in window, (
            f"{mod.__name__} does not stop the vehicles before lingering")


def test_the_mock_scenario_still_produces_its_verdict(tmp_path):
    """End-to-end backstop: the headline result must not have moved."""
    import json
    from carla_spoofing.scenarios.do_not_pass_spoofing import main
    assert main(["--mode", "mock", "--run", "both", "--sink", "null",
                 "--out", str(tmp_path)]) == 0
    verdict = json.loads((tmp_path / "comparison.json").read_text())["verdict"]
    assert verdict["attack_caused_unsafe_overtake"] is True
    assert verdict["attack_caused_collision"] is True
    assert verdict["honest_unsafe_overtake"] is False
    assert verdict["honest_collision"] is False
