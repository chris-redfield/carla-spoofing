"""Scene derivation against a fake road graph -- the --mode carla path, testable."""
import types
import pytest

import fake_carla
from carla_spoofing.scenarios.left_turn_spoofing import (
    ScenarioConfig, _derive_transforms, classify_approach,
)


def _args(**over):
    base = dict(ego_spawn=None, hazard_spawn=None)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_scene_derives_without_a_simulator():
    """The regression net for the --mode carla path.

    Three separate live runs died here on trivial errors a test could have
    caught in milliseconds: a missing import, a call to a renamed function, and
    a None compared with an int. This exercises the whole derivation.
    """
    carla = fake_carla.make_module()
    world = fake_carla.World()
    scene = _derive_transforms(carla, world, ScenarioConfig(), _args())

    for key in ("ego_tf", "crossing_tf", "stop_point", "turn_points",
                "conflict_point", "junction_center", "ego_yaw_deg",
                "exit_yaw_deg", "exit_point", "hazard_speed_kmh", "note"):
        assert key in scene, f"scene is missing {key}"
    assert len(scene["turn_points"]) > 5


def test_the_turn_actually_goes_left():
    """Exit heading must be 90 degrees LEFT of the approach, not right.

    A live run turned right and then U-turned; the cause was an exit chosen
    without checking whose arm the movement started on.
    """
    carla = fake_carla.make_module()
    scene = _derive_transforms(carla, fake_carla.World(), ScenarioConfig(), _args())
    approach, exit_yaw = scene["ego_yaw_deg"], scene["exit_yaw_deg"]
    rel = (exit_yaw - approach + 180.0) % 360.0 - 180.0
    assert rel == pytest.approx(-90.0, abs=5.0), (
        f"exit is {rel:+.0f} deg from the approach; left is -90")


def test_the_hazard_crosses_our_path():
    carla = fake_carla.make_module()
    scene = _derive_transforms(carla, fake_carla.World(), ScenarioConfig(), _args())
    kind = classify_approach(scene["ego_yaw_deg"],
                             scene["crossing_tf"].rotation.yaw)
    assert kind in ("left", "oncoming"), f"hazard approaches from our {kind}"


def test_ego_gets_the_longest_approach_available():
    """Spawns exist at 16 m and 42 m on our arm; it must take 42."""
    carla = fake_carla.make_module()
    scene = _derive_transforms(carla, fake_carla.World(), ScenarioConfig(), _args())
    assert scene["ego_walked_m"] >= 40.0, scene["note"]


def test_explicit_spawn_indices_are_honoured_and_range_checked():
    carla = fake_carla.make_module()
    world = fake_carla.World()
    scene = _derive_transforms(carla, world, ScenarioConfig(),
                               _args(ego_spawn=0, hazard_spawn=2))
    assert scene["ego_walked_m"] == pytest.approx(16.0, abs=2.0)
    with pytest.raises(SystemExit, match="out of range"):
        _derive_transforms(carla, world, ScenarioConfig(), _args(ego_spawn=999))


def test_hazard_speed_is_solved_into_a_plausible_range():
    carla = fake_carla.make_module()
    scene = _derive_transforms(carla, fake_carla.World(), ScenarioConfig(), _args())
    cfg = ScenarioConfig()
    assert cfg.hazard_min_speed_kmh <= scene["hazard_speed_kmh"] \
        <= cfg.hazard_max_speed_kmh


def test_the_turn_stays_in_lane_until_it_reaches_the_junction():
    """The regression: the ego clipped a pole on the corner island.

    It began arcing 10 m short of the junction and cut diagonally across the
    corner. A car stopped at the line pulls forward into the junction first and
    turns from inside it, so the path must show no sideways drift at all while
    it is still on the approach.
    """
    from carla_spoofing.control import approach_then_turn
    from carla_spoofing.geometry import heading_unit
    import math

    # Real numbers from the live run that hit the traffic light.
    ego = (-48.838, -11.136, 0.0)
    junction = (-48.8, -1.2, 0.0)
    approach_yaw = 90.0
    entry = (junction[0], junction[1] + 5.0, 0.0)
    exit_ = (-28.7, 28.1, 0.0)

    pts = approach_then_turn(ego, entry, approach_yaw, exit_, 0.2,
                             tangent_scale=0.35)
    fwd = heading_unit(approach_yaw)
    right = (-fwd[1], fwd[0])
    before = [p for p in pts if p[1] < junction[1]]
    assert before, "path should include the run up to the junction"
    drift = max(abs((p[0] - ego[0]) * right[0] + (p[1] - ego[1]) * right[1])
                for p in before)
    assert drift < 0.5, f"drifted {drift:.1f} m sideways before the junction"


def test_scene_turn_path_starts_with_a_straight_run():
    """Built into the scene, not just available as a helper."""
    import types
    import fake_carla
    from carla_spoofing.scenarios.left_turn_spoofing import (
        ScenarioConfig, _derive_transforms)
    from carla_spoofing.geometry import heading_unit

    scene = _derive_transforms(
        fake_carla.make_module(), fake_carla.World(), ScenarioConfig(),
        types.SimpleNamespace(ego_spawn=None, hazard_spawn=None))
    pts = scene["turn_points"]
    fwd = heading_unit(scene["ego_yaw_deg"])
    right = (-fwd[1], fwd[0])
    start = pts[0]
    # The first few metres must be dead straight along the approach.
    early = [p for p in pts
             if (p[0] - start[0]) * fwd[0] + (p[1] - start[1]) * fwd[1] < 4.0]
    drift = max(abs((p[0] - start[0]) * right[0] + (p[1] - start[1]) * right[1])
                for p in early)
    assert drift < 0.3, f"turn begins bending immediately ({drift:.2f} m)"
