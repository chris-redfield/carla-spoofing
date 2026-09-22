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
