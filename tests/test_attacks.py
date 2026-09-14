import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from carla_spoofing.scenarios.mock_world import MockWorld
from carla_spoofing.attacks import (
    FakeObjectAttack, FakeObjectSpec, RemoveObjectAttack, RemoveTarget,
    CameraInjectionAttack)
from carla_spoofing.lidar import points_in_box_mask


def test_fake_object_adds_to_cpm_and_lidar():
    w = MockWorld.default_scene()
    atk = FakeObjectAttack(FakeObjectSpec(position=(15, 0, 0)))
    honest = w.honest_cpm()
    spoof = atk.apply_cpm(honest)
    assert 999001 in spoof.object_ids() and 999001 not in honest.object_ids()
    pc = w.honest_pointcloud()
    spc = atk.apply_pointcloud(pc)
    assert spc.shape[0] > pc.shape[0]
    # the injected points really are at the phantom location
    mask = points_in_box_mask(spc, (15, 0, 0), (4.5, 2.0, 1.5), margin=0.3)
    assert mask.sum() >= atk.result.points_added


def test_remove_object_drops_from_cpm_and_carves_lidar():
    w = MockWorld.default_scene()
    atk = RemoveObjectAttack(RemoveTarget(position=(40, -3.5, 0)))
    honest = w.honest_cpm()
    spoof = atk.apply_cpm(honest)
    assert 201 not in spoof.object_ids() and 201 in honest.object_ids()
    pc = w.honest_pointcloud()
    spc = atk.apply_pointcloud(pc)
    assert spc.shape[0] < pc.shape[0]
    # no returns remain inside the erased truck's box
    mask = points_in_box_mask(spc, (40, -3.5, 0), (6.0, 2.4, 3.0), margin=0.0)
    assert mask.sum() == 0


def test_remove_by_id():
    w = MockWorld.default_scene()
    atk = RemoveObjectAttack(RemoveTarget(object_id=200))
    spoof = atk.apply_cpm(w.honest_cpm())
    assert 200 not in spoof.object_ids()


def test_camera_injection_changes_pixels():
    w = MockWorld.default_scene()
    frame, proj = w.camera(width=400, height=300, fov=90)
    atk = CameraInjectionAttack(position=(18, 1, 0))
    out = atk.apply_image(frame.copy(), proj)
    assert out.shape == frame.shape
    assert not np.array_equal(out, frame)          # something was drawn
    assert len(atk.result.image_injections) == 1


def test_camera_injection_behind_camera_is_noop():
    w = MockWorld.default_scene()
    frame, proj = w.camera(width=400, height=300, fov=90)
    atk = CameraInjectionAttack(position=(-20, 0, 0))  # behind attacker
    out = atk.apply_image(frame.copy(), proj)
    assert np.array_equal(out, frame)
    assert atk.result.image_injections == []
