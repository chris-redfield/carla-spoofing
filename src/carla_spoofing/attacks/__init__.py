"""Cooperative-perception spoofing attacks.

Every attack subclasses :class:`Attack` and may act on any subset of the three
modalities a station can share:

* ``apply_cpm``        -- object-level CPM (add / drop / alter perceived objects)
* ``apply_pointcloud`` -- raw LiDAR cloud (inject / carve returns)
* ``apply_image``      -- camera frame (composite a fake vehicle)

Attacks that don't touch a modality inherit an identity pass-through, so the
scenario runner can apply any attack uniformly.
"""
from .base import Attack, AttackResult, NoAttack
from .fake_object import FakeObjectAttack, FakeObjectSpec
from .remove_object import RemoveObjectAttack, RemoveTarget
from .camera_injection import (
    CameraInjectionAttack,
    CameraProjector,
    VehicleImageGenerator,
    ProceduralCarGenerator,
    ExternalCommandGenerator,
)

ATTACKS = {
    "none": NoAttack,
    "fake_object": FakeObjectAttack,
    "remove_object": RemoveObjectAttack,
    "camera": CameraInjectionAttack,
}

__all__ = [
    "Attack", "AttackResult", "NoAttack",
    "FakeObjectAttack", "FakeObjectSpec",
    "RemoveObjectAttack", "RemoveTarget",
    "CameraInjectionAttack", "CameraProjector",
    "VehicleImageGenerator", "ProceduralCarGenerator", "ExternalCommandGenerator",
    "ATTACKS",
]
