"""Minimal, CARLA-free demo of the fake-object attack.

Run:  python examples/demo_fake_object_offline.py
Shows honest vs spoofed CPM and the packet a victim would receive.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from carla_spoofing.scenarios.mock_world import MockWorld
from carla_spoofing.attacks import FakeObjectAttack, FakeObjectSpec
from carla_spoofing.v2x.cpm import diff_cpms
from carla_spoofing.v2x.packet import Packet

world = MockWorld.default_scene()
honest = world.honest_cpm(generation_time=0.0)

attack = FakeObjectAttack(FakeObjectSpec(position=(12.0, 0.0, 0.0),
                                         classification="car"))
spoofed = attack.apply_cpm(honest)

print("Honest perceived objects :", honest.object_ids())
print("Spoofed perceived objects:", spoofed.object_ids())
print("Diff                     :", diff_cpms(honest, spoofed))
print("Attack result            :", attack.result.to_dict())
print()
print("Packet a neighbour receives over V2X (assumed-perfect network):")
print(Packet.cpm(spoofed, seq=0).to_json())
