"""End-to-end spoofing scenario runner.

Pipeline (identical for mock and CARLA):
    honest perception  ->  attack  ->  packet  ->  sink (OMNeT++ hand-off)

Examples
--------
    # Inject a phantom stopped car 15 m ahead, dump packets for OMNeT++:
    carla-spoof --mode mock --attack fake_object --fake-x 15 --fake-y 0 \
        --out out --save-artifacts

    # Erase a real object (the truck) from the shared perception:
    carla-spoof --mode mock --attack remove_object --remove-x 40 --remove-y -3.5

    # Inject an AI-generated vehicle into the camera frame:
    carla-spoof --mode mock --attack camera --fake-x 18 --fake-y 1 --save-artifacts

    # Against a running CarlaAir/CARLA server (CPM-level spoof):
    carla-spoof --mode carla --host 127.0.0.1 --port 2000 --attack fake_object \
        --fake-x 12 --fake-y 0
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Optional

import numpy as np

from ..v2x.cpm import CollectivePerceptionMessage, diff_cpms
from ..v2x.packet import Packet, FileSink, UdpSink, NullSink
from ..lidar import save_pcd
from ..attacks import (
    NoAttack, FakeObjectAttack, FakeObjectSpec,
    RemoveObjectAttack, RemoveTarget,
    CameraInjectionAttack, ProceduralCarGenerator, ExternalCommandGenerator,
)


def build_attack(args):
    if args.attack == "none":
        return NoAttack()
    if args.attack == "fake_object":
        return FakeObjectAttack(FakeObjectSpec(
            position=(args.fake_x, args.fake_y, args.fake_z),
            classification=args.fake_class,
            inject_lidar=not args.no_lidar,
        ))
    if args.attack == "remove_object":
        pos = None if args.remove_x is None else (args.remove_x, args.remove_y, args.remove_z)
        return RemoveObjectAttack(RemoveTarget(
            object_id=args.remove_id, position=pos, carve_lidar=not args.no_lidar,
        ))
    if args.attack == "camera":
        gen = (ExternalCommandGenerator(args.generator_cmd)
               if args.generator_cmd else ProceduralCarGenerator())
        return CameraInjectionAttack(
            position=(args.fake_x, args.fake_y, args.fake_z),
            classification=args.fake_class, generator=gen,
        )
    raise ValueError(f"unknown attack {args.attack}")


def make_sink(args):
    if args.sink == "file":
        os.makedirs(args.out, exist_ok=True)
        return FileSink(os.path.join(args.out, "v2x_packets.jsonl"))
    if args.sink == "udp":
        return UdpSink(args.udp_host, args.udp_port)
    return NullSink()


def run_mock(args, attack, sink) -> dict:
    from .mock_world import MockWorld
    world = MockWorld.default_scene()
    summary = {"mode": "mock", "attack": attack.name, "frames": []}
    for frame in range(args.frames):
        gen_time = float(frame) * 0.1
        honest = world.honest_cpm(gen_time)
        spoofed = attack.apply_cpm(honest)
        sink.send(Packet.cpm(spoofed, seq=frame))
        frame_info = {"frame": frame, "cpm_diff": diff_cpms(honest, spoofed)}

        # LiDAR channel (fake_object / remove_object)
        if attack.name in ("fake_object", "remove_object") and not args.no_lidar:
            pc = world.honest_pointcloud(seed=frame)
            spoofed_pc = attack.apply_pointcloud(pc)
            frame_info["lidar"] = {"honest_points": int(pc.shape[0]),
                                   "spoofed_points": int(spoofed_pc.shape[0])}
            if args.save_artifacts and frame == 0:
                save_pcd(spoofed_pc, os.path.join(args.out, "lidar_spoofed.pcd"))
                save_pcd(pc, os.path.join(args.out, "lidar_honest.pcd"))

        # Camera channel
        if attack.name == "camera":
            frame_img, projector = world.camera()
            spoofed_img = attack.apply_image(frame_img, projector)
            meta = {"width": projector.width, "height": projector.height,
                    "fov": projector.fov_deg,
                    "injected": attack.result.image_injections}
            if args.save_artifacts:
                from PIL import Image
                ref = os.path.join(args.out, f"camera_spoofed_{frame:03d}.png")
                Image.fromarray(spoofed_img).save(ref)
                meta["image_ref"] = ref
            sink.send(Packet.camera(world.attacker_id, seq=frame,
                                    sim_time=gen_time, image_meta=meta))

        frame_info["attack_result"] = attack.result.to_dict()
        summary["frames"].append(frame_info)
    return summary


def run_carla(args, attack, sink) -> dict:
    try:
        import carla  # provided by the CarlaAir binary's bundled module
    except ImportError as e:
        print("ERROR: `carla` module not importable. Run inside the CarlaAir "
              "conda env / Docker image, with the sim reachable.", file=sys.stderr)
        raise SystemExit(2) from e
    from ..perception import build_cpm_from_carla

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    vehicles = [a for a in world.get_actors() if a.type_id.startswith("vehicle")]
    if not vehicles:
        raise SystemExit("No vehicles in the world to act as the attacker/sender. "
                         "Spawn traffic first (e.g. CARLA's generate_traffic.py).")
    sender = vehicles[0]
    print(f"[carla] attacker/sender = actor {sender.id} ({sender.type_id})")
    summary = {"mode": "carla", "attack": attack.name, "sender_id": sender.id,
               "frames": []}
    for frame in range(args.frames):
        honest = build_cpm_from_carla(world, sender)
        spoofed = attack.apply_cpm(honest)
        sink.send(Packet.cpm(spoofed, seq=frame))
        summary["frames"].append({
            "frame": frame, "cpm_diff": diff_cpms(honest, spoofed),
            "honest_objects": len(honest.perceived_objects),
            "attack_result": attack.result.to_dict(),
        })
        if args.frames > 1:
            world.wait_for_tick()
    print("[carla] NOTE: LiDAR/camera raw-injection reuse the same attack objects "
          "on sensor callback data; see omnet/README.md and attach a sensor loop.")
    return summary


def main(argv=None):
    p = argparse.ArgumentParser(description="CARLA V2X cooperative-perception spoofing")
    p.add_argument("--mode", choices=["mock", "carla"], default="mock")
    p.add_argument("--attack", choices=["none", "fake_object", "remove_object", "camera"],
                   default="fake_object")
    p.add_argument("--frames", type=int, default=1)
    # CARLA connection
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    # sink / output
    p.add_argument("--sink", choices=["file", "udp", "null"], default="file")
    p.add_argument("--out", default="out")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=47000)
    p.add_argument("--save-artifacts", action="store_true",
                   help="write spoofed/honest .pcd and .png alongside packets")
    p.add_argument("--no-lidar", action="store_true", help="CPM-only, skip point cloud")
    # fake-object / camera params
    p.add_argument("--fake-x", type=float, default=15.0)
    p.add_argument("--fake-y", type=float, default=0.0)
    p.add_argument("--fake-z", type=float, default=0.0)
    p.add_argument("--fake-class", default="car")
    p.add_argument("--generator-cmd", default=None,
                   help="external AI image-gen command template for camera attack")
    # remove-object params
    p.add_argument("--remove-id", type=int, default=None)
    p.add_argument("--remove-x", type=float, default=None)
    p.add_argument("--remove-y", type=float, default=0.0)
    p.add_argument("--remove-z", type=float, default=0.0)
    args = p.parse_args(argv)

    attack = build_attack(args)
    sink = make_sink(args)
    try:
        summary = run_mock(args, attack, sink) if args.mode == "mock" \
            else run_carla(args, attack, sink)
    finally:
        sink.close()

    if args.sink == "file":
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, "run_summary.json"), "w") as fh:
            json.dump(summary, fh, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
