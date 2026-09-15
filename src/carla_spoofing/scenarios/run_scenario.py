"""End-to-end spoofing scenario runner (multi-vehicle cooperative perception).

Every vehicle broadcasts its own honest CPM each frame; ONE vehicle is the
attacker, whose CPM is spoofed. All messages go to the sink (the OMNeT++
hand-off), and a human-readable ``messages.csv`` flags exactly what was faked.

    honest perception (per vehicle) --> [attacker: spoof] --> packets + CSV --> OMNeT++

Examples
--------
    # Mock: every vehicle broadcasts; attacker 101 injects a phantom car:
    carla-spoof --mode mock --attack fake_object --fake-x 15 --out out

    # Live sim: all vehicles broadcast, first vehicle is the attacker:
    carla-spoof --mode carla --host carla-sim --port 2000 \
        --attack remove_object --remove-x 40 --remove-y -3.5 --out out
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from ..v2x.cpm import diff_cpms
from ..v2x.packet import Packet, FileSink, UdpSink, NullSink
from ..lidar import save_pcd
from ..report import ReportWriter
from ..attacks import (
    NoAttack, FakeObjectAttack, FakeObjectSpec,
    RemoveObjectAttack, RemoveTarget,
    CameraInjectionAttack, ProceduralCarGenerator, ExternalCommandGenerator,
)

import time as _time


def _plan(args):
    """(n_frames, period_seconds) from --rate/--duration/--frames."""
    rate = max(0.0, args.rate)
    period = (1.0 / rate) if rate > 0 else 0.0
    if args.frames and args.frames > 0:
        n = args.frames
    elif rate > 0:
        n = max(1, round(args.duration * rate))
    else:
        n = 1
    return n, period


def _sleep_until(target_monotonic):
    dt = target_monotonic - _time.monotonic()
    if dt > 0:
        _time.sleep(dt)


def build_attack(args):
    if args.attack == "none":
        return NoAttack()
    if args.attack == "fake_object":
        return FakeObjectAttack(FakeObjectSpec(
            position=(args.fake_x, args.fake_y, args.fake_z),
            classification=args.fake_class, inject_lidar=not args.no_lidar))
    if args.attack == "remove_object":
        pos = None if args.remove_x is None else (args.remove_x, args.remove_y, args.remove_z)
        return RemoveObjectAttack(RemoveTarget(
            object_id=args.remove_id, position=pos, carve_lidar=not args.no_lidar))
    if args.attack == "camera":
        gen = (ExternalCommandGenerator(args.generator_cmd)
               if args.generator_cmd else ProceduralCarGenerator())
        return CameraInjectionAttack(
            position=(args.fake_x, args.fake_y, args.fake_z),
            classification=args.fake_class, generator=gen)
    raise ValueError(f"unknown attack {args.attack}")


def make_sink(args):
    if args.sink == "file":
        os.makedirs(args.out, exist_ok=True)
        return FileSink(os.path.join(args.out, "v2x_packets.jsonl"))
    if args.sink == "udp":
        return UdpSink(args.udp_host, args.udp_port)
    return NullSink()


def _attacker_raw_channels(args, attack, world, sink, frame, gen_time):
    """LiDAR / camera raw-data channels (attacker's own sensors), mock only."""
    if attack.name in ("fake_object", "remove_object") and not args.no_lidar:
        pc = world.honest_pointcloud(seed=frame)
        spoofed_pc = attack.apply_pointcloud(pc)
        if args.save_artifacts and frame == 0:
            save_pcd(spoofed_pc, os.path.join(args.out, "lidar_spoofed.pcd"))
            save_pcd(pc, os.path.join(args.out, "lidar_honest.pcd"))
        return {"honest_points": int(pc.shape[0]), "spoofed_points": int(spoofed_pc.shape[0])}
    if attack.name == "camera":
        frame_img, projector = world.camera()
        spoofed_img = attack.apply_image(frame_img, projector)
        meta = {"width": projector.width, "height": projector.height,
                "fov": projector.fov_deg, "injected": attack.result.image_injections}
        if args.save_artifacts:
            from PIL import Image
            ref = os.path.join(args.out, f"camera_spoofed_{frame:03d}.png")
            Image.fromarray(spoofed_img).save(ref)
            meta["image_ref"] = ref
        sink.send(Packet.camera(world.attacker_id, seq=frame, sim_time=gen_time,
                                image_meta=meta))
        return {"camera_injections": attack.result.image_injections}
    return {}


def run_mock(args, attack, sink, csv_writer) -> dict:
    from .mock_world import MockWorld
    world = MockWorld.default_scene()
    summary = {"mode": "mock", "attack": attack.name,
               "attacker_id": world.attacker_id, "frames": []}
    n_frames, period = _plan(args)
    step = period or 1.0
    start = _time.monotonic()
    for frame in range(n_frames):
        if period:
            _sleep_until(start + frame * period)
        gen_time = float(frame) * step
        finfo = {"frame": frame, "senders": [], "messages": 0}
        for sid in world.sender_ids():
            honest = world.honest_cpm_for(sid, gen_time)
            is_atk = (sid == world.attacker_id)
            broadcast = attack.apply_cpm(honest) if is_atk else honest
            sink.send(Packet.cpm(broadcast, seq=frame))
            csv_writer.add(frame=frame, sim_time=gen_time, sender_id=sid,
                           sender_type="vehicle", is_attacker=is_atk,
                           honest=honest, broadcast=broadcast)
            finfo["senders"].append(sid)
            finfo["messages"] += 1
            if is_atk:
                finfo["attacker_cpm_diff"] = diff_cpms(honest, broadcast)
                finfo["raw"] = _attacker_raw_channels(args, attack, world, sink,
                                                      frame, gen_time)
                finfo["attack_result"] = attack.result.to_dict()
        summary["frames"].append(finfo)
    summary["frames_run"] = n_frames
    summary["rate_hz"] = args.rate
    summary["total_messages"] = sum(f["messages"] for f in summary["frames"])
    return summary


def run_carla(args, attack, sink, csv_writer) -> dict:
    try:
        import carla  # provided by the CarlaAir binary's bundled module
    except ImportError as e:
        print("ERROR: `carla` module not importable. Run inside the CarlaAir "
              "conda env / Docker image, with the sim reachable.", file=sys.stderr)
        raise SystemExit(2) from e
    from ..perception import carla_states_from_snapshot, build_cpm_from_objects

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    vehicles = [a for a in world.get_actors() if a.type_id.startswith("vehicle")]
    if not vehicles:
        raise SystemExit("No vehicles in the world. Spawn traffic first "
                         "(CarlaAir auto-spawns it; or run generate_traffic.py).")
    ids = [v.id for v in vehicles]
    attacker_id = args.attacker_id if args.attacker_id in ids else vehicles[0].id
    senders = vehicles if not args.max_senders else vehicles[:args.max_senders]
    if attacker_id not in [v.id for v in senders]:
        senders = [v for v in vehicles if v.id == attacker_id] + senders
    print(f"[carla] {len(senders)} broadcasting vehicles; attacker = actor "
          f"{attacker_id}")
    summary = {"mode": "carla", "attack": attack.name, "attacker_id": attacker_id,
               "n_senders": len(senders), "frames": []}
    sender_meta = {v.id: ("drone" if ("drone" in v.type_id or "uav" in v.type_id)
                          else "vehicle") for v in senders}
    n_frames, period = _plan(args)
    start = _time.monotonic()
    for frame in range(n_frames):
        if period:
            _sleep_until(start + frame * period)
        # ONE world fetch per round, shared by all senders (keeps 1 Hz feasible).
        snap = world.get_snapshot()
        gen_time = snap.timestamp.elapsed_seconds
        states = carla_states_from_snapshot(world, snap)
        by_id = {st.object_id: st for st in states}
        finfo = {"frame": frame, "senders": [], "messages": 0}
        for v in senders:
            ss = by_id.get(v.id)
            if ss is None:
                continue
            honest = build_cpm_from_objects(
                station_id=v.id, reference_position=ss.position, objects=states,
                generation_time=gen_time, station_type=sender_meta[v.id])
            is_atk = (v.id == attacker_id)
            broadcast = attack.apply_cpm(honest) if is_atk else honest
            sink.send(Packet.cpm(broadcast, seq=frame))
            csv_writer.add(frame=frame, sim_time=gen_time,
                           sender_id=v.id, sender_type=honest.station_type,
                           is_attacker=is_atk, honest=honest, broadcast=broadcast)
            finfo["senders"].append(v.id)
            finfo["messages"] += 1
            if is_atk:
                finfo["attacker_cpm_diff"] = diff_cpms(honest, broadcast)
                finfo["attack_result"] = attack.result.to_dict()
        summary["frames"].append(finfo)
    summary["frames_run"] = n_frames
    summary["rate_hz"] = args.rate
    summary["total_messages"] = sum(f["messages"] for f in summary["frames"])
    print(f"[carla] broadcast {summary['total_messages']} messages over "
          f"{n_frames} rounds @ {args.rate} Hz ({len(senders)} vehicles).")
    print("[carla] NOTE: raw LiDAR/camera injection needs a sensor loop; see "
          "omnet/README.md. CPM-level multi-vehicle spoofing is active.")
    return summary


def main(argv=None):
    p = argparse.ArgumentParser(description="CARLA V2X cooperative-perception spoofing")
    p.add_argument("--mode", choices=["mock", "carla"], default="mock")
    p.add_argument("--attack", choices=["none", "fake_object", "remove_object", "camera"],
                   default="fake_object")
    p.add_argument("--frames", type=int, default=0,
                   help="exact broadcast rounds (0 = derive from --duration/--rate)")
    p.add_argument("--rate", type=float, default=1.0,
                   help="broadcasts per vehicle per second, Hz (default 1)")
    p.add_argument("--duration", type=float, default=60.0,
                   help="wall-clock seconds to run when --frames is 0 (default 60)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--attacker-id", type=int, default=None,
                   help="actor id of the attacker vehicle (default: first vehicle)")
    p.add_argument("--max-senders", type=int, default=0,
                   help="cap broadcasting vehicles in carla mode (0 = all)")
    p.add_argument("--sink", choices=["file", "udp", "null"], default="file")
    p.add_argument("--out", default="out")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=47000)
    p.add_argument("--no-csv", action="store_true", help="skip messages.csv")
    p.add_argument("--save-artifacts", action="store_true")
    p.add_argument("--no-lidar", action="store_true")
    p.add_argument("--fake-x", type=float, default=15.0)
    p.add_argument("--fake-y", type=float, default=0.0)
    p.add_argument("--fake-z", type=float, default=0.0)
    p.add_argument("--fake-class", default="car")
    p.add_argument("--generator-cmd", default=None)
    p.add_argument("--remove-id", type=int, default=None)
    p.add_argument("--remove-x", type=float, default=None)
    p.add_argument("--remove-y", type=float, default=0.0)
    p.add_argument("--remove-z", type=float, default=0.0)
    args = p.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    attack = build_attack(args)
    sink = make_sink(args)
    class _NullCsv:
        def add(self, **k): pass
        def close(self): pass
        n_messages = 0
        n_object_rows = 0
    csv_writer = _NullCsv() if args.no_csv else ReportWriter(args.out)

    try:
        run = run_mock if args.mode == "mock" else run_carla
        summary = run(args, attack, sink, csv_writer)
    finally:
        sink.close()
        csv_writer.close()

    summary["messages_csv_rows"] = getattr(csv_writer, "n_messages", 0)
    summary["object_csv_rows"] = getattr(csv_writer, "n_object_rows", 0)
    if args.sink == "file":
        with open(os.path.join(args.out, "run_summary.json"), "w") as fh:
            json.dump(summary, fh, indent=2)
    print(json.dumps(summary, indent=2))
    if not args.no_csv:
        print(f"\nWrote {csv_writer.n_messages} messages to {os.path.join(args.out, 'messages.csv')}"
              f" and {csv_writer.n_object_rows} object-rows to perceived_objects.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
