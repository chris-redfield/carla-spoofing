# CARLA V2X Cooperative-Perception Spoofing

Sensor / V2X communication spoofing research for autonomous-vehicle
cybersecurity, built on **CARLA via [CarlaAir](https://github.com/louiszengCN/CarlaAir)
v0.1.7** (CARLA 0.9.16 + Unreal Engine 4.26, with an integrated **flying drone**
/ AirSim multirotor). Everything runs in **Docker** for reproducibility.

> **Simulation-only research.** All attacks operate inside CARLA and produce
> data files/packets. Nothing here attacks a real vehicle or network.

## What it does

An **attacker station** (a vehicle or the drone) broadcasts forged
**cooperative / collective perception** data to all neighbours. Three attacks:

| Attack | CPM (object list) | LiDAR point cloud | Camera |
|---|---|---|---|
| `fake_object` | adds a phantom object | injects synthetic returns | *(via `camera`)* |
| `remove_object` | drops a real object | carves out its returns | — |
| `camera` | — | — | composites a fake vehicle (pluggable AI image generator) |

The spoof is serialised into **V2X packets** and dropped into a hand-off that
**OMNeT++** consumes later. Per the plan, the **network is assumed perfect** and
is left to OMNeT++ (see [`omnet/`](omnet/README.md)).

```
honest perception  ──►  attack  ──►  V2X packet  ──►  sink  ──►  OMNeT++
 (CARLA or mock)      (spoofing)     (CPM/CAMERA)   (.jsonl/UDP)  (network)
```

## Quick start — no CARLA, no GPU (mock mode)

The attack logic is fully testable offline with just `numpy` + `Pillow`:

```bash
pip install -e .
carla-spoof --mode mock --attack fake_object --fake-x 15 --save-artifacts --out out
carla-spoof --mode mock --attack remove_object --remove-x 40 --remove-y -3.5 --out out
carla-spoof --mode mock --attack camera --fake-x 18 --fake-y 1 --save-artifacts --out out
python examples/demo_fake_object_offline.py
pytest -q          # 10 tests
```

Outputs land in `out/`: `v2x_packets.jsonl` (the OMNeT++ hand-off),
`lidar_{honest,spoofed}.pcd`, `camera_spoofed_*.png`, `run_summary.json`.

## Full environment (Docker + CarlaAir + drone)

```bash
# 1. GPU access for containers (host, once; needs sudo):
bash scripts/install_nvidia_toolkit.sh

# 2. Fetch + unpack the CarlaAir binary (~6.85 GB, resumable):
bash scripts/download_carlaair.sh
bash scripts/extract_carlaair.sh

# 3. Build the image and start the simulator (headless, GPU):
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up carla-sim

# 4. In another shell — spoof against the running sim:
docker compose -f docker/docker-compose.yml run --rm spoofing \
  carla-spoof --mode carla --host carla-sim --port 2000 \
  --attack fake_object --fake-x 12 --out /workspace/out
```

The flying **drone** is controlled via the AirSim client on port `41451`
(`airsim.MultirotorClient(port=41451)`), and can act as an aerial attacker or
sensor sharing perception with ground vehicles.

## Layout

```
src/carla_spoofing/
  v2x/cpm.py         CPM information model (ETSI-inspired) + (de)serialisation
  v2x/packet.py      packet envelope + sinks (File .jsonl / UDP / Null) for OMNeT++
  lidar.py           point-cloud inject / carve primitives
  perception.py      honest CPM from a CARLA world or plain object states
  attacks/           fake_object · remove_object · camera_injection (+ base)
  scenarios/         run_scenario.py (CLI) · mock_world.py (CARLA-free scene)
docker/              Dockerfile · docker-compose.yml · entrypoint.sh
scripts/             download / extract CarlaAir · install NVIDIA toolkit
omnet/               packet schema + OMNeT++/Artery integration notes
examples/  tests/
```

## Coordinate frame

All poses are in the **CARLA world frame**: left-handed, X-forward, Y-right,
Z-up, metres, degrees. Kept end-to-end so CARLA replay and OMNeT++ agree.

## Environment notes

- Built for CarlaAir **v0.1.7** (CARLA 0.9.16 / UE 4.26). The newest CARLA
  (0.10.0, UE 5.5) has **no** drone plugin, which is why 0.9.16 is used.
- The `carla` Python module ships **inside** the CarlaAir binary; do not
  `pip install carla` alongside it.
