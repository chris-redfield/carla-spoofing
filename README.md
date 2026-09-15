# CARLA V2X Cooperative-Perception Spoofing

Autonomous-vehicle cybersecurity research **in simulation**. A malicious vehicle
broadcasts **fake cooperative-perception messages** to the vehicles around it —
inventing an object that isn't there, or erasing one that is — to study how such
spoofing corrupts a neighbour's view of the world. Built on **CARLA via
[CarlaAir](https://github.com/louiszengCN/CarlaAir) v0.1.7** (CARLA 0.9.16 +
Unreal Engine 4.26, with an integrated flying **drone**), fully **Dockerized**
for reproducibility.

_Hardware: an NVIDIA GPU with ~4 GB free VRAM (8 GB card is plenty), ~25 GB disk._

> Simulation only. Nothing here touches a real vehicle or a real network. The
> attacks produce data files/packets inside CARLA.

**New here? Go to [QUICKSTART.md](QUICKSTART.md)** — it takes you from a fresh
machine to a running sim in a few `make` commands.

---

## The idea in one minute

In **cooperative (collective) perception**, every vehicle runs its own sensors,
detects nearby objects, and **broadcasts what it sees** as a **CPM (Collective
Perception Message)**. Neighbours *fuse* those reports into their own world model
— that's the benefit: you see things your own sensors can't.

The attack targets that message. A malicious vehicle broadcasts a CPM that **lies**:

- **`fake_object`** — adds a phantom object (e.g. a stopped car ahead) → a victim
  might brake for nothing.
- **`remove_object`** — deletes a real object (e.g. a pedestrian) → a victim never
  learns it's there.
- **`camera`** — forges the shared camera image with an AI-generated vehicle
  (pluggable image generator).

**Key point:** this is a *data-integrity attack on a broadcast message*, not a
control attack. We never drive any car. The implementation is: read CARLA ground
truth as the sender's honest perception → forge the message → emit a packet. The
network is assumed perfect and is left to OMNeT++ (below).

Every vehicle broadcasts its own honest CPM each cycle; **one** vehicle is the
attacker whose CPM is poisoned — so the output is a realistic stream of mostly-honest
messages with one liar, which is exactly what a victim's fusion (and OMNeT++) consumes.

---

## Running it

Everything is driven by `make` (see [QUICKSTART.md](QUICKSTART.md) for the
from-scratch path and prerequisites):

```bash
make doctor    # check host prerequisites (Docker, GPU driver, toolkit, disk, display)
make toolkit   # install NVIDIA container toolkit (once, sudo)
make setup     # download + extract CarlaAir (6.85 GB) + build the image
make up        # start the sim WITH a window (default; 'make headless' for servers)
make spoof     # attack the live sim; default 60 s @ 1 Hz per vehicle
               # (ATTACK=fake_object|remove_object|camera, RATE=, DURATION=)
make down      # stop the sim
```

**The drone** is an AirSim multirotor in the same world; it auto-hovers on spawn
and you fly it in the window (WASD) or via `airsim.MultirotorClient(port=41451)`.

## Outputs (in `out/`)

| File | Purpose |
|---|---|
| `messages.csv` | Human-readable: one row per perceived object per message; `message_kind` (honest/spoofed) and `spoof_flag` (REAL/INJECTED/REMOVED) columns make the attack obvious. |
| `v2x_packets.jsonl` | The V2X packets (one JSON/line) — the OMNeT++ hand-off. |
| `run_summary.json` | Per-run summary. |
| `lidar_*.pcd`, `camera_spoofed_*.png` | Point-cloud / camera artifacts (mock mode). |

## OMNeT++ integration (the "integrate at the end" step)

The network is **assumed perfect** and deferred to OMNeT++. CARLA only *produces*
the spoofed packets; an OMNeT++ model replays them so a victim node receives them.
The packet schema and an Artery/INET integration guide are in
[`omnet/`](omnet/README.md). This receive side is documented but not yet built.

## Project layout

```
Makefile               one-command workflows (make help)
scripts/               doctor, toolkit install, download/extract CarlaAir, GUI launch
docker/                Dockerfile, docker-compose.yml (+ .headless.yml override), entrypoint
src/carla_spoofing/
  v2x/cpm.py           the CPM message model
  v2x/packet.py        packet format + sinks (file .jsonl / UDP) for OMNeT++
  attacks/             fake_object · remove_object · camera_injection
  perception.py        build an honest CPM from CARLA (or mock) ground truth
  lidar.py             point-cloud inject / carve primitives
  report.py            messages.csv writer
  scenarios/           run_scenario.py (multi-vehicle runner) · mock_world.py (no-sim scene)
omnet/                 packet schema + OMNeT++/Artery integration notes
vendor/                the downloaded CarlaAir binary (git-ignored, ~24 GB)
```

## How it's packaged

- **CarlaAir binary** is downloaded to `vendor/` and **mounted** into the container
  (not baked into the image) — keeps the image lean (~3–4 GB) and rebuilds fast.
- **Window mode is the default**; `docker/docker-compose.headless.yml` overrides to
  headless for servers/CI.
- The `carla` Python module ships inside the CarlaAir binary — never `pip install
  carla` alongside it.

## Known limitations / next steps

1. **Live LiDAR/camera sensor loop** not wired: point-cloud inject/carve and camera
   injection currently run on synthetic data; against the live sim only the CPM
   (object-list) is spoofed.
2. **Real AI image generator** not plugged in (procedural sprite default; a hook
   exists for any external generator).
3. **OMNeT++ receive side** documented but not implemented.

## Coordinate frame

All poses are in the **CARLA world frame**: left-handed, X-forward, Y-right, Z-up,
metres, degrees — kept end-to-end so CARLA and OMNeT++ agree.
