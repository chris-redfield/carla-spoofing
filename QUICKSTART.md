# QUICKSTART — running everything from scratch

Two tracks. **Track A** validates the spoofing logic in ~2 minutes with only
Python (no GPU, no simulator, no 6.85 GB download). **Track B** brings up the
full CarlaAir simulator + flying drone in Docker and spoofs against it.

Start with A to prove the pipeline, then do B.

---

## Prerequisites (host)
- Ubuntu 22.04, Python 3.8+
- For Track B only: NVIDIA GPU + driver (you have an RTX 4070), Docker + Compose,
  ~25 GB free disk, and the NVIDIA Container Toolkit (installed in step B1).

---

## Track A — spoofing logic only (no GPU, no sim)   ✅ verified working

```bash
cd ~/proj/carla-spoofing

# 1. Isolated env + install the toolkit
python3 -m venv .venv && source .venv/bin/activate
pip install -e .

# 2. Run each attack in mock mode (writes packets + artifacts to ./out)
carla-spoof --mode mock --attack fake_object   --fake-x 15 --fake-y 0  --save-artifacts --out out
carla-spoof --mode mock --attack remove_object --remove-x 40 --remove-y -3.5           --out out
carla-spoof --mode mock --attack camera        --fake-x 18 --fake-y 1  --save-artifacts --out out

# 3. Run the test suite (should print "10 passed")
pip install pytest && pytest -q

# 4. Minimal scripted demo (honest vs spoofed CPM side by side)
python examples/demo_fake_object_offline.py
```

**What to check in `out/`:**
- `v2x_packets.jsonl` — the V2X packets handed off to OMNeT++ (one JSON per line).
  `fake_object` adds an object id `999001`; `remove_object` makes a real id vanish.
- `lidar_honest.pcd` vs `lidar_spoofed.pcd` — open in CloudCompare/Open3D; the
  phantom cluster appears (fake_object) or a real object is carved out (remove_object).
- `camera_spoofed_000.png` — a fake car composited into the frame.
- `run_summary.json` — per-frame diff + attack ground-truth.

---

## Track B — full CarlaAir sim + drone in Docker   ⚠️ first real run on this machine

### B1. NVIDIA Container Toolkit (once, needs sudo)
```bash
bash scripts/install_nvidia_toolkit.sh
# self-test — should print your RTX 4070:
docker run --rm --gpus all nvidia/cuda:12.3.2-base-ubuntu22.04 nvidia-smi
```

### B2. Get the CarlaAir binary (6.85 GB, resumable)
```bash
bash scripts/download_carlaair.sh    # verifies exact byte count at the end
bash scripts/extract_carlaair.sh     # -> vendor/CarlaAir-v0.1.7/CarlaAir.sh
```

### B3. Build the image
```bash
docker compose -f docker/docker-compose.yml build
```

### B4. Start the simulator (headless, GPU)
```bash
docker compose -f docker/docker-compose.yml up carla-sim
# leave this running; it serves CARLA RPC on :2000 and the drone (AirSim) on :41451
```

### B5. Spoof against the running sim (second terminal)
```bash
# CPM-level fake-object attack broadcast from a vehicle in the sim:
docker compose -f docker/docker-compose.yml run --rm spoofing \
  carla-spoof --mode carla --host carla-sim --port 2000 \
  --attack fake_object --fake-x 12 --out /workspace/out

# object removal:
docker compose -f docker/docker-compose.yml run --rm spoofing \
  carla-spoof --mode carla --host carla-sim --port 2000 \
  --attack remove_object --remove-x 40 --remove-y -3.5 --out /workspace/out
```
> If the sim has no vehicles yet, spawn some first with CARLA's traffic generator
> (`generate_traffic.py`) — the attacker/sender is picked from existing vehicles.

### B6. Fly the drone (optional, in the spoofing container's env)
```python
import airsim
d = airsim.MultirotorClient(port=41451)   # sim exposes 41451
d.confirmConnection(); d.enableApiControl(True); d.armDisarm(True)
d.takeoffAsync().join()
d.moveToPositionAsync(80, 30, -25, 5).join()   # aerial attacker / sensor vantage
```

---

## The OMNeT++ hand-off (later — network assumed perfect)
Both tracks write `out/v2x_packets.jsonl`. Feed it to OMNeT++ by replaying each
line at its `header.sim_time`, or emit live over UDP with `--sink udp --udp-port 47000`.
Schema + Artery/INET wiring: see [`omnet/README.md`](omnet/README.md) and
[`omnet/cpm_packet_schema.md`](omnet/cpm_packet_schema.md).

---

## Troubleshooting
- **`carla` import error in Track B** — the module ships inside the CarlaAir
  binary; make sure `vendor/CarlaAir-v0.1.7/` is extracted and mounted. Never
  `pip install carla` alongside it.
- **`docker: could not select device driver … gpu`** — B1 not done or Docker
  not restarted.
- **Sim exits immediately / black** — it runs `-RenderOffScreen`; check
  `docker compose logs carla-sim`. 8 GB VRAM is enough for 0.9.16/UE4.26.
- **Download wrong size** — `scripts/download_carlaair.sh` fails if the byte
  count isn't exactly 6,846,384,047; just re-run it (resumable).
