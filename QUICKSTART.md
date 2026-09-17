# QUICKSTART — from a fresh machine to a running spoofing sim

This gets you from **nothing** to a live CARLA simulation with the flying drone,
running V2X spoofing attacks, in a handful of `make` commands. No prior knowledge
of the project is assumed.

> **What this is:** autonomous-vehicle cybersecurity **research in simulation**.
> A malicious vehicle broadcasts *fake* cooperative-perception messages to its
> neighbours (a phantom car, or a real object erased). Everything runs in Docker
> for reproducibility. See [README.md](README.md) for the concepts.

---

## 0. Prerequisites (install these first)

You need a **Linux machine (Ubuntu 22.04 recommended) with an NVIDIA GPU**
(needs **~4 GB of free VRAM**; an 8 GB card is plenty — lower it with `QUALITY=Low` on smaller GPUs).
The project runs in Docker, but a few things must exist on the host first —
the project does **not** install these for you:

```bash
# tools
sudo apt update && sudo apt install -y git make curl

# Docker Engine + Compose plugin  (https://docs.docker.com/engine/install/ubuntu/)
# ...then let your user run docker without sudo:
sudo usermod -aG docker $USER && newgrp docker   # (or log out/in)

# NVIDIA GPU driver — must already work:
nvidia-smi        # should print your GPU. If not, install the driver first.
```

You do **not** need to install Python, CARLA, or Unreal Engine — they live inside
the container / the downloaded binary.

**Check everything at once:**
```bash
git clone <this-repo> carla-spoofing && cd carla-spoofing
make doctor
```
`make doctor` prints a PASS/WARN/FAIL list for git, make, Docker, the GPU driver,
the container toolkit, disk space (~25 GB needed), and your display. Fix any
**FAIL** before continuing; **WARN**s for the toolkit and the binary are resolved
by the next two steps.

---

## 1. One-time host setup (GPU access for containers)

```bash
make toolkit
```
Installs the **NVIDIA Container Toolkit** (needs your sudo password) so containers
can use the GPU, and restarts Docker. This is the one host-level piece that can't
live in an image. Run it once per machine.

---

## 2. Fetch CarlaAir + build the image

```bash
make setup
```
This **downloads the CarlaAir binary (6.85 GB, resumable)**, unpacks it into
`vendor/`, and builds the Docker image. First run takes a while (download +
~10 min build). It's safe to re-run — finished steps are skipped.

> CarlaAir = CARLA 0.9.16 + Unreal Engine 4.26 with an integrated AirSim flying
> drone. (The newest CARLA has no drone, which is why this version is used.)

---

## 3. Start the simulator

```bash
make up
```
Opens the **CarlaAir window on your desktop** with the map (Town01, where the
do-not-pass scenario lives — `MAP=Town10HD make up` for the city instead), ~10
vehicles + pedestrians of traffic, and the drone. Leave this terminal running —
it's the live simulator (CARLA on port 2000, drone/AirSim on 41451).

- **Fly the drone:** `W A S D` move, **mouse** look, **scroll** speed, `N` weather,
  `H` help, `Tab` release the mouse. The drone auto-takes-off to a stable hover
  on spawn (it won't fall).
- **No graphical desktop?** (a remote server) use `make headless` instead — same
  sim, no window.
- The red **REC** icon in the corner is CarlaAir's built-in screen recorder
  (toggle `F`); it's unrelated to our data.

---

## 4. Run a spoofing attack (in a second terminal)

```bash
cd carla-spoofing
make spoof                          # phantom-car injection (default: 60 s @ 1 Hz)
make spoof ATTACK=remove_object     # erase a real object
make spoof DURATION=30 RATE=2       # 30 s, each vehicle emits twice per second
```
By default the attack runs for **~60 seconds** with **every vehicle broadcasting
once per second** — the attacker's message spoofed, the rest honest — so the
command blocks for about a minute. Tune with `DURATION=` (seconds) and `RATE=`
(Hz per vehicle). With ~10 vehicles that is ~600 messages (60 spoofed). Results
land in `out/`:

| File | What it is |
|---|---|
| `out/messages.csv` | **Human-readable.** One row per perceived object per message. Filter `message_kind = spoofed` or `spoof_flag != REAL` to see exactly what the attacker faked (`INJECTED` / `REMOVED`). |
| `out/v2x_packets.jsonl` | The V2X packets, one JSON per line — the hand-off consumed later by OMNeT++. |
| `out/run_summary.json` | Per-run summary (senders, what was spoofed). |

Open `out/messages.csv` in any spreadsheet to see the attack.

---

## 5. Stop

```bash
make down        # stop the simulator container
```

---

## All commands

```bash
make doctor    # check host prerequisites
make toolkit   # install NVIDIA container toolkit (once, sudo)
make setup     # download + extract CarlaAir + build image
make up        # start sim WITH a window (default)
make headless  # start sim with NO window (servers)
make spoof     # attack the live sim: default 60 s @ 1 Hz/vehicle
               # (ATTACK=fake_object|remove_object|camera, RATE=, DURATION=)
make logs      # tail the simulator logs
make down      # stop the simulator
make help      # list all targets
```

## If something breaks
- `make doctor` first — it catches most from-scratch issues.
- Sim won't start / exits: `make logs` and look for the failure.
- Window doesn't appear: ensure you're at a graphical (X11) session; `make up`
  runs `xhost +local:` for you. On a headless box use `make headless`.
- `docker: permission denied`: you're not in the `docker` group (see step 0).
