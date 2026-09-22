# QUICKSTART — from a fresh machine to a running spoofing sim

This gets you from **nothing** to a live CARLA simulation with the flying drone,
running V2X spoofing attacks, in a handful of `make` commands. No prior knowledge
of the project is assumed.

> **What this is:** autonomous-vehicle cybersecurity **research in simulation**.
> An attacker broadcasts *fake* cooperative-perception messages to its neighbours
> — a phantom car, or a real object erased. In the headline scenario a **drone
> impersonating a Road Side Unit** erases an oncoming car, and the victim
> overtakes into it. Everything runs in Docker for reproducibility. See
> [README.md](README.md) for the concepts.

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

## 4. Watch the attack cause a crash (in a second terminal)

```bash
cd carla-spoofing
make do-not-pass
```

The **Do-Not-Pass Warning** scenario, and the clearest thing to look at first. A
drone hovering at the roadside impersonates an RSU and deletes the oncoming car
from what it reports. The victim's overtaking assistant, reasoning correctly over
a poisoned world model, concludes the road is clear.

It runs **twice**, and the comparison is the point:

| | what you see |
|---|---|
| **honest** | the ego waits behind the slow lead while the oncoming car passes, *then* overtakes safely |
| **spoofed** | the ego pulls out almost immediately and hits the oncoming car head-on |

Nothing differs between the two runs but the messages — the ego is driven by its
own controller, with no scripted steering anywhere. After the crash the scene is
held on screen for a few seconds before the vehicles are cleared.

```bash
make do-not-pass RUN=spoofed   # just the crash
make do-not-pass RUN=honest    # just the safe baseline
make do-not-pass-mock          # the same closed loop with no simulator at all
```

Results land in `out/do_not_pass/`:

| File | What it is |
|---|---|
| `comparison.json` | the verdict — `attack_caused_collision`, `attack_caused_unsafe_overtake` |
| `<run>/do_not_pass_decisions.csv` | one row per decision: what the ego concluded, what it *would* have concluded from the honest messages, and the ground truth |
| `<run>/messages.csv` | who transmitted what, which station id they **claimed**, and what was deleted |

Full walkthrough with diagrams: [docs/do_not_pass_spoofing.md](docs/do_not_pass_spoofing.md).

---

## 5. The same attack at a crossroads (Left Turn Assist)

The second scenario, and the second whitepaper use case. Same attacker — a drone
impersonating an RSU — against a different safety application. This one needs
**Town10HD**, so the simulator has to be started on that map:

```bash
make up MAP=Town10HD     # terminal 1 — note the map
make left-turn           # terminal 2
```

If the simulator is already running on another map, `make left-turn` stops and
tells you so rather than reloading it — that reload is unreliable in this build
and would restart your sim as a side effect. Do `make down` first.

A car waits at the line to make a **permissive left turn**: it has a green ball,
so it is allowed to go, and the only question is whether the junction is clear.
A building on the corner hides the cross street, so it cannot answer that
question itself and has to trust what the RSU tells it. The drone deletes the
crossing vehicle from that report.

| | what you see |
|---|---|
| **honest** | the car waits at the line until the crossing vehicle has gone through, then turns |
| **spoofed** | the car turns the moment it reaches the line, into a vehicle it has been told is not there |

Both runs reach the line at the same moment — only the waiting differs, so the
comparison isolates the messages and nothing else.

```bash
make left-turn RUN=spoofed   # just the crash
make left-turn RUN=honest    # just the safe baseline
make left-turn-mock          # no simulator at all — and no Town10HD needed
```

Results land in `out/left_turn/`, same shape as above, with
`<run>/left_turn_decisions.csv` in place of the do-not-pass one.

> **Note:** this scenario is verified end to end in the mock backend; the
> Town10HD geometry has not yet been checked against the live map.

Full walkthrough with diagrams: [docs/left_turn_spoofing.md](docs/left_turn_spoofing.md).

---

## 6. Run the multi-vehicle spoofing attack

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

## 7. Stop

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
make do-not-pass       # drone-as-fake-RSU crash scenario (RUN=honest|spoofed|both)
make do-not-pass-mock  # the same, with no simulator at all
make left-turn         # the same attack at a crossroads — needs 'make up MAP=Town10HD'
make left-turn-mock    # the same, with no simulator at all
make spoof     # multi-vehicle attack on the live sim: default 60 s @ 1 Hz/vehicle
               # (ATTACK=fake_object|remove_object|camera, RATE=, DURATION=)
make test      # unit tests (no sim, no GPU)
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
