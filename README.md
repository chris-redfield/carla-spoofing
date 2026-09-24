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

**Key point:** the attack itself is a *data-integrity attack on a broadcast
message*, never a control attack. The attacker only lies: read CARLA ground truth
as the sender's honest perception → forge the message → emit a packet. The network
is assumed perfect and is left to OMNeT++ (below).

What the *victim* then does is a separate question, and the two scenarios answer
it differently. The multi-vehicle runner (`make spoof`) stops at the message and
leaves the consequence to be measured downstream. The Do-Not-Pass scenario
(below) goes further and gives the victim a controller, so the attack's
consequence — a collision — is produced rather than asserted.

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

## Scenario: Do-Not-Pass Warning spoofing (whitepaper use case 1b)

A **drone impersonating a Road Side Unit** makes a car overtake into oncoming
traffic. Two-lane road on **Town01**: the ego follows a slow **lead** vehicle it
cannot see round, an **oncoming** car approaches in the other lane, and an **RSU**
broadcasts a CPM of the whole segment. The drone rebroadcasts that view stamped
with the **RSU's station id**, minus the oncoming car. Because a receiver keeps
one entry per station, the forgery *replaces* the genuine report instead of adding
to it — so the hazard vanishes, the ego's warning flips `DO_NOT_PASS → PASS`, and
it pulls out.

```bash
make up                # start the sim (Town01 by default — where this scene lives)
make do-not-pass       # honest baseline + attack run, then the comparison
make do-not-pass-mock  # the same closed loop with no simulator at all
```

The sim starts on **Town01** so nothing has to reload the map at runtime — that call
is unreliable in this build (see the scenario doc). `do-not-pass` still clears
auto-spawned traffic and places the drone at the roadside fake-RSU pose itself.

**This scenario is closed-loop, and deliberately unscripted.** The ego is driven
by `control.DoNotPassController`, whose *only* trigger for an overtake is the
warning it computes from received messages — no timer, no scripted steering. The
honest run and the attack run differ in nothing but the message stream, so the
outcome is a causal result rather than a staged one:

| | honest run | spoofed run |
|---|---|---|
| overtakes | yes, once the road really is clear | yes, immediately |
| ground truth at that moment | `PASS` | `DO_NOT_PASS` |
| collision | none | head-on with the oncoming car |

A detail worth noting: in the attack run the ego *does* spot the oncoming vehicle
with its own sensors once it leaves its lane and the lead stops occluding it — but
by then the manoeuvre is committed and it is too late to abort. That behaviour is
emergent, not coded.

Outputs land in `out/do_not_pass/{honest,spoofed}/`, plus a top-level
`comparison.json` with the verdict.

## Scenario: VRU Crossing Warning spoofing

A **drone impersonating a Road Side Unit** makes a car run over a cyclist and a
pedestrian. Four-way crossroads on **Town01**: the ego approaches while the two
VRUs (Vulnerable Road Users) cross from the blind side street, and an **RSU**
broadcasts a CPM covering the crossing. The drone rebroadcasts that view stamped
with the **RSU's station id**, minus both VRUs — so the ego's crossing assistant
concludes nobody is coming.

- **Cenário 1 — Operação Nominal (`make vru-warning RUN=honest`):** o pedestre e
  o ciclista realizam a travessia no cruzamento em rota de colisão com o
  veículo. A RSU detecta os usuários vulneráveis e emite um alerta seguro de
  frenagem, fazendo com que o veículo pare a tempo e evite o acidente.
- **Cenário 2 — Ataque de *Spoofing* (`make vru-warning RUN=spoofed`):** durante
  a mesma travessia do pedestre e do ciclista, um drone executa um ataque de
  *spoofing* contra a comunicação V2X. A mensagem adulterada instrui o veículo
  a ignorar o risco e manter sua trajetória no cruzamento, resultando no
  atropelamento de ambos.

```bash
make up                # start the sim (Town01 by default — where this scene lives)
make vru-warning        # honest baseline + attack run, then the comparison
make vru-warning-mock   # the same closed loop with no simulator at all
```

**This scenario is closed-loop, and deliberately unscripted**, same as
Do-Not-Pass: the ego is driven only by its own controller reacting to the
messages it receives, so the honest and spoofed runs differ in nothing but the
message stream.

| | honest run | spoofed run |
|---|---|---|
| crossing decision | brakes to a full stop, waits for both VRUs to clear | never warned, drives straight through |
| ground truth at that moment | cyclist + pedestrian on the crossing | cyclist + pedestrian on the crossing |
| collision | none | hits the cyclist and the pedestrian |

Outputs land in `out/vru_warning/{honest,spoofed}/`, plus a top-level
`comparison.json` with the verdict.

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
  attacks/             fake_object · remove_object · camera_injection · identity_spoof
  perception.py        build an honest CPM from CARLA (or mock) ground truth
  fusion.py            receiver-side CPM fusion (one entry per station) + occlusion
  do_not_pass_warning.py  receiver-side DNPW decision: objects + ego pose -> PASS/DO_NOT_PASS
  control.py           simple lane-follow / overtake controllers (CARLA + kinematic mock)
  lidar.py             point-cloud inject / carve primitives
  report.py            messages.csv · perceived_objects.csv · do_not_pass_decisions.csv
  scenarios/           run_scenario.py (multi-vehicle runner) · mock_world.py (no-sim scene)
                       do_not_pass_spoofing.py (closed-loop DNPW scenario)
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
