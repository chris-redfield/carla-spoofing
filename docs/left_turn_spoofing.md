# Left Turn Assist spoofing — drone as a fake RSU at a crossroads

Whitepaper **use case 3b** (Fig. 12): *"a spoofer drone posing as an RSU and manipulating
communication, leading to a collision at the crossover. The inability of the Rx vehicle to
receive precise information accentuates the need for a secure communication framework."*

A drone impersonating a Road Side Unit erases a crossing vehicle from the cooperative
perception a car relies on while waiting to turn left. The car turns. They meet in the
middle of the junction.

This is the second scenario in the toolkit and reuses the first one's attack machinery
unchanged — identity spoofing plus object removal. What is new is the *manoeuvre*, the
*receiver application* deciding it, and the *reason the victim is blind*.

---

## The scene

```
                          ╔═══════════════╗
                          ║   CORNER      ║   hides the cross street
                          ║   BUILDING    ║   from the waiting ego
                          ╚═══════════════╝
                                              ▲   RSU 9001  (honest)
   ego ──────────────────►│       │           │   drone 9002 (attacker,
        approach          │       │           │     hovering alongside)
   ─────────────────────  │  JCT  │  ─────────────────
                    stop ─┤       ├─ conflict point
   ─────────────────────  │       │  ─────────────────
                          │   ▲   │
                          │   │   │
                          │ crossing vehicle
                          │ (hidden behind the corner)
```

* **RSU 9001** — honest infrastructure overlooking the junction. Sees everything.
* **ego** — waiting at the line to turn left. Sees only what the corner allows.
* **crossing** — a vehicle coming up the cross street. Hidden from the ego.
* **drone 9002** — the attacker, hovering at the roadside.

### Why a *permissive* left turn

The ego has a **green ball, not a protected arrow**. It is entitled to enter the junction;
the entire safety question is whether the gap is big enough. That matters, because it is
what makes the attack interesting: under attack the ego does not run a light or break a
rule. It correctly executes a legal manoeuvre on a world model that has been falsified.

All lights at the junction are frozen green for the run (`--live-lights` disables this).
Leaving Town10's cycle running would let a red phase stop the crossing vehicle mid-run,
and the scene would stop being reproducible for reasons having nothing to do with the
attack.

### Why a corner building, and not another car

In the do-not-pass scenario the ego could not see past the vehicle it wanted to overtake.
A junction needs a different occluder, and the choice is forced by geometry:

| hazard | where it is | what can hide it |
|---|---|---|
| oncoming through-traffic | straight down the road, nearly head-on | only another **vehicle** |
| cross-street traffic | approaching from the side | a **building** on the corner |

A corner building cannot hide a car that is directly ahead of you. So for the building to
do real work, the conflicting vehicle has to be the one the ego crosses paths with on the
intersecting road — which is still precisely "a collision at the crossover".

The decision logic handles both cases with one rule (anything not travelling the ego's way
and heading for the conflict point), so the oncoming-traffic variant needs no new code if
we want it later.

---

## How a run works, step by step

Both runs are the same code, the same scene and the same controller. The only
difference is which message stream the ego believes.

### Setup — identical for both

```
1. Connect; check the sim is on Town10HD (stop if not -- never reload it)
2. Census the world, destroy any pre-existing vehicles
3. Ego    <- spawn point 151, ~42 m from the junction
   Hazard <- spawn point 46, chosen FROM THE MAP: a road that actually
             feeds this junction, approaching from the left
4. Solve the hazard's speed so it arrives ~2 s after the ego
5. Freeze every light at the junction GREEN  (a permissive left turn)
6. Query the corner buildings -> static occluders
7. Place the drone back down the ego's approach, 20 m up
8. Synchronous mode, 0.05 s per tick
```

From here two rates run: **control at 20 Hz**, **messages at 1 Hz**.

### Each messaging round (once per simulated second)

Four CPMs are built from CARLA's ground truth:

| sender | range | sees |
|---|---|---|
| **RSU**, station 9001 | 160 m | everything — infrastructure outranges its receivers |
| **ego**, its own id | 75 m | filtered through the corner buildings: **not the hazard** |
| **drone**, station 9002 | 160 m | everything, honestly |
| **the forgery** | — | the drone's CPM, hazard deleted, re-stamped **as station 9001** |

Then they are fused, and a **third** decision is computed from omniscient
ground truth. Nobody acts on that one; it is the scoring reference.

### Each tick

The controller takes the latest decision and steps. Its **only** trigger is that
decision — no timers, no scripted steering. Every tick writes a `trajectory.csv`
row: pose, throttle/steer/brake, and the pure-pursuit target point.

### The honest run

```
t=0        APPROACH   drives ~42 m toward the junction
t~4        DO_NOT_TURN -- the RSU reports the hazard
                      brakes to the stop line
t=9.8      WAITING    holds while the hazard crosses in front of it
t=13.0     TURN       the junction is genuinely clear: pulls in and turns
t=17.0     CLEARED    out the other side. No collision.
```

It **does** turn. The honest ego is not a car that never goes; it is a car that
goes once the road is actually clear. Scoring against "did it turn" would
therefore prove nothing, which is why ground truth is scored instead.

### The attack run

```
t=0        APPROACH   identical start, identical speed
t~4        TURN       the forgery says the junction is empty
                      never slows, never stops
t=8.15     TURNING    commits within 8 m of the line
t=11.75    the hazard arrives. Collision at the crossover.
```

Ground truth said `DO_NOT_TURN` at the moment it committed — which is what makes
it an *unsafe* turn rather than merely a turn.

### Why the attacked ego never stops

Requiring a full halt at the line was an earlier simplification: it made both
runs pause identically and hid the difference. A driver who can see the junction
is clear flows through; one who cannot stops and waits. `commit_within_m` (8 m)
lets the ego commit while still rolling, so the behavioural signature of the
attack is now "never even slowed down".

Both runs still decide over the same stretch of road, so the comparison remains
about beliefs and not about geometry. Measured: 2.5–9 m works; at 14 m the
junction still reads clear in *both* runs and the honest ego commits too, then
has to abort.

---

## Why impersonation is required

Identical to the do-not-pass case, and just as load-bearing. A receiver keeps **one entry
per sending station**:

```
  FORGED UNDER THE DRONE'S OWN ID              FORGED UNDER THE RSU'S ID

  RSU   9001  {ego, crossing}                  RSU   9001  {ego, crossing}
  drone 9002  {ego}          ← deleted         drone 9001  {ego}   ← claims to be 9001
  ─────────────────────────────                ─────────────────────────────
  fused:      {ego, crossing}                  fused:      {ego}
              hazard SURVIVES                              hazard GONE
```

A message from a *new* sender is unioned with the RSU's, so deleting an object from it
changes nothing. Stamping it with the RSU's id makes it **supersede** the genuine report.
`fusion.fuse_latest_by_station` is the mechanism the attack exploits, and
`test_forging_under_the_drones_own_id_does_not_work` asserts the naive version fails.

---

## Running it

The scenario needs **Town10HD**, while the do-not-pass scenario needs Town01 and is the
compose default. Runtime `load_world` is unreliable in this CarlaAir build, so start the
simulator on the right map rather than relying on the scenario to switch it:

```bash
make up MAP=Town10HD     # terminal 1 — the simulator, on the right map
make left-turn           # terminal 2 — honest + spoofed, then the verdict
```

To go back to the do-not-pass scenario afterwards, `make down` then `make up`
(Town01 is the default).

**The scenario will not reload the map for you.** If the simulator is on the
wrong one it stops immediately and prints the two commands above. That is
deliberate: `load_world` has to tear down a level while a second client still
owns actors and the AirSim plugin sits in the same UE4 process, and it has been
seen completing, hanging and segfaulting on identical inputs. Restarting
someone's simulator as a side effect of running a scenario is worse than
stopping. `--allow-map-reload` opts back in.

`Town10HD_Opt` counts as `Town10HD` — same level, different build suffix.

```bash
make left-turn RUN=spoofed   # just the crash
make left-turn RUN=honest    # just the safe baseline
make left-turn-mock          # the same closed loop with no simulator at all
```

A single run writes its files but produces **no verdict**: every headline claim
is a *difference* between the two runs, so one run alone is a recording, not a
result.

Background traffic is swept out of the scene once per messaging round, but it is
tidier not to spawn it at all:

```bash
make down && make up MAP=Town10HD SPAWN_TRAFFIC=0
```

`SPAWN_TRAFFIC` only applies when the **simulator** starts, and `make left-turn`
never restarts it. Pedestrians are left alone either way — they are not vehicles
and cannot affect the decision.

The finished scene is held for 4 s before teardown (`--linger`; do-not-pass uses
7 s). Counted in **real** seconds, not sim seconds.

### Survey first, when something looks wrong

```bash
make left-turn-survey
```

Read-only: spawns nothing, drives nothing, sends no messages. It prints each
spawn point's pose, the junction ahead of it, that junction's arms with their
bearings, which exit is the left turn, every spawn that feeds the junction, and
— the line that matters — whether the ego and the hazard reach the **same** one.

Every failure this scenario has had was geometry that was inferred and never
checked. Ten seconds here beats a full run.

### Which spawn points, and why they are chosen from the map

The reference scene's indices are CARLA 0.9.13 numbering. On this 0.9.16 build
the ego's (126) still lands on the right road but only **16 m** from the line,
and the hazard's (33) lands **154 m away at a different junction** entirely.

So both are selected from the map instead: the ego gets the longest approach on
its own road (**151**, ~42 m), the hazard the longest road feeding the junction
from a conflicting direction (**46**, ~50 m). `--ego-spawn` / `--hazard-spawn`
override. Asking the map cannot go stale the way a constant does.

Useful flags: `--crossing-back` (how far back the hazard starts), `--crossing-speed`,
`--ego-back`, `--stop-line`, `--live-lights`, `--no-fly-drone`.

### Where the drone hovers

The surveyed pose from the reference scene is **not** reused as a position. That run
was taking close-up stills, so the drone sits almost on top of the junction — from
there it sees the roofs of the cars it is supposed to be watching and nothing of the
scene. It is pulled back down the ego's approach and lifted instead:

| flag | default | |
|---|---|---|
| `--drone-back` | 28 m | back down the ego's approach arm |
| `--drone-side` | 6 m | to the right of that arm, off the road |
| `--drone-height` | 20 m | high enough to see over the corner |

Its pose is cosmetic to the attack — a forged CPM claims the *impersonated* station's
position, never the attacker's — but pull it back far enough and the crossing vehicle
falls outside its 160 m perception range, at which point the forgery has nothing to
delete. That is checked, and warned about, exactly as in the do-not-pass scenario.

### `trajectory.csv` — reading what the car actually did

Written every control tick (20 Hz), for every actor. The decision log says what the
ego *believed*, once a second; this says what it *did*, and why:

| column | |
|---|---|
| `x, y, yaw_deg, speed_mps` | the pose, per tick |
| `throttle, steer, brake` | raw actuation |
| `target_x, target_y, target_bearing_deg` | **the pure-pursuit target point** |
| `path_progress_m, path_remaining_m` | position along the turn path |
| `ctrl_state, decision, dist_to_hold_m` | controller state and what drove it |

The target point is the important one. A vehicle that turns the wrong way is almost
always a controller faithfully chasing a target in the wrong place, and without both
in the same file you cannot tell that from a broken controller. `--no-trace` skips it.

A healthy left turn looks like this — heading sweeping left, target advancing along
the arc, progress increasing:

```
t     state     x       y      yaw     steer  target(x,y)       prog
2.7   TURNING   -7.76   -0.01  -2.0    -0.63  (-3.18, -2.64)    0.00
3.3   TURNING   -5.84   -0.35  -16.8   -0.58  (-3.01, -2.93)    0.34
3.9   TURNING   -3.62   -1.53  -37.8   -0.62  (-1.82, -5.41)    2.64
4.5   TURNING   -1.81   -3.62  -57.6   -0.46  (-0.84, -8.08)    5.21
```

---

## Results (mock backend)

Both runs start identically. Everything after that is belief.

| | honest | spoofed |
|---|---|---|
| reaches the line | 9.8 s, **stops** | never stops |
| starts the turn | **13.0 s** | **8.15 s** |
| ground truth at that moment | `TURN` | `DO_NOT_TURN` |
| collision | none | **11.75 s, in the junction** |
| completes the turn | 17.0 s | — |

```
attack_caused_unsafe_turn  : true
attack_caused_collision    : true
honest_unsafe_turn         : false
honest_collision           : false
evidence_valid             : true
```

The honest ego **does turn** — it is supposed to, once the junction is genuinely
clear. That is why the run is scored against an omniscient ground-truth decision
rather than against "did it turn at all".

Live CARLA: the junction, the approach, the turn shape and the pole clearance
are all resolved (see the log below). The end-to-end attack result on the live
map is **not yet confirmed**.

## Guards against a hollow result

Six things can make a collision look like evidence when it is not. Every one of
them announces itself rather than sitting silently in the output.

**1. The corner might not actually occlude.** If the ego can see the crossing vehicle with
its own sensors, the suppressed RSU report is irrelevant to its decision and any collision
proves nothing. The run says so on the first messaging round:

```
[lta] WARNING: the ego can see the crossing vehicle 202 with its own sensors, so the
corner is not occluding it. The suppressed RSU report is then irrelevant to the decision
and any collision is NOT evidence of the attack.
```

**2. The ego might hit scenery.** The collision sensor fires for anything, so
clipping a pole registered as a crash — in the *honest* run, which would have
cancelled the signal. Collisions are now classified: `collided` counts
**vehicle-vs-vehicle only**, scenery goes to `hit_scenery` and invalidates the
run. A car that cannot drive the junction cleanly cannot tell us anything about
spoofing.

**3. The ego might never turn at all.** A spawn on the wrong road leaves it
driving to a stop line that is not there, waiting out the whole run.

**4. The forgery might delete nothing.** An impersonated CPM supersedes the RSU's whether
or not the attacker edited it, so a run can collide with an empty `removed_ids` column.
`test_attack_actually_removed_something` asserts it did. This trap was found the hard way
in the do-not-pass scenario.

**5. The two runs might not differ at all.** Zero decision flips, or both runs
committing at the same instant, means the message stream changed nothing.

**6. A collision might land where ground truth said SAFE** — then whatever caused
it, it was not the attack.

All six print

```
THIS RUN IS NOT EVIDENCE OF THE ATTACK, whatever the verdict says:
  - ...
```

and exit non-zero. They exist because a live run once reported
`attack_caused_collision: true` while the hazard had stalled 50 m short of the
junction and the two cars had simply collided by coincidence. **The verdict flags
are not trusted on their own.**

---

## Relation to the reference scene

Map, weather, vehicle models, drone hover pose and spectator framing come from
`~/proj/CarlaNetpp/cooperative_perception_dev/use_case_3/main.py`.

| | value |
|---|---|
| map / weather | `Town10HD` / `CloudyNoon` |
| drone hover pose | `Location(-43.010139, 22.077925, 12.224535)`, `Rotation(pitch=-16.181599, yaw=89.799080)` |
| RSU (impersonated) | same spot at pole height, `z = 6.0`, station id `9001` |
| spectator | `Location(-62.067219, 3.026420, 27.673388)`, `Rotation(pitch=-50.634418, yaw=55.887512)` |
| original spawn points | 126 ego · 33 audi · 145 police — recorded for provenance, **not used** |

**The manoeuvre is not reused.** The reference staged it: a subprocess fed
`steer=-0.15 → -0.10 → -0.05` on `sleep()` timers at `elapsed_time > 5`, and the "good"
variant ran the identical turn at `> 9`. The turn happened whether or not anything was
attacking, and `BAD_SCENARIO` only changed *when*. Its "attack" was sensor noise
(`attach_all_sensors(drone_attack=True)`), which never touched a message.

Here the ego has a real controller (`control.LeftTurnController`) whose only trigger is
the receiver's decision, so honest and spoofed differ in **nothing but the message
stream**. Spawn indices are not reused either: they are CARLA 0.9.13 numbering and the
junction is found from the road graph instead.

---

## Files

| file | what it is |
|---|---|
| `left_turn_assist.py` | receiver-side decision: `(ego, objects) → TURN \| DO_NOT_TURN` |
| `geometry.py` | planar helpers + `EgoState`, shared with `do_not_pass_warning` |
| `fusion.py` | per-station fusion, vehicle occlusion, **building occlusion** |
| `control.py` | `LeftTurnController`, `PolylineReference`, `hermite_turn_path` |
| `scenarios/left_turn_spoofing.py` | the scenario, mock + CARLA backends |
| `tests/test_left_turn.py` | 19 tests |

Outputs land in `out/left_turn/{honest,spoofed}/`: `left_turn_decisions.csv`,
`messages.csv`, `perceived_objects.csv`, `v2x_packets.jsonl`, `run_summary.json`, plus
`comparison.json` with the verdict.
