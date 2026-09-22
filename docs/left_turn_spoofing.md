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

Both runs put the ego at the stop line at **2.6 s**. Everything after that is belief.

| | honest | spoofed |
|---|---|---|
| reaches the line | 2.6 s | 2.6 s |
| starts the turn | **6.0 s** | **2.6 s** (at once) |
| ground truth at that moment | `TURN` | `DO_NOT_TURN` |
| collision | none | **4.90 s, in the junction** |
| completes the turn | 10.1 s | — |

```
attack_caused_unsafe_turn  : true
attack_caused_collision    : true
honest_unsafe_turn         : false
honest_collision           : false
```

The honest ego **does turn** — it is supposed to, once the junction is genuinely clear.
That is why the run is scored against an omniscient ground-truth decision rather than
against "did it turn at all".

Live CARLA: the junction is found correctly. The **first live run drove the ego across
the junction** — `Junction.get_waypoints()` returns one `(entry, exit)` pair per turning
movement through the whole junction, and the exit was being chosen by "most leftward"
without checking the movement started on the *ego's own arm*. It could therefore return
the exit of a turn beginning on a different approach, and the ego faithfully steered at
it. Fixed in `pick_left_exit`, which filters by entry arm first and is unit-tested at
six ego headings. Two supporting fixes: the turn path now runs `--exit-run-out` past the
junction so the controller has something to aim at while it straightens up, and the
controller uses tighter gains (4–9 m lookahead instead of 5–18) because an 18 m lookahead
on a ~20 m arc puts the target at the exit and cuts the corner.

Still to confirm on the live map: that the corrected turn tracks the lane.

---

## Guards against a hollow result

Two things in this scenario can make a collision look like evidence when it is not. Both
announce themselves rather than sitting silently in the output.

**1. The corner might not actually occlude.** If the ego can see the crossing vehicle with
its own sensors, the suppressed RSU report is irrelevant to its decision and any collision
proves nothing. The run says so on the first messaging round:

```
[lta] WARNING: the ego can see the crossing vehicle 202 with its own sensors, so the
corner is not occluding it. The suppressed RSU report is then irrelevant to the decision
and any collision is NOT evidence of the attack.
```

**2. The forgery might delete nothing.** An impersonated CPM supersedes the RSU's whether
or not the attacker edited it, so a run can collide with an empty `removed_ids` column.
`test_attack_actually_removed_something` asserts it did. This trap was found the hard way
in the do-not-pass scenario.

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
