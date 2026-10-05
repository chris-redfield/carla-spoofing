"""Whether a pair of runs can be evidence of the attack at all.

A scenario's verdict compares *outcomes*: the spoofed run crashed, the honest
one did not, so the attack caused the crash. That is necessary and nowhere near
sufficient -- it can be true by coincidence, and once was.

The checks here ask the other question: did the forged messages, and only the
forged messages, produce the difference? They were written after a live left-turn
run that reported ``attack_caused_collision: true`` and proved nothing. Both runs
turned at exactly the same instant, every decision row read TURN/TURN/TURN, the
hazard was never once counted as a conflict, and the two cars collided purely
because they happened to occupy the same space. Every outcome flag was correct.
The experiment was worthless.

So this is a validity check on the experiment, not a measurement of the attack,
and it is deliberately separate from the verdict: a verdict can be *withheld*.
The unit tests check the decision logic in mock; these checks look at what a live
run actually produced, which no amount of unit testing can stand in for.

Shared by all three scenarios so the wording and the exit code cannot drift
apart. Each scenario supplies its own vocabulary through a `ManoeuvreSpec`:
"turn", "overtake" and "crossing" are the same idea wearing different nouns.
"""

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

HONEST, SPOOFED = "honest", "spoofed"

# Anything the ego can hit that is not a road user. A kerb, a street sign or the
# RSU's own pole is a driving fault; it says nothing about the attack, and it
# must never be counted as the collision the scenario is scored on.
ACTOR_PREFIXES = ("vehicle.", "walker.")


def is_road_user(type_id: str) -> bool:
    """True for a vehicle or a pedestrian, false for scenery."""
    return type_id.startswith(ACTOR_PREFIXES)


@dataclass(frozen=True)
class ManoeuvreSpec:
    """One scenario's names for the single thing its ego is scored on.

    `required` is what each run had to actually do for the pair to be
    comparable, as ``run -> (summary key, why its absence voids the run)``. It
    is per-run because the runs are not always symmetric: the do-not-pass and
    left-turn egos both have to perform the manoeuvre, while the VRU honest ego
    has to *stop* -- that is the safe behaviour the attack exists to subvert --
    and only the spoofed ego has to cross.
    """

    label: str                                  # "turn", "overtake", "crossing"
    commit_time_key: str                        # when the ego committed, seconds
    unsafe_key: str                             # ground truth said that commit was unsafe
    required: Dict[str, Tuple[str, str]]
    artifacts: str                              # what to read when something is wrong


@dataclass
class Evidence:
    """The guard's answer. `valid` is only true when it was able to judge."""

    comparable: bool
    problems: List[str] = field(default_factory=list)
    note: Optional[str] = None

    @property
    def valid(self) -> bool:
        return self.comparable and not self.problems

    def to_summary(self) -> dict:
        """The fields a scenario merges into its comparison.json.

        `evidence_valid` is None rather than False when only one run was asked
        for: an incomplete experiment is not a failed one, and reporting it as
        failed is how the earlier version of this guard invented two problems
        for every ``--run honest`` invocation.
        """
        out: dict = {
            "evidence_valid": self.valid if self.comparable else None,
            "evidence_problems": self.problems,
        }
        if self.note:
            out["evidence_note"] = self.note
        return out


def evidence_problems(results: Dict[str, dict], spec: ManoeuvreSpec,
                      extra: Sequence[Callable[[Dict[str, dict]], List[str]]] = ()
                      ) -> Evidence:
    """Reasons this pair of runs proves nothing, even if it ended in a collision."""
    h, s = results.get(HONEST), results.get(SPOOFED)
    if not (h and s):
        have = ", ".join(sorted(results)) or "neither run"
        return Evidence(
            comparable=False,
            note=(f"only {have} was run, so there is nothing to compare: the "
                  f"attack is judged by the difference between the two runs, "
                  f"never by one of them on its own. Re-run with --run both."))

    problems: List[str] = []

    # 1. Did the scene work at all? If the ego was not driving the scene
    #    correctly, nothing downstream means anything -- so this comes first.
    for name in (SPOOFED, HONEST):
        r = results[name]
        if r.get("hit_scenery"):
            hit = r["static_collision"]
            problems.append(
                f"the {name} ego hit scenery ({hit['with_type_id']}) at "
                f"{hit['sim_time']}s -- it is not driving the scene correctly, "
                f"so nothing about this run is trustworthy")

    for name in (SPOOFED, HONEST):
        key, why = spec.required.get(name, (None, ""))
        if key and not results[name].get(key):
            problems.append(f"the {name} ego {why}")

    # 2. Did the forgery actually do anything? These are the no-op checks, and
    #    the reason the verdict alone cannot be trusted.
    if s.get("n_flips", 0) == 0:
        problems.append(
            "the spoofed run produced ZERO decision flips: the forged messages "
            "never changed what the ego concluded, so nothing it did can be "
            "attributed to the attack")
    if not s.get("attack", {}).get("removed_object_ids"):
        problems.append(
            "the forgery deleted nothing: an impersonated CPM supersedes the "
            "RSU's whether or not it was edited, so any effect here is not "
            "suppression")

    # 3. Is the difference attributable to the messages, or to chance?
    if not s.get(spec.unsafe_key, False) and s.get("collided"):
        problems.append(
            f"the spoofed run collided but ground truth said the "
            f"{spec.label} was SAFE when it committed -- the collision is not "
            f"the attack's doing")
    commit_h, commit_s = h.get(spec.commit_time_key), s.get(spec.commit_time_key)
    if commit_h is not None and commit_h == commit_s:
        problems.append(
            f"both runs committed to the {spec.label} at the same instant "
            f"({commit_s}s), so the message stream changed nothing about the "
            f"manoeuvre")

    for check in extra:
        problems.extend(check(results))

    return Evidence(comparable=True, problems=problems)


def report(ev: Evidence, spec: ManoeuvreSpec, printer=print) -> int:
    """Print the guard's finding and return the process exit code.

    Exit 2 only when the guard actually judged and found something: a single-run
    invocation is incomplete, not failed, and must not fail the make target.
    """
    if not ev.comparable:
        printer(f"\n[evidence] {ev.note}")
        return 0
    if not ev.problems:
        return 0
    printer("\n" + "=" * 72)
    printer("THIS RUN IS NOT EVIDENCE OF THE ATTACK, whatever the verdict says:")
    for prob in ev.problems:
        printer(f"  - {prob}")
    printer(spec.artifacts)
    printer("=" * 72)
    return 2
