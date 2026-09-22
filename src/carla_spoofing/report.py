"""Human-readable CSV reports of the broadcast messages.

Two files (both truncated per run):

* ``messages.csv``          — ONE ROW PER MESSAGE (one vehicle's CPM in one round).
                              Columns say who sent it, whether it was honest or
                              spoofed, and what was faked (injected/removed ids).
* ``perceived_objects.csv`` — ONE ROW PER PERCEIVED OBJECT inside each message,
                              with each object's position and a spoof_flag
                              (REAL / INJECTED / REMOVED). The detailed view.

Filter ``message_kind = spoofed`` in messages.csv to see every poisoned message.
"""
from __future__ import annotations

import csv
import os
from typing import Iterator

from .v2x.cpm import CollectivePerceptionMessage, diff_cpms

MESSAGE_FIELDS = [
    "frame", "sim_time", "sender_id", "claimed_sender_id", "sender_type",
    "is_attacker", "message_kind", "identity_spoofed",
    "n_objects", "n_injected", "n_removed", "injected_ids", "removed_ids",
]
OBJECT_FIELDS = [
    "frame", "sim_time", "sender_id", "sender_type", "is_attacker",
    "message_kind", "object_id", "classification",
    "pos_x", "pos_y", "pos_z", "confidence", "spoof_flag",
]


def _object_rows(frame, sim_time, sender_id, sender_type, is_attacker,
                 kind, added, removed,
                 honest: CollectivePerceptionMessage,
                 broadcast: CollectivePerceptionMessage) -> Iterator[dict]:
    def row(o, flag):
        return {
            "frame": frame, "sim_time": round(sim_time, 3),
            "sender_id": sender_id, "sender_type": sender_type,
            "is_attacker": is_attacker, "message_kind": kind,
            "object_id": o.object_id, "classification": o.classification,
            "pos_x": round(o.position[0], 2), "pos_y": round(o.position[1], 2),
            "pos_z": round(o.position[2], 2), "confidence": round(o.confidence, 3),
            "spoof_flag": flag,
        }
    for o in broadcast.perceived_objects:
        yield row(o, "INJECTED" if o.object_id in added else "REAL")
    honest_by_id = {o.object_id: o for o in honest.perceived_objects}
    for rid in sorted(removed):
        yield row(honest_by_id[rid], "REMOVED")


class ReportWriter:
    """Writes messages.csv (per message) and perceived_objects.csv (per object)."""

    def __init__(self, out_dir: str, per_object: bool = True):
        self.msg_path = os.path.join(out_dir, "messages.csv")
        self._mf = open(self.msg_path, "w", newline="")
        self._mw = csv.DictWriter(self._mf, fieldnames=MESSAGE_FIELDS)
        self._mw.writeheader()
        self.n_messages = 0
        self._obj = None
        if per_object:
            self.obj_path = os.path.join(out_dir, "perceived_objects.csv")
            self._of = open(self.obj_path, "w", newline="")
            self._ow = csv.DictWriter(self._of, fieldnames=OBJECT_FIELDS)
            self._ow.writeheader()
            self._obj = self._ow
        self.n_object_rows = 0

    def add(self, frame, sim_time, sender_id, sender_type, is_attacker,
            honest, broadcast) -> None:
        diff = diff_cpms(honest, broadcast)
        added, removed = set(diff["added"]), set(diff["removed"])
        # An impersonated station id is itself a spoof, even when the object
        # list is untouched -- so it must not be classified as an honest message.
        claimed = broadcast.station_id
        identity_spoofed = claimed != honest.station_id
        tampered = is_attacker and (added or removed or identity_spoofed)
        kind = "spoofed" if tampered else "honest"

        self._mw.writerow({
            "frame": frame, "sim_time": round(sim_time, 3),
            "sender_id": sender_id, "claimed_sender_id": claimed,
            "sender_type": sender_type,
            "is_attacker": is_attacker, "message_kind": kind,
            "identity_spoofed": identity_spoofed,
            "n_objects": len(broadcast.perceived_objects),
            "n_injected": len(added), "n_removed": len(removed),
            "injected_ids": ";".join(str(i) for i in sorted(added)),
            "removed_ids": ";".join(str(i) for i in sorted(removed)),
        })
        self.n_messages += 1

        if self._obj is not None:
            for r in _object_rows(frame, sim_time, sender_id, sender_type,
                                  is_attacker, kind, added, removed,
                                  honest, broadcast):
                self._ow.writerow(r)
                self.n_object_rows += 1

    def close(self) -> None:
        for fh in (getattr(self, "_mf", None), getattr(self, "_of", None)):
            try:
                if fh:
                    fh.close()
            except Exception:
                pass


# --------------------------------------------------------------------------- #
# Do-Not-Pass Warning decisions                                                #
# --------------------------------------------------------------------------- #
DECISION_FIELDS = [
    "run", "frame", "sim_time", "ego_id", "ego_speed_mps", "ego_state",
    "acted_decision", "counterfactual_decision", "truth_decision",
    "flipped", "misled", "unsafe_pass_enabled",
    "pass_intent", "lead_object_id", "blocking_object_id", "hidden_object_id",
    "gap_m", "closing_speed_mps", "time_to_meet_s", "n_oncoming",
    "n_objects_fused", "acted_reason",
]


class DecisionWriter:
    """Writes ``do_not_pass_decisions.csv``: one row per DNPW evaluation.

    Each row carries the decision the ego *acted on* and the counterfactual
    decision from the other message stream, so the flip caused by the attack is
    readable straight off the file without joining two runs together.
    """

    def __init__(self, out_dir: str, filename: str = "do_not_pass_decisions.csv"):
        self.path = os.path.join(out_dir, filename)
        self._fh = open(self.path, "w", newline="")
        self._w = csv.DictWriter(self._fh, fieldnames=DECISION_FIELDS)
        self._w.writeheader()
        self.n_rows = 0
        self.n_flips = 0

    def add(self, run, frame, sim_time, ego_id, ego_speed, ego_state,
            acted, counterfactual=None, hidden_object_id=None, truth=None) -> None:
        flipped = counterfactual is not None and counterfactual.decision != acted.decision
        unsafe = (counterfactual is not None
                  and counterfactual.decision == "DO_NOT_PASS"
                  and acted.decision == "PASS")
        # `misled` is the ground-truth version of `unsafe_pass_enabled`: the ego
        # believes the road is clear while an oncoming vehicle really is there.
        misled = (truth is not None and truth.decision == "DO_NOT_PASS"
                  and acted.decision == "PASS")
        d = acted.to_dict()
        self._w.writerow({
            "run": run, "frame": frame, "sim_time": round(sim_time, 3),
            "ego_id": ego_id, "ego_speed_mps": round(ego_speed, 2),
            "ego_state": ego_state,
            "acted_decision": d["decision"],
            "counterfactual_decision": (counterfactual.decision
                                        if counterfactual is not None else ""),
            "truth_decision": truth.decision if truth is not None else "",
            "flipped": flipped, "misled": misled, "unsafe_pass_enabled": unsafe,
            "pass_intent": d["pass_intent"],
            "lead_object_id": d["lead_object_id"],
            "blocking_object_id": d["blocking_object_id"],
            "hidden_object_id": hidden_object_id if hidden_object_id is not None else "",
            "gap_m": d["gap_m"], "closing_speed_mps": d["closing_speed_mps"],
            "time_to_meet_s": d["time_to_meet_s"], "n_oncoming": d["n_oncoming"],
            "n_objects_fused": d["n_objects_considered"],
            "acted_reason": d["reason"],
        })
        self.n_rows += 1
        if flipped:
            self.n_flips += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Left Turn Assist decisions                                                   #
# --------------------------------------------------------------------------- #
LEFT_TURN_DECISION_FIELDS = [
    "run", "frame", "sim_time", "ego_id", "ego_speed_mps", "ego_state",
    "acted_decision", "counterfactual_decision", "truth_decision",
    "flipped", "misled", "unsafe_turn_enabled",
    "turn_intent", "conflicting_object_id", "hidden_object_id",
    "gap_m", "approach_speed_mps", "time_to_arrival_s", "ego_to_conflict_m",
    "n_conflicting", "n_objects_fused", "acted_reason",
]


class LeftTurnDecisionWriter:
    """Writes ``left_turn_decisions.csv``: one row per Left Turn Assist evaluation.

    Deliberately a separate class from :class:`DecisionWriter` rather than a
    generalisation of it. The two applications answer different questions and
    their audit columns differ -- time-to-*arrival* at a conflict point versus
    time-to-*meet* an oncoming car, turn intent versus pass intent. Collapsing
    them into one writer would mean a row of mostly-empty columns whichever
    scenario ran, and would put the do-not-pass scenario's verified output at
    risk for no gain.
    """

    def __init__(self, out_dir: str, filename: str = "left_turn_decisions.csv"):
        self.path = os.path.join(out_dir, filename)
        self._fh = open(self.path, "w", newline="")
        self._w = csv.DictWriter(self._fh, fieldnames=LEFT_TURN_DECISION_FIELDS)
        self._w.writeheader()
        self.n_rows = 0
        self.n_flips = 0

    def add(self, run, frame, sim_time, ego_id, ego_speed, ego_state,
            acted, counterfactual=None, hidden_object_id=None, truth=None) -> None:
        flipped = counterfactual is not None and counterfactual.decision != acted.decision
        unsafe = (counterfactual is not None
                  and counterfactual.decision == "DO_NOT_TURN"
                  and acted.decision == "TURN")
        # `misled` is the ground-truth version of `unsafe_turn_enabled`: the ego
        # believes the junction is clear while a conflicting vehicle really is
        # approaching it.
        misled = (truth is not None and truth.decision == "DO_NOT_TURN"
                  and acted.decision == "TURN")
        d = acted.to_dict()
        self._w.writerow({
            "run": run, "frame": frame, "sim_time": round(sim_time, 3),
            "ego_id": ego_id, "ego_speed_mps": round(ego_speed, 2),
            "ego_state": ego_state,
            "acted_decision": d["decision"],
            "counterfactual_decision": (counterfactual.decision
                                        if counterfactual is not None else ""),
            "truth_decision": truth.decision if truth is not None else "",
            "flipped": flipped, "misled": misled, "unsafe_turn_enabled": unsafe,
            "turn_intent": d["turn_intent"],
            "conflicting_object_id": d["conflicting_object_id"],
            "hidden_object_id": hidden_object_id if hidden_object_id is not None else "",
            "gap_m": d["gap_m"], "approach_speed_mps": d["approach_speed_mps"],
            "time_to_arrival_s": d["time_to_arrival_s"],
            "ego_to_conflict_m": d["ego_to_conflict_m"],
            "n_conflicting": d["n_conflicting"],
            "n_objects_fused": d["n_objects_considered"],
            "acted_reason": d["reason"],
        })
        self.n_rows += 1
        if flipped:
            self.n_flips += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Trajectory trace                                                             #
# --------------------------------------------------------------------------- #
TRAJECTORY_FIELDS = [
    "run", "frame", "sim_time", "actor_id", "role",
    "x", "y", "z", "yaw_deg", "speed_mps",
    # Ego only: what the controller was doing and what it was aiming at.
    "ctrl_state", "throttle", "steer", "brake",
    "target_x", "target_y", "target_bearing_deg",
    "path_progress_m", "path_remaining_m", "dist_to_hold_m", "decision",
]


class TrajectoryWriter:
    """Writes ``trajectory.csv``: one row per actor per tick.

    A pose log at the control rate, not the message rate. Debugging a
    pure-pursuit controller from the decision log is impossible -- that file
    says what the ego *believed*, once a second, while the steering is computed
    twenty times a second from a target point that never appears in any output.
    So the target point and the raw control outputs are recorded too: a wrong
    turn is almost always the controller faithfully chasing a target that is in
    the wrong place, and you cannot see that without both.

    Cheap enough to leave on (a 60 s run at 20 Hz with three actors is ~3600
    rows), and the first thing to read when a vehicle does something strange.
    """

    def __init__(self, out_dir: str, filename: str = "trajectory.csv"):
        self.path = os.path.join(out_dir, filename)
        self._fh = open(self.path, "w", newline="")
        self._w = csv.DictWriter(self._fh, fieldnames=TRAJECTORY_FIELDS)
        self._w.writeheader()
        self.n_rows = 0

    def add(self, run, frame, sim_time, actor_id, role, position, yaw_deg,
            speed, ctrl_state="", command=None, target=None,
            path_progress=None, path_remaining=None, dist_to_hold=None,
            decision="", target_bearing_deg=None) -> None:
        def r(v, n=3):
            return "" if v is None else round(float(v), n)
        self._w.writerow({
            "run": run, "frame": frame, "sim_time": round(sim_time, 3),
            "actor_id": actor_id, "role": role,
            "x": r(position[0]), "y": r(position[1]), "z": r(position[2]),
            "yaw_deg": r(yaw_deg), "speed_mps": r(speed),
            "ctrl_state": ctrl_state,
            "throttle": r(command.throttle if command else None),
            "steer": r(command.steer if command else None),
            "brake": r(command.brake if command else None),
            "target_x": r(target[0] if target else None),
            "target_y": r(target[1] if target else None),
            "target_bearing_deg": r(target_bearing_deg),
            "path_progress_m": r(path_progress),
            "path_remaining_m": r(path_remaining),
            "dist_to_hold_m": r(dist_to_hold),
            "decision": decision,
        })
        self.n_rows += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass
