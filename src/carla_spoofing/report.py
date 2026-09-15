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
    "frame", "sim_time", "sender_id", "sender_type", "is_attacker",
    "message_kind", "n_objects", "n_injected", "n_removed",
    "injected_ids", "removed_ids",
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
        tampered = is_attacker and (added or removed)
        kind = "spoofed" if tampered else "honest"

        self._mw.writerow({
            "frame": frame, "sim_time": round(sim_time, 3),
            "sender_id": sender_id, "sender_type": sender_type,
            "is_attacker": is_attacker, "message_kind": kind,
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
