"""Human-readable CSV of every broadcast message and what was spoofed.

Each row is one perceived object inside one broadcast CPM. Honest senders'
messages are all ``REAL``; the attacker's message shows ``INJECTED`` rows for
fabricated objects and ``REMOVED`` rows for objects it deleted (present in its
honest baseline, absent from what it broadcast). Open in any spreadsheet and
filter ``spoof_flag != REAL`` (or ``message_kind = spoofed``) to see the attack.
"""
from __future__ import annotations

import csv
from typing import Iterator, List

from .v2x.cpm import CollectivePerceptionMessage, diff_cpms

CSV_FIELDS = [
    "frame", "sim_time", "sender_id", "sender_type", "is_attacker",
    "message_kind", "object_id", "classification",
    "pos_x", "pos_y", "pos_z", "confidence", "spoof_flag",
]


def message_rows(frame: int, sim_time: float, sender_id: int, sender_type: str,
                 is_attacker: bool,
                 honest: CollectivePerceptionMessage,
                 broadcast: CollectivePerceptionMessage) -> Iterator[dict]:
    """Rows for one broadcast. For honest senders pass honest==broadcast."""
    diff = diff_cpms(honest, broadcast)
    added, removed = set(diff["added"]), set(diff["removed"])
    tampered = is_attacker and (added or removed)
    kind = "spoofed" if tampered else "honest"

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

    # objects the attacker dropped: show them so a reader can see the deletion
    honest_by_id = {o.object_id: o for o in honest.perceived_objects}
    for rid in sorted(removed):
        yield row(honest_by_id[rid], "REMOVED")


class MessageCsvWriter:
    """Incremental CSV writer for broadcast messages."""

    def __init__(self, path: str):
        self.path = path
        self._fh = open(path, "w", newline="")
        self._w = csv.DictWriter(self._fh, fieldnames=CSV_FIELDS)
        self._w.writeheader()
        self.n_rows = 0

    def add(self, **kwargs) -> None:
        for r in message_rows(**kwargs):
            self._w.writerow(r)
            self.n_rows += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass
