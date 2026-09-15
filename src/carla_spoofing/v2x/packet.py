"""Transport-agnostic V2X packet + hand-off sinks for OMNeT++.

Per the research plan, network transport is assumed perfect and simulated in
OMNeT++. The CARLA side therefore only needs to *emit* well-framed packets:

    [ header (who/when/what) ][ payload (CPM JSON, or camera-frame reference) ]

Two sinks are provided:

* ``FileSink``  -> newline-delimited JSON (``.jsonl``). This is the default
  hand-off: OMNeT++ (or a Python bridge) replays the file as a packet stream.
* ``UdpSink``   -> length-prefixed JSON datagrams, for a live socket bridge to
  an OMNeT++ ``ExternalApp`` / INET ``sink`` if you later want CARLA and OMNeT++
  running concurrently.

The wire form is identical for both, so switching sinks changes nothing else.
"""
from __future__ import annotations

import json
import socket
import struct
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .cpm import CollectivePerceptionMessage


class MsgType(str, Enum):
    CPM = "CPM"            # collective perception message (payload = CPM JSON)
    CAMERA = "CAMERA"      # camera frame (payload = image metadata + ref/inline)


@dataclass
class PacketHeader:
    msg_type: MsgType
    sender_id: int          # V2X station id of the transmitter (the attacker)
    seq: int                # per-sender sequence number
    sim_time: float         # CARLA sim seconds when produced
    # 'broadcast' models a single-hop V2X broadcast to all neighbours. A real
    # attacker uses broadcast; OMNeT++ decides who is in range (assumed all).
    destination: str = "broadcast"

    def to_dict(self) -> dict:
        return {
            "msg_type": self.msg_type.value,
            "sender_id": self.sender_id,
            "seq": self.seq,
            "sim_time": self.sim_time,
            "destination": self.destination,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "PacketHeader":
        return cls(
            msg_type=MsgType(d["msg_type"]),
            sender_id=int(d["sender_id"]),
            seq=int(d["seq"]),
            sim_time=float(d["sim_time"]),
            destination=str(d.get("destination", "broadcast")),
        )


@dataclass
class Packet:
    header: PacketHeader
    payload: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"header": self.header.to_dict(), "payload": self.payload}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    @classmethod
    def from_dict(cls, d: dict) -> "Packet":
        return cls(header=PacketHeader.from_dict(d["header"]),
                   payload=d.get("payload", {}))

    @classmethod
    def from_json(cls, s: str) -> "Packet":
        return cls.from_dict(json.loads(s))

    # ---- convenience constructors ----
    @classmethod
    def cpm(cls, cpm: CollectivePerceptionMessage, seq: int,
            sim_time: Optional[float] = None) -> "Packet":
        st = cpm.generation_time if sim_time is None else sim_time
        hdr = PacketHeader(MsgType.CPM, sender_id=cpm.station_id, seq=seq, sim_time=st)
        return cls(hdr, payload=cpm.to_dict())

    @classmethod
    def camera(cls, sender_id: int, seq: int, sim_time: float,
               image_meta: dict) -> "Packet":
        """image_meta: {width,height,fov,image_ref|image_b64, injected:[...]}."""
        hdr = PacketHeader(MsgType.CAMERA, sender_id=sender_id, seq=seq, sim_time=sim_time)
        return cls(hdr, payload=image_meta)


# --------------------------------------------------------------------------- #
# Sinks                                                                        #
# --------------------------------------------------------------------------- #
class PacketSink:
    """Base sink. A sink is where CARLA drops packets for OMNeT++ to pick up."""

    def send(self, packet: Packet) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class NullSink(PacketSink):
    """Discards everything (useful for dry runs / tests)."""

    def __init__(self):
        self.count = 0

    def send(self, packet: Packet) -> None:
        self.count += 1


class FileSink(PacketSink):
    """Append packets as newline-delimited JSON (the OMNeT++ hand-off file)."""

    def __init__(self, path: str, append: bool = False):
        # Truncate per run by default so out/ reflects one run (matches the CSV).
        self.path = path
        self._fh = open(path, "a" if append else "w", buffering=1)

    def send(self, packet: Packet) -> None:
        self._fh.write(packet.to_json() + "\n")

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


class UdpSink(PacketSink):
    """Length-prefixed (uint32 BE) JSON datagrams to a live OMNeT++ bridge."""

    def __init__(self, host: str = "127.0.0.1", port: int = 47000):
        self.addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, packet: Packet) -> None:
        data = packet.to_json().encode("utf-8")
        self._sock.sendto(struct.pack(">I", len(data)) + data, self.addr)

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass


class PacketReader:
    """Reads a FileSink ``.jsonl`` back into Packets.

    This doubles as a *reference receiver*: the same logic an OMNeT++ external
    app (or a victim-vehicle Python client) uses to consume the packet stream.
    """

    def __init__(self, path: str):
        self.path = path

    def __iter__(self):
        with open(self.path, "r") as fh:
            for line in fh:
                line = line.strip()
                if line:
                    yield Packet.from_json(line)
