import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from carla_spoofing.v2x.cpm import (
    PerceivedObject, CollectivePerceptionMessage, diff_cpms)
from carla_spoofing.v2x.packet import Packet, PacketReader, FileSink, MsgType


def _sample_cpm():
    return CollectivePerceptionMessage(
        station_id=1, generation_time=0.0, reference_position=(0, 0, 0),
        perceived_objects=[
            PerceivedObject(10, (5, 0, 0), classification="car"),
            PerceivedObject(11, (9, 2, 0), classification="pedestrian"),
        ],
    )


def test_cpm_roundtrip():
    cpm = _sample_cpm()
    again = CollectivePerceptionMessage.from_json(cpm.to_json())
    assert again.to_dict() == cpm.to_dict()
    assert again.object_ids() == [10, 11]


def test_cpm_copy_is_independent():
    cpm = _sample_cpm()
    c = cpm.copy()
    c.add_object(PerceivedObject(99, (1, 1, 1)))
    assert 99 not in cpm.object_ids()
    assert 99 in c.object_ids()


def test_diff():
    honest = _sample_cpm()
    spoof = honest.copy()
    spoof.add_object(PerceivedObject(999, (3, 0, 0)))
    spoof.remove_object(11)
    d = diff_cpms(honest, spoof)
    assert d["added"] == [999] and d["removed"] == [11] and d["kept"] == [10]


def test_packet_roundtrip():
    cpm = _sample_cpm()
    pkt = Packet.cpm(cpm, seq=3)
    again = Packet.from_json(pkt.to_json())
    assert again.header.msg_type == MsgType.CPM
    assert again.header.sender_id == 1 and again.header.seq == 3
    assert CollectivePerceptionMessage.from_dict(again.payload).object_ids() == [10, 11]


def test_file_sink_reader(tmp_path):
    path = str(tmp_path / "p.jsonl")
    with FileSink(path) as s:
        s.send(Packet.cpm(_sample_cpm(), seq=0))
        s.send(Packet.cpm(_sample_cpm(), seq=1))
    pkts = list(PacketReader(path))
    assert len(pkts) == 2 and [p.header.seq for p in pkts] == [0, 1]
