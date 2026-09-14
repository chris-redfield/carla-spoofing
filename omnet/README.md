# OMNeT++ integration

**Scope now:** the CARLA side *generates* spoofed V2X payloads and serialises
them (see `cpm_packet_schema.md`). Per the research plan we **assume a perfect
network** and defer the channel/PHY/MAC to OMNeT++. Nothing here simulates the
network — it defines the hand-off so the OMNeT++ side is a drop-in later.

## Two hand-off modes

1. **File replay (default, decoupled).** CARLA writes `out/v2x_packets.jsonl`.
   An OMNeT++ scenario (or the reference Python receiver `v2x.packet.PacketReader`)
   replays each line as a message injected at the attacker node, scheduled at
   `header.sim_time`. Best while you don't yet have a live bridge.

2. **Live UDP bridge (coupled).** Run the attack with `--sink udp --udp-port 47000`.
   CARLA emits length-prefixed JSON datagrams. On the OMNeT++ side, an
   `inet::ExternalApp`/`cSocketRTScheduler` (or a small `cSimpleModule` reading a
   UDP socket) parses the envelope and emits a corresponding INET packet from the
   attacker node, broadcast to neighbours.

## Suggested OMNeT++ model (Artery / INET / Veins)

Because these are Collective Perception Messages, [**Artery**](https://github.com/riebl/artery)
(ETSI ITS-G5 stack on OMNeT++/INET, with a CP service) is the natural fit:

- Map each `CPM` packet to an Artery `CollectivePerceptionMessage` (or a custom
  `.msg`), one `PerceivedObject` per `ObjectContainer`.
- The **attacker** node transmits the spoofed CPM verbatim; genuine nodes
  transmit their honest CPMs. "Perfect network" = a lossless channel / 100 %
  PDR and zero latency, so every neighbour receives the spoof.
- The **victim** node's CP service fuses received objects into its local
  environment model; log the fused world to measure attack success (ghost
  accepted / real object dropped).

A minimal custom message (if not using Artery's CPM):

```
// SpoofedCpm.msg
packet SpoofedCpm {
    int stationId;
    double generationTime;
    double refX; double refY; double refZ;
    // repeated perceived objects flattened, or carry the JSON payload as a string:
    string payloadJson;   // the CARLA `payload` object, parsed at the receiver
}
```

Carrying `payloadJson` as a string is the fastest path: reuse the exact JSON
this repo emits, parse it at the OMNeT++ receiver, and you avoid maintaining two
schemas. Swap to typed fields once the attack set is stable.

## Time alignment

`header.sim_time` / `payload.generation_time` are CARLA seconds. Replay them on
the OMNeT++ clock 1:1 (or with an offset) so cooperative fusion sees the same
temporal ordering CARLA produced.
