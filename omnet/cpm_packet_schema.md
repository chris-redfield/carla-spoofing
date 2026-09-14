# V2X packet schema (CARLA → OMNeT++ hand-off)

Every line of `out/v2x_packets.jsonl` is one broadcast packet. The wire form is
JSON (UTF-8). Over UDP (`UdpSink`) each datagram is `uint32` big-endian length
prefix + the same JSON bytes.

## Envelope

```json
{
  "header": {
    "msg_type": "CPM | CAMERA",
    "sender_id": 101,          // V2X station id of the transmitter (attacker)
    "seq": 0,                  // per-sender sequence number
    "sim_time": 0.0,           // CARLA sim seconds when produced
    "destination": "broadcast" // single-hop V2X broadcast; OMNeT++ picks receivers
  },
  "payload": { ... }           // depends on msg_type
}
```

## `payload` for `msg_type = CPM`

A Collective Perception Message (simplified ETSI EN 302 637-5 model):

```json
{
  "protocol_version": 2,
  "station_id": 101,
  "station_type": "vehicle | rsu | drone",
  "generation_time": 0.0,
  "reference_position": [x, y, z],          // sender pose, CARLA world frame (m)
  "perceived_objects": [
    {
      "object_id": 200,
      "position": [x, y, z],                // world frame, metres
      "velocity": [vx, vy, vz],             // m/s
      "yaw_deg": 0.0,
      "dimensions": [length, width, height],// metres
      "classification": "car|truck|pedestrian|bicycle|unknown",
      "confidence": 0.95                    // 0..1
    }
  ]
}
```

* **Fake-object attack** → an extra `perceived_object` the victim will fuse.
* **Object-removal attack** → a real `object_id` is absent from the list.

## `payload` for `msg_type = CAMERA`

```json
{
  "width": 800, "height": 600, "fov": 90.0,
  "image_ref": "out/camera_spoofed_000.png", // or "image_b64": "<...>"
  "injected": [ { "world_pos": [x,y,z], "pixel": [u,v],
                  "size_px": [w,h], "depth_m": 18.0,
                  "classification": "car" } ]
}
```

`injected` is attack ground-truth for evaluation; a real victim only sees the
pixels. Coordinate frame is CARLA world (left-handed, X-fwd, Y-right, Z-up, m).
