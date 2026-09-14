"""Shared constants and coordinate conventions.

Coordinate frame
----------------
All poses are expressed in the **CARLA world frame**: left-handed, X-forward,
Y-right, Z-up, metres, rotations in degrees (yaw about Z). We deliberately keep
raw CARLA coordinates end-to-end so the OMNeT++ side and any CARLA replay agree
without a transform. Convert to ENU/right-handed only at export if a consumer
needs it.
"""

# Default CARLA RPC endpoint.
DEFAULT_CARLA_HOST = "127.0.0.1"
DEFAULT_CARLA_PORT = 2000

# CarlaAir / AirSim drone RPC endpoint.
DEFAULT_AIRSIM_PORT = 41451

# CPM protocol version we emit (ETSI EN 302 637-5 is v2; this is a simplified
# JSON encoding of the same information model, tagged so consumers can branch).
CPM_PROTOCOL_VERSION = 2

# Perception range an honest sensor/CPM would plausibly report (metres).
DEFAULT_PERCEPTION_RANGE = 75.0
