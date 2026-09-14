"""V2X cooperative-perception spoofing toolkit for CARLA / CarlaAir.

Layout
------
- ``v2x``       : the Collective Perception Message (CPM) model and the
                  transport-agnostic packet format handed to OMNeT++.
- ``lidar``     : point-cloud helpers (inject / carve object points).
- ``perception``: builds an *honest* CPM (+ point cloud) from a CARLA world
                  or from a mock world.
- ``attacks``   : the spoofing attacks (fake object, object removal, camera
                  injection) operating on CPMs, point clouds and images.
- ``scenarios`` : runnable end-to-end scenarios, incl. a CARLA-free mock world.

Design note
-----------
Per the research plan, *network transport is assumed perfect* and is delegated
to OMNeT++. This toolkit therefore only **produces** spoofed V2X payloads in
CARLA and **serialises** them into packets (see ``v2x.packet``). A companion
OMNeT++ model reads those packets and delivers them to the victim vehicles.
"""

__version__ = "0.1.0"
