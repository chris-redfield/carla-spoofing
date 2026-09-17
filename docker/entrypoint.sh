#!/usr/bin/env bash
# Activate the carlaAir env, make the CarlaAir `carla` module importable, then
# either launch the simulator headless (SIM=1) or run the given command.
set -e
source /opt/conda/etc/profile.d/conda.sh
conda activate carlaAir

CARLAAIR_DIR="${CARLAAIR_DIR:-/opt/carlaair}"

# UE4/Vulkan want an XDG_RUNTIME_DIR; provide a private one to silence warnings.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-$(id -u)}"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

# Install CarlaAir's bundled `carla` module into the env if not importable.
# (site-packages lives in the image layer, so this runs once per container.)
if ! python -c "import carla" >/dev/null 2>&1; then
    if [ -f "$CARLAAIR_DIR/env_setup/setup_env.sh" ]; then
        echo "[entrypoint] installing CarlaAir carla module via setup_env.sh ..."
        ( cd "$CARLAAIR_DIR" && bash env_setup/setup_env.sh ) \
            || echo "[entrypoint] WARN: setup_env.sh reported an issue."
        conda activate carlaAir 2>/dev/null || true
    else
        echo "[entrypoint] NOTE: CarlaAir not mounted at $CARLAAIR_DIR."
        echo "             CARLA mode needs it; mock mode works without it."
    fi
fi

if [ "${SIM:-0}" = "1" ]; then
    MAP="${MAP:-Town10HD}"
    RPC_PORT="${CARLA_PORT:-2000}"
    QUALITY="${QUALITY:-Epic}"
    BINARY="$CARLAAIR_DIR/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
    [ -x "$BINARY" ] || chmod +x "$BINARY" 2>/dev/null || true
    if [ ! -x "$BINARY" ]; then
        echo "[entrypoint] ERROR: sim binary not found/executable at $BINARY"
        echo "             Did you run scripts/extract_carlaair.sh and mount vendor/?"
        exit 1
    fi

    # Seed AirSim settings so the drone (port 41451) is available.
    if [ -f "$CARLAAIR_DIR/AirSimConfig/settings.json" ]; then
        mkdir -p "$HOME/Documents/AirSim"
        cp -f "$CARLAAIR_DIR/AirSimConfig/settings.json" "$HOME/Documents/AirSim/settings.json"
    fi

    # HEADLESS=1 (default): off-screen render, no window. HEADLESS=0: on-screen
    # UE4 window on the host X server (needs DISPLAY + X11 socket + xhost access).
    if [ "${HEADLESS:-1}" = "0" ]; then
        RENDER_FLAGS="-windowed -ResX=${RES_X:-1280} -ResY=${RES_Y:-720}"
        echo "[entrypoint] launching CarlaUE4 ($MAP, quality=$QUALITY) WINDOWED on DISPLAY=$DISPLAY, RPC $RPC_PORT ..."
    else
        RENDER_FLAGS="-RenderOffScreen"
        echo "[entrypoint] launching CarlaUE4 ($MAP, quality=$QUALITY) headless on RPC $RPC_PORT ..."
    fi
    # Fully-qualified package path: the bare map name ("Town01") is ignored by
    # this build and the engine silently falls back to GameDefaultMap. If the
    # path does not resolve the behaviour is the same fallback, so this is never
    # worse -- and when it works, no runtime level reload is needed at all.
    "$BINARY" CarlaUE4 "/Game/Carla/Maps/$MAP" \
        -carla-rpc-port="$RPC_PORT" $RENDER_FLAGS -nosound \
        -quality-level="$QUALITY" -TexturePoolSize=2048 -unattended &
    SIM_PID=$!

    # Wait for the RPC port to accept connections (no extra tooling needed).
    echo "[entrypoint] waiting for CARLA RPC on $RPC_PORT ..."
    for i in $(seq 1 150); do
        if python -c "import socket; s=socket.socket(); s.settimeout(1); s.connect(('127.0.0.1', $RPC_PORT))" 2>/dev/null; then
            echo "[entrypoint] CARLA RPC ready on $RPC_PORT"
            break
        fi
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "[entrypoint] ERROR: sim process exited during startup (see log above)."
            exit 1
        fi
        sleep 2
    done

    # Load the requested map over RPC.
    #
    # This packaged build IGNORES the positional map argument: launched with
    # `CarlaUE4-Linux-Shipping CarlaUE4 Town01` the server still reports
    # Town10HD, falling back to GameDefaultMap in CarlaUE4/Config/DefaultEngine.ini.
    # (CarlaAir.sh has the same bug -- its advertised `./CarlaAir.sh Town03` does
    # not work either.) So the map is switched here instead.
    #
    # The timing matters. load_world() is unreliable in this build once the world
    # is busy -- it has to tear down a level while other clients own actors and
    # the AirSim plugin sits in the same UE4 process, and it can hang forever.
    # Right here it is safe: no traffic yet, the drone has not taken off, and this
    # is the only client connected. Do NOT move this below the traffic block.
    if [ -n "${MAP:-}" ]; then
        MAP="$MAP" RPC_PORT="$RPC_PORT" python -c '
import carla, os, time
want = os.environ["MAP"]
client = carla.Client("127.0.0.1", int(os.environ["RPC_PORT"]))
client.set_timeout(180.0)

# The RPC socket accepts connections well before the engine has finished
# booting, and loading a level into a half-initialised engine SEGFAULTS it
# (observed: "Signal 11 caught" immediately after "CARLA RPC ready"). Wait for
# the world to actually tick, which only happens once it is really up.
ready = False
for _ in range(60):
    try:
        client.get_world().wait_for_tick(10.0)
        ready = True
        break
    except Exception:
        time.sleep(2.0)
if not ready:
    raise SystemExit("world never ticked; leaving the map alone")
time.sleep(8.0)

current = client.get_world().get_map().name.split("/")[-1]
if current == want:
    print("[entrypoint] map is %s" % want)
else:
    print("[entrypoint] engine loaded %s, switching to %s over RPC ..." % (current, want))
    client.load_world(want)
    time.sleep(5.0)
    print("[entrypoint] map is now %s"
          % client.get_world().get_map().name.split("/")[-1])
' || echo "[entrypoint] WARN: could not switch the map to $MAP; continuing on the engine default"
    fi

    # Spawn traffic so an attacker/sender vehicle exists in the world.
    if [ "${SPAWN_TRAFFIC:-1}" = "1" ] && [ -f "$CARLAAIR_DIR/examples/auto_traffic.py" ]; then
        echo "[entrypoint] spawning traffic (${N_VEHICLES:-10} vehicles, ${N_WALKERS:-10} walkers) ..."
        python "$CARLAAIR_DIR/examples/auto_traffic.py" \
            --vehicles "${N_VEHICLES:-10}" --walkers "${N_WALKERS:-10}" \
            --port "$RPC_PORT" >/tmp/auto_traffic.log 2>&1 &
    fi

    # Auto-takeoff the AirSim drone to a stable hover so it doesn't sit/fall on
    # spawn (SimpleFlight is disarmed with no throttle until first command).
    # Set DRONE_TAKEOFF=0 to keep the vendor default (drone idle until you fly it).
    if [ "${DRONE_TAKEOFF:-1}" = "1" ]; then
        (
          for i in $(seq 1 30); do
            python -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',41451))" 2>/dev/null && break
            sleep 1
          done
          python -c "
import airsim
try:
    c = airsim.MultirotorClient(port=41451); c.confirmConnection()
    c.enableApiControl(True); c.armDisarm(True)
    c.takeoffAsync().join(); c.moveToZAsync(-3.0, 1.0).join(); c.hoverAsync()
    print('[drone] auto-takeoff: hovering at ~3 m')
except Exception as e:
    print('[drone] auto-takeoff skipped:', e)
"
        ) &
    fi

    echo "[entrypoint] simulator up; holding foreground (docker stop to exit)."
    wait "$SIM_PID"
    exit $?
fi

exec "$@"
