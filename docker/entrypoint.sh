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
    "$BINARY" CarlaUE4 "$MAP" \
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

    # Spawn traffic so an attacker/sender vehicle exists in the world.
    if [ "${SPAWN_TRAFFIC:-1}" = "1" ] && [ -f "$CARLAAIR_DIR/examples/auto_traffic.py" ]; then
        echo "[entrypoint] spawning traffic (${N_VEHICLES:-10} vehicles, ${N_WALKERS:-10} walkers) ..."
        python "$CARLAAIR_DIR/examples/auto_traffic.py" \
            --vehicles "${N_VEHICLES:-10}" --walkers "${N_WALKERS:-10}" \
            --port "$RPC_PORT" >/tmp/auto_traffic.log 2>&1 &
    fi

    echo "[entrypoint] simulator up; holding foreground (docker stop to exit)."
    wait "$SIM_PID"
    exit $?
fi

exec "$@"
