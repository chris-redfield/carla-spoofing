#!/usr/bin/env bash
# Activate the carlaAir env, make the CarlaAir `carla` module importable, and
# either launch the simulator (SIM=1) or run the given command.
set -e
source /opt/conda/etc/profile.d/conda.sh
conda activate carlaAir

CARLAAIR_DIR="${CARLAAIR_DIR:-/opt/carlaair}"

# Place CarlaAir's bundled `carla` module into the env if it isn't importable.
if ! python -c "import carla" >/dev/null 2>&1; then
    if [ -f "$CARLAAIR_DIR/env_setup/setup_env.sh" ]; then
        echo "[entrypoint] installing CarlaAir carla module via setup_env.sh ..."
        ( cd "$CARLAAIR_DIR" && bash env_setup/setup_env.sh ) || \
            echo "[entrypoint] WARN: setup_env.sh failed; 'carla' may be unavailable."
    else
        echo "[entrypoint] NOTE: CarlaAir not mounted at $CARLAAIR_DIR."
        echo "             CARLA mode needs it; mock mode works without it."
    fi
fi

if [ "${SIM:-0}" = "1" ]; then
    MAP="${MAP:-Town10HD}"
    RPC_PORT="${CARLA_PORT:-2000}"
    echo "[entrypoint] launching CarlaAir ($MAP) headless on RPC port $RPC_PORT ..."
    exec "$CARLAAIR_DIR/CarlaAir.sh" "$MAP" \
        -RenderOffScreen -nosound -carla-rpc-port="$RPC_PORT" -quality-level=Epic
fi

exec "$@"
