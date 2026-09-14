#!/usr/bin/env bash
# Convenience: authorize X access, then start the sim (window mode is the default).
set -e
export DISPLAY="${DISPLAY:-:1}"
echo "Allowing local containers to use X server on $DISPLAY ..."
xhost +local: >/dev/null 2>&1 || echo "  (xhost not found; if no window appears, run: xhost +local:)"
cd "$(dirname "$0")/.."
docker compose -f docker/docker-compose.yml up carla-sim
