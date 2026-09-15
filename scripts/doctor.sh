#!/usr/bin/env bash
# Check host prerequisites for a from-scratch CARLA-spoofing run.
# Informational: prints PASS/WARN/FAIL per item and a final verdict.
set -u

ok(){   printf "  \033[32m[PASS]\033[0m %s\n" "$1"; }
warn(){ printf "  \033[33m[WARN]\033[0m %s\n" "$1"; WARN=$((WARN+1)); }
bad(){  printf "  \033[31m[FAIL]\033[0m %s\n" "$1"; FAIL=$((FAIL+1)); }
WARN=0; FAIL=0

echo "CARLA-spoofing host check"
echo "-------------------------"

# --- core tooling ---
command -v git  >/dev/null && ok "git present" || bad "git missing (needed to clone the repo)"
command -v make >/dev/null && ok "make present" || bad "make missing (apt install make)"

# --- docker ---
if command -v docker >/dev/null; then
  if docker info >/dev/null 2>&1; then
    ok "docker installed and usable without sudo"
  else
    bad "docker installed but not usable — is the daemon up and are you in the 'docker' group? (newgrp docker)"
  fi
else
  bad "docker not installed"
fi
if docker compose version >/dev/null 2>&1; then
  ok "docker compose plugin present"
else
  bad "docker compose plugin missing"
fi

# --- NVIDIA driver (host) ---
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  GPU=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
  ok "NVIDIA driver works on host ($GPU)"
else
  bad "nvidia-smi not working — install the NVIDIA GPU driver (the toolkit does NOT install it)"
fi

# --- NVIDIA container toolkit (GPU in Docker) ---
if command -v nvidia-ctk >/dev/null; then
  if grep -q '"nvidia"' /etc/docker/daemon.json 2>/dev/null; then
    ok "nvidia-container-toolkit installed and wired into Docker"
  else
    warn "nvidia-ctk present but Docker runtime not configured — run: make toolkit"
  fi
else
  warn "nvidia-container-toolkit not installed — run: make toolkit (needs sudo)"
fi

# --- disk space (~25 GB for binary + image) ---
AVAIL=$(df -BG --output=avail . 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -n "${AVAIL:-}" ]; then
  if [ "$AVAIL" -ge 25 ]; then ok "disk space: ${AVAIL} GB free (need ~25 GB)"
  else warn "only ${AVAIL} GB free — need ~25 GB for the 6.85 GB binary + image"; fi
fi

# --- GUI prerequisites (window mode) ---
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ] && command -v xhost >/dev/null; then
  ok "GUI ready (DISPLAY=$DISPLAY, X socket + xhost present) — 'make up' will show a window"
else
  warn "no X display/xhost — GUI ('make up') won't work here; use 'make headless'"
fi

# --- CarlaAir binary (informational) ---
if [ -f vendor/CarlaAir-v0.1.7/CarlaAir.sh ]; then
  ok "CarlaAir binary already extracted (vendor/)"
elif [ -f vendor/CarlaAir-v0.1.7.zip ]; then
  warn "CarlaAir zip downloaded but not extracted — 'make setup' will handle it"
else
  warn "CarlaAir not downloaded yet — 'make setup' will fetch it (6.85 GB)"
fi

echo "-------------------------"
if [ "$FAIL" -gt 0 ]; then
  printf "\033[31mNOT READY: %d blocking issue(s), %d warning(s).\033[0m Fix FAILs above, then re-run 'make doctor'.\n" "$FAIL" "$WARN"
  exit 1
elif [ "$WARN" -gt 0 ]; then
  printf "\033[33mMOSTLY READY: %d warning(s).\033[0m Review WARNs (some are resolved by 'make toolkit' / 'make setup').\n" "$WARN"
else
  printf "\033[32mALL GOOD — ready for 'make setup' then 'make up'.\033[0m\n"
fi
