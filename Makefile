# CARLA V2X spoofing — one-command workflows.
# From scratch:  make toolkit   (once, sudo)  ->  make setup  ->  make up
COMPOSE := docker compose -f docker/docker-compose.yml
HEADLESS := $(COMPOSE) -f docker/docker-compose.headless.yml
ATTACK ?= fake_object
RATE ?= 1
DURATION ?= 60
RUN ?= both
# Drone hover spot, metres behind the ego start (negative = in front of it).
# -55 puts it back over the RSU, directly above the crash, where it used to sit.
DRONE_BACK ?= -2.5

.PHONY: help doctor toolkit setup download extract build up gui headless \
        spoof do-not-pass do-not-pass-mock test down logs clean

help:            ## Show this help
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-17s\033[0m %s\n",$$1,$$2}'

doctor:          ## Check host prerequisites for a from-scratch run
	@bash scripts/doctor.sh

toolkit:         ## Install NVIDIA Container Toolkit on the host (once, needs sudo)
	bash scripts/install_nvidia_toolkit.sh

setup: download extract build ## Fetch CarlaAir + build the image (from scratch)
	@echo "Setup complete. Run 'make up' (window) or 'make headless'."

download:        ## Download the CarlaAir binary (6.85 GB, resumable) if missing
	@test -d vendor/CarlaAir-v0.1.7 || bash scripts/download_carlaair.sh

extract:         ## Unpack the CarlaAir binary if not already extracted
	@test -f vendor/CarlaAir-v0.1.7/CarlaAir.sh || bash scripts/extract_carlaair.sh

build:           ## Build the Docker image
	$(COMPOSE) build

up: gui          ## Alias for 'gui' (window mode is the default)

gui:             ## Start the sim with a window on your desktop (runs xhost)
	bash scripts/run_sim_gui.sh

headless:        ## Start the sim headless (no window; servers/CI)
	$(HEADLESS) up carla-sim

spoof:           ## Attack the live sim: make spoof [ATTACK=remove_object] [RATE=1] [DURATION=60]
	$(COMPOSE) run --rm spoofing carla-spoof --mode carla --host carla-sim \
	  --port 2000 --attack $(ATTACK) --rate $(RATE) --duration $(DURATION) \
	  --out /workspace/out

do-not-pass:     ## Do-Not-Pass spoofing vs the live sim: make do-not-pass [RUN=both] [DRONE_BACK=-2.5]
                 ## Prepares its own world (loads Town01, clears traffic) — just 'make up' first.
	$(COMPOSE) run --rm spoofing \
	  python -m carla_spoofing.scenarios.do_not_pass_spoofing \
	  --mode carla --host carla-sim --port 2000 --run $(RUN) \
	  --drone-back $(DRONE_BACK) \
	  --out /workspace/out/do_not_pass

do-not-pass-mock: ## Same scenario with no simulator at all (kinematic mock)
	$(COMPOSE) run --rm --no-deps spoofing \
	  python -m carla_spoofing.scenarios.do_not_pass_spoofing \
	  --mode mock --run $(RUN) --drone-back $(DRONE_BACK) \
	  --out /workspace/out/do_not_pass

test:            ## Run the unit tests (no sim, no GPU)
	PYTHONPATH=src python -m pytest tests/ -q

down:            ## Stop and remove the sim container
	$(COMPOSE) down

logs:            ## Tail the sim container logs
	$(COMPOSE) logs -f carla-sim

clean:           ## Remove containers (keeps the downloaded binary)
	$(COMPOSE) down --remove-orphans
