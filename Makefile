# CARLA V2X spoofing — one-command workflows.
# From scratch:  make toolkit   (once, sudo)  ->  make setup  ->  make up
COMPOSE := docker compose -f docker/docker-compose.yml
HEADLESS := $(COMPOSE) -f docker/docker-compose.headless.yml
ATTACK ?= fake_object

.PHONY: help doctor toolkit setup download extract build up gui headless spoof down logs clean

help:            ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-12s\033[0m %s\n",$$1,$$2}'

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

spoof:           ## Run an attack vs the live sim: make spoof ATTACK=remove_object
	$(COMPOSE) run --rm spoofing carla-spoof --mode carla --host carla-sim \
	  --port 2000 --attack $(ATTACK) --out /workspace/out

down:            ## Stop and remove the sim container
	$(COMPOSE) down

logs:            ## Tail the sim container logs
	$(COMPOSE) logs -f carla-sim

clean:           ## Remove containers (keeps the downloaded binary)
	$(COMPOSE) down --remove-orphans
