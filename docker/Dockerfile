# CARLA V2X cooperative-perception spoofing — reproducible environment.
#
# Base: CUDA + OpenGL/Vulkan userspace. The actual GPU driver libraries are
# injected at runtime by the NVIDIA Container Toolkit (NVIDIA_DRIVER_CAPABILITIES).
# The heavy CarlaAir v0.1.7 binary is NOT baked in; it is mounted at /opt/carlaair
# from ./vendor (fetched once via scripts/download_carlaair.sh) to keep the image
# lean and the download cache reusable across researchers.
FROM nvidia/cuda:12.3.2-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    CONDA_DIR=/opt/conda \
    PATH=/opt/conda/bin:$PATH

# System libs needed to run the UE4-based CarlaAir binary + our Python stack.
RUN apt-get update && apt-get install -y --no-install-recommends \
        wget ca-certificates unzip xz-utils git \
        libvulkan1 vulkan-tools libgl1 libgl1-mesa-dri libglu1-mesa \
        libsm6 libxext6 libxrender1 libxi6 libglib2.0-0 \
        libomp5 libjpeg-turbo8 libpng16-16 xdg-user-dirs \
    && rm -rf /var/lib/apt/lists/*

# Miniconda + the CarlaAir Python environment (Python 3.10, per env_setup).
RUN wget -qO /tmp/mc.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    && bash /tmp/mc.sh -b -p $CONDA_DIR && rm /tmp/mc.sh \
    && conda create -y -n carlaAir python=3.10 \
    && conda run -n carlaAir pip install --no-cache-dir \
        numpy Pillow pygame airsim pytest \
    && conda clean -afy

# Our spoofing toolkit (installed editable so /workspace edits take effect live).
WORKDIR /workspace
COPY . /workspace
RUN conda run -n carlaAir pip install --no-cache-dir -e /workspace

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Ports: CARLA RPC (2000-2002), CARLA streaming, AirSim drone RPC (41451),
# and our UDP OMNeT++ bridge (47000).
EXPOSE 2000 2001 2002 41451 47000/udp

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
