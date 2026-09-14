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
        libomp5 libjpeg-turbo8 libpng16-16 libtiff5 xdg-user-dirs \
    && rm -rf /var/lib/apt/lists/*

# Miniforge (conda-forge by default) + the CarlaAir Python env (Python 3.10).
# Miniforge avoids Anaconda's default channels and their Terms-of-Service gate,
# keeping the image freely redistributable for other researchers.
RUN wget -qO /tmp/mf.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh \
    && bash /tmp/mf.sh -b -p $CONDA_DIR && rm /tmp/mf.sh \
    && conda create -y -n carlaAir -c conda-forge --override-channels python=3.10 \
    && conda run -n carlaAir pip install --no-cache-dir \
        numpy Pillow pygame pytest \
    && conda clean -afy

# AirSim drone client (port 41451). Fragile to install: airsim's setup.py imports
# its whole runtime stack just to read __version__, and msgpack-rpc-python still
# uses the removed setuptools use_2to3. So: pin setuptools<58 for these builds,
# preinstall the runtime deps, install with build isolation OFF, then restore
# a modern setuptools (our editable install builds in isolation, so it's fine).
RUN conda run -n carlaAir pip install --no-cache-dir "setuptools<58" wheel \
    && conda run -n carlaAir pip install --no-cache-dir numpy tornado "opencv-contrib-python<4.9" \
    && conda run -n carlaAir pip install --no-cache-dir --no-build-isolation msgpack-rpc-python \
    && conda run -n carlaAir pip install --no-cache-dir --no-build-isolation airsim \
    && conda run -n carlaAir pip install --no-cache-dir -U "setuptools>=61" \
    && conda run -n carlaAir python -c "import airsim; print('airsim', airsim.__version__, 'installed')"

# NVIDIA Vulkan ICD manifest. CUDA base images get the driver libs (mounted by
# the container toolkit) but NOT this manifest, so the Vulkan loader can't find
# the NVIDIA driver (ERROR_INCOMPATIBLE_DRIVER) and UE4 -RenderOffScreen dies.
RUN mkdir -p /usr/share/vulkan/icd.d && printf '%s\n' \
    '{ "file_format_version": "1.0.0", "ICD": { "library_path": "libGLX_nvidia.so.0", "api_version": "1.3.194" } }' \
    > /usr/share/vulkan/icd.d/nvidia_icd.json

# X client libraries that UE4's SDL dlopens for the on-screen (GUI) window mode
# (HEADLESS=0). Headless off-screen rendering doesn't need these.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libxcursor1 libxinerama1 libxrandr2 libxkbcommon0 libxss1 \
    && rm -rf /var/lib/apt/lists/*

# Our spoofing toolkit (installed editable so /workspace edits take effect live).
WORKDIR /workspace
COPY . /workspace
RUN conda run -n carlaAir pip install --no-cache-dir -e /workspace

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# CARLA/UE4 shipping binaries refuse to run as root. Create a uid-1000 user
# (matches the host ownership of the mounted vendor/ and workspace volumes) and
# hand it the conda env + workspace so runtime setup_env.sh can write there.
RUN userdel -r ubuntu 2>/dev/null || true \
    && useradd -m -u 1000 -s /bin/bash carla \
    && chown -R carla:carla /opt/conda /workspace
USER carla

# Ports: CARLA RPC (2000-2002), CARLA streaming, AirSim drone RPC (41451),
# and our UDP OMNeT++ bridge (47000).
EXPOSE 2000 2001 2002 41451 47000/udp

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
