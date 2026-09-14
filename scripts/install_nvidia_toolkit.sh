#!/usr/bin/env bash
# Install the NVIDIA Container Toolkit so Docker containers can use the GPU.
# Requires sudo. Run once on the host. (Ubuntu 22.04.)
set -euo pipefail
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
echo "OK. Test with: docker run --rm --gpus all nvidia/cuda:12.3.2-base-ubuntu22.04 nvidia-smi"
