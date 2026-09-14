#!/usr/bin/env bash
# Download the CarlaAir v0.1.7 prebuilt Ubuntu binary (6.85 GB) from Hugging Face.
# Resumable. Run from the repo root.
set -euo pipefail
URL="https://huggingface.co/tianlezeng/CarlaAIr-v0.1.7/resolve/main/CarlaAir-v0.1.7.zip"
DEST="vendor/CarlaAir-v0.1.7.zip"
EXPECTED_BYTES=6846384047
mkdir -p vendor
echo "Downloading CarlaAir v0.1.7 (~6.85 GB, resumable) ..."
wget -c -O "$DEST" "$URL"
SIZE=$(stat -c%s "$DEST")
if [ "$SIZE" -ne "$EXPECTED_BYTES" ]; then
    echo "WARNING: size $SIZE != expected $EXPECTED_BYTES (download may be incomplete)"; exit 1
fi
echo "OK: $DEST ($SIZE bytes). Next: bash scripts/extract_carlaair.sh"
