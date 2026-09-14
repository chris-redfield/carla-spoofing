#!/usr/bin/env bash
# Extract the CarlaAir binary into vendor/CarlaAir-v0.1.7/ (mounted into Docker).
set -euo pipefail
ZIP="vendor/CarlaAir-v0.1.7.zip"
OUT="vendor/CarlaAir-v0.1.7"
[ -f "$ZIP" ] || { echo "Missing $ZIP — run scripts/download_carlaair.sh first"; exit 1; }
echo "Extracting $ZIP ..."
mkdir -p "$OUT"
unzip -q -o "$ZIP" -d "$OUT"
# Some archives nest a single top-level dir; flatten if so.
inner=$(find "$OUT" -maxdepth 1 -mindepth 1 -type d | head -1)
if [ -f "$inner/CarlaAir.sh" ] && [ ! -f "$OUT/CarlaAir.sh" ]; then
    echo "Flattening nested dir $inner ..."
    shopt -s dotglob; mv "$inner"/* "$OUT"/; rmdir "$inner"
fi
chmod +x "$OUT"/*.sh 2>/dev/null || true
echo "Done. CarlaAir at $OUT"
ls -la "$OUT" | head -20
