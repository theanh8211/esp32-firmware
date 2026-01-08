#!/usr/bin/env bash
# Simple flash helper using PlatformIO
set -euo pipefail
BASEDIR=$(dirname "$0")
ENV=esp32dev

echo "Building and uploading firmware (env=$ENV)"
cd "$BASEDIR"
pio run -e $ENV -t upload

echo "Done. Open serial monitor with:"
echo "  pio device monitor -b 115200"
