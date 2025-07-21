#!/usr/bin/env bash
set -euo pipefail

# Base directory (directory of this script)
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# List of subdirectories containing install.sh
DIRS=(
  "chromium"
  "flir"
  "microstrain"
  "ouster"
  "ros2-humble"
)

for d in "${DIRS[@]}"; do
  echo "=== Running install in $d ==="
  SCRIPT="$BASE_DIR/$d/install.sh"
  if [[ -f "$SCRIPT" ]]; then
    chmod +x "$SCRIPT"
    (cd "$BASE_DIR/$d" && bash install.sh)
  else
    echo "Warning: $d/install.sh not found, skipping."
  fi
done

echo "All installations completed."

