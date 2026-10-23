#!/usr/bin/env bash
set -eo pipefail

echo "=== RELATED PROCESSES ==="
ps -ef | grep -E 'cf_control|cfclient|python3' | grep -v grep || true
echo "=== CRAZYRADIO OWNER ==="
if command -v fuser >/dev/null; then
  fuser -v /dev/bus/usb/001/003 || true
fi
