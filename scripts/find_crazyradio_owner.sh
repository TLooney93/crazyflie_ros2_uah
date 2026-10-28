#!/usr/bin/env bash
set -eo pipefail

echo "=== RELATED PROCESSES ==="
ps -ef | grep -E 'cf_control|cfclient|python3' | grep -v grep || true
echo "=== CRAZYRADIO OWNER ==="
crazyradio_line=$(lsusb -d 1915:7777 2>/dev/null | head -n 1 || true)
if [[ -z "$crazyradio_line" ]]; then
  echo "Crazyradio 1915:7777 is not attached to this WSL instance."
  exit 0
fi

bus=$(awk '{print $2}' <<<"$crazyradio_line")
device=$(awk '{gsub(":", "", $4); print $4}' <<<"$crazyradio_line")
crazyradio_path="/dev/bus/usb/$bus/$device"
echo "Device: $crazyradio_path"
ls -l "$crazyradio_path"

if command -v fuser >/dev/null; then
  fuser -v "$crazyradio_path" || true
fi
