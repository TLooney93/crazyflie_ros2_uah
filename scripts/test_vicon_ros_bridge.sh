#!/usr/bin/env bash
set -eo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
source /opt/ros/jazzy/setup.bash
source "$HOME/cf_jazzy_ws/install/setup.bash"

log_file=/tmp/vicon-ros-bridge.log
timeout 10s ros2 run vicon_datastream_bridge vicon_datastream_bridge \
  --ros-args -p host:=10.86.153.14:801 >"$log_file" 2>&1 &
bridge_pid=$!
sleep 3

echo "Published Vicon topics:"
ros2 topic list -t | grep -E '^/(vicon/cf1/pose|wand/pose)' || true

wait "$bridge_pid" || true
echo
echo "Bridge log:"
cat "$log_file"

grep -q 'Publishing cf1 pose' "$log_file"
grep -q 'Publishing cf2 pose' "$log_file"
echo
echo "PASS: valid cf1 and cf2 poses were published."
