#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/cf_jazzy_ws/install/setup.bash"

exec ros2 run vicon_datastream_bridge vicon_datastream_bridge \
  --ros-args -p host:="${VICON_HOST:-10.86.153.14:801}"
