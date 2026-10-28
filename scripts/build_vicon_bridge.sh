#!/usr/bin/env bash
set -eo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
linux_ws="$HOME/cf_jazzy_ws"
mkdir -p "$linux_ws/src/vicon_datastream_bridge"
cp -R "$repo_dir/src/vicon_datastream_bridge/." "$linux_ws/src/vicon_datastream_bridge/"
cd "$linux_ws"
colcon --log-base log build \
  --base-paths src/vicon_datastream_bridge \
  --build-base build \
  --install-base install \
  --packages-select vicon_datastream_bridge \
  --cmake-args -DVICON_SDK_DIR="$repo_dir/output/vicon-sdk-linux64/Linux64"
