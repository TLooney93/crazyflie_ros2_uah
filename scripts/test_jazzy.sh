#!/usr/bin/env bash
set -eo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
source scripts/use_jazzy.bash
python3 -m pytest \
  src/cf_control/test/test_vertical_gesture.py \
  src/cf_control/test/test_gesture_trial_logger.py \
  src/cf_control/test/test_r2_command_test.py \
  src/cf_control/test/test_wand_node.py \
  src/cf_control/test/test_wand_button_node.py -q
colcon --log-base /tmp/cf-jazzy-colcon-log build \
  --base-paths src/cf_control \
  --packages-select cf_control \
  --build-base /tmp/cf-jazzy-build \
  --install-base /tmp/cf-jazzy-install
