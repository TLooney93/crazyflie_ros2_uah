#!/usr/bin/env bash
# Run from the repository root inside WSL.
set -e
source .venv/bin/activate
source /opt/ros/lyrical/setup.bash
export PYTHONPATH="$PWD/src/cf_control:$PYTHONPATH"
python3 -m pytest src/cf_control/test/test_vertical_gesture.py src/cf_control/test/test_wand_node.py src/cf_control/test/test_wand_button_node.py -q
