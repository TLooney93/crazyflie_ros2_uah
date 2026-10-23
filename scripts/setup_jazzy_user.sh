#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/venvs"
python3 -m venv --system-site-packages "$HOME/venvs/cf-jazzy"
source "$HOME/venvs/cf-jazzy/bin/activate"
python3 -m pip install --upgrade pip
python3 -m pip install cflib pytest
python3 -c 'import rclpy, cflib; print("Jazzy Python imports OK")'
