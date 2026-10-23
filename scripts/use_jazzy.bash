# Source from the repository root inside Ubuntu-24.04.
source /opt/ros/jazzy/setup.bash
source "$HOME/venvs/cf-jazzy/bin/activate"
export PYTHONPATH="$PWD/src/cf_control:$PYTHONPATH"
