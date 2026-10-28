#!/usr/bin/env bash
# Run only inside the new Ubuntu-24.04 distribution.
set -eo pipefail
source /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 24.04 ]; then
    echo 'This script requires Ubuntu 24.04. Existing environment unchanged.' >&2
    exit 1
fi
sudo apt-get update
sudo apt-get install -y curl ca-certificates software-properties-common python3-venv
sudo add-apt-repository -y universe
release=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
package_dir=$(mktemp -d)
curl -fL -o "$package_dir/ros2-apt-source.deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$release/ros2-apt-source_${release}.noble_all.deb"
sudo dpkg -i "$package_dir/ros2-apt-source.deb"
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-dev-tools python3-pytest libusb-1.0-0 usbutils
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/venvs"
python3 -m venv --system-site-packages "$HOME/venvs/cf-jazzy"
source "$HOME/venvs/cf-jazzy/bin/activate"
python3 -m pip install cflib pytest
python3 -c 'import rclpy, cflib; print("Jazzy Python imports OK")'
echo 'Setup complete. Source scripts/use_jazzy.bash from the repository root.'
