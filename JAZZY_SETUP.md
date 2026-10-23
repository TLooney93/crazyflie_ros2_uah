# Separate Jazzy environment

Keep the existing `Ubuntu` WSL distribution for Lyrical. Use `Ubuntu-24.04`
for Jazzy; do not source the old workspace's `install/setup.bash` or activate
the repository's existing `.venv` there. They were built for another ROS/Python
version.

Open the new environment from PowerShell:

```powershell
wsl -d Ubuntu-24.04
```

Complete any first-run username/password prompt locally. From the repository
root inside Ubuntu-24.04, install the dependencies once:

```bash
bash scripts/setup_jazzy.sh
```

The setup script checks Ubuntu 24.04 before changing packages. It installs
ROS Jazzy base and development tools, then creates a separate Python environment
at `~/venvs/cf-jazzy`. It does not install a Vicon bridge or contact Tracker.

For subsequent terminals, from the repository root:

```bash
source scripts/use_jazzy.bash
echo "$ROS_DISTRO"
python3 -m pytest src/cf_control/test/test_vertical_gesture.py src/cf_control/test/test_wand_node.py src/cf_control/test/test_wand_button_node.py -q
```

Expected ROS distribution: `jazzy`. The Lyrical-specific test helper under
`firmware/` should not be used in this environment.

If building the ROS package, keep all generated directories separate:

```bash
colcon --log-base /tmp/cf-jazzy-colcon-log build --base-paths src/cf_control --packages-select cf_control --build-base /tmp/cf-jazzy-build --install-base /tmp/cf-jazzy-install
```

These build outputs are temporary. For direct development, the activation
script already places the source package on Python's import path, so
`python3 -m cf_control.wand_gesture_node` does not need a colcon rebuild.

## Vicon DataStream bridge

Tracker 4.4 publishes the rigid bodies `cf1` (drone) and `cf2` (wand) through
the official Vicon DataStream SDK on `10.86.153.14:801`. The local bridge maps
them as follows:

```text
cf1 -> /vicon/cf1/pose
cf2 -> /wand/pose
```

Build the bridge after changes:

```bash
bash scripts/build_vicon_bridge.sh
```

Start it and leave the terminal open:

```bash
bash scripts/run_vicon_bridge.sh
```

For a ten-second, read-only verification:

```bash
bash scripts/test_vicon_ros_bridge.sh
```

The expected final line is `PASS: valid cf1 and cf2 poses were published.`
The bridge does not publish an object while Tracker marks its pose occluded.
This bridge only publishes pose data; it does not arm or command a drone.

References:
- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- https://help.vicon.com/download/attachments/376309877/Vicon%20Tracker%20ROS%202%20Quick%20Start%20Guide.pdf
