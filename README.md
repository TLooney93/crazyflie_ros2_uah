Undergrad Senior Design Project - CrazyConductors
University of Alabama Huntsville EE dept Fall 2026
Drone swarm software architecture using Crazyflie platform and LightHouse Positioning System

OBJECTIVE:
Autonomous drone swarm commanded by wand gesture

EXECUTION:
Utilize Cflib (Python API used for intefacing with CF firmware) and ros2 to inteface multiple nodes,
implement safety filter, and recognize and execute gesture commands.


CURRENT PYTHON ARCHITECTURE:

- `crazyflie_node.py`: owns one radio connection and its ROS 2 topics
- `telemetry.py`: reads Lighthouse-backed position estimates from cflib
- `flight_controller.py`: converts position error into bounded velocity
- `crazyflie_interface.py`: provides cflib commander methods for future flight integration
- `swarm_node.py`: future coordination layer for multiple drone nodes
- `lighthouse.py`: validates Lighthouse input and estimator variance stability during startup

Per-drone ROS 2 data flow:

    /<drone_id>/target (geometry_msgs/Point)
        -> FlightController
        -> bounded world-frame velocity (requires fresh, valid position telemetry)
        -> /<drone_id>/debug/velocity (geometry_msgs/Twist)

    Crazyflie stateEstimate.{x,y,z}
        -> TelemetryReader
        -> /<drone_id>/position (geometry_msgs/Point)

The node includes a deliberately constrained first-flight mode. It records the
ground position when armed, takes off vertically to 0.30 m, holds the recorded
x/y position, and lands at the recorded ground position. External target topics
are disabled by default. A maximum flight timer initiates landing automatically.

## First vertical flight test

Do the first checks with propellers removed. For a propeller-on test, use a clear
indoor test area, propeller guards, a fully charged battery, and a second terminal
with the emergency-stop command ready.

Set up the environment and start the node:

```bash
source /opt/ros/jazzy/setup.bash
source /home/tanner-looney/dev/crazyflie/venv/bin/activate
cd /home/tanner-looney/dev/crazyflie/cfros_ws
colcon build --packages-select cf_control
source install/setup.bash
ros2 run cf_control crazyflie_node --ros-args -p drone_id:=cf1
```

In a second terminal, source the same ROS, virtual-environment, and workspace
setup files. Confirm telemetry and state before arming:

```bash
ros2 topic echo /cf1/position
ros2 topic echo /cf1/flight_state
```

Then invoke each step separately:

```bash
ros2 service call /cf1/arm std_srvs/srv/Trigger {}
ros2 service call /cf1/takeoff std_srvs/srv/Trigger {}
ros2 service call /cf1/land std_srvs/srv/Trigger {}
```

Keep this command ready in another terminal. It stops and disarms immediately;
it does not perform a controlled landing:

```bash
ros2 service call /cf1/emergency_stop std_srvs/srv/Trigger {}
```

If position telemetry becomes stale in flight, the node commands a 0.10 m/s
vertical descent for at most three seconds and then stops and disarms. The normal
flight test automatically begins landing after ten seconds. Configuration in
`config.py` uses metres for position, metres per second for velocity, and degrees
per second for yaw rate.
