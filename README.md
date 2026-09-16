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

The node currently runs the controller in dry-run mode and does not send motion
commands to the drone. Its diagnostic velocity is zero when no target exists or
position telemetry is missing, invalid, or stale. Flight-state handling and
takeoff/landing integration are still to be implemented. Configuration in
`config.py` uses metres for position, metres per second for velocity, and degrees
per second for yaw rate.
