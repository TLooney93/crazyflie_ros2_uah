Undergrad Senior Design Project - CrazyConductors
University of Alabama Huntsville EE dept Fall 2026
Drone swarm software architecture using Crazyflie platform

OBJECTIVE:
Autonomous drone swarm commanded by wand gesture

EXECUTION:
Utilize Cflib (Python API used for intefacing with CF firmware) and ros2 to inteface multiple nodes, 
implement safety filter, and recognize and execute gesture commands.


CURRENT PYTHON ARCHITECTURE:

- `crazyflie_node.py`: owns one radio connection and its ROS 2 topics
- `telemetry.py`: reads Lighthouse-backed position estimates from cflib
- `flight_controller.py`: converts position error into bounded velocity
- `crazyflie_interface.py`: sends commands through the cflib commander
- `swarm_node.py`: future coordination layer for multiple drone nodes
- `lighthouse.py`: future Lighthouse validation and estimator setup

Per-drone ROS 2 data flow:

    /<drone_id>/target (geometry_msgs/Point)
        -> FlightController
        -> bounded world-frame velocity
        -> cflib
        -> Crazyflie

    Crazyflie stateEstimate.{x,y,z}
        -> TelemetryReader
        -> /<drone_id>/position (geometry_msgs/Point)

The node does not send motion commands until it has both position telemetry and
an explicit target. Configuration in `config.py` currently assumes metres for
position and velocity and degrees per second for yaw rate.

