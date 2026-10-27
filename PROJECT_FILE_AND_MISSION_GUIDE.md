# Crazyflie Project File and Mission Guide

This document explains the purpose of the active project files and how the
current flight missions are assembled. Use `LAUNCH_TEST_REFERENCE.md` for
terminal commands and `FLIGHT_TEST_REFERENCE.md` for the complete Vicon
hardware checklist.

## Active control flow

The primary runtime path is:

```text
Position source
  Vicon PoseStamped / Lighthouse estimator / Gazebo odometry
        |
        v
crazyflie_node.py
  safety checks + flight state + mission phase
        |
        v
flight_controller.py
  position error -> bounded world-frame velocity
        |
        v
crazyflie_interface.py
  cflib radio command or Gazebo Twist publication
```

Wand input joins the same guarded controller:

```text
wand_node.py or wand_driver_node.py
        |
        +-- /wand/gesture
        +-- /wand/pose
        |
        v
crazyflie_node.py
  enable gate + WAND_TAKEOFF/WAND_HOVER + bounded target
```

Swarm control keeps the same per-drone safety path:

```text
swarm_missions.py
  named phase offsets and hold times
        |
        v
swarm_node.py
  group services + position/state monitoring + separation checks
        |
        +-- /cf1/target --> crazyflie_node.py --> cf1
        +-- /cf2/target --> crazyflie_node.py --> cf2
```

The coordinator never sends motor commands. Every vehicle still owns its own
arming, position freshness, target bounds, landing, timeout, and emergency
stop behavior.

## Core runtime files

### `src/cf_control/cf_control/config.py`

Central configuration for the active controller.

It contains:

- Crazyflie IDs and radio URIs.
- Controller rate and proportional gain.
- Position and velocity limits.
- Estimator and telemetry timeouts.
- Autonomous mission height, timing, radius, tolerance, and landing values.
- Wand topics, tilt threshold, repeat timing, step size, and travel boundary.
- External-target bounds and swarm timing, tolerance, and separation limits.

Change numeric tuning and mission geometry here when the mission structure is
unchanged. Rebuild after modifying it.

### `src/cf_control/cf_control/crazyflie_node.py`

The main ROS 2 controller and the authoritative mission implementation.

Responsibilities:

- Selects Vicon, Lighthouse, or Gazebo position input.
- Owns one real Crazyradio connection for its configured drone.
- Validates telemetry and estimator readiness before arming.
- Publishes position, flight state, and debug velocity.
- Provides arm, takeoff, land, clear-target, and emergency-stop services.
- Runs autonomous, external-target, and wand mission phases.
- Detects stale position data and initiates the telemetry-loss descent.
- Enforces the maximum flight time.
- Stops and disarms during shutdown.

The mission phase logic is in `_update_test_sequence()`. In external mode it
takes off to `EXTERNAL_HOVER` and accepts bounded topic targets. Autonomous
mode cannot be overwritten by topic commands. The public service state is
separate from the internal mission phase.

### `src/cf_control/cf_control/flight_controller.py`

Small proportional position controller.

It stores one `(x, y, z)` target and computes:

```text
velocity = POSITION_KP * (target - current_position)
```

Each axis is limited to `MAX_VELOCITY`. This file does not choose waypoints,
manage state, or perform safety checks. Change it only when changing the
control law itself.

### `src/cf_control/cf_control/crazyflie_interface.py`

Hardware/simulation command boundary.

`CrazyflieInterface` sends:

- Firmware parameter changes.
- Arming and disarming requests.
- World-frame velocity setpoints.
- External Vicon positions.
- Stop and disarm commands.

`SimulatedCrazyflieInterface` exposes the same flight-command behavior using a
ROS callback for Gazebo. Mission code should call this interface instead of
calling cflib directly.

### `src/cf_control/cf_control/flight_state.py`

Defines the public safety states:

```text
DISARMED
READY
FLYING
LANDING
```

Do not add a state for every waypoint. Internal mission phases belong in
`crazyflie_node.py`; public states should represent safety-relevant vehicle
conditions.

### `src/cf_control/cf_control/telemetry.py`

Background cflib telemetry reader.

For the receiver drone it reads `stateEstimate.x/y/z`. With
`include_attitude=True`, the physical wand driver also reads stabilizer roll,
pitch, and yaw. It keeps only the newest sample so delayed telemetry does not
build up in a queue.

### `src/cf_control/cf_control/lighthouse.py`

Lighthouse startup verification and Kalman-estimator reset helper.

It checks:

- Lighthouse status.
- Finite position variances.
- Variance stability across recent samples.
- A startup timeout.

It is used only when `positioning_source:=lighthouse`. Vicon uses the same
estimator-reset helper but receives measurements from ROS.

### `src/cf_control/cf_control/wand_control.py`

Pure wand math and validation helpers:

- Normalizes gesture names.
- Updates the keyboard wand's simulated position.
- Converts Euler angles and quaternions.
- Converts relative wand tilt into a world-frame XY direction.

Keep math that can be unit-tested without ROS in this file.

### `src/cf_control/cf_control/wand_node.py`

Interactive keyboard wand command source.

`cfw` publishes commands only; `cfwand` remains an equivalent compatibility
name. `cfwandsim` enables the optional simulated
pose publisher. It publishes:

- `/wand/gesture`
- `/wand/enabled`
- `/wand/connected`
- `/wand/pose` when simulated pose is enabled

This node does not command the drone directly. The main flight node applies
all safety and state checks.

### `src/cf_control/cf_control/wand_driver_node.py`

Physical `cf2` wand reader.

It opens its own cflib connection, reads `cf2` position and attitude, converts
the attitude to a quaternion, and publishes `/wand/pose`. It also publishes
the connection state once per second.

Use a second Crazyradio if separate `cf1` and `cf2` processes cannot share the
same dongle. Remove all props from the handheld wand.

### `src/cf_control/cf_control/swarm_node.py`

Active ROS 2 swarm coordinator.

It:

- Observes each drone's position and public flight state.
- Calls all arm, takeoff, land, and emergency-stop services asynchronously.
- Offers full-sequence and staged `/swarm/*` services.
- Publishes one target per drone during a saved mission.
- Verifies fresh position data, target separation, takeoff timeout, and total
  sequence timeout.
- Lands the group when a mission completes or a safety check fails.
- In wand group mode, leaves movement to the common gesture topic while it
  monitors group takeoff, position freshness, and actual separation.

### `src/cf_control/cf_control/swarm_missions.py`

The normal file to edit when designing a swarm choreography. Each named
mission is a list of phases with a name, a hold time, and one XYZ offset per
drone. Offsets are relative to each vehicle's independently recorded home
position. Definitions are validated before the coordinator starts.

### `src/cf_control/cf_control/__init__.py`

Marks `cf_control` as a Python package. It intentionally contains no runtime
logic.

## ROS package and build files

### `src/cf_control/setup.py`

Installs the Python package, launch files, Lighthouse YAML files, and console
commands:

```text
crazyflie_node
swarm_node
wand_node
wand_driver_node
```

Add a new Python executable here when a new module needs to be launched with
`ros2 run`.

### `src/cf_control/setup.cfg`

Places installed Python executables in the ROS package's expected `lib`
directory.

### `src/cf_control/package.xml`

Declares ROS runtime and test dependencies. Add a dependency here when active
code imports another ROS package.

### `src/cf_control/resource/cf_control`

ROS package-index marker. Do not put code in it.

## Launch and simulation files

### `src/cf_control/launch/control.launch.py`

Hardware-mode selector. It creates one protected `crazyflie_node` per drone
and adds `swarm_node` for multi-drone modes. Launch arguments select:

```text
mode: single | swarm
input: autonomous | terminal | wand
drone_ids: comma-separated names
positioning_source: vicon | lighthouse
vicon_topics: comma-separated PoseStamped topics
radio_uris: optional comma-separated URI overrides
mission_name: swarm_missions.py key
```

The `cfsingle`, `cfterminal`, `cfwandflight`, and `cfswarm` helpers build these
arguments for routine use.

### `src/cf_gazebo/launch/simulation.launch.py`

Starts only the Gazebo world. It selects GUI or headless operation, accepts a
world-file override, and sets the Gazebo model search path.

Use it through:

```bash
cfgazebo
cfgazebo headless
```

### `src/cf_gazebo/launch/flight_test.launch.py`

Starts the complete simulation stack:

- Includes `simulation.launch.py`.
- Starts the ROS/Gazebo topic bridge.
- Starts `crazyflie_node` in simulation mode.
- Passes autonomous, terminal-target, and wand mode arguments.

Use `cfsim gui autonomous`, `cfsim gui terminal`, or `cfsim gui wand`.

### `src/cf_gazebo/launch/swarm_test.launch.py`

Starts the two-model Gazebo world, bridges both vehicles, starts one simulated
controller per vehicle, and starts the real swarm coordinator. It accepts
autonomous mission, per-drone terminal target, and common wand modes. Use it
through `cfswarmsim` before any hardware swarm test.

### `src/cf_gazebo/worlds/crazyflie_world.sdf`

Defines the Gazebo world and includes the Crazyflie model.

Change this file for environmental objects, lighting, gravity, or model spawn
placement. Do not define the flight mission here.

### `src/cf_gazebo/worlds/crazyflie_swarm_world.sdf`

Two-model integration-test world. The models are named `cf1` and `cf2` so
each controller can use its own velocity and odometry topics.

### `src/cf_gazebo/models/crazyflie/model.sdf`

Active Gazebo Crazyflie model and simulation plugins. It defines the model
used by the world, including velocity control and odometry publication.

### `src/cf_gazebo/urdf/crazyflie2.urdf.xacro`

URDF/Xacro description retained for robot-description and visualization use.
The current Gazebo world loads `models/crazyflie/model.sdf` directly.

### `src/cf_gazebo/meshes/crazyflie2.dae`

Visual mesh for the Crazyflie model. It affects appearance, not the mission or
control behavior.

### `src/cf_gazebo/package.xml` and `CMakeLists.txt`

Declare and install the Gazebo package, launch files, worlds, models, meshes,
and URDF resources.

## Lighthouse configuration files

### `src/cf_control/config/lighthouse/lh_calibration28sep.yaml`

Project calibration data captured for the September Lighthouse setup. Treat it
as installation-specific data.

### `src/cf_control/config/lighthouse/lhsample_3.yaml`

Additional/sample Lighthouse geometry. Verify which calibration matches the
physical base-station arrangement before using it.

### `src/cf_control/config/lighthouse/lighthouse_config_vis.py`

Visualization/helper script for Lighthouse geometry. It is not called by the
active flight node.

## Automated tests

### `src/cf_control/test/test_flight_controller.py`

Tests proportional velocity calculation, zero output without a target, and
per-axis velocity limits.

### `src/cf_control/test/test_crazyflie_interface.py`

Tests hardware-wrapper calls and simulated-interface arming, velocity, and
stop behavior without a real drone.

### `src/cf_control/test/test_wand_control.py`

Tests gesture normalization, simulated movement, quaternion conversion, tilt
direction, heading rotation, and the tilt dead zone.

### `src/cf_control/test/test_swarm_missions.py`

Tests valid mission loading and rejects unknown missions, wrong drone sets,
and duplicate drone IDs.

### `test_flake8.py`, `test_pep257.py`, and `test_copyright.py`

Package-wide quality checks. They also scan legacy example scripts, so the full
suite currently reports existing cleanup work outside the active runtime path.

## Shell and operator documents

### `/home/tanner-looney/dev/crazyflie/scripts/cf_env.sh`

Defines the project environment and the build, single-drone, terminal, wand,
swarm, simulation, monitor, and safety helper functions.

### `~/.bash_aliases`

Defines `cfenv`, `cfvicon`, and `cfkill` for new shell sessions.

### `FLIGHT_TEST_REFERENCE.md`

Detailed Vicon network, startup, safety, mission, wand, shutdown, and
troubleshooting runbook.

### `LAUNCH_TEST_REFERENCE.md`

Command inventory and test-by-test terminal procedure.

### `README.md`

Original project overview. Some descriptions reflect earlier vertical-flight
milestones; use the two current reference documents for operator procedures.

## Legacy and experimental tools

Files under `src/cf_control/tools/` are direct cflib experiments and upstream
examples. They bypass parts or all of the active ROS safety path. Do not use
them for routine flight testing.

| File | Purpose and status |
|---|---|
| `scanner.py` | Useful read-only Crazyradio address scan. Do not run while another process owns the radio. |
| `autonomous_sequence.py` | Upstream low-level position-setpoint sequence example; not used by the ROS mission. |
| `autonomous_sequence_high_level.py` | Upstream high-level trajectory/figure-eight example; not used by the ROS mission. |
| `motion_flying.py` | Experimental MotionCommander boundary behavior; not part of the guarded controller. |
| `testflight1.py` | Early direct MotionCommander flight script with an outdated URI format. Do not use for the current test procedure. |
| `connect_log_param.py` | Early parameter/logging experiment containing placeholder functions. |
| `hello_node.py` | Basic ROS publisher tutorial node; not installed as a console command. |
| `swarm.py` | Incomplete local swarm helper copy with unresolved imports; not active. |
| `swarm_test.py` | Direct cflib swarm example; not active. |
| `swarm_test_link.py` | Experimental multi-link script with incomplete imports; not safe as an operator test. |

## Current mission structure

### Public flight states

The externally visible state sequence is:

```text
DISARMED -> READY -> FLYING -> LANDING -> DISARMED
```

- `cfarm` records `home_position`, arms, and enters `READY`.
- `cftakeoff` starts the normal mission and enters `FLYING`.
- `cfland` or automatic completion enters `LANDING`.
- Landing completion, emergency stop, or shutdown enters `DISARMED`.

### Autonomous mission phases

The normal takeoff service sets `test_phase` to `TAKEOFF`:

```text
TAKEOFF
  Reach home Z + FLIGHT_TEST_HEIGHT_M
        |
        v
HOVER
  Wait FLIGHT_TEST_HOVER_S
        |
        v
FIGURE_EIGHT
  Follow the generated path for FLIGHT_TEST_FIGURE_EIGHT_DURATION_S
        |
        v
RETURN_HOME
  Reach the original XY at flight height
        |
        v
LANDING
```

`FLYING` remains the public state during all internal phases.

### Wand mission phases

A wand `takeoff` sets:

```text
WAND_TAKEOFF -> WAND_HOVER -> LANDING
```

`WAND_HOVER` holds the last target and accepts bounded direction or tilt
commands. It does not execute the autonomous figure-eight.

### Terminal/external phases

A terminal or coordinator takeoff sets:

```text
EXTERNAL_TAKEOFF -> EXTERNAL_HOVER -> LANDING
```

Only `EXTERNAL_HOVER` accepts `/DRONE_ID/target`. Targets are absolute world
coordinates and must stay within the configured horizontal and vertical
offsets from the home recorded at arming.

### Swarm coordinator states

The complete saved-sequence path is:

```text
IDLE -> ARMING -> ARM_SETTLE -> TAKEOFF -> HOVER -> MISSION
     -> LANDING -> IDLE
```

The coordinator publishes the current value on `/swarm/state`. Individual
vehicles continue to publish `DISARMED`, `READY`, `FLYING`, and `LANDING`.

## Design or change a swarm sequence

Edit `src/cf_control/cf_control/swarm_missions.py`. This is the main swarm
choreography file; do not add formation waypoints to `swarm_node.py`.

The bundled pattern is:

```python
SWARM_MISSIONS = {
    'two_drone_demo': [
        {
            'name': 'SPREAD',
            'hold_s': 2.0,
            'offsets': {
                'cf1': (-0.15, 0.0, 0.35),
                'cf2': (0.15, 0.0, 0.35),
            },
        },
        # Add more phases here.
    ],
}
```

Each offset is `(x, y, z)` in metres relative to that drone's own arming
position. `hold_s` starts only after every drone is within the swarm position
tolerance. Every phase must contain exactly the configured drone IDs.

To add a mission:

1. Add a new key and phase list to `SWARM_MISSIONS`.
2. Keep every target inside each drone's external-command limits.
3. Keep all simultaneous targets at least `SWARM_MIN_SEPARATION_M` apart.
4. End at a stable, known formation; the coordinator lands automatically.
5. Rebuild and select the name with `cfswarmsim gui YOUR_MISSION`.
6. Run `cftest`, then the complete Gazebo mission, before hardware.

Swarm safety tuning remains in `config.py`:

```python
SWARM_POSITION_TOLERANCE_M = 0.07
SWARM_MIN_SEPARATION_M = 0.25
SWARM_TAKEOFF_TIMEOUT_S = 12.0
SWARM_MAX_FLIGHT_TIME_S = 90.0
```

The separation check uses absolute targets after adding each drone's home
position, so it also catches an unsafe physical starting layout.

## Change mission tuning without changing the sequence

Edit `src/cf_control/cf_control/config.py`.

Common parameters:

```python
FLIGHT_TEST_HEIGHT_M = 0.30
FLIGHT_TEST_HOVER_S = 3.0
FLIGHT_TEST_FIGURE_EIGHT_RADIUS_M = 0.35
FLIGHT_TEST_FIGURE_EIGHT_DURATION_S = 20.0
FLIGHT_TEST_POSITION_TOLERANCE_M = 0.08
MAX_FLIGHT_TIME_S = 35.0
MAX_VELOCITY = 0.3
POSITION_KP = 0.5
LANDING_SPEED_M_S = 0.10
POSITION_TIMEOUT_S = 0.5
```

Guidelines:

- Increase only one parameter at a time.
- Keep `MAX_FLIGHT_TIME_S` longer than the planned mission plus landing time.
- Larger `POSITION_KP` reacts more aggressively and can increase oscillation.
- `MAX_VELOCITY` limits each axis separately, not total 3D speed.
- Increasing figure-eight radius also increases the required clear area.
- Always rebuild and rerun Gazebo after changing constants.

```bash
cfenv
cfbuild
cftest
cfsim headless
```

## Change the autonomous mission sequence

The sequence is implemented in
`CrazyflieNode._update_test_sequence(position, now)` in
`src/cf_control/cf_control/crazyflie_node.py`.

### Rules for adding a phase

1. Give the phase a unique uppercase string.
2. Set its target with `self.flight_controller.set_target(x, y, z)`.
3. Use absolute world coordinates relative to `self.home_position`.
4. Use `self._near(position, target)` for position completion.
5. Use `self.test_phase_started_at` for timed completion.
6. Change phases with `self._set_test_phase('NEXT_PHASE', now)`.
7. Reset any phase-specific fields in `_stop_and_disarm()`.
8. Leave landing and telemetry-loss behavior in the shared safety methods.

### Example: insert a final hover after returning home

Keep the end of `FIGURE_EIGHT` returning to the home target:

```python
if progress >= 1.0:
    self.test_feedforward = (0.0, 0.0, 0.0)
    self.flight_controller.set_target(home_x, home_y, flight_z)
    self._set_test_phase('RETURN_HOME', now)
return
```

Change `RETURN_HOME` so reaching the target starts the timed hover:

```python
if self.test_phase == 'RETURN_HOME':
    target = (home_x, home_y, flight_z)
    if self._near(position, target):
        self._set_test_phase('FINAL_HOVER', now)
    return
```

Then add the timed phase:

```python
if self.test_phase == 'FINAL_HOVER':
    if now - self.test_phase_started_at >= 2.0:
        self._begin_landing('Final hover complete')
    return
```

The split phases ensure the two-second timer starts only after the vehicle has
returned home.

### Example: waypoint mission pattern

For a square or other waypoint list, define offsets in `config.py`:

```python
MISSION_WAYPOINT_OFFSETS_M = (
    (0.30, 0.00, 0.30),
    (0.30, 0.30, 0.30),
    (0.00, 0.30, 0.30),
    (0.00, 0.00, 0.30),
)
```

Add `self.mission_waypoint_index = 0` during node initialization and reset it
in `_stop_and_disarm()`. After takeoff/hover, enter `WAYPOINTS`. The phase can
follow this pattern:

```python
if self.test_phase == 'WAYPOINTS':
    offset = MISSION_WAYPOINT_OFFSETS_M[self.mission_waypoint_index]
    target = (
        home_x + offset[0],
        home_y + offset[1],
        floor_z + offset[2],
    )
    self.flight_controller.set_target(*target)

    if self._near(position, target):
        self.mission_waypoint_index += 1
        if self.mission_waypoint_index == len(MISSION_WAYPOINT_OFFSETS_M):
            self._begin_landing('Waypoint mission complete')
    return
```

Do not copy this directly into hardware use without tests for index reset,
empty waypoint lists, boundary limits, timeout behavior, and landing.

## Change the figure-eight trajectory

The figure-eight is generated inside the `FIGURE_EIGHT` branch using:

```text
x = home_x + radius * sin(angle)
y = home_y + radius * sin(angle) * cos(angle)
z = flight_z
```

The smoothstep expression makes angular speed zero at entry and exit. The
feedforward velocity follows the derivative of the generated curve.

When replacing the curve:

1. Change both the position formula and its feedforward derivative.
2. Or set `self.test_feedforward = (0.0, 0.0, 0.0)` and accept slower
   proportional-only tracking.
3. Keep the target inside the verified flight volume.
4. Ensure the path returns to a known point before landing.
5. Test at reduced height, radius, and velocity first.

## Change wand behavior

Wand constants are in `config.py`:

```python
WAND_TARGET_STEP_M = 0.05
WAND_MAX_HORIZONTAL_OFFSET_M = 0.50
WAND_TILT_TRIGGER_DEG = 20.0
WAND_TILT_REPEAT_S = 0.25
```

Gesture handling is in `wand_gesture_callback()`. Physical attitude handling
is in `wand_pose_callback()`, and target bounding is in
`_move_wand_target()`.

To add a discrete gesture:

1. Add its string to `VALID_WAND_GESTURES` in `config.py`.
2. Add keyboard behavior in `wand_node.py` if needed.
3. Add guarded receiver behavior in `wand_gesture_callback()`.
4. Decide which public flight states permit it.
5. Add a pure helper and unit tests in `test_wand_control.py` when math is
   involved.

Never allow a movement gesture to bypass `wand_active`, fresh-position checks,
the `FLYING` state, or the horizontal boundary.

## Change or add ROS parameters

Current parameters are declared near the top of `CrazyflieNode.__init__()`:

```text
drone_id
simulation
positioning_source
vicon_pose_topic
wand_control_enabled
external_targets_enabled
flight_mode
radio_uri
sim_model_name
flight_height_m
max_flight_time_s
```

When adding a parameter:

1. Declare it before reading it.
2. Validate values during initialization.
3. Add it to the appropriate launch file.
4. Document its default in `LAUNCH_TEST_REFERENCE.md`.
5. Test both the default and override path.

## Safety functions to preserve

Mission changes should continue to use these shared functions:

- `_fresh_position()` rejects stale telemetry.
- `_positioning_source_is_ready()` validates the selected position source.
- `_begin_landing()` sets a controlled target near the recorded floor.
- `_stop_and_disarm()` clears mission state and stops the motors.
- `control_callback()` enforces the flight timeout and telemetry-loss descent.
- `emergency_stop_callback()` immediately stops and disarms.

Avoid embedding new landing or emergency behavior inside individual waypoint
branches. Keeping those actions centralized makes every mission use the same
safety path.

## Required validation after a mission change

1. Run focused tests:

   ```bash
   cfenv
   cfbuild
   cftest
   ```

2. Run the full mission in headless Gazebo.
3. Confirm normal completion returns to `DISARMED`.
4. Repeat and call `cfland` during every new phase.
5. Repeat and call `cfstop` during every new phase.
6. Verify the maximum-flight-time landing.
7. Verify telemetry-loss behavior in simulation or a controlled propeller-off
   test.
8. Run the sequence on hardware with props removed.
9. Only then perform a guarded low-altitude propeller-on test.
