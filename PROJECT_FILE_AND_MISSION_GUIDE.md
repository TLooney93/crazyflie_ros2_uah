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
swarm_missions/<mission_name>.py
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

### `src/cf_control/cf_control/crazyflie_missions/`

One file per editable single-drone mission, plus `loader.py`, which registers
and validates them. Autonomous missions are lists of waypoint or figure-eight
phases. Add and tune sequences here instead of modifying the controller state
machine.

### `src/cf_control/cf_control/crazyflie_node.py`

The main ROS 2 controller and mission executor.

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

The node loads the selected sequence through `crazyflie_missions/loader.py`. In external
mode it takes off to `EXTERNAL_HOVER` and accepts bounded topic targets.
Autonomous mode cannot be overwritten by topic commands. The public service
state is separate from the internal mission phase.

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
- Recognizes three-part position gestures from tracked XYZ samples.
- Maps completed gestures to guarded flight actions.
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

### `src/cf_control/cf_control/wand_gesture_node.py`

Vicon position gesture recognizer. It subscribes to a configurable tracked
wand `PoseStamped` topic and publishes one event on `/wand/gesture` only after
a complete motion. The initial mappings are `up_down` for takeoff,
`side_to_side` for starting the selected single-drone mission, and `down_up`
for landing. Use `cfwandgesture [vicon_pose_topic]` to run it.

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

Swarm membership is supplied by the launch layer. Do not add vehicle names to
the default `drone_ids` parameter in this file.

### `src/cf_control/cf_control/swarm_profiles/`

Named vehicle rosters for routine launches. Each profile file defines the
ordered drone IDs, Vicon topics, radio URIs, default mission, and matching
Gazebo world. `loader.py` registers and validates the files. Add or remove
physical swarm members in that profile's file.

The bundled profiles are `two_drone_lab` and `five_drone_lab`. The five-drone
profile can run in simulation immediately, but its `cf3` through `cf5` radio
URIs are intentionally blank until the actual addresses are scanned.

### `src/cf_control/cf_control/swarm_missions/`

One file per swarm choreography, plus `loader.py`, which registers and
validates them. Each mission is a list of phases with a name, a hold time, and
one XYZ offset per drone. Offsets are relative to each vehicle's independently
recorded home position.

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
profile_name: optional registered swarm profile name
drone_ids: comma-separated names
positioning_source: vicon | lighthouse
vicon_topics: comma-separated PoseStamped topics
radio_uris: optional comma-separated URI overrides
mission_name: registered mission name
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

Loads a named swarm profile, selects its Gazebo world, generates one bridge
and controller per vehicle, and starts the real swarm coordinator. It accepts
autonomous mission, per-drone terminal target, and common wand modes. Use it
through `cfswarmsim` before any hardware swarm test.

### `src/cf_gazebo/worlds/crazyflie_world.sdf`

Defines the Gazebo world and includes the Crazyflie model.

Change this file for environmental objects, lighting, gravity, or model spawn
placement. Do not define the flight mission here.

### `src/cf_gazebo/worlds/crazyflie_swarm_world.sdf`

Two-model integration-test world. The models are named `cf1` and `cf2` so
each controller can use its own velocity and odometry topics.

### `src/cf_gazebo/worlds/crazyflie_five_swarm_world.sdf`

Five-model integration-test world for `five_drone_lab`. The vehicles start on
a square with `cf5` in the center, with more than the configured minimum
separation between every pair.

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
WAND_TAKEOFF -> WAND_HOVER
```

`WAND_HOVER` holds the last target and accepts bounded direction or tilt
commands. A recognized `side_to_side` event may start the selected
single-drone mission from this stable hover. The mission then uses its normal
phase executor and lands when complete. A `down_up` event requests landing at
any point. Swarm mission gestures remain disabled during the initial test.

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

Create or edit a mission in
`src/cf_control/cf_control/swarm_missions/`. Do not add formation waypoints to
`swarm_node.py`.

The bundled pattern is:

```python
MISSION_NAME = 'two_drone_demo'

MISSION = [
    {
        'name': 'SPREAD',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (-0.15, 0.0, 0.35),
            'cf2': (0.15, 0.0, 0.35),
        },
    },
    # Add more phases here.
]
```

Each offset is `(x, y, z)` in metres relative to that drone's own arming
position. `hold_s` starts only after every drone is within the swarm position
tolerance. Every phase must contain exactly the configured drone IDs.

The mission and profile must agree exactly. For example,
`five_drone_lab` selects `five_drone_demo`, whose every phase contains `cf1`
through `cf5`.

To add a mission:

1. Copy an existing mission file and give it a unique `MISSION_NAME`.
2. Import its `MISSION_NAME` and `MISSION` in `swarm_missions/loader.py`, then
   add the pair to `SWARM_MISSIONS`.
3. Keep every target inside each drone's external-command limits.
4. Keep all simultaneous targets at least `SWARM_MIN_SEPARATION_M` apart.
5. End at a stable, known formation; the coordinator lands automatically.
6. Rebuild and select it with
   `cfswarmsim gui PROFILE_NAME YOUR_MISSION`.
7. Run `cftest`, then the complete Gazebo mission, before hardware.

Swarm safety tuning remains in `config.py`:

```python
DEFAULT_SWARM_PROFILE = 'two_drone_lab'
SWARM_POSITION_TOLERANCE_M = 0.07
SWARM_MIN_SEPARATION_M = 0.25
SWARM_TAKEOFF_TIMEOUT_S = 12.0
SWARM_MAX_FLIGHT_TIME_S = 90.0
```

The separation check uses absolute targets after adding each drone's home
position, so it also catches an unsafe physical starting layout.

## Add or change swarm membership

Create or edit a file in `src/cf_control/cf_control/swarm_profiles/`, not
`swarm_node.py`.

For every profile:

1. Copy an existing profile file and give it a unique `PROFILE_NAME`.
2. List each drone once in `drone_ids`.
3. Provide exactly one absolute Vicon topic per ID.
4. Provide exactly one separately addressable radio URI per hardware drone.
5. Select a mission whose phase dictionaries contain exactly those IDs.
6. Select a Gazebo world containing models with those same names.
7. Import its `PROFILE_NAME` and `PROFILE` in `swarm_profiles/loader.py`, then
   add the pair to `SWARM_PROFILES`.

Test the bundled five-drone profile with:

```bash
cfbuild
cftest
cfswarmsim headless five_drone_lab
```

For hardware, first fill the blank `cf3`, `cf4`, and `cf5` radio URIs and
verify every Vicon subject name. The launcher deliberately refuses to start
if any profile radio URI remains blank.

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

Create or edit a file in
`src/cf_control/cf_control/crazyflie_missions/`. The built-in names are
`figure_eight`, `hover_test`, and `square_demo`.

Create a waypoint mission file like this:

```python
MISSION_NAME = 'my_mission'

MISSION = [
    {
        'name': 'FORWARD',
        'kind': 'waypoint',
        'offset': (0.25, 0.0, 0.30),
        'hold_s': 2.0,
    },
    {
        'name': 'RETURN_HOME',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, 0.30),
        'hold_s': 1.0,
    },
]
```

Then import its `MISSION_NAME` and `MISSION` in
`crazyflie_missions/loader.py` and add the pair to `CRAZYFLIE_MISSIONS`.

Offsets are `(x, y, z)` metres from the position recorded by `cfarm`.
`hold_s` begins only after the drone reaches the waypoint and resets if it
drifts outside the position tolerance. The last phase must finish above
`(0, 0)` so the shared landing sequence begins from the launch point.

The loader rejects duplicate/reserved phase names, non-finite values, targets
outside the guarded flight volume, invalid durations, and missions that do
not return home. The node automatically performs vertical takeoff to the
first phase altitude and lands after the last phase.

Select and test the mission with:

```bash
cfbuild
cftest
cfsim headless autonomous my_mission
```

For hardware, the optional fifth `cfsingle` argument selects the mission:

```bash
cfsingle cf1 vicon /vicon/cf231/cf231 "" my_mission
```

The fourth argument remains the optional radio URI override. Always run the
complete mission in Gazebo before selecting it for hardware.

## Change the figure-eight trajectory

The figure-eight executor in `crazyflie_node.py` uses:

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
