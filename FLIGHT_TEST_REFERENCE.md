# Crazyflie Flight-Test Reference

## Start the development environment

Open a new terminal and run:

```bash
cfenv
```

`cfenv` performs the following setup:

- Sources ROS 2 Jazzy.
- Activates `/home/tanner-looney/dev/crazyflie/venv`.
- Sources the built ROS workspace when available.
- Selects the virtual-environment `colcon` wrapper.
- Changes directory to `cfros_ws`.

If `cfenv` is unavailable in an existing terminal, reload the aliases:

```bash
source ~/.bash_aliases
cfenv
```

## Development commands

| Command | Purpose |
|---|---|
| `cfenv` | Initialize the complete Crazyflie environment. |
| `cfws` | Change directory to the ROS workspace. |
| `cfbuild` | Build `cf_control` and re-source the workspace. |
| `cftest` | Run the focused controller and interface tests. |
| `cfcheck` | Run the complete `cf_control` test suite. |
| `cfsource` | Re-source `install/setup.bash`. |
| `cfgazebo` | Launch Gazebo with its graphical interface. |
| `cfgazebo headless` | Launch the Gazebo server without OpenGL. |
| `cfsim` | Launch Gazebo, its ROS bridge, and the simulated controller. |
| `cfsim headless` | Launch the complete controller simulation without a GUI. |

Run the normal build and focused tests with:

```bash
cfenv
cfbuild
cftest
```

## Gazebo

Build the workspace and launch the simulation server with:

```bash
cfenv
cfbuild
cfgazebo headless
```

Launch the graphical interface with:

```bash
cfgazebo
```

Run the complete controller integration test with:

```bash
cfenv
cfbuild
cfsim
```

Use `cfsim headless` for automated or non-graphical testing. In another terminal,
use the same service commands as the hardware test:

```bash
cfenv
cfstate
cfarm
sleep 1
cftakeoff
```

The simulation automatically lands after ten seconds, or land early with
`cfland`. The Gazebo model is a kinematic 6-DOF integration model: it validates
ROS topics, control behavior, services, state transitions, and safety timing. It
does not model individual motors, propellers, battery behavior, or aerodynamic
disturbances.

If the GUI reports that it cannot create an OpenGL context, verify the graphics
driver before retrying:

```bash
glxinfo -B
nvidia-smi
```

Use `cfgazebo headless` while repairing the graphics driver. Stop either launch
mode with `Ctrl+C`.

## Drone commands

Commands default to `cf1`:

| Command | Purpose |
|---|---|
| `cfrun` | Start the Crazyflie ROS node. |
| `cfstate` | Continuously display the flight state. |
| `cfpos` | Continuously display estimated position. |
| `cfarm` | Arm after fresh position telemetry is available. |
| `cftakeoff` | Begin the constrained vertical takeoff. |
| `cfland` | Request a controlled landing. |
| `cfstop` | Immediately stop and disarm. This is not a landing. |

Pass a drone ID to operate another configured drone:

```bash
cfrun cf2
cfstate cf2
cfarm cf2
cftakeoff cf2
cfland cf2
cfstop cf2
```

## Current flight-test limits

- One drone only.
- External position targets are disabled.
- Takeoff is vertical from the position recorded during arming.
- Target height is 0.30 m above the recorded floor height.
- Landing descent is limited to 0.10 m/s.
- Automatic landing starts after 10 seconds.
- Position data becomes stale after 0.5 seconds.
- After telemetry loss, the node descends vertically for at most three seconds,
  then stops and disarms.

These values are defined in
`src/cf_control/cf_control/config.py`.

## Hardware-test procedure

Before installing propellers, run the entire service sequence once with the
propellers removed. For propeller-on testing, use a clear indoor area, propeller
guards, a fully charged battery, and no people inside the flight area.

### Terminal 1: node

```bash
cfenv
cfbuild
cfrun
```

Do not arm until the node reports:

```text
Lighthouse input and variance stability checks passed
```

### Terminal 2: monitoring and control

```bash
cfenv
cfstate
```

Use another monitoring terminal if desired:

```bash
cfenv
cfpos
```

Confirm that position is finite and stable while the drone is stationary.

### Terminal 3: emergency stop

Prepare this command before arming:

```bash
cfenv
cfstop
```

Type `cfstop` but wait to press Enter until an emergency occurs. Emergency stop
immediately stops and disarms the motors, so an airborne drone will fall.

### Execute the flight

With the drone level on the floor and the flight area clear:

```bash
cfarm
```

Expected state:

```text
READY
```

Wait at least one second, then run:

```bash
cftakeoff
```

Expected sequence:

```text
READY -> FLYING -> LANDING -> DISARMED
```

The node automatically initiates landing after ten seconds. To land earlier:

```bash
cfland
```

## Abort conditions

Use `cfstop` immediately if the Crazyflie:

- Tilts sharply during liftoff.
- Moves toward a person or obstacle.
- Climbs substantially above 0.30 m.
- Develops increasing oscillation.
- Does not respond to `cfland`.

Do not press `Ctrl+C` in the node terminal while the drone is airborne. Request
`cfland` first and confirm the `DISARMED` state.

## Direct ROS commands

The aliases expand to these ROS commands:

```bash
ros2 run cf_control crazyflie_node --ros-args -p drone_id:=cf1

ros2 service call /cf1/arm std_srvs/srv/Trigger '{}'
ros2 service call /cf1/takeoff std_srvs/srv/Trigger '{}'
ros2 service call /cf1/land std_srvs/srv/Trigger '{}'
ros2 service call /cf1/emergency_stop std_srvs/srv/Trigger '{}'

ros2 topic echo /cf1/flight_state
ros2 topic echo /cf1/position
```

## Troubleshooting

### `cfenv: command not found`

```bash
source ~/.bash_aliases
cfenv
```

### `No module named cflib`

Confirm that the environment uses the venv executables:

```bash
which python3
which colcon
```

Expected paths:

```text
/home/tanner-looney/dev/crazyflie/venv/bin/python3
/home/tanner-looney/dev/crazyflie/venv/bin/colcon
```

### Takeoff rejected after arming

Wait at least one second after `cfarm`, confirm fresh `/cf1/position` messages,
and retry `cftakeoff`.

### Lighthouse verification timeout

Do not arm. Confirm that both Lighthouse base stations are visible, the geometry
is configured on the Crazyflie, and the position estimate is stable.
