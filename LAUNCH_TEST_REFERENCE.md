# Crazyflie Launch and Test Command Reference

This document is the command-first reference for building, simulating, and
running the Crazyflie project. For the detailed Vicon flight checklist, use
`FLIGHT_TEST_REFERENCE.md`. For process ownership and data flow, use
`ARCHITECTURE.md`.

## Safety rules

- Run every changed flight sequence in Gazebo first.
- Repeat the sequence with the receiver drone's propellers removed before a
  propeller-on test.
- Remove all propellers from `cf2` when it is used as the handheld wand.
- Keep `cfstop cf1` ready in a dedicated terminal during hardware tests.
- Keep `cfswarmstop` ready during every multi-drone hardware test.
- Use `cfland cf1` for a normal descent. `cfstop cf1` stops the motors and an
  airborne drone will fall.
- Do not stop the flight node with `Ctrl+C` while the drone is airborne.

## Open the project environment

In every new Crazyflie terminal, run:

```bash
cfenv
```

If the command is not available:

```bash
source ~/.bash_aliases
cfenv
```

`cfenv` performs the following setup:

- Sources ROS 2 Jazzy.
- Activates `/home/tanner-looney/dev/crazyflie/venv`.
- Sources the built ROS workspace when present.
- Changes to `/home/tanner-looney/dev/crazyflie/cfros_ws`.
- Loads the project helper functions listed below.

## Command and alias inventory

### Global shell helpers

These are defined in `~/.bash_aliases` and are available before `cfenv`.

| Command | Arguments | Purpose |
|---|---|---|
| `cfenv` | none | Load the complete project environment and change to the ROS workspace. |
| `cfvicon` | `[tracker_ip] [namespace]` | Start the Vicon ROS receiver. Defaults to `10.86.153.14` and namespace `vicon`. |
| `cfkill` | `[drone_id]` | Request emergency stop, terminate the local flight/Vicon processes, and stop the ROS daemon. Use only after landing unless an emergency requires it. |

For the wired Vicon network used by the current runbook, override the older
Wi-Fi default explicitly:

```bash
cfvicon 192.168.10.1 vicon
```

### Project helpers loaded by `cfenv`

| Command | Arguments | Purpose |
|---|---|---|
| `cfws` | none | Change to the ROS workspace. |
| `cfbuild` | optional `colcon build` arguments | Build `cf_control` and `cf_gazebo`, then source the result. |
| `cftest` | optional `colcon test` arguments | Run the focused controller, mission, interface, wand, and swarm tests. |
| `cfchecktests` | optional `colcon test` arguments | Run the complete `cf_control` suite, including legacy lint checks. |
| `cfsource` | none | Re-source `install/setup.bash` after an external build. |
| `cfgazebo` | `[gui\|headless]` | Start only the Gazebo world. It does not start the bridge or controller. |
| `cfsim` | `[gui\|headless] [autonomous\|terminal\|wand] [mission]` | Start the complete single-drone Gazebo stack in the selected mode. |
| `cfswarmsim` | `[gui\|headless] [profile] [mission] [autonomous\|terminal\|wand]` | Start the profile-selected Gazebo swarm. The mode may be omitted, so `cfswarmsim five_drone_lab` is valid. |
| `cfkillgazebo` | none | Stop Gazebo Sim server and GUI processes owned by the current user, including stale instances left by an interrupted launch. |
| `cfrun` | `[drone_id] [vicon\|lighthouse] [vicon_topic] [wand_bool] [mission]` | Start the hardware Crazyflie node. Defaults to `cf1`, Vicon, wand off, and `figure_eight`. |
| `cfsingle` | `[drone_id] [source] [vicon_topic] [radio_uri] [mission]` | Launch one hardware drone with a named autonomous mission. |
| `cfterminal` | `[drone_id] [source] [vicon_topic] [radio_uri]` | Launch one hardware drone for bounded terminal position targets. |
| `cfwandflight` | `[drone_id] [source] [vicon_topic] [radio_uri] [mission]` | Launch one hardware drone with guarded wand input and a selectable gesture-started mission. |
| `cfbuttonwandflight` | `[drone_id] [source] [vicon_topic] [radio_uri] [mission] [wand_id] [wand_uri]` | Launch one drone and a physical button wand in the same process so they share one Crazyradio. |
| `cfswarm` | `[profile] [input] [source] [mission] [radio_uris] [vicon_topics]` | Launch a named hardware swarm profile. A comma-separated ID list as the first argument retains the explicit legacy form. |
| `cfswarmwand` | `[profile] [source] [mission] [wand_id] [wand_uri] [radio_uris] [vicon_topics]` | Launch a hardware swarm and physical wand in one shared-radio process. |
| `cfarm` | `[drone_id]` | Arm after valid position checks. Defaults to `cf1`. |
| `cftakeoff` | `[drone_id]` | Start the autonomous mission selected when the node was launched. |
| `cfland` | `[drone_id]` | Request a controlled landing or disarm before takeoff. |
| `cfstop` | `[drone_id]` | Immediately stop setpoints and disarm. This is not a controlled landing. |
| `cfstate` | `[drone_id]` | Continuously display the public flight state. |
| `cfpos` | `[drone_id]` | Continuously display the fused position. |
| `cftarget` | `drone_id x y z` | Publish one absolute, bounded world-coordinate target in terminal mode. |
| `cfw` | none | Start the keyboard wand command prompt without publishing a fake pose. |
| `cfwand` | none | Backward-compatible name for `cfw`. |
| `cfwandsim` | none | Start the keyboard wand and also publish its simulated pose. |
| `cfwandgesture` | `[vicon_pose_topic]` | Recognize up/down, side-to-side, and down/up mission gestures from a Vicon wand. Defaults to `/vicon/cfw/cfw`. |
| `cfwanddriver` | `[wand_id] [wand_uri]` | Read pose and attitude from the physical wand. Defaults to `cf2`. |
| `cfswarmstart` | none | Run swarm arm, takeoff, configured mission, and landing. |
| `cfswarmarm` | none | Arm all swarm members. |
| `cfswarmtakeoff` | none | Take off all armed swarm members and hold. |
| `cfswarmmission` | none | Start the configured sequence from swarm hover. |
| `cfswarmland` | none | Request controlled landing from all active swarm members. |
| `cfswarmstop` | none | Request immediate stop/disarm from every swarm member. |
| `cfswarmstate` | none | Continuously display the coordinator state. |

`cfchecktests` currently includes older example files that have existing style and
incomplete-code warnings. Use `cftest` as the focused regression test for the
active flight stack, and review `cfchecktests` output separately.

## Three-drone battery and link qualification

Before arming a three-drone hardware test, install charged packs and start the
stack without issuing any flight command:

```bash
cfenv
cfswarm three_drone_demo terminal vicon
```

In separate terminals, verify each battery topic for at least 30 seconds:

```bash
ros2 topic echo /cf1/battery_voltage
ros2 topic echo /cf2/battery_voltage
ros2 topic echo /cf3/battery_voltage
```

The hardware arming gate requires fresh battery telemetry at or above 3.85 V
for every drone. This intentionally conservative threshold is for beginning a
test with charged packs; it is not the firmware's critical-low cutoff. Also
confirm that the launch terminal continues to report estimator telemetry near
5 Hz and Vicon forwarding near 5 Hz for every drone. If one voltage topic or
all diagnostics stop while the process remains alive, treat that as a link or
driver failure and do not arm.

Resting voltage identifies a low pack but does not prove battery health. A pack
that reads normally can still sag under motor load. Keep the first powered test
to a staged takeoff and hover over the net, and compare the three live voltage
topics for a sudden drop or reboot before running a saved mission.

The `three_drone_demo` profile takes off directly to its 0.45 m mission
height. The shared dispatcher takes a snapshot of every drone's latest
velocity every 20 ms (50 Hz), sends all three addressed command packets, and
then sends pending external-position updates. Stock Crazyflie firmware still
uses one over-air packet per drone; the dispatcher makes those packets one
ordered group frame. A velocity that is not renewed for 100 ms expires rather
than being replayed indefinitely. Arm, land, and emergency-stop commands
remain direct.

The coordinator does not enter `HOVER` or begin the mission until every
controller publishes `EXTERNAL_HOVER` on its control-phase topic. During hover
and mission flight, a drone that remains more than 0.25 m below the takeoff
altitude for 0.40 seconds causes a coordinated safety landing.

For the next diagnostic run, include control phases, commanded velocities,
raw firmware thrust output, and battery voltage:

```bash
ros2 bag record \
  /vicon/cf1/cf1 /vicon/cf2/cf2 /vicon/cf3/cf3 \
  /cf1/position /cf2/position /cf3/position \
  /cf1/battery_voltage /cf2/battery_voltage /cf3/battery_voltage \
  /cf1/stabilizer_thrust /cf2/stabilizer_thrust \
  /cf3/stabilizer_thrust \
  /cf1/control_phase /cf2/control_phase /cf3/control_phase \
  /cf1/debug/velocity /cf2/debug/velocity /cf3/debug/velocity \
  /cf1/flight_state /cf2/flight_state /cf3/flight_state /swarm/state
```

The same per-drone battery gate is used by terminal, autonomous, and wand
launches. For the later three-drone keyboard-wand test:

```bash
cfswarm three_drone_demo wand vicon
cfswarmarm
cfswarmstate
cfw
```

Wait for `WAND_READY`, then enter `enable` and `takeoff` at the `wand>` prompt.
Do not use `cf1`, `cf2`, or `cf3` as the physical wand ID; a button wand must
have its own radio address and Vicon rigid body.

## Mode selection quick reference

The launch mode decides who is allowed to update the position target. Modes
are intentionally exclusive; do not switch sources while airborne.

| Vehicle set | Command source | Simulation | Hardware launch |
|---|---|---|---|
| Single | Named saved mission | `cfsim gui autonomous MISSION` | `cfsingle cf1 vicon TOPIC RADIO_URI MISSION` |
| Single | Terminal target | `cfsim gui terminal` | `cfterminal cf1 vicon TOPIC` |
| Single | Keyboard/physical wand | `cfsim gui wand` | `cfwandflight cf1 vicon TOPIC` |
| Swarm | Saved formation mission | `cfswarmsim gui PROFILE` | `cfswarm PROFILE autonomous vicon` |
| Swarm | Per-drone terminal targets | `cfswarmsim gui PROFILE "" terminal` | `cfswarm PROFILE terminal vicon` |
| Swarm | Common wand gestures | `cfswarmsim gui PROFILE "" wand` | `cfswarm PROFILE wand vicon` |

`TOPIC` means the actual Vicon `PoseStamped` topic for that rigid body.
`TOPICS` and `RADIO_URIS` are comma-separated lists in the same order as
`IDS`. Each drone needs a unique radio address, but the hardware swarm runner
keeps all controller links in one process so CFLib can multiplex them through
one Crazyradio. Multiple URIs may therefore begin with `radio://0`.

The physical `cf2` cannot be a handheld wand and a flying swarm member at the
same time. For a wand-controlled `cf1,cf2` swarm, use the keyboard `cfw`
source or a third propeller-free wand.

## Test 1: Build and focused automated tests

Use this after any controller, interface, mission, wand, or launch-file change:

```bash
cfenv
cfbuild
cftest
colcon test-result --test-result-base build/cf_control --verbose
```

Expected focused result:

```text
18 passed
```

Run the complete package suite when cleaning up project-wide style or legacy
tools:

```bash
cfchecktests
```

## Test 2: Gazebo world only

This checks whether Gazebo and the Crazyflie model load. It does not start a
flight controller and cannot respond to `cfarm`.

Graphical:

```bash
cfenv
cfgazebo
```

Headless:

```bash
cfenv
cfgazebo headless
```

Stop the launch with `Ctrl+C`.

## Test 3: Full autonomous Gazebo mission

### Terminal 1 - complete simulator

```bash
cfenv
cfbuild
cfsim headless
```

Use `cfsim` without `headless` for the graphical window.

The default is `figure_eight`. To run another mission registered in
`crazyflie_missions/loader.py`, append its name, for example:

```bash
cfsim headless autonomous square_demo
```

### Terminal 2 - state monitor

```bash
cfenv
cfstate cf1
```

### Terminal 3 - control

```bash
cfenv
cfarm cf1
```

Wait at least one second after a successful arm response:

```bash
cftakeoff cf1
```

Expected mission phases in Terminal 1:

```text
TAKEOFF -> HOVER -> FIGURE_EIGHT -> RETURN_HOME -> LANDING
```

Expected public states in Terminal 2:

```text
DISARMED -> READY -> FLYING -> LANDING -> DISARMED
```

Land early if required:

```bash
cfland cf1
```

## Test 4: Terminal position control in Gazebo

### Terminal 1 - external-command simulator

```bash
cfenv
cfbuild
cfsim headless terminal
```

### Terminal 2 - arm, take off, command, land

```bash
cfenv
cfarm cf1
```

Wait at least one second, then:

```bash
cftakeoff cf1
```

Wait until Terminal 1 reports `EXTERNAL_HOVER`. Read the current absolute
position with `cfpos cf1`, then send a target inside 0.75 m horizontally and
between 0.10 and 0.75 m above the recorded floor. Example for the standard
Gazebo origin:

```bash
cftarget cf1 0.20 0.10 0.35
```

Finish with:

```bash
cfland cf1
```

Targets sent before `EXTERNAL_HOVER`, during autonomous mode, outside the
configured volume, or with non-finite coordinates are rejected.

## Test 5: Keyboard wand control in Gazebo

Select wand mode when starting the simulator.

### Terminal 1 - simulator with wand control

```bash
cfenv
cfbuild
cfsim headless wand
```

Use `cfsim gui wand` for the GUI.

### Terminal 2 - monitoring

```bash
cfenv
cfstate cf1
```

Optional position monitor:

```bash
cfenv
cfpos cf1
```

### Terminal 3 - arm

```bash
cfenv
cfarm cf1
```

Wait at least one second.

### Terminal 4 - keyboard wand

```bash
cfenv
cfw
```

Enter these commands one at a time at the `wand>` prompt:

```text
enable
takeoff
move_forward
move_left
hold
move_backward
move_right
land
```

Wait until Terminal 1 reports `WAND_HOVER` before sending movement commands.
Each movement advances the target by 0.05 m. Targets are limited to 0.50 m
from the position recorded at arming.

`cfwandsim` may be used instead of `cfw` when a simulated `/wand/pose`
topic is also needed:

```bash
cfwandsim
```

Do not run `cfwandsim` alongside the physical wand driver because both would
publish `/wand/pose`. Use `cfw` with the physical wand.

## Test 6: Wand commands without the interactive prompt

These commands are useful for repeatable Gazebo checks. The simulator must
already be running with `cfsim gui wand` or `cfsim headless wand`.

```bash
ros2 topic pub --once /wand/gesture std_msgs/msg/String "{data: enable}"
ros2 topic pub --once /wand/gesture std_msgs/msg/String "{data: takeoff}"
ros2 topic pub --once /wand/gesture std_msgs/msg/String "{data: move_forward}"
ros2 topic pub --once /wand/gesture std_msgs/msg/String "{data: hold}"
ros2 topic pub --once /wand/gesture std_msgs/msg/String "{data: land}"
```

The allowed gesture strings are:

```text
enable
disable
takeoff
land
move_left
move_right
move_forward
move_backward
hold
up_down
side_to_side
down_up
```

The three tracked-motion events map to guarded actions:

```text
up_down      -> takeoff
side_to_side -> start the selected single-drone mission from WAND_HOVER
down_up      -> controlled landing
```

## Test 7: Profile-selected swarm mission in Gazebo

### Terminal 1 - swarm simulator

```bash
cfenv
cfbuild
cfswarmsim headless two_drone_lab
```

### Terminal 2 - coordinator state

```bash
cfenv
cfswarmstate
```

### Terminal 3 - start the complete sequence

```bash
cfenv
cfswarmstart
```

Expected coordinator sequence:

```text
IDLE -> ARMING -> ARM_SETTLE -> TAKEOFF -> HOVER -> MISSION
     -> LANDING -> IDLE
```

The `two_drone_lab` profile selects the same-named separated-orbit mission. It
logs `INITIAL_HOVER`, `ORBIT_ENTRY`, `CONTINUOUS_ORBIT`, and `FINAL_HOVER`
before landing. The orbit is a 30-second continuous trajectory with bounded
tangential velocity feedforward, not a sequence of stop-and-go waypoints.
Both drones stay at the same commanded height and 0.60 m apart through the
shared orbit; no vertical stack is attempted. At any time, request a normal
multi-drone landing or an emergency stop with:

```bash
cfswarmland
cfswarmstop
```

### Swarm wand variant

Start the simulator with common wand input:

```bash
cfswarmsim headless two_drone_lab "" wand
```

In a control terminal:

```bash
cfswarmarm
cfw
```

Wait for `WAND_READY`, then enter `enable`, `takeoff`, movement commands, and
`land` in the wand prompt. The expected coordinator states are
`IDLE -> ARMING -> WAND_READY -> WAND_ACTIVE -> LANDING -> IDLE`.

## Test 8: Vicon receiver and network only

This test does not connect to the Crazyflie.

### Terminal 1 - Vicon receiver

```bash
source ~/.bash_aliases
cfvicon 192.168.10.1 vicon
```

Expected receiver output includes:

```text
Connection successfully established with 192.168.10.1
Creating publisher for segment cf231 from subject cf231
```

### Terminal 2 - topic checks

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicon_receiver_ws/install/setup.bash
ros2 topic list | grep vicon
ros2 topic type /vicon/cf1/cf1
ros2 topic hz /vicon/cf1/cf1
```

Expected type:

```text
geometry_msgs/msg/PoseStamped
```

Inspect coordinates:

```bash
ros2 topic echo /vicon/cf1/cf1
```

Move the unpowered receiver drone by hand and verify smooth X, Y, and Z motion
with the expected signs.

## Test 9: Hardware autonomous mission with Vicon

Complete the full safety and network checklist in `FLIGHT_TEST_REFERENCE.md`
before this test.

### Terminal 1 - Vicon

```bash
cfvicon 192.168.10.1 vicon
```

### Terminal 2 - receiver node

```bash
cfenv
cfbuild
cfsingle cf1 vicon /vicon/cf1/cf1
```

Do not arm until the node reports:

```text
Kalman estimator selected
Vicon stream detected; resetting estimator
Vicon positioning is ready for arming
```

### Terminal 3 - fused position

```bash
cfenv
cfpos cf1
```

### Terminal 4 - state

```bash
cfenv
cfstate cf1
```

### Terminal 5 - control

```bash
cfenv
cfarm cf1
```

Wait at least one second, then:

```bash
cftakeoff cf1
```

Use the appropriate command: normal landing or emergency motor stop:

```bash
cfland cf1
cfstop cf1
```

## Test 10: Hardware connection with Lighthouse

This preserves the Lighthouse path without using Vicon. Confirm the base
stations, Lighthouse deck, geometry, and calibration before arming.

### Terminal 1 - receiver node

```bash
cfenv
cfbuild
cfsingle cf1 lighthouse
```

Required startup message:

```text
Lighthouse input and variance stability checks passed
```

### Terminals 2 and 3 - monitor and control

```bash
cfenv
cfstate cf1
```

```bash
cfenv
cfpos cf1
```

Only after the position is finite and stable:

```bash
cfarm cf1
```

Wait at least one second, then:

```bash
cftakeoff cf1
```

## Test 11: Vicon wand gestures with Vicon-controlled cf1

Remove all props from the handheld `cf2`. The combined button-wand launch
keeps the flying drone and wand connections in one process so both addresses
can share `radio://0`.

Wire two normally-open momentary buttons between the wand deck pins and GND:

```text
enable button: DECK_GPIO_IO1 --- momentary switch --- GND
E-stop button: DECK_GPIO_IO2 --- momentary switch --- GND
```

The firmware uses `INPUT_PULLUP`; do not wire either input to a supply rail.
`wand_buttons.c` must be built into the wand firmware so the `wand.enable`,
`wand.disable`, and `wand.stop` log variables exist.

### Terminal 1 - Vicon

```bash
cfvicon 192.168.10.1 vicon
```

### Terminal 2 - cf1 with wand subscriptions enabled

```bash
cfenv
cfbuttonwandflight cf1 vicon /vicon/cf1/cf1 "" hover_test cf2
```

### Terminal 3 - Vicon gesture recognizer

Use the actual Vicon subject and segment topic for the tracked wand:

```bash
cfenv
cfwandgesture /vicon/cfw/cfw
```

Verify recognition without props first:

```bash
ros2 topic echo /wand/gesture
```

Expected events are `up_down`, `side_to_side`, and `down_up`.

Recognition is disabled by default. Hold IO1 while performing a gesture;
releasing IO1 disables recognition but preserves the active takeoff, hover
target, or saved mission. Use an explicit `hold` gesture to capture the
current position. IO2 publishes `emergency_stop` and bypasses the enable gate.

### Physical wand verification

`cfbuttonwandflight` already starts the physical wand driver. Do not start a
second `cfwanddriver` process for this test.

### Terminal 4 - optional tilt verification

```bash
cfenv
ros2 topic hz /wand/pose
ros2 topic echo /wand/connected
ros2 topic echo /wand/enabled
ros2 topic echo /wand/gesture
```

### Terminal 5 - arm and emergency stop

```bash
cfenv
cfarm cf1
```

Wait at least one second. Keep this terminal available for:

```bash
cfstop cf1
```

### Terminal 6 - guarded command prompt

```bash
cfenv
cfw
```

Enter this once to enable guarded wand actions:

```text
enable
```

After arming, move the tracked wand up at least 0.15 m and return it near its
starting height within two seconds. This publishes `up_down` and requests
takeoff. Wait for `WAND_HOVER`.

Move the wand at least 0.15 m to one horizontal side and then at least 0.15 m
past its starting position to the opposite side. This publishes
`side_to_side` and starts the selected single-drone mission.

Move the wand down at least 0.15 m and return it near its starting height to
publish `down_up` and request controlled landing.

When the optional attitude driver is running, tilting more than 20 degrees
also advances the target by
0.05 m every 0.25 seconds. Return the wand to neutral to stop advancing the
target.

### Five-drone simulation variant

Run the complete five-drone profile before configuring hardware:

```bash
cfswarmsim headless five_drone_lab
```

The expected phases are `HOVER`, `FORWARD`, and `RETURN_HOME`.

### Three-drone conservative hover test

Build and run the dedicated three-drone world:

```bash
cfenv
cfbuild
cfswarmsim headless three_drone_demo
```

In another terminal, start the complete sequence:

```bash
cfenv
cfswarmstart
```

The current hardware-baseline mission contains one `HOVER` phase at 0.30 m,
held for 5 seconds before coordinated landing. Expand this mission only after
all three vehicles complete the hover test cleanly.
Hardware use reads the three unique radio
URIs for `cf1`, `cf2`, and `cf3` from `cf_control/config.py`.

## Test 12: Hardware swarm with Vicon

Do not run this until Test 7 completes normally in Gazebo. One Crazyradio can
serve the swarm because all hardware controller links run inside one shared
process. Each Crazyflie must still have a unique address. Replace the Vicon
topic examples below with the three topics verified by `ros2 topic list`.

### Terminal 1 - Vicon receiver

```bash
cfvicon 192.168.10.1 vicon
```

### Terminal 2 - hardware swarm

```bash
cfenv
cfbuild
cfswarm two_drone_lab autonomous vicon
```

During takeoff, each controller adds a `0.50 m/s` vertical boost below
`0.25 m` above its recorded floor position. The combined controller and boost
command may use a `0.50 m/s` cap during the first three seconds, then returns
to the normal `0.40 m/s` takeoff cap. During this three-second heavy-airframe
assist window, stalled-climb and sustained-thrust timers do not accumulate;
the normal four-second progress window and two-second thrust window begin
after assistance ends. Horizontal correction is limited to
`0.01 m/s` in the first 0.10 m and `0.04 m/s` above it; leaving a `0.30 m`
launch column still initiates landing even during assistance. Position-source
freshness, emergency stop, and maximum flight time also remain active. After
the assist window, a vehicle must gain 0.05 m within four seconds, and a raw
thrust demand at or above 60000 for two seconds without that progress initiates
landing. The swarm allows 20 seconds for all drones to
reach within `0.40 m` of takeoff height. A timeout reports each
drone's measured height and state. During a mission, each drone's waypoint
arrival is latched independently so natural motion does not require every
vehicle to occupy its tolerance sphere on the same control sample.

The receiver may publish roughly 100 Vicon poses per second per rigid body.
The controller uses Vicon directly as the authoritative ROS control and swarm
safety position, while retaining onboard `stateEstimate` telemetry for the
preflight agreement check. By default, only 5 Hz is forwarded to each onboard
estimator, and onboard telemetry is requested at 5 Hz. This preserves
shared-Crazyradio bandwidth for commands and return telemetry. Expect periodic
messages similar to:

```text
Vicon rates: input 100.0 Hz, forwarded 4.5-5.0 Hz; estimator telemetry 5.0 Hz, age 0.10 s; cflib handoff velocity 50.0 Hz, extpos 5.0 Hz
```

Wait for every Vicon-ready message. Verify each position stream separately:

```bash
cfpos cf1
cfpos cf2
cfpos cf3
```

All position streams must remain continuous before flight. To test a
different forwarding limit without editing the source configuration, launch
directly and add `vicon_forward_rate_hz:=8.0`:

```bash
ros2 launch cf_control control.launch.py \
  mode:=swarm profile_name:=two_drone_lab input:=autonomous \
  positioning_source:=vicon vicon_forward_rate_hz:=8.0
```

Also verify that the shared runner and both logical controller nodes exist:

```bash
ros2 node list | grep -E 'hardware_swarm_runner|cf[12]_control'
```

Keep a dedicated terminal ready with `cfswarmstop`. Start the full saved
sequence only after both vehicles have a clear volume:

```bash
cfswarmstart
```

For staged operation instead of the full sequence:

```bash
cfswarmarm
cfswarmtakeoff
cfswarmmission
cfswarmland
```

For terminal-controlled swarm flight, use
`cfswarm two_drone_lab terminal vicon`, then use the staged arm/takeoff calls,
wait for `HOVER`, and send one
bounded absolute target per drone with `cftarget`.

For common wand control, use `cfswarm two_drone_lab wand vicon`. Arm with
`cfswarmarm`, wait for `WAND_READY`, then run `cfw` and enter `enable` and
`takeoff`. Both drones receive the same bounded relative movement gestures,
while the coordinator monitors position freshness, group separation, and
whether all members entered flight. Use `land` in `cfw` or `cfswarmland` to
finish.

For a physical-wand three-drone mission, configure unique radio URIs for
`cf1`, `cf2`, `cf3`, and `cfw`, then launch:

```bash
cfenv
cfswarmwand three_drone_demo vicon "" cfw
```

Start `cfwandgesture /vicon/cfw/cfw` separately and arm with `cfswarmarm`.
Hold the enable button only while making each gesture. `up_down` commands the
common takeoff. Wait until every controller reports `WAND_HOVER` and the
coordinator reports `WAND_ACTIVE`; `side_to_side` then starts the selected
`three_drone_demo` sequence. `down_up` requests a coordinated landing at any
time. A mission gesture received before the complete hover handshake is
ignored.

## Test 13: Crazyradio scan

Power the intended Crazyflie and connect the Crazyradio, then run:

```bash
cfenv
python3 src/cf_control/tools/scanner.py
```

The candidate receiver addresses currently used by the three-drone setup are:

```text
radio://0/80/2M/E7E7E7E7E7
radio://0/80/2M/E7E7E7E7E6
radio://0/80/2M/E7E7E7E7E5
```

Do not run the scanner while another process owns the Crazyradio.

### Radio-to-airframe-to-Vicon identity check

Stop every flight, wand, and controller process before using the identifier;
only one process may own the Crazyradio. `cfidentify` never arms the vehicle
or sends a motor command. It blinks the onboard LEDs for the requested
configured name or URI:

```bash
cfenv
cfidentify cf1
cfidentify cf2
cfidentify cf3
```

For each blinking airframe, move it by hand and confirm exactly one matching
Vicon topic moves. Record the observed mapping before editing `DRONES`:

```text
radio URI                physical airframe    Vicon subject
radio://0/80/2M/...E7    ________________     ______________
radio://0/80/2M/...E6    ________________     ______________
radio://0/80/2M/...E5    ________________     ______________
```

If the LEDs identified by `cfidentify cf3` are not attached to the rigid body
published on `/vicon/cf3/cf3`, correct `DRONES` before another powered test.
During a later single-drone net test, compare `/cfN/stabilizer_thrust`,
`/cfN/battery_voltage`, and height. Rising thrust with no height change points
to an airframe, propeller, motor, payload, or battery-current problem. Little
or no thrust despite a positive vertical velocity command points to the
control or firmware path instead.

## Useful ROS inspection commands

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 topic info /cf1/position
ros2 topic info /wand/pose
ros2 topic echo /swarm/state
ros2 topic echo /cf1/debug/velocity
ros2 topic echo /cf1/stabilizer_thrust
ros2 topic echo /cf1/flight_state
ros2 topic echo /wand/enabled
ros2 topic echo /wand/connected
```

## Direct ROS equivalents

Start the Vicon-controlled node with wand input disabled:

```bash
ros2 run cf_control crazyflie_node --ros-args \
  -p drone_id:=cf1 \
  -p positioning_source:=vicon \
  -p vicon_pose_topic:=/vicon/cf1/cf1 \
  -p wand_control_enabled:=false \
  -p flight_mode:=autonomous \
  -p external_targets_enabled:=false
```

Start it with Lighthouse:

```bash
ros2 run cf_control crazyflie_node --ros-args \
  -p drone_id:=cf1 \
  -p positioning_source:=lighthouse \
  -p wand_control_enabled:=false
```

Services:

```bash
ros2 service call /cf1/arm std_srvs/srv/Trigger '{}'
ros2 service call /cf1/takeoff std_srvs/srv/Trigger '{}'
ros2 service call /cf1/land std_srvs/srv/Trigger '{}'
ros2 service call /cf1/emergency_stop std_srvs/srv/Trigger '{}'
ros2 service call /cf1/clear_target std_srvs/srv/Trigger '{}'
```

## Shutdown between tests

After the state monitor reports `DISARMED`, stop each foreground process with
`Ctrl+C`.

If a previous hardware/Vicon stack did not exit cleanly:

```bash
cfkill cf1
```

For Gazebo, stop the launch terminal with `Ctrl+C`. Confirm no simulator is
left behind before starting another test:

```bash
cfkillgazebo
pgrep -af 'gz sim|ruby.*gz|crazyflie_node|vicon_client'
```
