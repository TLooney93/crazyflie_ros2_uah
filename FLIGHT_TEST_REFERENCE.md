# Crazyflie Vicon Flight-Test Runbook

This runbook covers the complete hardware startup and execution procedure for
the `cf1` autonomous test mission. The mission takes off vertically, hovers,
flies a smooth figure-eight centered on its takeoff point, returns to the
origin, and lands.

## Current configuration

| Item | Value |
|---|---|
| Crazyflie ID | `cf1` |
| Vicon computer wired IP | `192.168.10.1` |
| Operator computer wired IP | `192.168.10.2/24` |
| Wired interface | `enx047bcb661434` |
| Vicon pose topic | `/vicon/cf231/cf231` |
| Pose message type | `geometry_msgs/msg/PoseStamped` |
| Takeoff height | 0.30 m above the recorded floor position |
| Initial hover | 3 seconds |
| Figure-eight radius | 0.35 m |
| Figure-eight duration | 20 seconds |
| Mission safety timeout | 35 seconds |

The mission treats the position recorded by `cfarm` as its origin. The
figure-eight extends 0.35 m along X and approximately 0.175 m along Y from
that point.

## Safety requirements

Run the entire procedure once with the propellers removed after any software,
radio, estimator, Vicon, or network change.

For a propeller-on flight:

- Use a clear indoor flight volume with at least 0.75 m of horizontal
  clearance around the takeoff point.
- Install propeller guards and use a fully charged battery.
- Keep everyone outside the flight volume.
- Keep an emergency-stop terminal visible and ready.
- Do not fly unless Vicon position is stable and follows hand movement with the
  correct axis directions.

Use `cfland cf1` for a controlled early landing. Use `cfstop cf1` only when an
immediate motor stop is safer than continued flight; an airborne drone will
fall after `cfstop`.

## 1. Start the Vicon system

On the Vicon computer:

1. Power the Vicon cameras and network hardware.
2. Start Vicon Tracker.
3. Open the calibrated tracking session.
4. Confirm all required cameras are connected and contributing.
5. Confirm the Crazyflie rigid body is detected as subject and segment
   `cf231`.
6. Enable the Vicon DataStream server.
7. Leave Tracker running throughout the flight.

Place the Crazyflie within the calibrated volume and confirm that Tracker shows
a stable rigid-body pose before continuing.

## 2. Connect and verify the wired network

Connect the operator computer to the same Vicon switch as the Vicon computer.
The saved NetworkManager profile should configure the adapter as
`192.168.10.2/24` without a default gateway.

Activate and inspect it:

```bash
nmcli connection up "Wired connection 1"
ip -brief address show enx047bcb661434
ip route get 192.168.10.1
ping -c 3 192.168.10.1
```

Expected results include:

```text
enx047bcb661434  UP  192.168.10.2/24
192.168.10.1 dev enx047bcb661434 src 192.168.10.2
```

Do not continue unless all ping packets return. Wi-Fi may remain connected;
the specific `192.168.10.0/24` route sends Vicon traffic over Ethernet.

If the wired profile must be recreated:

```bash
nmcli connection modify "Wired connection 1" \
  connection.interface-name enx047bcb661434 \
  ipv4.method manual \
  ipv4.addresses 192.168.10.2/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method disabled

nmcli connection up "Wired connection 1"
```

## 3. Prepare the Crazyflie

1. Verify the propellers and guards are installed correctly.
2. Place the Crazyflie level at the desired mission origin.
3. Connect the Crazyradio USB dongle to the operator computer.
4. Power on `cf1` and confirm it remains stationary.
5. Keep the takeoff point clear; the mission will return to this position.

The configured radio URI is defined in
`src/cf_control/cf_control/config.py`.

## 4. Start and verify Vicon ROS data

### Terminal 1 — Vicon receiver

```bash
source ~/.bash_aliases
cfvicon 192.168.10.1 vicon
```

Leave this terminal running. It must report:

```text
Connection successfully established with 192.168.10.1
Creating publisher for segment cf231 from subject cf231
```

The explicit address is required because the current `cfvicon` shell default
still points to the campus/Wi-Fi address.

### Terminal 2 — Raw Vicon verification

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicon_receiver_ws/install/setup.bash
ros2 topic list | grep vicon
ros2 topic type /vicon/cf231/cf231
ros2 topic hz /vicon/cf231/cf231
```

Required type:

```text
geometry_msgs/msg/PoseStamped
```

The observed stream normally runs near 120 Hz. Stop the frequency display with
`Ctrl+C` after confirming a stable rate. Inspect coordinates if needed:

```bash
ros2 topic echo /vicon/cf231/cf231
```

Move the unpowered drone by hand and verify that X, Y, and Z change smoothly
and in the expected directions.

## 5. Build and start the Crazyflie node

### Terminal 3 — Crazyflie node

```bash
cfenv
cfbuild
cfrun cf1 vicon /vicon/cf231/cf231
```

Leave this terminal running. Do not arm until it reports both:

```text
Kalman estimator selected
Vicon stream detected; resetting estimator
Vicon positioning is ready for arming
```

If the radio connection fails, power-cycle the drone, reconnect the Crazyradio,
and restart this terminal before proceeding.

## 6. Start flight monitors

### Terminal 4 — Fused position

```bash
cfenv
cfpos cf1
```

Confirm the position is finite and stable while the drone is stationary. Lift
and move the unpowered drone by hand once more. The fused position must follow
Vicon with the correct signs and return to a stable value when stationary.
The fused coordinates should be close to the raw Vicon coordinates; a large
altitude such as approximately 170 m indicates the Kalman estimator was not
selected, and flight must not continue.

### Terminal 5 — Flight state

```bash
cfenv
cfstate cf1
```

The initial state must be:

```text
DISARMED
```

## 7. Execute the mission

### Terminal 6 — Flight control and emergency stop

```bash
cfenv
```

Keep this terminal selected during the flight.

Arm the drone while it is level and stationary at the desired origin:

```bash
cfarm cf1
```

Confirm the service succeeds and Terminal 5 changes to `READY`. Wait at least
one second for arming to settle, then start the mission:

```bash
cftakeoff cf1
```

The autonomous sequence is:

```text
TAKEOFF -> HOVER -> FIGURE_EIGHT -> RETURN_HOME -> LANDING -> DISARMED
```

The node prints each mission phase in Terminal 3. Flight state remains
`FLYING` during the mission phases, changes to `LANDING` during descent, and
finally changes to `DISARMED`.

To request a controlled landing at any time:

```bash
cfland cf1
```

For an emergency motor stop:

```bash
cfstop cf1
```

Use `cfstop` immediately if the drone tilts sharply, approaches a person or
obstacle, climbs unexpectedly, develops increasing oscillation, or does not
respond to `cfland`.

Do not press `Ctrl+C` in Terminal 3 while the drone is airborne. Land or stop
the drone first and confirm `DISARMED`.

## 8. Shut down

After Terminal 5 reports `DISARMED`:

1. Stop the Crazyflie node in Terminal 3 with `Ctrl+C`.
2. Stop the Vicon receiver in Terminal 1 with `Ctrl+C`.
3. Power off the Crazyflie.
4. Close the Vicon session or leave Tracker running for the next test.
5. Disconnect the wired profile when finished, if desired:

```bash
nmcli connection down "Wired connection 1"
```

## Wand-control mode

Wand control is opt-in and does not replace the autonomous figure-eight.
Start the hardware controller in wand mode with:

```bash
cfwandflight cf1 vicon /vicon/cf231/cf231
```

The wand cannot arm the drone. Arming remains a deliberate terminal command,
and the existing landing, emergency-stop, position freshness, Vicon agreement,
and maximum-flight-time checks remain active.

### Gazebo wand test

Run this test before using props:

Terminal 1:

```bash
cfenv
cfsim headless wand
```

Terminal 2:

```bash
cfenv
cfarm cf1
```

Wait at least one second after arming. Then start the keyboard wand:

```bash
cfw
```

At the `wand>` prompt, enter these one at a time:

```text
enable
takeoff
move_forward
move_left
hold
land
```

Wait for `WAND_HOVER` in the Crazyflie-node output before entering movement
commands. `move_forward` and `move_backward` change world X;
`move_left` and `move_right` change world Y. Each command moves the target by
0.05 m. Horizontal targets are clamped to 0.50 m from the recorded arm point.

### Physical cf2 wand

The teammate implementation uses `cf2` as the physical wand. It publishes
position and attitude on `/wand/pose`. Start it in its own terminal:

Remove all propellers from `cf2` before using it as a handheld wand. The
driver only reads telemetry, but a propeller-free wand prevents an unexpected
firmware or operator command from becoming a handheld motor hazard.

```bash
cfenv
cfwanddriver cf2
```

If cf1 and cf2 cannot share one Crazyradio across the two processes, connect a
second Crazyradio and pass its URI explicitly. For example, when the second
radio is index 1:

```bash
cfwanddriver cf2 radio://1/80/2M/E7E7E7E7E6
```

Verify the stream before flight:

```bash
ros2 topic hz /wand/pose
ros2 topic echo /wand/connected
```

Use another terminal for the guarded command source:

```bash
cfenv
cfw
```

The hardware sequence is:

1. Start Vicon and the cf1 node with wand control enabled.
2. Start `cfwanddriver cf2` and verify `/wand/pose` is updating.
3. Hold cf2 level, run `cfarm cf1`, and wait at least one second.
4. Enter `enable`; this records the wand's neutral attitude and heading.
5. Enter `takeoff` and wait for `WAND_HOVER`.
6. Tilt the wand more than 20 degrees to move. Forward/back tilt commands the
   calibrated heading; left/right roll commands its perpendicular direction.
7. Return the wand to neutral to hold the current target.
8. Enter `land` for a controlled landing.

While a tilt is held, the target advances by 0.05 m every 0.25 seconds. Enter
`disable` to stop accepting wand movement and hold the current position. Enter
`land` even if the wand has been disabled. Keep `cfstop cf1` ready in a
separate terminal for emergencies.

## Mission parameters

The selected sequence and its geometry are defined in:

```text
src/cf_control/cf_control/crazyflie_missions/<mission_name>.py
```

Controller gains and shared safety limits are defined in:

```text
src/cf_control/cf_control/config.py
```

Important parameters:

```python
FLIGHT_TEST_HEIGHT_M = 0.30
FLIGHT_TEST_HOVER_S = 3.0
FLIGHT_TEST_FIGURE_EIGHT_RADIUS_M = 0.35
FLIGHT_TEST_FIGURE_EIGHT_DURATION_S = 20.0
FLIGHT_TEST_POSITION_TOLERANCE_M = 0.08
MAX_FLIGHT_TIME_S = 35.0
MAX_VELOCITY = 0.3
POSITION_TIMEOUT_S = 0.5
VICON_ESTIMATOR_MAX_ERROR_M = 0.5
TELEMETRY_LOSS_DESCENT_S = 3.0
```

After changing these values, rebuild before flight:

```bash
cfenv
cfbuild
```

## Troubleshooting

### Wired ping fails

```bash
nmcli connection up "Wired connection 1"
ip -brief address show enx047bcb661434
ip route get 192.168.10.1
ping -c 3 192.168.10.1
```

The route must use `enx047bcb661434`, not `wlp9s0`. Confirm the Vicon computer
is powered on and both computers are connected to the same switch.

### No Vicon topic

Confirm Terminal 1 connected successfully and Tracker's DataStream server is
enabled. Then run:

```bash
ros2 topic list | grep vicon
```

The current rigid-body topic is `/vicon/cf231/cf231`.

### Wrong Vicon topic name

Use the topic shown by `ros2 topic list` as the third `cfrun` argument:

```bash
cfrun cf1 vicon /vicon/SUBJECT/SEGMENT
```

### Node waits for Vicon poses

Verify that Terminal 1 is still running, the raw topic is updating, and
Terminal 3 was started with `/vicon/cf231/cf231`.

### Takeoff is rejected

Confirm:

- Terminal 5 shows `READY`.
- At least one second has elapsed since `cfarm cf1`.
- Fused position data is still updating.
- Terminal 3 reported `Vicon positioning is ready for arming`.

### Position becomes stale during flight

The node automatically enters its telemetry-loss descent and stops after the
configured timeout. Keep the flight volume clear and investigate both the
Vicon stream and Crazyflie radio link before attempting another flight.

### Shell command is missing

```bash
source ~/.bash_aliases
cfenv
```

## Direct ROS equivalents

```bash
ros2 run cf_control crazyflie_node --ros-args \
  -p drone_id:=cf1 \
  -p positioning_source:=vicon \
  -p vicon_pose_topic:=/vicon/cf231/cf231

ros2 service call /cf1/arm std_srvs/srv/Trigger '{}'
ros2 service call /cf1/takeoff std_srvs/srv/Trigger '{}'
ros2 service call /cf1/land std_srvs/srv/Trigger '{}'
ros2 service call /cf1/emergency_stop std_srvs/srv/Trigger '{}'

ros2 topic echo /cf1/flight_state
ros2 topic echo /cf1/position
```
