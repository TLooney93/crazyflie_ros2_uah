# Crazyflie operations

This is the operator runbook for simulation and hardware. Run commands from a
terminal where `cfenv` has loaded the ROS workspace and Python environment.

## Before every hardware flight

1. Remove props while checking radio identities or changing configuration.
2. Verify each `DRONES` URI in `src/cf_control/cf_control/config.py` maps to
   the intended airframe.
3. Confirm the Vicon subject, radio URI, and physical label agree for every
   drone. A swapped identity defeats separation and position safety.
4. Use charged batteries and clear the flight volume.
5. Keep `cfswarmstop` (or `cfstop DRONE_ID`) ready. Emergency stop cuts
   thrust; use a controlled landing when that remains safe.
6. Simulate a new mission before flying it.

## Build and automated checks

```bash
cfenv
cfbuild
cftest
```

Use the full package test when changing runtime infrastructure:

```bash
cd "$CF_WS"
python -m pytest -q src/cf_control/test
```

## Simulation

```bash
# Single drone
cfsim gui autonomous hover_test

# Three-drone saved mission
cfswarmsim gui three_drone_demo

# Start the armed swarm sequence from another terminal
cfswarmstart
```

Launch never arms automatically. Use `cfarm` and `cftakeoff` for a single
vehicle, or `cfswarmstart` for a complete swarm sequence.

## Hardware launch

Start the Vicon-to-ROS receiver first and verify that its subjects are live:

```bash
ros2 topic hz /vicon/cf1/cf1
ros2 topic echo --once /vicon/cf1/cf1
```

Then launch one of the guarded control paths:

```bash
# One autonomous vehicle
cfsingle cf1 vicon /vicon/cf1/cf1

# Profile-selected swarm
cfswarm three_drone_demo autonomous vicon
```

In separate terminals, monitor state and position:

```bash
cfstate cf1
cfpos cf1
cfswarmstate
```

Start and stop with:

```bash
cfarm cf1
cftakeoff cf1
cfland cf1
cfstop cf1

cfswarmstart
cfswarmland
cfswarmstop
```

## Rate diagnostics

Every five seconds each hardware controller reports a line beginning with
`Vicon rates`. During a three-drone flight, expect approximately:

- Vicon input: the receiver rate, normally 60–120 Hz;
- estimator forwarding: 20 Hz per drone;
- estimator health telemetry: 2 Hz per drone;
- cflib velocity handoff: 50 Hz per flying drone;
- measured radio uplink: current per-link packet rate, payload use, and quality;
- max command gap: normally near 20 ms and below 60 ms.

`skipped frames`, `stale commands`, a command gap above 100 ms, repeated
telemetry restarts, or an estimator age above one second indicate an unhealthy
run. Land and resolve the link or host scheduling problem instead of loosening
safety timeouts.

The transport follows this policy:

```text
controller computes newest command at 50 Hz
  -> dispatcher keeps one latest command per drone
  -> every 20 ms: send each fresh command, then pending Vicon updates
  -> skip an overdue frame; never replay a burst of old commands
```

## Recording a flight

Record only the topics needed to reconstruct control and positioning:

```bash
ros2 bag record \
  /rosout /swarm/state \
  /cf1/flight_state /cf1/control_phase /cf1/position \
  /cf1/debug/velocity /cf1/battery_voltage /cf1/stabilizer_thrust \
  /vicon/cf1/cf1
```

Add the equivalent topics for each swarm member. Inspect counts and duration
with `ros2 bag info BAG_DIRECTORY`.

## Failure guide

- **No Vicon topic:** fix the receiver/network before starting control.
- **Vicon input falls sharply:** reduce unrelated ROS/console load and check
  the wired receiver link.
- **Estimator telemetry is missing for one drone:** power-cycle that airframe,
  verify its URI, and relaunch. Do not arm with stale onboard state.
- **Frequent skipped command frames:** stop other Crazyradio clients and close
  old launch processes. Only the hardware runner should own swarm links.
- **Zero Vicon pose or discontinuous motion:** restore marker visibility and
  verify subject identity before changing the position filter.
- **Takeoff leaves its launch column:** verify radio/Vicon identity, props,
  marker balance, and floor position before changing gains.
- **Gazebo remains after a test:** run `cfkillgazebo`.

## Shutdown

Land first, wait for `DISARMED`, stop the launch, stop bag recording, then
power down the airframes. Never leave another cflib script connected to the
same radio while this stack is running.
