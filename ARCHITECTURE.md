# Crazyflie control architecture

This document is the authoritative runtime overview. Command syntax belongs
in `LAUNCH_TEST_REFERENCE.md`; mission schemas belong in
`MISSION_AND_PROFILE_AUTHORING_REFERENCE.md`.

## Runtime layers

```text
Vicon / Lighthouse / Gazebo
            |
            v
   per-drone CrazyflieNode  <---- wand gestures or terminal targets
      | position control
      | takeoff/landing safety
      | estimator forwarding
      v
 CrazyflieInterface ----> firmware commander / simulated command topic
      ^
      |
   SwarmNode (swarm modes only)
      group state, formation targets, separation and altitude gates
```

## Process ownership

### Single hardware flight

`control.launch.py` starts one `CrazyflieNode`. The node owns its radio link,
position source, telemetry reader, flight state machine, mission execution,
and command output.

### Hardware swarm

`control.launch.py` starts two processes:

1. `hardware_swarm_runner` creates one `CrazyflieNode` per vehicle in a shared
   process and routes cflib work through one `RadioCommandDispatcher`. This
   prevents independent processes from competing for the Crazyradio USB
   driver. The dispatcher takes one group snapshot every 20 ms (50 Hz), sends
   one addressed velocity packet for each drone, and only then sends pending
   external-position packets. Stock CRTP cannot place three independently
   addressed velocity commands in one over-air packet, so "group frame" here
   means one scheduled batch of three packets on the shared radio.
2. `swarm_node` coordinates group services and publishes each drone's target.
   It monitors group position freshness, separation, altitude, and phase
   completion. It never opens a radio link.

An optional physical wand driver is created inside the shared hardware process
so its radio traffic uses the same dispatcher.

### Simulation

Gazebo launch files create simulated vehicles, ROS bridges, per-drone
controllers, and the same swarm coordinator used by hardware. Simulation uses
Gazebo odometry as authoritative position; Vicon is not required.

## Per-drone Vicon data flow

```text
/vicon/<subject>/<segment> (PoseStamped)
   -> pose validation and jump/reacquisition filtering
   -> /<drone_id>/position (Point)
   -> FlightController (bounded PI position control)
   -> takeoff/landing command shaping
   -> Crazyflie world-frame velocity setpoint
```

Accepted Vicon poses are rate-limited before being forwarded to the firmware
Kalman estimator. ROS keeps the full receiver stream for control and safety;
radio forwarding defaults to 5 Hz to preserve shared-radio bandwidth. Onboard
estimator telemetry remains a separate preflight health check.

Velocity commands use latest-value semantics: every controller renews its
command at 50 Hz and the dispatcher repeats only the newest value during its
shared frame. A value expires after 100 ms without renewal, preventing a dead
controller callback from sustaining an old movement command indefinitely.

## Flight ownership

`CrazyflieNode` owns the safety-critical state machine:

```text
DISARMED -> READY -> FLYING -> LANDING -> DISARMED
```

It handles arming checks, takeoff, single missions or external targets,
controlled landing, stale-position descent, and emergency stop. `takeoff.py`
contains reusable near-floor command shaping and the stalled-takeoff monitor.
Because swarm controllers are instances of the same node, these policies apply
to both single and swarm hardware.

Marker-loaded airframes receive a three-second launch-assist window. It allows
the full configured `0.50 m/s` vertical command and delays only the
stalled-climb and thrust-saturation timers. Vicon freshness, the hard launch
column, emergency stop, and overall flight timeout remain active.

Hardware arming is a confirmed transaction. After sending the request, the
interface polls the firmware supervisor and the node enters `READY` only when
both `Is armed` and `Can fly` are present. Locked, crashed, or tumbled states
fail immediately; a missing confirmation times out after one second and sends
a defensive disarm request.

`SwarmNode` adds group behavior above the individual state machines. It calls
per-drone services, publishes targets, advances saved phases, and requests
coordinated landing when a group-level safety condition fails.

## Configuration ownership

- `config.py`: shared radio identities, controller gains, update rates,
  takeoff/landing policy, telemetry gates, swarm safety, and wand thresholds.
- `crazyflie_missions/<name>.py`: one-drone phase geometry and timing.
- `swarm_missions/<name>.py`: multi-drone phases, formations, and trajectories.
- `swarm_profiles/<name>.py`: roster, Vicon topics, radio URIs, default mission,
  takeoff height, and simulation world.
- each folder's `loader.py`: registry and validation. A new file is not usable
  until imported and registered there.

Keep mission geometry out of `config.py`, and do not add drone rosters directly
to `swarm_node.py`.

## Input modes

- `autonomous`: run the named saved mission.
- `terminal`: accept bounded `/<drone_id>/target` points.
- `wand`: use gestures for takeoff, mission start, landing, and bounded motion.
  The enable button gates gesture capture; it is not held for the full flight.

The default gesture mapping is `up_down` for takeoff, `side_to_side` to start
the saved mission after hover, and `down_up` for landing.

## Safety cascade

1. Mission/profile loaders reject malformed or out-of-bounds definitions.
2. Launch rejects mismatched rosters, topics, and radio lists.
3. Each controller checks battery, estimator agreement, and fresh position.
4. Each controller filters Vicon jumps and detects stale position, launch-
   column departure, insufficient climb, and sustained high thrust.
5. The swarm coordinator monitors every member and lands the group for a
   confirmed separation, altitude, or member-state fault.

Emergency stop bypasses controlled descent. Use it only when continued thrust
is the greater hazard.

The hardware runner also owns `/hardware_swarm/emergency_stop`. This service
directly stops and disarms every connected radio link and remains available if
the higher-level `swarm_node` coordinator crashes. External-mode vehicle nodes
require a continuous target heartbeat; loss of the coordinator initiates a
local landing instead of retaining the last target indefinitely. During staged
missions, motion by a member that should be grounded triggers a group hard stop
as a likely radio/Vicon identity mismatch.

## Where to extend the system

- New mission: add a mission file and register it in its loader.
- New swarm size or hardware set: add a profile, not coordinator code.
- New gesture: recognize it in `wand_gesture_node.py`, map it in `config.py`,
  then handle it in the relevant controller or coordinator.
- New position source: adapt it at the `CrazyflieNode` position boundary and
  keep published positions in the shared world frame.
- New safety rule: put vehicle-local rules in the controller or a focused
  policy module; put cross-vehicle rules in `swarm_node.py`.
