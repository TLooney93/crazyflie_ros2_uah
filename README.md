# CrazyConductors Crazyflie control stack

ROS 2 Jazzy software for Vicon-guided Crazyflie flight, coordinated swarms,
Gazebo simulation, and wand gesture control. Lighthouse remains available as
an alternate hardware position source selected before launch.

The active hardware baseline is Vicon with one Crazyradio shared by the
controllers in a swarm process. Positions are metres, velocities are metres
per second, and mission offsets are relative to each vehicle's arming position
unless a mission explicitly uses formation coordinates.

## Start here

Load the environment in every terminal, then build and test:

```bash
cfenv
cfbuild
cftest
```

Run a safe single-drone simulation:

```bash
cfsim gui autonomous hover_test
```

Run the current three-drone simulation profile:

```bash
cfswarmsim gui three_drone_demo
```

Launch does not arm automatically. In another terminal, use
`cfarm`/`cftakeoff` for a single drone or `cfswarmstart` for a swarm. Keep
`cfstop` or `cfswarmstop` ready during every hardware test.

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) — process ownership, data flow, safety,
  and where each kind of change belongs.
- [OPERATIONS.md](OPERATIONS.md) — simulation, hardware, flight logging, rate
  diagnostics, and failure handling.
- [MISSION_AUTHORING.md](MISSION_AUTHORING.md) — concise mission/profile
  templates and validation steps.

## Configuration ownership

| Change | Edit |
|---|---|
| Radio address or shared control/safety tuning | `src/cf_control/cf_control/config.py` |
| Single-drone route | `src/cf_control/cf_control/crazyflie_missions/` |
| Swarm route or formation | `src/cf_control/cf_control/swarm_missions/` |
| Swarm roster, Vicon topics, radios, world | `src/cf_control/cf_control/swarm_profiles/` |
| Shell commands and aliases | `/home/tanner-looney/dev/crazyflie/scripts/cf_env.sh` |

Single and swarm hardware flights both use `CrazyflieNode`, so controller,
takeoff, telemetry, and landing settings in `config.py` apply to both. Swarm-
only gates such as separation and group timeouts are applied by `SwarmNode`.

## Safety baseline

- Simulate a changed mission before hardware flight.
- Verify each Vicon rigid-body topic maps to the intended physical drone.
- Use charged batteries and confirm fresh Vicon and onboard telemetry before
  arming.
- Use controlled landing when possible. Emergency stop cuts thrust and an
  airborne vehicle will fall.
- Tune one safety factor at a time and review the ROS log and bag after each
  hardware test.
