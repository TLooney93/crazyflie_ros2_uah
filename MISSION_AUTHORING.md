# Mission and profile authoring

Mission files contain geometry and timing. Profiles contain hardware and
simulation membership. Shared controller and safety policy stays in
`config.py`.

## Single-drone mission

Add `src/cf_control/cf_control/crazyflie_missions/<name>.py`:

```python
"""Short description of the route."""

MISSION_NAME = 'inspection'
MISSION = [
    {
        'name': 'HOVER',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, 0.30),
        'hold_s': 3.0,
    },
]
```

Import and register it in `crazyflie_missions/loader.py`, run the mission
loader tests, then simulate it with `cfsim gui autonomous inspection`.

## Swarm mission

Add `src/cf_control/cf_control/swarm_missions/<name>.py`:

```python
"""Short description of the formation."""

MISSION_NAME = 'two_drone_hold'
MISSION = [
    {
        'name': 'HOLD',
        'transition_s': 2.0,
        'hold_s': 5.0,
        'arrival_tolerance_m': 0.08,
        'timeout_s': 10.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.30),
            'cf2': (0.0, 0.0, 0.30),
        },
    },
]
```

Use `offsets` for positions relative to each drone's arming location. Use
`formation_targets` only when geometry is intentionally expressed around the
group center. Trajectories must provide continuous targets and bounded
feedforward velocities.

Register the module in `swarm_missions/loader.py`. The loader rejects missing
drones, non-finite coordinates, unsafe separation, invalid timeouts, and
unsupported trajectory definitions.

## Swarm profile

Add `src/cf_control/cf_control/swarm_profiles/<name>.py`:

```python
"""Hardware and simulation membership for the formation."""

from cf_control.config import DRONES

PROFILE_NAME = 'two_drone_hold'
PROFILE = {
    'drone_ids': ('cf1', 'cf2'),
    'vicon_topics': {
        'cf1': '/vicon/cf1/cf1',
        'cf2': '/vicon/cf2/cf2',
    },
    'radio_uris': {
        'cf1': DRONES.get('cf1', ''),
        'cf2': DRONES.get('cf2', ''),
    },
    'mission_name': 'two_drone_hold',
    'takeoff_height_m': 0.30,
    'simulation_world': 'crazyflie_swarm_world.sdf',
}
```

Register it in `swarm_profiles/loader.py`. Keep the Vicon subject, physical
label, radio address, and simulated model name identical.

## Validation checklist

1. `python -m pytest -q src/cf_control/test/test_crazyflie_mission.py`
2. `python -m pytest -q src/cf_control/test/test_swarm_missions.py`
3. `python -m pytest -q src/cf_control/test/test_swarm_profiles.py`
4. Simulate the exact profile and mission.
5. Confirm all commanded pairs remain above the configured minimum separation.
6. Run a low hover before adding horizontal motion on hardware.
