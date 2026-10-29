# Mission and Profile Authoring Reference

Use this reference when adding a new single-drone mission, swarm mission, or
swarm vehicle profile. Mission and profile names are selected at launch time;
the controller and coordinator state machines normally do not need editing.

## File locations

```text
src/cf_control/cf_control/
├── crazyflie_missions/
│   ├── <single_mission>.py
│   └── loader.py
├── swarm_missions/
│   ├── <swarm_mission>.py
│   └── loader.py
├── swarm_profiles/
│   ├── <swarm_profile>.py
│   └── loader.py
└── config.py

src/cf_gazebo/worlds/
└── <swarm_world>.sdf
```

Each new file must be registered in the `loader.py` in the same folder.
Creating a file without registering it does not make it selectable.

## Common Python file header requirements

Every mission or profile file should start with:

1. A module docstring describing its purpose.
2. Any required imports from `cf_control.config`.
3. The required name constant.
4. The required mission or profile object.

Use lowercase `snake_case` for filenames and selection names. Keeping the
filename and selection name identical is strongly recommended.

```python
"""Short description of this mission or profile."""

from cf_control.config import SOME_SHARED_VALUE


MISSION_NAME = 'example_name'  # Or PROFILE_NAME for a profile.
```

The blank lines are intentional and keep the file compatible with the
project's Python style checks.

## Single-drone mission

### Required exports

A single-drone mission file must export:

```python
MISSION_NAME = 'unique_mission_name'
MISSION = [...]
```

Offsets are `(x, y, z)` meters relative to the position recorded when the
drone is armed. The controller performs takeoff and landing automatically.

### Waypoint mission template

Create
`src/cf_control/cf_control/crazyflie_missions/my_mission.py`:

```python
"""Forward waypoint, hover, return, and landing demonstration."""

from cf_control.config import FLIGHT_TEST_HEIGHT_M


MISSION_NAME = 'my_mission'

MISSION = [
    {
        'name': 'INITIAL_HOVER',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 2.0,
    },
    {
        'name': 'FORWARD',
        'kind': 'waypoint',
        'offset': (0.25, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 2.0,
    },
    {
        'name': 'RETURN_HOME',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 1.0,
    },
]
```

Waypoint fields:

- `name`: unique, non-empty phase name.
- `kind`: must be `'waypoint'`.
- `offset`: three finite XYZ values relative to the arming position.
- `hold_s`: optional, finite, and non-negative; the default is zero.

### Figure-eight phase template

```python
{
    'name': 'FIGURE_EIGHT',
    'kind': 'figure_eight',
    'center_offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
    'radius_m': 0.25,
    'duration_s': 15.0,
},
```

Figure-eight fields:

- `center_offset`: three finite XYZ values relative to the arming position.
- `radius_m`: finite and greater than zero.
- `duration_s`: finite and greater than zero.

### Single-drone validation rules

- The mission must contain at least one phase.
- Phase names must be unique and non-empty.
- Do not use the reserved names `TAKEOFF`, `EXTERNAL_TAKEOFF`,
  `EXTERNAL_HOVER`, `WAND_TAKEOFF`, or `WAND_HOVER`.
- Horizontal distance from home must not exceed
  `EXTERNAL_MAX_HORIZONTAL_OFFSET_M`.
- Phase height must remain between `EXTERNAL_MIN_HEIGHT_M` and
  `EXTERNAL_MAX_HEIGHT_M`.
- A figure eight's center distance plus its radius must fit within the
  horizontal limit.
- The final phase must have an X/Y offset of exactly `(0.0, 0.0)` so landing
  begins above the arming position.

### Register the single-drone mission

Edit `crazyflie_missions/loader.py`.

Add an import:

```python
from cf_control.crazyflie_missions.my_mission import (
    MISSION as MY_MISSION,
    MISSION_NAME as MY_MISSION_NAME,
)
```

Add it to the registry:

```python
CRAZYFLIE_MISSIONS = {
    # Existing missions...
    MY_MISSION_NAME: MY_MISSION,
}
```

### Run the single-drone mission

Gazebo:

```bash
cfenv
cfbuild
cftest
cfsim headless autonomous my_mission
```

Hardware with the configured `cf1` radio URI:

```bash
cfenv
cfsingle cf1 vicon /vicon/cf231/cf231 "" my_mission
```

The fourth `cfsingle` argument is an optional radio URI override. The fifth is
the mission name.

## Swarm mission

### Required exports

A swarm mission file must export:

```python
MISSION_NAME = 'unique_swarm_mission_name'
MISSION = [...]
```

Every phase supplies one offset for every drone in the selected profile.
Offsets are relative to each drone's own arming position, not a shared global
origin.

### Swarm mission template

Create
`src/cf_control/cf_control/swarm_missions/my_swarm_demo.py`:

```python
"""Two-drone hover, translation, return, and landing demonstration."""


MISSION_NAME = 'my_swarm_demo'

MISSION = [
    {
        'name': 'HOVER',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.35),
            'cf2': (0.0, 0.0, 0.35),
        },
    },
    {
        'name': 'FORWARD',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.25, 0.35),
            'cf2': (0.0, 0.25, 0.35),
        },
    },
    {
        'name': 'RETURN_HOME',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.35),
            'cf2': (0.0, 0.0, 0.35),
        },
    },
]
```

Swarm phase fields:

- `name`: a descriptive phase name; unique names are recommended.
- `hold_s`: finite and non-negative. Timing starts only after every drone is
  within `SWARM_POSITION_TOLERANCE_M` of its target.
- `offsets`: a dictionary containing exactly the drone IDs in the profile.
- Each offset is a finite `(x, y, z)` tuple in metres.

### Swarm mission validation and safety rules

- The mission must contain at least one phase.
- Every phase must contain exactly the selected profile's drone IDs—no missing
  or additional IDs.
- Every target must remain inside the external horizontal and height limits.
- Simultaneous absolute targets must remain at least
  `SWARM_MIN_SEPARATION_M` apart.
- Actual measured positions are also monitored for minimum separation during
  flight.
- Returning each drone to `(0.0, 0.0, flight_height)` in the final phase is
  strongly recommended before the automatic landing.

Because offsets are relative to separate home positions, target separation
depends on both the mission offsets and the physical or simulated starting
formation.

### Register the swarm mission

Edit `swarm_missions/loader.py`.

Add an import:

```python
from cf_control.swarm_missions.my_swarm_demo import (
    MISSION as MY_SWARM_DEMO,
    MISSION_NAME as MY_SWARM_DEMO_NAME,
)
```

Add it to the registry:

```python
SWARM_MISSIONS = {
    # Existing missions...
    MY_SWARM_DEMO_NAME: MY_SWARM_DEMO,
}
```

## Swarm profile

A swarm profile connects a roster of drone IDs to its Vicon topics, radio
URIs, default mission, and Gazebo world.

### Required exports

```python
PROFILE_NAME = 'unique_profile_name'
PROFILE = {...}
```

### Swarm profile template

Create `src/cf_control/cf_control/swarm_profiles/my_swarm.py`:

```python
"""Two-drone hardware and simulation profile."""

from cf_control.config import DRONES


PROFILE_NAME = 'my_swarm'

PROFILE = {
    'drone_ids': ('cf1', 'cf2'),
    'vicon_topics': {
        'cf1': '/vicon/cf1/cf1',
        'cf2': '/vicon/cf2/cf2',
    },
    'radio_uris': {
        'cf1': DRONES['cf1'],
        'cf2': DRONES['cf2'],
    },
    'mission_name': 'my_swarm_demo',
    'simulation_world': 'my_swarm_world.sdf',
}
```

Profile fields:

- `drone_ids`: ordered tuple or list containing at least two unique,
  non-empty names.
- `vicon_topics`: exactly one absolute topic beginning with `/` per drone ID.
- `radio_uris`: exactly one string per drone ID. An empty string is allowed
  while developing in simulation, but hardware launch rejects missing URIs.
- `mission_name`: registered swarm mission name whose offsets contain exactly
  the same drone IDs.
- `simulation_world`: SDF filename only, ending in `.sdf`; do not include a
  path.

Use separately addressable radios for simultaneous hardware connections. The
current examples use radio index `0` for `cf1` and index `1` for `cf2`.

### Register the swarm profile

Edit `swarm_profiles/loader.py`.

Add an import:

```python
from cf_control.swarm_profiles.my_swarm import (
    PROFILE as MY_SWARM,
    PROFILE_NAME as MY_SWARM_NAME,
)
```

Add it to the registry:

```python
SWARM_PROFILES = {
    # Existing profiles...
    MY_SWARM_NAME: MY_SWARM,
}
```

### Matching Gazebo world

Create the world named by `simulation_world` under
`src/cf_gazebo/worlds/`. It must include one Crazyflie model for every
profile ID, using exactly matching names:

```xml
<include>
  <uri>model://crazyflie</uri>
  <name>cf1</name>
  <pose>-0.30 0 0.08 0 0 0</pose>
</include>

<include>
  <uri>model://crazyflie</uri>
  <name>cf2</name>
  <pose>0.30 0 0.08 0 0 0</pose>
</include>
```

Initial positions must provide more than `SWARM_MIN_SEPARATION_M` clearance.
Leave additional margin for tracking error; do not place drones exactly at
the configured minimum.

### Run the swarm profile and mission

Gazebo, using the profile's default mission:

```bash
cfenv
cfbuild
cftest
cfswarmsim headless my_swarm
```

In a second terminal:

```bash
cfenv
cfswarmstart
```

Override the profile's default mission:

```bash
cfswarmsim headless my_swarm my_swarm_demo
```

Hardware, using the profile's default mission:

```bash
cfenv
cfvicon
```

In another terminal:

```bash
cfenv
cfswarm my_swarm
```

Then start the complete sequence:

```bash
cfswarmstart
```

Keep `cfswarmstop` ready in a separate terminal during hardware testing.

## Single-drone configuration versus profiles

The current architecture has reusable mission files for a single drone, but
does not have a separate `crazyflie_profiles/` package. Single-drone identity
and default radio URI are defined in `DRONES` in `config.py`; positioning
source, Vicon topic, radio override, and mission are supplied to `cfsingle`.

To add another single drone, add its default radio URI:

```python
DRONES = {
    'cf1': 'radio://0/80/2M/E7E7E7E7E7',
    'cf2': 'radio://0/80/2M/E7E7E7E7E6',
}
```

Then launch it explicitly:

```bash
cfsingle cf2 vicon /vicon/cf2/cf2 "" my_mission
```

Swarm profiles should be used when a reusable combination of multiple drone
IDs, topics, radios, mission, and simulation world is needed.

## Final authoring checklist

1. Use a unique lowercase `snake_case` filename and selection name.
2. Include the module docstring and required name/object exports.
3. Register the new file in the loader in the same folder.
4. Ensure swarm mission IDs exactly match the selected profile IDs.
5. Ensure the Gazebo model names exactly match the profile IDs.
6. Keep targets inside configured bounds and formations above minimum
   separation.
7. Return to a stable position above home before landing.
8. Run `cfbuild` and `cftest`.
9. Complete a headless Gazebo run before GUI or hardware testing.
10. Test hardware initially at conservative height, distance, and speed.
