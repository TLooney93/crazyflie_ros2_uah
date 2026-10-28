# R2 command-mapping test

Requirement: every recognized command must invoke the correct ROS 2 command,
and invalid inputs must invoke nothing. The pass gate is 30/30 for every
command and no action for all invalid inputs.

This test uses the real `wand_coordinator`, but replaces the drone with fake
services under the isolated ID `r2test`. It cannot command a physical drone.
Do not run `crazyflie_node`, `flight_wand_node`, or the physical wand button
node for this test.

The simplest method starts both isolated processes and cleans them up when the
test finishes:

```bash
bash scripts/test_r2_commands.sh
```

The equivalent manual two-terminal method is below.

In Ubuntu 24.04 terminal 1:

```bash
cd "/mnt/c/Users/Bengy/OneDrive/Desktop/Senior Design/crazyflie_ros2_uah"
source scripts/use_jazzy.bash
python3 -m cf_control.wand_coordinator --ros-args -p drone_id:=r2test
```

In terminal 2:

```bash
cd "/mnt/c/Users/Bengy/OneDrive/Desktop/Senior Design/crazyflie_ros2_uah"
source scripts/use_jazzy.bash
python3 -m cf_control.r2_command_test
```

The harness sends 30 each of:

- `takeoff` (after an `enable` control event)
- `land`
- `emergency_stop`
- `invalid_command`

It records the exact fake service received and the command-to-service latency.
The invalid input passes only if no service is called during the observation
window. Results are saved under `output/test_results/`.
