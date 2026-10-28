# CrazyConductors ROS 2 control stack

University of Alabama in Huntsville senior design project for commanding
Crazyflie vehicles with a wand. Vicon is the external-position source; the
older Lighthouse modules remain only as historical code and are not used by
the active flight node.

## Active software path

1. `vicon_datastream_bridge` reads Tracker's official DataStream output and
   maps `cf1` to `/vicon/cf1/pose` and `cf2` to `/wand/pose`.
2. `crazyflie_node.py` validates that pose, sends it to the onboard Kalman
   estimator, and publishes `/<drone_id>/position`.
3. Takeoff is accepted only when actuation was explicitly enabled and Vicon
   has supplied a stable startup window.
4. `wand_button_node.py` or the keyboard `wand_node.py` publishes normalized
   gestures on `/wand/gesture`.
5. `wand_coordinator.py` maps gestures to guarded takeoff, landing,
   emergency-stop, and bounded movement commands.
6. `flight_controller.py` computes bounded world-frame velocity setpoints.

The flight node defaults to `enable_actuation:=false`. Starting it cannot take
off or send motor commands. Enable actuation only during a cleared, supervised
flight test with an independent emergency-stop procedure.

## Important files

- `cf_control/config.py`: radio addresses and conservative control constants
- `cf_control/crazyflie_interface.py`: small wrapper around cflib commands
- `cf_control/crazyflie_node.py`: Vicon, state machine, services, watchdog
- `cf_control/vicon.py`: pure pose validation and startup readiness logic
- `cf_control/wand_coordinator.py`: gesture-to-flight bridge
- `cf_control/wand_button_node.py`: reads custom firmware button log variables
- `cf_control/flight_wand_node.py`: runs `cf1` and `cf2` through one shared
  Crazyradio process (required when only one dongle is available)
- `firmware/wand_buttons/`: Bitcraze app-layer button example
- `src/vicon_datastream_bridge/`: official-SDK Vicon-to-ROS pose bridge
- `scripts/run_vicon_bridge.sh`: one-command Jazzy bridge launcher
- `test/`: software-only unit tests

See `output/pdf/crazyflie_vicon_operations_guide.pdf` for complete terminal,
Vicon, flight-test, troubleshooting, and firmware instructions.
