# Wand button firmware app

This Bitcraze app-layer example uses two active-low buttons. Wire each
normally-open button between its listed IO pin and GND:

- IO1 press: enable wand control
- IO1 release: disable wand control
- IO2 press: emergency stop after debounce, without waiting for release
- IO2 takes priority when both buttons are pressed; release both before resuming

IO1 must remain held to keep wand control enabled. After emergency stop, release
both buttons and press IO1 again to send a new enable request. There are no
button takeoff or landing actions; `wand.takeoff` and `wand.land` remain in the
log interface for compatibility but are always zero. Disable is not a motor
stop or landing command. These events are ROS requests, not direct onboard
flight commands; the coordinator and drone node must be running to act on them.
The release event depends on the radio link and is not a link-loss watchdog.

Inputs are debounced in firmware. Each recognized action is exposed as a
short log-variable pulse for the ROS button node. Wand movement still comes
from the motion/gesture logic, not from these buttons.

Build and flash only after checking the pin names against the exact deck and
firmware revision. See the repository's operations PDF for the supported
Toolbelt workflow. The emergency stop must also have a tested independent
operator path; do not rely on a radio button as the only safety mechanism.

## Crazyflie 2.1+ build

Clone `bitcraze/crazyflie-firmware` into the repository root so that it is at
`crazyflie_ros2_uah/crazyflie-firmware`, then run from this directory:

```bash
make clean
make cf2_defconfig
make -j$(nproc)
```

The selected platform must be `cf2`; do not use `bolt_defconfig` for a
Crazyflie 2.1+. The resulting binary is built below the firmware project's
`build` directory. Flash that compiled binary with Crazyradio using the
Crazyflie Client or Bitcraze's `cfloader` workflow.
