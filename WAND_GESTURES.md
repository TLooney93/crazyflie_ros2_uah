# Vertical wand gestures

Use the `cf2-wand-hold-enable-estop.bin` firmware: IO1 press enables,
release disables, IO2 requests emergency stop. No new flash is needed for
gesture recognition if that firmware is already installed.

`wand_gesture_node.py` receives PoseStamped tracking and button events.
`vertical_gesture.py` recognizes world-Z displacement from the first valid
pose after enable: 20 cm upward requests takeoff; 20 cm downward requests
controlled landing. Three consecutive samples must exceed the threshold.
One command is emitted per hold. Release and press again for the next gesture.
This is displacement, not an orientation gesture; horizontal movement is ignored.

The default output `/wand/detected_gesture` is for observation only. It does
not feed the flight coordinator. Keep this default for initial tests.
Tracking must use meters, Z-up, frame `world`, and fresh, increasing ROS
timestamps. Invalid/stale poses disable recognition until another enable event.
The pose timeout defaults to 0.25 seconds. Button events currently use pulses;
a lost release event is not covered by a button-link watchdog. Do not treat
this implementation as a verified physical dead-man switch.

## Automated simulation (no radio or Vicon needed)

From the repository root in WSL:

```bash
bash firmware/test_wand_gestures.sh
```

The tests exercise upward/downward motion, noise, release, emergency stop,
one event per hold, stale tracking, and the ROS callback with synthetic poses.

## Observe actual tracking later

Activate the environment in each terminal:

```bash
source .venv/bin/activate
source /opt/ros/lyrical/setup.bash
export PYTHONPATH="$PWD/src/cf_control:$PYTHONPATH"
```

Start the button node in one terminal:

```bash
python3 -m cf_control.wand_button_node --ros-args -p wand_id:=cf2
```

Start the detector in another (replace the topic with the wand's actual topic):

```bash
python3 -m cf_control.wand_gesture_node --ros-args -p pose_topic:=/vrpn_mocap/cf2/pose
```

Observe detections in a third:

```bash
ros2 topic echo /wand/detected_gesture
```

The wand must be added as a separate tracked object in Vicon first. Do not
use the flying drone's pose as the wand input. This subscriber accepts
best-effort VRPN publishers. Confirm frame, scale, freshness and direction.

## Flight integration after physical verification

Set the detector's `output_topic` parameter to `/wand/gesture` to connect
recognized gestures to the existing coordinator. The coordinator must select
the intended flight drone, and its node must pass its existing takeoff guards.
The coordinator defaults to cf1 only; this change does not configure three drones.
Release disables future gesture recognition; it does not cancel a landing
already requested. Controlled landing uses the existing land service, not a
motor cut. Physical landing stability still requires lab verification.

Emergency stop now bypasses the coordinator's pending-service guard. This
allows the request to be sent immediately, but is not a guarantee of delivery
or execution order across separate ROS services and a radio link.
