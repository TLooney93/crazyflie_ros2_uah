# R1 gesture-recognition test

Requirement: recognize each vertical gesture with at least 95% accuracy.
The defined pass gate is at least 29 correct detections in 30 trials for each
gesture, repeated by at least two operators.

This is a dry test. Keep `enable_actuation:=false` for the entire session.

Run the Vicon bridge, wand button node, and gesture detector in separate
Ubuntu 24.04 terminals. Neither the drone controller nor coordinator is
required for R1, so `cf1` cannot receive a flight command during this test.

```bash
bash scripts/run_vicon_bridge.sh
```

```bash
source scripts/use_jazzy.bash
python3 -m cf_control.wand_button_node --ros-args -p wand_id:=cf2
```

```bash
source scripts/use_jazzy.bash
python3 -m cf_control.wand_gesture_node
```

Start the logger, substituting the operator's name or identifier:

```bash
source scripts/use_jazzy.bash
python3 -m cf_control.gesture_trial_logger --ros-args \
  -p operator:=operator1
```

For every prompt, press Enter, hold IO1, perform the requested vertical
gesture, and release IO1. Return the wand to a comfortable neutral position
before the next prompt. The sequence alternates between takeoff (raise) and
land (lower), for 30 trials each.

Results are written under `output/test_results/`. Run a separate session with
`operator:=operator2`. Each gesture passes only at 29/30 or better for both
operators. Keep failed trials in the CSV; do not repeat or discard them.
