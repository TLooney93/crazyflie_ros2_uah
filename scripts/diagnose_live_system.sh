#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

echo "=== NODES ==="
ros2 node list
echo "=== REQUIRED TOPICS ==="
ros2 topic list -t | grep -E '^/(vicon/cf1/pose|wand/pose|wand/gesture|wand/detected_gesture|cf1/position)' || true
echo "=== CF1 SERVICES ==="
ros2 service list | grep -E '^/cf1/(takeoff|land|emergency_stop)' || true
echo "=== VICON PUBLISHERS ==="
ros2 topic info /vicon/cf1/pose --verbose || true
echo "=== WAND POSE PUBLISHERS ==="
ros2 topic info /wand/pose --verbose || true
echo "=== GESTURE PUBLISHERS ==="
ros2 topic info /wand/gesture --verbose || true
