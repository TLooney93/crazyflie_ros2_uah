#!/usr/bin/env bash
set -eo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
source scripts/use_jazzy.bash

python3 -m cf_control.wand_coordinator --ros-args \
  -p drone_id:=r2test >/tmp/r2_wand_coordinator.log 2>&1 &
coordinator_pid=$!
cleanup() {
  kill -INT "$coordinator_pid" 2>/dev/null || true
  wait "$coordinator_pid" 2>/dev/null || true
}
trap cleanup EXIT

python3 -m cf_control.r2_command_test "$@"
