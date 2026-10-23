#!/usr/bin/env bash
set -uo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
log_file=/tmp/vicon-datastream-test.log

VICON_TEST_SECONDS="${VICON_TEST_SECONDS:-3}" \
  bash "$script_dir/test_vicon_datastream.sh" >"$log_file" 2>&1
status=$?

echo "Vicon SDK test exit status: $status (124 means the timed test completed normally)"
echo "Captured lines: $(wc -l < "$log_file")"
echo
echo "Connection and tracked-object summary:"
grep -E -i -m 60 'connect|subject|segment|cf1|wand|frame number|frame rate' "$log_file" || true
echo
echo "First 20 output lines:"
head -n 20 "$log_file"

if [[ $status -ne 0 && $status -ne 124 ]]; then
  exit "$status"
fi
