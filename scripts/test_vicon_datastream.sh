#!/usr/bin/env bash
set -eo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sdk_source="$repo_dir/output/vicon-sdk-linux64/Linux64"
sdk_dir=/tmp/cf-vicon-sdk-linux64
mkdir -p "$sdk_dir"
cp "$sdk_source"/ViconDataStreamSDK_CPPTest \
  "$sdk_source"/libViconDataStreamSDK_CPP.so "$sdk_dir/"
cd "$sdk_dir"
chmod +x ViconDataStreamSDK_CPPTest
timeout "${VICON_TEST_SECONDS:-8}s" env LD_LIBRARY_PATH=. \
  ./ViconDataStreamSDK_CPPTest "${VICON_HOST:-10.86.153.14}"
