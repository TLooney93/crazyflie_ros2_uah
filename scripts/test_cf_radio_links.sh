#!/usr/bin/env bash
set -eo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
source scripts/use_jazzy.bash
python3 scripts/test_cf_radio_links.py
