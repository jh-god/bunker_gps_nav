#!/usr/bin/env bash
set -e
BUNKER_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BUNKER_WS_ROOT=$(cd -- "$BUNKER_ROOT/../.." && pwd)
source /opt/ros/humble/setup.bash
cd "$BUNKER_WS_ROOT"
colcon build --base-paths src --build-base build --install-base install \
  --parallel-workers "${BUNKER_BUILD_WORKERS:-2}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="${BUNKER_BUILD_TESTING:-OFF}" \
  --event-handlers console_direct+
