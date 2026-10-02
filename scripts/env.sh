# From the workspace root: source src/bunker_gps_navigation/scripts/env.sh
BUNKER_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BUNKER_WS_ROOT=$(cd -- "$BUNKER_ROOT/../.." && pwd)
source /opt/ros/humble/setup.bash
if [[ -f "$BUNKER_WS_ROOT/install/setup.bash" ]]; then
  source "$BUNKER_WS_ROOT/install/setup.bash"
fi
