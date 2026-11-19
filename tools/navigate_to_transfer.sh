#!/usr/bin/env bash
# Send the recorded map-frame goal to Nav2; only run in the intended simulation.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${1:-$ROOT/config/transfer_goal.yaml}"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found; source ~/mrs_tray_ws/tray_env.sh first." >&2
  exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "Goal config missing: $CONFIG" >&2
  exit 1
fi
GOAL=$(python3 - "$CONFIG" <<'PYGOAL'
import sys, math, re, json
from pathlib import Path
p = Path(sys.argv[1])
fields={}
for line in p.read_text().splitlines():
    m=re.fullmatch(r'\s*(frame_id|x|y|yaw_rad):\s*([^#]+?)\s*(?:#.*)?', line)
    if m:
        fields[m.group(1)] = m.group(2).strip().strip("'\"")
if not all(k in fields for k in ('frame_id','x','y','yaw_rad')):
    raise SystemExit('Config must have frame_id, x, y, yaw_rad')
frame=fields['frame_id']
if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_/]*',frame):
    raise SystemExit('Invalid ROS frame_id')
x,y,yaw=(float(fields[k]) for k in ('x','y','yaw_rad'))
if not all(math.isfinite(v) for v in (x,y,yaw)):
    raise SystemExit('Non-finite coordinates are invalid')
qz,qw=math.sin(yaw/2),math.cos(yaw/2)
print(json.dumps({"pose": {"header": {"frame_id": frame}, "pose": {"position": {"x": x, "y": y, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}}}}))
PYGOAL
)
echo 'Sending Nav2 goal (map frame); verify the receiving station is clear.'
echo "$GOAL"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL" --feedback
