#!/usr/bin/env bash
# Only run after freshly spawning TurtleBot 4 at home, and before running FSM.
set -euo pipefail
source "$HOME/mrs_tray_ws/tray_env.sh"
WORLD=milestone2_multi_robot_transfer
SCENE="/world/$WORLD/scene/info"
BLOCK_FILE="$(ros2 pkg prefix --share mrs_tray_demo)/models/pick_block.sdf"

# Query the known world directly: `gz model -l` can time out on /gazebo/worlds.
scene="$(gz service -s "$SCENE" --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 8000 --req '')"
if [[ -z "$scene" ]]; then
  echo "ERROR: no response from Gazebo scene/info ($SCENE)" >&2
  exit 1
fi
if [[ "$scene" == *'name: "standard_dock"'* ]]; then
  echo '[scene] Removing charging dock model standard_dock'
  gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req 'name: "standard_dock", type: MODEL'
else
  echo '[scene] standard_dock is already absent'
fi
if [[ "$scene" == *'name: "pick_block"'* ]]; then
  echo '[scene] pick_block already exists; NOT spawning duplicate or changing its pose'
  echo '[scene] For a fresh take, reset the robot and block to the initial scene deliberately.'
else
  if [[ ! -f "$BLOCK_FILE" ]]; then
    echo "ERROR: missing block model $BLOCK_FILE" >&2
    exit 1
  fi
  echo '[scene] Spawning orange block on initial TurtleBot tray near (-3.0,-1.6)'
  ros2 run ros_gz_sim create -world "$WORLD" \
    -file "$BLOCK_FILE" -name pick_block \
    -x -2.97 -y -1.60 -z 0.40
fi
echo '[scene] Visually verify the orange block rests on the mobile tray before running the FSM.'
