#!/usr/bin/env bash
# Watch the virtual car: the world view (the developer window, live) on the one running here.
#
#   pixi run virtualcar       # first, in its own terminal: everything the car provides
#   pixi run ravensim         # then: the world view, watching it (no simulator of its own)
#   pixi run ravensim --screenshot FILE    # any other argument goes to the window
#
# Ben, 26 Nov: "Maybe we need a ravensim that is just the monitoring everything and displays
# the results. THats it." So it starts no simulator: until 26 Nov it started its own (with
# sim.launch.py's switches); now pixi run virtualcar does that, and a switch given here
# (name:=value) is refused with where it goes instead.
#
# The window is tools/dev_window.sh --live: the scene and console left, the lidar, the map
# and the keys right, on the running world's own maps (the campus: its NDT map, RavenTiles
# and lanes). --live may send a goal (a click) and set the e-stop, only while every sensor
# says SIM; nothing else here writes to ROS. On this computer only (tools/ros.sh --local).
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -u
cd "$(dirname "$0")/.."

for arg in "$@"; do
  case "$arg" in
    *:=*)
      echo "pixi run ravensim only watches (since 26 Nov): give $arg to the virtual car," >&2
      echo "  pixi run virtualcar $arg" >&2
      exit 2
      ;;
  esac
done

world=$(tools/ros.sh --local ros2 param get /simulator world 2>/dev/null \
  | sed -n 's/^String value is: //p' | sed 's#.*/##; s#\.sdf$##')
if [ -z "$world" ]; then
  echo "No virtual car running on this computer (no /simulator answered)." >&2
  echo "Start it first, in its own terminal: pixi run virtualcar" >&2
  exit 2
fi
echo "watching the virtual car: world $world"
AAV_WORLD=$world exec tools/dev_window.sh --live "$@"
