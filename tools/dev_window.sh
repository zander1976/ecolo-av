#!/usr/bin/env bash
# The developer window (ros/hud/aav_dev_window): pretend data, or live ROS.
#
#   pixi run dev-window                # pretend data: no ROS, no car, no build
#   pixi run dev-window --live         # beside demo or sim on THIS computer: it may send
#                                      # goals and set the e-stop, only while every sensor says SIM
#   pixi run dev-window --watch        # listens to the network it's on, such as the car's,
#                                      # plugged in. It has no publishers at all
#
#   pixi run nav-view [--live|--watch] # the same, as the navigation view: only the status
#                                      # strip and the map panel, in a small window of its own
#                                      # (this script with --map-only first)
#
# Any other arguments go to the program (--cubes N, --seconds S, --screenshot FILE, --map-span M,
# --title WORDS, --position X,Y...).
# --live and --watch need pixi run build first, for the message types.
#
# --live draws the running world's own maps (Ben, 26 Nov: "a consinstent map between all
# 3"): it asks the simulator on this computer which world it runs (its `world` parameter)
# and adds that world's files where they are:
#
#   | World     | NDT's map (--ndt-map)  | and                                          |
#   |-----------|------------------------|----------------------------------------------|
#   | campus    | maps/ndt/campus.pcd    | --chunks maps/campus/chunks, --lanes campus.osm |
#   | test_lot  | maps/ndt/test_lot.pcd  |                                              |
#   | road, empty, nothing heard | none  |                                              |
#
# AAV_WORLD=<name> says it instead of asking (no wait for the simulator).
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -euo pipefail
cd "$(dirname "$0")/.."

manifest=ros/hud/aav_dev_window/Cargo.toml

# The world the simulator on this computer runs: campus, test_lot, road, empty or a twin's
# name; empty if no simulator answers.
running_world() {
  if [ -n "${AAV_WORLD:-}" ]; then
    echo "$AAV_WORLD"
    return
  fi
  tools/ros.sh --local ros2 param get /simulator world 2>/dev/null \
    | sed -n 's/^String value is: //p' | sed 's#.*/##; s#\.sdf$##' || true
}

# The world view's maps for that world (the table above), as arguments.
world_maps() {
  case "$1" in
    campus)
      [ -f maps/ndt/campus.pcd ] && printf ' --ndt-map maps/ndt/campus.pcd'
      [ -d maps/campus/chunks ] && printf ' --chunks maps/campus/chunks'
      [ -f maps/campus/campus.osm ] && printf ' --lanes maps/campus/campus.osm'
      ;;
    test_lot)
      [ -f maps/ndt/test_lot.pcd ] && printf ' --ndt-map maps/ndt/test_lot.pcd'
      ;;
  esac
  return 0
}

# cargo runs quiet (-q): on a fresh clone the first build (Bevy) shows nothing for minutes.
if [ ! -x ros/hud/aav_dev_window/target/release/dev_window ]; then
  echo "dev_window: the first time, cargo builds the window (Bevy): about 2-5 min, no output" >&2
fi

# The navigation view (pixi run nav-view): the same program, only its strip and map panel.
only=""
if [ "${1:-}" = "--map-only" ]; then
  only="--map-only"
  shift
fi

case "${1:-}" in
  --live)
    shift
    world=$(running_world)
    maps=$(world_maps "$world")
    echo "dev_window: the world ${world:-(no simulator heard)}:${maps:- no maps of its own}" >&2
    # shellcheck disable=SC2086  # $maps is several words on purpose (no spaces in them)
    exec tools/ros.sh --local cargo run --release -q --manifest-path "$manifest" \
      --features ros -- $only --live --developer $maps "$@"
    ;;
  --watch)
    shift
    exec tools/ros.sh cargo run --release -q --manifest-path "$manifest" \
      --features ros -- $only --live "$@"
    ;;
  *)
    exec cargo run --release -q --manifest-path "$manifest" -- $only "$@"
    ;;
esac
