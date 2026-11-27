#!/usr/bin/env bash
# pixi run ladder: the test ladder. With nothing after it, the tests window; with anything,
# tools/ladder.py's text, as always.
#
#   pixi run ladder                    the tests window (Bevy, ladder_window): the ten rungs,
#                                      the simulator and the car side by side, Run and Stop
#   pixi run ladder --pretend          the window with sample data
#   pixi run ladder table [--json]     the same table as text (what "pixi run ladder" printed
#                                      until 26 Nov)
#   pixi run ladder next|run|all|record|session|report ...   tools/ladder.py, unchanged
#
# Ben, 26 Nov: "why is the testing screen now a command prompt instead of a screen".
# tools/ladder.py stays the only writer: the window runs it for each table and each run.
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -eu
cd "$(dirname "$0")/.."

window() {
  # Build first, where people can see it: a silent rebuild (after a pull) looked like
  # "it did nothing" (Ben, 26 Nov). Bevy's first build takes 2-5 min; a rebuild ~40 s.
  echo "ladder: building the tests window if anything changed (cargo shows its progress) ..." >&2
  cargo build --release --manifest-path ros/hud/aav_dev_window/Cargo.toml --bin ladder_window
  echo "ladder: opening the tests window" >&2
  exec ros/hud/aav_dev_window/target/release/ladder_window "$@"
}

case "${1:-}" in
  "") window ;;
  --pretend) window "$@" ;;
  table) shift; exec python3 tools/ladder.py "$@" ;;
  *) exec python3 tools/ladder.py "$@" ;;
esac
