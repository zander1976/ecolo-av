"""
The simulator's named set-ups: a few of sim.launch.py's switches under one name.

No ROS. docs/wrap-up.md section 3: "sim.launch.py's switches become a short list of named
set-ups (as stacks.yaml does for algorithms)". stacks.yaml names the algorithms; a set-up
names the world around them.

    pixi run virtualcar             = sim.launch.py setup:=car (the virtual car; pixi run
                                      sim is its old name)
    pixi run virtualcar world:=road the car's set-up, in the road world: yours wins
    ros2 launch aav_bringup sim.launch.py      no set-up: each switch's own default

| Set-up | Switches | What it is |
|---|---|---|
| (none) | world:=empty esp32:=false | the launch file's defaults, as |
|        | perception:=false detector_impl:=python | every test starts it |
| car | world:=campus esp32:=true | everything the car provides, on Carleton's |
|     | perception:=true detector_impl:=rust | campus (the street map's place): every |
|     |                         | topic the car has (pixi run check virtual) |

The car's set-up was world:=road until 26 Nov (Ben, 26 Nov: "create a consinstent map
between all 3"): the campus is the one world the simulator, the world view and the map
window all draw the same.

The switches a set-up may set are declared empty in sim.launch.py (CHOSEN), so one
given by hand is never mistaken for "not given": world:=empty wins over setup:=car.
"""

from __future__ import annotations

from typing import Dict, Mapping

from aav_description.vehicle import DEFAULT_DETECTOR_IMPL

NONE = ''        # setup:= empty: no set-up, each switch's own default
CHOSEN = ''      # a set-up switch's declared default: "the set-up's choice"
CAR = 'car'      # what pixi run sim passes (Ben, 25 Nov: "pixi run sim" is the virtual car)

# What each switch is with no set-up and nothing given by hand.
FALLBACK = {'world': 'empty', 'esp32': 'false', 'perception': 'false',
            'detector_impl': DEFAULT_DETECTOR_IMPL}

SETUPS = {
    CAR: {'what': 'everything the car provides: the campus, the simulated ESP32 and the '
                  'stop-sign detector (Rust, ONNX), so every topic the car has (pixi run '
                  'check virtual)',
          'world': 'campus', 'esp32': 'true', 'perception': 'true', 'detector_impl': 'rust'},
}


def names() -> tuple:
    """Return the set-ups' names."""
    return tuple(SETUPS)


def switches(setup: str) -> Dict[str, str]:
    """Return the switches a set-up sets (none for NONE); ValueError for an unknown one."""
    setup = setup.strip().lower()
    if setup == NONE:
        return {}
    if setup not in SETUPS:
        raise ValueError(f'setup:={setup}: pick one of {", ".join(names())}, or none')
    return {k: v for k, v in SETUPS[setup].items() if k != 'what'}


def resolve(setup: str, given: Mapping[str, str]) -> Dict[str, str]:
    """
    Return each set-up switch's value: given by hand, else the set-up's, else FALLBACK.

    given: the switches as the launch file read them; empty (CHOSEN) means not given.
    """
    picked = switches(setup)
    return {name: (given.get(name) or CHOSEN).strip() or picked.get(name, fallback)
            for name, fallback in FALLBACK.items()}


def describe(setup: str, values: Mapping[str, str]) -> str:
    """Return one line for the launch's log: the set-up and what it gave."""
    shown = ' '.join(f'{k}:={values[k]}' for k in FALLBACK)
    if setup.strip() == NONE:
        return f'no set-up: {shown}'
    return f'setup:={setup.strip().lower()}: {shown}'
