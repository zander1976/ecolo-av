"""The simulator's named set-ups: setup:=car, no set-up, and a switch by hand winning."""

import os
import re

from aav_description import setups
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(HERE, '..', '..', 'aav_bringup', 'launch', 'sim.launch.py')
PIXI = os.path.join(HERE, '..', '..', '..', '..', 'pixi.toml')
NOT_GIVEN = {name: setups.CHOSEN for name in setups.FALLBACK}


OLD = {'world': 'empty', 'esp32': 'false', 'perception': 'false', 'detector_impl': 'python',
       'web': 'false'}
CAR = {'world': 'campus', 'esp32': 'true', 'perception': 'true', 'detector_impl': 'rust',
       'web': 'true'}


def test_no_setup_keeps_the_launch_files_old_defaults():
    assert setups.resolve('', NOT_GIVEN) == OLD


def test_car_is_the_campus_with_the_esp32_and_the_detector():
    """Ben, 26 Nov: everything the car provides, on one map the screens share."""
    assert setups.resolve('car', NOT_GIVEN) == CAR
    assert setups.resolve(' Car ', {}) == CAR


def test_a_switch_given_by_hand_wins_over_the_setup():
    assert setups.resolve('car', {'world': 'road', 'esp32': ''}) == dict(CAR, world='road')
    assert setups.resolve('car', {'world': 'empty', 'esp32': 'false', 'perception': 'false'}) \
        == dict(CAR, world='empty', esp32='false', perception='false')


def test_an_unknown_setup_is_refused_by_name():
    with pytest.raises(ValueError, match='setup:=moon: pick one of car'):
        setups.resolve('moon', NOT_GIVEN)


def test_every_setup_sets_only_switches_with_a_fallback():
    for name in setups.names():
        assert set(setups.switches(name)) <= set(setups.FALLBACK), name
        assert setups.SETUPS[name]['what'], name


def test_sim_launch_declares_the_setup_switches_empty():
    """A switch declared with a value could not be told apart from one given by hand."""
    with open(SIM_LAUNCH) as f:
        text = f.read()
    for name in setups.FALLBACK:
        assert re.search(rf"DeclareLaunchArgument\('{name}', default_value=setups.CHOSEN",
                         text), name


@pytest.mark.skipif(not os.path.isfile(PIXI), reason='no pixi.toml beside this package')
def test_pixi_run_virtualcar_is_the_virtual_car_and_sim_its_old_name():
    with open(PIXI) as f:
        lines = f.readlines()
    for task in ('virtualcar = ', 'sim = '):
        line = next(t for t in lines if t.startswith(task))
        assert f'setup:={setups.CAR}' in line, task


def test_describe_says_what_the_setup_gave():
    values = setups.resolve('car', NOT_GIVEN)
    assert setups.describe('car', values) == ('setup:=car: world:=campus esp32:=true '
                                              'perception:=true detector_impl:=rust web:=true')
    assert setups.describe('', setups.resolve('', NOT_GIVEN)).startswith('no set-up')
