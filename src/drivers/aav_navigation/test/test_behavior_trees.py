"""
Check the car's behaviour trees: forward-only and reversing twins, and no spinning.

Each navigator has two trees: *_car.xml (forward only) and *_car_reversing.xml (with
BackUp). They must stay the same apart from BackUp, and BackUp must stay inside the car's
reverse speed (vehicle.yaml). No ROS needed.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
TREES = PACKAGE / 'behavior_trees'
VEHICLE = PACKAGE.parents[1] / 'aav_description' / 'config' / 'vehicle.yaml'
NAV2 = PACKAGE / 'config' / 'nav2.yaml'
NAVIGATORS = ['navigate_to_pose', 'navigate_through_poses']


def tree(name):
    return ET.parse(TREES / name).getroot()


def shape(element):
    """Return the tree as nested tuples: tag, attributes, children (comments dropped)."""
    return (element.tag, sorted(element.attrib.items()),
            [shape(child) for child in element])


def without_backup(element):
    for parent in element.iter():
        for child in list(parent):
            if child.tag == 'BackUp':
                parent.remove(child)
    return element


@pytest.mark.parametrize('navigator', NAVIGATORS)
def test_the_twins_differ_only_by_backup(navigator):
    forward = tree(f'{navigator}_car.xml')
    reversing = tree(f'{navigator}_car_reversing.xml')
    assert not list(forward.iter('BackUp'))
    assert len(list(reversing.iter('BackUp'))) == 1
    assert shape(without_backup(reversing)) == shape(forward)


@pytest.mark.parametrize('navigator', NAVIGATORS)
def test_backing_up_stays_inside_the_reverse_speed(navigator):
    limit = yaml.safe_load(VEHICLE.read_text())['vehicle']['max_reverse_speed']
    (backup,) = tree(f'{navigator}_car_reversing.xml').iter('BackUp')
    assert 0.0 < float(backup.get('backup_speed')) <= limit
    assert 0.0 < float(backup.get('backup_dist')) <= 2.0


@pytest.mark.parametrize('name', sorted(p.name for p in TREES.glob('*.xml')))
def test_no_tree_spins(name):
    # A car can't turn on the spot.
    assert not list(tree(name).iter('Spin'))


def test_the_behavior_server_can_back_up():
    plugins = yaml.safe_load(NAV2.read_text())['behavior_server']['ros__parameters']
    assert 'backup' in plugins['behavior_plugins']
    assert 'spin' not in plugins['behavior_plugins']


TO_POSE = ['navigate_to_pose_car.xml', 'navigate_to_pose_car_reversing.xml']


@pytest.mark.parametrize('name', TO_POSE)
def test_no_replanning_on_the_final_approach(name):
    # TODO 17: near the goal a new path was a loop for a car, and reset the goal checker.
    # The tree keeps its path there (unless blocked, for another goal, or 30 s old).
    # Inside a RateController: a Fallback restarts its children, and a RateController
    # under it planned on every tick, not once a second (25 Oct).
    (guard,) = [f for r in tree(name).iter('RateController') for f in r
                if f.tag == 'Fallback' and f.get('name') == 'PlanUnlessOnTheFinalApproach']
    assert not list(guard.iter('RateController'))
    keep, plan = list(guard)
    assert list(keep.iter('IsPathValid'))          # a blocked path is planned again
    assert list(keep.iter('PathExpiringTimer'))    # and one it can't follow, in time
    assert list(plan.iter('ComputePathToPose'))
    # The zone must leave room for an S-bend back onto the line (two turns at the
    # planner's radius), and be wider than the goal checker's circle.
    zone = float(next(n for n in keep.iter('ArePosesNear')
                      if n.get('ref_pose') == '{current_pose}').get('tolerance'))
    nav2 = yaml.safe_load(NAV2.read_text())
    checker = nav2['controller_server']['ros__parameters']['general_goal_checker']
    radius = nav2['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
    assert zone >= 2.0 * radius
    assert zone >= 3.0 * checker['xy_goal_tolerance']


def test_the_map_window_waits_longer_than_nav2_keeps_a_path():
    # The big screen's map shows NO NEW PLAN when /plan is older than plan_stale_after: it
    # must wait longer than the tree keeps a path, or it cries wolf (TODO 17).
    longest = max(float(t.get('seconds')) for name in TO_POSE
                  for t in tree(name).iter('PathExpiringTimer'))
    window = PACKAGE.parents[1] / 'hud' / 'aav_hud_raylib' / 'config' / 'map.yaml'
    stale = yaml.safe_load(window.read_text())['map']['ros__parameters']['plan_stale_after']
    assert stale > longest
