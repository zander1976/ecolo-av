"""
Whole drives on the pretend car (ego -> Gazebo's actuator model): stop at B, brake gently.

The same targets as tools/sim_check.py --drive-to-point: 10, 30 and 60 m, straight, to the side and
behind; at the car's 1 m/s and at Gazebo's sim_speed 3.
"""

import dataclasses
import math

from aav_drive_to_point.approach import Approach, planned_deceleration, STOPPING, TURN
from aav_drive_to_point.drive import Command, Drive, Steering
from aav_drive_to_point.line import Fence, min_radius, Pose
from aav_ego.ackermann import Limits
from aav_gazebo.actuator_model import ActuatorConfig
from fake_car import drive_to
import pytest

CFG = ActuatorConfig()                 # vehicle.yaml's actuators (the GUESSES, 15 Oct)
DECEL = planned_deceleration(CFG.max_brake)
R = min_radius(1.57, CFG.max_steering)
LEAD = 0.8                             # config/drive_to_point.yaml's brake_lead
TARGETS = [(10.0, 0.0), (30.0, 45.0), (60.0, -30.0), (30.0, 150.0)]   # m, degrees left


class Fenced:
    """A Drive checked against its fence every tick, as point_driver does; stops on a breach."""

    def __init__(self, drive, fence, flip=False):
        self.drive, self.fence, self.flip = drive, fence, flip
        self.breach = None

    def tick(self, pose, measured, dt):
        self.breach = self.breach or self.fence.breach((pose.x, pose.y), self.drive.line)
        if self.breach:
            return Command(0.0, 0.0, STOPPING, 0.0, 0.0)
        cmd = self.drive.tick(pose, measured, dt)
        # flip: a steering sign the wrong way round (legacy.yaml steering_sign, unchecked).
        return dataclasses.replace(cmd, curvature=-cmd.curvature) if self.flip else cmd


def go(distance, degrees, cruise, start=Pose(0.0, 0.0, 0.0), flip=False):
    a = start.yaw + math.radians(degrees)
    target = (start.x + distance * math.cos(a), start.y + distance * math.sin(a))
    plan = Approach(cruise=cruise, accel=0.5, decel=DECEL, lead=LEAD)
    fenced = []

    def make(s):
        fenced.append(Fenced(Drive(target, plan, Steering(max_curvature=1.0 / R), s),
                             Fence((s.x, s.y), target, turn_radius=R), flip))
        return fenced[0]
    run = drive_to(make, start=start, cfg=CFG,
                   limits=Limits(max_speed=cruise, max_acceleration=1.0, max_deceleration=2.0))
    run.breach = fenced[0].breach
    return target, run


@pytest.mark.parametrize('cruise', [1.0, 3.0])
@pytest.mark.parametrize('distance,degrees', TARGETS)
def test_it_stops_at_b_without_braking_hard(distance, degrees, cruise):
    target, run = go(distance, degrees, cruise)
    assert run.breach is None              # a normal drive never touches the fence
    _, x, y, yaw, *_ = run.end
    assert math.hypot(target[0] - x, target[1] - y) < 0.3
    # No overshoot: it never passes B by more than a few centimetres.
    assert (x - target[0]) * math.cos(yaw) + (y - target[1]) * math.sin(yaw) < 0.15
    # Never harder than the plan (with a margin for the speed loop), never backwards.
    assert max(d for _, d in run.decelerations()) < DECEL * 1.25
    assert min(r[4] for r in run.rows) > -0.01


def test_it_slows_at_about_the_planned_deceleration():
    _, run = go(60.0, 0.0, 3.0)
    slowing = [d for _, d in run.decelerations() if d > 0.3]
    assert slowing and DECEL * 0.6 < max(slowing) <= DECEL * 1.25


def test_behind_it_turns_round_first():
    _, run = go(30.0, 150.0, 1.0)
    phases = [r[6] for r in run.rows]
    assert phases[0] == TURN and TURN not in phases[-10:]


def test_a_start_facing_any_way():
    _, run = go(30.0, 0.0, 1.0, start=Pose(5.0, -3.0, 2.0))
    target = (5.0 + 30.0 * math.cos(2.0), -3.0 + 30.0 * math.sin(2.0))
    assert math.hypot(target[0] - run.end[1], target[1] - run.end[2]) < 0.3


def test_the_fence_stops_a_car_that_steers_the_wrong_way():
    # B 15 degrees off the nose: it follows the line at once, steering away from it.
    _, run = go(30.0, 15.0, 1.0, flip=True)
    assert run.breach is not None and 'of the line to B' in run.breach
    _, x, y, *_ = run.end
    assert math.hypot(x, y) < 30.0          # stopped well before it got anywhere
