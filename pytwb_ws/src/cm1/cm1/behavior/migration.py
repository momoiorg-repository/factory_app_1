import time

from lib.actor_bt import ActorBT
from pytwb.common import behavior
from ros_actor import run_actor

import py_trees
from threading import Semaphore

@behavior
class Approach(ActorBT):
    desc = 'approach target by visual feedback'

    def __init__(self, name, node, target=0.20):
        super().__init__(name, 'approach', target)
    
    def initialise(self):
        time.sleep(1)
        super().initialise()

@behavior
class Mini_Walk(ActorBT):
    desc = 'approach target by specified time'

    def __init__(self, name, node, target=1):
        super().__init__(name, 'mini_walk', target)

@behavior
class Shift(ActorBT):
    desc = 'approach target by specified len'

    def __init__(self, name, node, target=0.006):
        super().__init__(name, 'shift', target)

@behavior
class Face(ActorBT):
    desc = 'direct face to the coke can'

    def __init__(self, name, node):
        super().__init__(name, 'face')
    

@behavior
class MatchAngleGoal(ActorBT):
    desc = 'adjust angle to goal'

    def __init__(self, name, goal="rack_workpiece"):
        super().__init__(name, goal)
        self.goal = goal

    def initialise(self):
        run_actor("match_angle_goal", self.goal)

    def update(self):
        return py_trees.common.Status.SUCCESS

@behavior
class ApproachAction(ActorBT):
    desc = 'approach action'

    def __init__(self, name, flag):
        super().__init__(name, flag)

    def initialise(self):
        bb = py_trees.blackboard.Blackboard()
        self.bb = bb
        base_x = self.bb.get("base_x")
        base_y = self.bb.get("base_y")
        base_z = self.bb.get("base_z")
        run_actor("approach_action", (base_x, base_y, base_z))

    def update(self):
        return py_trees.common.Status.SUCCESS

@behavior
class MiniWalk(ActorBT):
    desc = "mini walk"

    def __init__(self, name, node, time, back, high_speed):
        super().__init__(name, "mini_walk_times", time, back, high_speed)

@behavior
class FaceBodyGoal(ActorBT):
    desc = "Adjust the robot's angle"

    def __init__(self, name, node, goal):
        super().__init__(name, "face_body_goal", goal)