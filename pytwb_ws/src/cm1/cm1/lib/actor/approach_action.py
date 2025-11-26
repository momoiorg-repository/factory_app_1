from math import radians, degrees, atan2
import math

from ros_actor import actor, SubNet
from ..pointlib import PointEx
from geometry_msgs.msg import Twist

from ..print_color import cprint

SPEED = 0.1
HIGH_SPEED = 0.2
# TURN = 0.5
TURN = 1.0

class ApproachAction(SubNet):
    ##################
    # Direct control
    ##################
    def move(self, dir=1, turn=0, high_speed = False):
        twist = Twist()
        twist.linear.x = (HIGH_SPEED if high_speed else SPEED) * dir
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = TURN * turn
        self.run_actor('motor', twist)
    
    # direct command to motor
    @actor    
    def mini_walk(self, len=5, high_speed = False):
        if len == 0:
            self.move(0)
            return
        if len > 0:
            self.move(1, high_speed=high_speed)
        else:
            self.move(-1, high_speed=high_speed)
            len = -len
        self.run_actor('sleep', len*0.1)
        self.move(0, 0)
    
    @actor
    def mini_walk_times(self, times=5, back=False, high_speed = False):
        for _ in range(times):
            if back:
                self.run_actor('mini_walk', -5, high_speed=high_speed)
            else:
                self.run_actor('mini_walk', high_speed=high_speed)
    
    @actor    
    def mini_turn(self, len=5):
        if len == 0:
            self.move(0, 0)
            return
        if len > 0:
            self.move(0, 1)
        else:
            self.move(0, -1)
            len = -len
        self.run_actor('sleep', len*0.1)
        self.move(0)

    ##########################
    # Use Navigation and Map
    ##########################
    @actor
    def goto_pos(self, pos="rack_workpiece"):
        """
        Move to a predetermined coordinate
        arg:
            pos: position name in the map
        return: 
        """
        cprint(f"Go to {pos}!", "green")
        x, y, ang = self.run_actor("get_goal_pos", pos)
        self.run_actor("goto", x, y, ang)

    @actor
    def match_angle_goal(self, goal="rack_workpiece"):
        """
        Adjust the robot's angle to the goal position
        arg:
            goal: position name in the map
        return:
        """
        _, _, goal_ang = self.run_actor("get_goal_pos", goal)
        goal_ang = (goal_ang + math.pi) % (2 * math.pi) - math.pi

        # for _ in range(5):
        while True:
            _, _, now_ang = self.run_actor("get_position")
            now_ang = (now_ang + math.pi) % (2 * math.pi) - math.pi

            rad = (goal_ang - now_ang + math.pi) % (2 * math.pi) - math.pi

            # cprint(f"Now angle: {now_ang:.3f}, Goal angle: {goal_ang:.3f}, Diff angle: {rad:.3f}", "yellow")

            if abs(rad) < 0.02:  # ≈1°
                self.move(0, 0) 
                cprint(f"Angle adjustment complete!", "green")
                return

            if rad > 0:
                self.run_actor('mini_turn', 5)
            else:
                self.run_actor('mini_turn', -5)

    @actor
    def approach_action(self, target_base_coords: tuple):
        """
        Approach the target using base coordinates
        arg:
            target_base_coords: (x, y, z) coordinates in the base frame
        return:
        """
        base_x, base_y, base_z = target_base_coords
        map_x, map_y, map_z = self.run_actor('trans_map_coordinates', base_x, base_y, base_z, 'base_link')  # Change to map coordinates

        target_distance = base_x
        while True:
            # if target_distance < base_x / 2 + 0.3:
            if target_distance < 0.95:
                cprint(f"Close to target complete! target distance: {target_distance:.3f}m", "green")
                return True

            self.run_actor('mini_walk')

            point = PointEx(map_x, map_y, map_z)
            trans = self.run_actor('base_trans', 'map')
            point.setTransform(trans.transform)
            target_distance, base_y, base_z = point.x, point.y, point.z
            cprint(f"Distance to target: {target_distance:.3f}m", "yellow")

    @actor
    def face_body_goal(self, goal="rack_workpiece"):
        """
        Adjust the robot's angle to face the goal position
        arg:
            goal: position name in the map
        return:
        """
        cprint(f"Face to {goal}!", "green")
        goal_x, goal_y, _ = self.run_actor("get_goal_pos", goal)

        while True:
            now_x, now_y, now_ang = self.run_actor("get_position")
            dx = goal_x - now_x
            dy = goal_y - now_y
            goal_ang = atan2(dy, dx)
            goal_ang = (goal_ang + math.pi) % (2 * math.pi) - math.pi
            now_ang = (now_ang + math.pi) % (2 * math.pi) - math.pi

            rad = (goal_ang - now_ang + math.pi) % (2 * math.pi) - math.pi

            # cprint(f"Now angle: {now_ang:.3f}, Goal angle: {goal_ang:.3f}, Diff angle: {rad:.3f}", "yellow")

            if abs(rad) < 0.02:  # ≈1°
                self.move(0, 0) 
                cprint(f"Angle adjustment complete!", "green")
                return

            if rad > 0:
                self.run_actor('mini_turn', 5)
            else:
                self.run_actor('mini_turn', -5)

    ####################################################################    
    
    # adjust location
    @actor
    def shift(self, target):
        _, _, _, distance = self.run_actor('measure_center')
        len = distance - target
        if len < 0.005: return True
        trans = self.run_actor('map_trans')
        start = PointEx(0.0, 0.0)
        start.setTransform(trans.transform)
        dest = PointEx(len, 0.0)
        dest.setTransform(trans.transform)
        self.run_actor('mini_walk', len*164)
        self.run_actor('sleep', 3)
        trans = self.run_actor('map_trans')
        actual = PointEx(0.0, 0.0)
        actual.setTransform(trans.transform)
        dx = actual.x - start.x
        dy = actual.y - start.y
        dir = atan2(dy, dx)
        self.run_actor('goto', actual.x, actual.y, dir)
        return True        
    
    # approach target with visual feedback
    @actor
    def approach(self, target=0.20):
        trans = self.run_actor('map_trans')
        start = PointEx(0.0, 0.0)
        start.setTransform(trans.transform)
        self.move(1)
        with self.run_actor_mode('measure_distance', 'iterator', 'bar') as obj_it:
            for distance in obj_it:
                if distance <= target: break 
        self.move(0)
        self.run_actor('sleep', 5)
        trans = self.run_actor('map_trans')
        actual = PointEx(0.0, 0.0)
        actual.setTransform(trans.transform)
        dx = actual.x - start.x
        dy = actual.y - start.y
        dir = atan2(dy, dx)
        self.run_actor('goto', actual.x, actual.y, dir)
    
    # adjust body angle
    @actor
    def face(self, raw=0):
        root = PointEx()
        trans = self.run_actor('map_trans', 'base_link')
        if not trans:
            print('trans error')
            return False
        root.setTransform(trans.transform)
        x, y, rel_angle = self.run_actor('object_loc', 'base_link')
        target = PointEx(x, y)
        target.setTransform(trans.transform)
        dx = target.x - root.x
        dy = target.y - root.y
        abs_angle = atan2(dy, dx)
        dangle = degrees(rel_angle)
        dangle *= 0.985
        self.run_actor('mini_turn', dangle)
        self.run_actor('goto', root.x, root.y, abs_angle)
        '''
        print(f'raw x:{x}, y:{y}')
        print(f'map x:{target.x}, y:{target.y}')
        print(f'root x:{root.x}, y:{root.y}')
        print(f'diff x:{dx}, y:{dy}')
        print(f'move:{degrees(abs_angle)}')
        '''
        return True
