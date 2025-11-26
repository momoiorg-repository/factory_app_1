import math
from math import radians, atan2, degrees
import random

from ros_actor import SubNet, actor

from ..print_color import cprint

class TaskFlow(SubNet):
    @actor
    def demo1(self):
        # 1. Move to workpiece rack
        self.run_actor('goto_pos', 'rack_workpiece')
        self.run_actor('match_angle_goal', 'rack_workpiece')
        self.run_actor('home', 130)
        self.run_actor('open')

        # 2. target determination
        target = self.run_actor('determine_target')
        if not target:
            cprint('No target found', 'red')
            return False
        label, score, cx, cy = target
        #  target coordinates change
        camera_x, camera_y, camera_z = self.run_actor('trans_camera_coordinates', cx, cy)
        base_x, base_y, base_z = self.run_actor('trans_base_coordinates', camera_x, camera_y, camera_z, "camera_color_optical_frame")
        cprint(f"Target base coordinates: ({base_x:.3f}, {base_y:.3f}, {base_z:.3f})", 'green')

        # 3. Approach the target
        self.run_actor("approach_action", (base_x, base_y, base_z))  # close to the target
        self.run_actor('home', 110)

        # 4. Fine tuning
        target = self.run_actor('determine_target')
        if not target:
            cprint('No target found', 'red')
            return False
        label, score, cx, cy = target
        camera_x, camera_y, camera_z = self.run_actor('trans_camera_coordinates', cx, cy)
        base_x, base_y, base_z = self.run_actor('trans_base_coordinates', camera_x, camera_y, camera_z, "camera_color_optical_frame")

        # 5. Pick up the target
        self.run_actor("add_scene_object", position=[base_x, base_y, base_z-0.02])  # add object to planning scene
        self.run_actor('sleep', 5)
        self.run_actor("move_to_pose", position=[base_x, base_y, base_z], quat_xyzw=[1, 0, 0, 0])  # move to the target
        self.run_actor('sleep', 1)
        self.run_actor('close')
        self.run_actor("remove_scene_object")  # remove object from planning scene
        self.run_actor('sleep', 1)
        self.run_actor('home', 90)

        # 6. Move to Machining Center
        self.run_actor("mini_walk_times", 50, back=True, high_speed=True)  # move backward
        self.run_actor("face_body_goal", 'machining_center')
        self.run_actor('goto_pos', 'machining_center')
        self.run_actor('match_angle_goal', 'machining_center')

        # 7. Place the target
        map_x, map_y, _ = self.run_actor('get_goal_pos', 'machining_center', True)  # return machining center position on map
        map_z = 0.89  # height of machining center's chuck
        base_x, base_y, base_z = self.run_actor('trans_base_coordinates', map_x, map_y, map_z, "map")
        self.run_actor('approach_action', (base_x, base_y, base_z))
        self.run_actor('match_angle_goal', 'machining_center')
        # self.run_actor('move_joint', radians(0), radians(0), radians(0), radians(-135), radians(0), radians(132), radians(45))  # move to place position
        map_x, map_y, _ = self.run_actor('get_goal_pos', 'machining_center', True)
        map_z = 0.92  # height of machining center's chuck
        base_x, base_y, base_z = self.run_actor('trans_base_coordinates', map_x, map_y, map_z, "map")
        quat_list = self.run_actor('euler_to_quat', roll=0, pitch=180, yaw=-90)
        self.run_actor('move_to_pose', position=[base_x, base_y, base_z], quat_xyzw=quat_list)


    @actor
    def demo2(self):
        self.run_actor('demo1')
        self.run_actor('sleep', 15)
        self.run_actor('home', 90)
        #2. Move to rack product
        self.run_actor('mini_walk_times', 160, back=True, high_speed=True)
        self.run_actor('face_body_goal', 'rack_product')
        self.run_actor('goto_pos', 'rack_product')
        self.run_actor('match_angle_goal', 'rack_product')
        #3.
        map_x, map_y, _ = self.run_actor('get_goal_pos', 'rack_product', True)
        map_z = 0.92
        base_x, base_y, base_z = self.run_actor('trans_base_coordinates', map_x, map_y, map_z, "map")
        self.run_actor('approach_action', (base_x, base_y, base_z))
        self.run_actor('place_product')
        # self.run_actor('move_to_pose', position=[], quat_xyzw=[1, 0, 0, 0])
        self.run_actor('open')
        self.run_actor('home', 90)
