from math import radians, atan2, degrees, sqrt, isinf

from ros_actor import actor, SubNet
from lib.pointlib import PointEx, PointBag
from lib.simlib import find_coke

from ..print_color import cprint

class CognitiveNetwork(SubNet):
    # == Get Images ==
    # get depth image with same size as RGB image
    @actor
    def get_depth_image(self):
        data = self.run_actor('depth')
        cv_bridge = self.get_value('cv_bridge')
        depth_image = cv_bridge.imgmsg_to_cv2(data)
        return depth_image

    # == Use Yolo Detections ==
    # determine target object from detected objects
    @actor
    def determine_target(self):
        results = self.run_actor('detections_visualizer')
        if not results:
            cprint("No target found", "red")
            return False

        camera_info = self.get_value('camera_info')
        camera_width, camera_height = camera_info["realsense_rgb"]["width"], camera_info["realsense_rgb"]["height"]
        center_x, center_y = camera_width / 2, camera_height / 2
        
        target = None
        min_distance = float('inf')  # set value to infinity initially

        for label, score, cx, cy in results:
            if score < 0.4 or label != 'work': continue

            distance = ((cx - center_x) ** 2 + (cy - center_y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                target = (label, score, cx, cy)

        if target:
            cprint(f"Target selected: {target[0]} ({target[1]:.2f}) at ({int(target[2])}, {int(target[3])})", "green")
            return target
        else:
            cprint("No suitable target found", "red")
            return False

    # == Change coordinates ==
    @actor
    def trans_camera_coordinates(self, img_x, img_y):
        """
        Transform pixel coordinates to camera coordinates
        arg:
            img_x: pixel x coordinate
            img_y: pixel y coordinate
        return: 
            camera_x, camera_y, camera_z
        """
        camera_info = self.get_value('camera_info')
        camera_width, camera_height = camera_info["realsense_rgb"]["width"], camera_info["realsense_rgb"]["height"]
        f_x, f_y = camera_info["realsense_rgb"]["focal_len"]
        c_x, c_y = camera_info["realsense_rgb"]["optical_cen"]

        depth_image = self.run_actor('get_depth_image')
        distace = depth_image[img_y][img_x]

        camera_x = ((img_x - c_x) * distace) / f_x
        camera_y = ((img_y - c_y) * distace) / f_y
        camera_z = distace

        return camera_x, camera_y, camera_z

    @actor
    def trans_base_coordinates(self, x, y, z, frame_name = "camera_color_optical_frame"):
        """
        Transform camera coordinates to base coordinates
        arg:
            frame_name: frame name (e.g., camera_color_optical_frame, map) 
            x: frame x coordinate
            y: frame y coordinate
            z: frame z coordinate
        return: 
            base_x, base_y, base_z
        """
        point = PointEx(x, y, z)
        trans = self.run_actor("base_trans", frame_name)
        point.setTransform(trans.transform)
        base_x, base_y, base_z = point.x, point.y, point.z
        return base_x, base_y, base_z

    @actor
    def trans_map_coordinates(self, x, y, z, frame_name = "base_link"):
        """
        Transform base coordinates to map coordinates
        arg:
            frame_name: frame name (e.g., base_link)
            x: frame x coordinate
            y: frame y coordinate
            z: frame z coordinate
        return:
            map_x, map_y, map_z
        """
        point = PointEx(x, y, z)
        trans = self.run_actor("map_trans", frame_name)
        point.setTransform(trans.transform)
        map_x, map_y, map_z = point.x, point.y, point.z
        return map_x, map_y, map_z

    ####################################################################
    # from Turtlebot3 Lime    
    # get target location by arm coordinate
    @actor  
    def object_loc(self, target='link1'):
        while True:
            point = self.run_actor('find_object')
            if point: break
            self.run_actor('sleep', 1)
        trans = self.run_actor('var_trans', target)
        point.setTransform(trans.transform)
        angle = atan2(point.y, point.x) + radians(1.05)
        return point.x, point.y, angle

    # get target location by map coordinate
    @actor
    def object_glance(self):
        while True:
            point = self.run_actor('find_object')
            if point: break
        trans = self.run_actor('map_trans')
        point.setTransform(trans.transform)
        return point.x, point.y
    
    def register_flist(self, cand_points, point):
        cand = None
        min_d = 0.01
        for found in cand_points:
            d = (found.x-point.x)**2 + (found.y-point.y)**2 + (found.z-point.z)**2
            if d >= min_d: continue
            cand = found
            min_d = d
        if not cand:
            cand = PointBag(point)
#            cand.type = point.type
            cand_points.append(cand)
        else:
            cand.append(point)

    # accumulate detected object locations to get more accurate value
    @actor
    def get_found(self, max_time=10, min_count=10):
        cand_points = []
        while True:
            point = self.run_actor('find_object')
            if not point: return None
            trans = self.run_actor('map_trans')
            point.setTransform(trans.transform)
            if point.valid:
                self.register_flist(cand_points, point)
            max_count = 0
            target = None
            for c in cand_points:
                if c.count > max_count:
                    target = c
                    max_count = c.count
            if max_count >= min_count:
                return target
            
    # realtime object detection for visual feedback
    @actor('measure_distance', 'multi')
    def measure_distance(self, callback, target):
        bridge = self.get_value('cv_bridge')
        def stub(data):
            depth_image = bridge.imgmsg_to_cv2(data)
            mid_line = depth_image[len(depth_image)//2]
            return callback(min(mid_line))
            
        depth_tran = self.run_actor_mode('depth', 'multi', stub)
        return ('close', lambda tran: depth_tran.close(depth_tran)),
    
    # detect can center
    @actor
    def measure_center(self, target='link1'):
        data = self.run_actor('depth')
        cv_bridge = self.get_value('cv_bridge')
        depth_image = cv_bridge.imgmsg_to_cv2(data)
        det_line = depth_image[-5]
        index = det_line.argmin()
        distance = det_line[index]
        x, y = self.pix_to_coordinate(index, distance, depth_image)
        point = PointEx(x,y)
        trans = self.run_actor('var_trans', target)
        point.setTransform(trans.transform)
        angle = atan2(point.y, point.x) + radians(2.2)
        return point.x, point.y, angle, distance

    def pix_to_coordinate(self, pix, distance, depth_image):
        # get horizontal coordinate (y) from camera data
        # coke can position by camera coordinate target_x: depth, target_y: horizontal pos
        pix = depth_image.shape[1] / 2 - pix # Pixel value in the horizontal direction with the center at 0
        lp = 500 # base line length by pixel
        mag = distance / sqrt(pix**2 + lp**2)
        y = pix * mag * 2
        x = lp * mag
        return x, y

    # find single object
    @actor
    def find_object(self):
        center = self.run_actor('pic_find')
        if not center: return None
        data = self.run_actor('depth')
        cv_bridge = self.get_value('cv_bridge')
        depth_image = cv_bridge.imgmsg_to_cv2(data)
        yp = center[0] # by pic cell
        zp = center[1] # by pic cell
        distance = depth_image[zp][yp]
        if isinf(distance): return None

#        color  = (0, 255, 0)
#        radius = 20
#        cv2.circle(self.cv_image, center, radius, color)

        target_x, target_y = self.pix_to_coordinate(yp, distance, depth_image)        
        point = PointEx(target_x, target_y)
        point.v_x = target_x
        point.distance = distance        
        return point

    # find out target object from RGB image input
    @actor
    def pic_find(self):
        ret = None
        with self.run_actor_mode('pic_receiver', 'timed_iterator', 10) as pic_iter:
            for cv_image in pic_iter:
                ret = find_coke(cv_image)
                if ret: break
        return ret
        
    # get raw RGB image
    @actor('pic_receiver', 'multi')
    def pic_receiver(self, callback):
        def stub(data):
            cv_image = None
            cv_bridge = self.get_value('cv_bridge')
            try:
                cv_image = cv_bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
            return callback(cv_image)
        pic_tran = self.run_actor_mode('pic', 'multi', stub)
        return ('close', lambda tran: pic_tran.close(pic_tran)),
