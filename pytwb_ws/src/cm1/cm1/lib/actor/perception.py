from ros_actor import actor, SubNet

from ..print_color import cprint

class PerceptionNetwork(SubNet):
    """
    Perception Network
    Ex):
    - Object detection
    - pose estimation
    etc.
    """
    ###################
    # Isaac ROS YOLOv8
    ###################
    @actor
    def detections_subscriber(self):
        # get images dimensions
        img_msg = self.run_actor('pic')
        org_w, org_h = img_msg.width, img_msg.height  # original image size
        model_w, model_h = 640, 640  # model input size

        # for adjust ratios
        aspect_ratio = org_w / org_h  
        model_h = model_w / aspect_ratio
        width_ratio = org_w / model_w
        height_ratio = org_h / model_h
        padding_ofs = (org_w - org_h)//2

        # get yolov8 detection message
        detections_msg = self.run_actor('sub_from_yolov8')
        names = self.get_value('work_class_names')

        # summarizing the detection results
        outputs = []
        def __str__func(self):
            return vars(self)  # return as dict
        Detection_class = type('detection_class', (object,), dict())
        setattr(Detection_class,"__str__",__str__func)

        for detection in detections_msg.detections:
            # Coordinates of the detected object
            output = Detection_class()
            output.center_x = detection.bbox.center.position.x * width_ratio
            output.center_y = detection.bbox.center.position.y  * height_ratio - padding_ofs
            output.width = detection.bbox.size_x * width_ratio
            output.height = detection.bbox.size_y * height_ratio

            # Name and confidence
            output.label = names[int(detection.results[0].hypothesis.class_id)]
            output.conf_score = detection.results[0].hypothesis.score

            outputs.append(output)
        
        return outputs

    @actor
    def detections_visualizer(self):
        detections_info = []

        results = self.run_actor('detections_subscriber')
        if len(results) == 0:
            print("No detections")
            return False
        
        cprint(f"==Yolo Detections==", "blue")
        for r in results:
            cx, cy = int(r.center_x), int(r.center_y)
            label = r.label
            score = r.conf_score
            print(f"{label} ({score:.2f}) at ({cx}, {cy})")
            detections_info.append((label, score, cx, cy))
        cprint(f"===================", "blue")
        return detections_info
        

    @actor
    def get_object_pose(self):
        results =  self.run_actor('sub_from_foundationpose')  # to update the detections
        Pose_class = type('pose_class', (object,), dict())
        print(results.detections)
        object_pose = Pose_class()

        for detection in results.detections:
            label = detection.results[0].hypothesis.class_id
            score = detection.results[0].hypothesis.score
            position = detection.results[0].pose.pose.position
            orientation = detection.results[0].pose.pose.orientation
            setattr(object_pose, label, (score, position, orientation))
            pos_from_base = self.run_actor("trans_base_coordinates", position.x, position.y, position.z, "camera_color_optical_frame")

        return  object_pose
