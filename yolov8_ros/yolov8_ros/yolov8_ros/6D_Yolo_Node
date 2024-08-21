#!/usr/bin/env python3

from typing import List, Dict
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge
import rospy
from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from sensor_msgs.msg import Image
from yolov8_msgs.msg import Point2D
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Mask
from yolov8_msgs.msg import KeyPoint2D
from yolov8_msgs.msg import KeyPoint2DArray
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from std_srvs.srv import SetBool


class Yolov8Node:

    def __init__(self):

         # params
        self.model = rospy.get_param("~model", "yolov8l.pt")
        self.device = rospy.get_param("~device", "cuda:0")
        self.threshold = rospy.get_param("~threshold", 0.5)
        self.enable = rospy.get_param("~enable", True)

        self.image_reliability = rospy.get_param("~image_reliability", 2) 

        self.cv_bridge = CvBridge()
        self.yolo = YOLO(self.model)
        self.yolo.fuse()

        # pubs
        self._pub = rospy.Publisher("detections", DetectionArray, queue_size=10)

        # subs
        self._sub = rospy.Subscriber("image_raw", Image, self.image_cb)

        # services
        self._srv = rospy.Service("enable", SetBool, self.enable_cb)

        rospy.loginfo("YOLO node started")

    def enable_cb(self, req):
        self.enable = req.data
        return SetBool.Response(success=True)


    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        # Convert the frames to numpy arrays
        #color_image = np.asanyarray(color_frame.get_data())
        """ depth_image = np.asanyarray(depth_frame.get_data())

        depth_image = depth_image * depth_scale """

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]

            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1] )
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

            """ object_depth = np.median(depth_image[int(box[1]):int(box[1]+box[3]), int(box[0]):int(box[0]+box[2])])
            label = f"{object_depth:.2f}m"

            print(f"{self.yolo.names[int(box_data.cls)]}: {object_depth:.2f}m") """

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list

    def image_cb(self, msg: Image) -> None:

        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            results: Results = results[0].cpu()

            
            if results.boxes:
                hypothesis = self.parse_hypothesis(results)

                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):

                aux_msg = Detection()

                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

            # publish detections
            detections_msg.header = msg.header
            self._pub.publish(detections_msg)


if __name__ == '__main__':
    rospy.init_node('yolov8_node')

    """ # Set up the RealSense D455 camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    depth_scale = 0.0010000000474974513

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame() """

    node = Yolov8Node()
    rospy.spin()
 




