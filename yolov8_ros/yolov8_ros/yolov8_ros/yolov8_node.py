#!/usr/bin/env python3

from typing import List, Dict
from cv_bridge import CvBridge
import cv2
import random
import rospy
import numpy as np
from typing import Tuple
from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from sensor_msgs.msg import Image
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from std_srvs.srv import SetBool


class Yolov8Node:

    def __init__(self):

         # params
        self.model = rospy.get_param("~model", "yolov8m.pt")
        self.device = rospy.get_param("~device", "cuda:0")
        self.threshold = rospy.get_param("~threshold", 0.1)
        self.enable = rospy.get_param("~enable", True)

        self.image_reliability = rospy.get_param("~image_reliability", 2) 
        self._class_to_color = {}

        self.cv_bridge = CvBridge()
        self.yolo = YOLO(self.model)
        self.yolo.fuse()

        # pubs
        self._pub = rospy.Publisher("detections", DetectionArray, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz
        self._dbg_pub = rospy.Publisher('/debug_image', Image, queue_size=10)
        

        # subs
        self._sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)

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

        return boxes_list
    
    def draw_box(self, cv_image: np.array, detection: Detection, color: Tuple[int]) -> np.array:

        # get detection info
        label = detection.class_name
        score = detection.score
        box_msg: BoundingBox2D = detection.bbox
        track_id = detection.id

        min_pt = (round(box_msg.center.position.x - box_msg.size.x / 2.0),
                  round(box_msg.center.position.y - box_msg.size.y / 2.0))
        max_pt = (round(box_msg.center.position.x + box_msg.size.x / 2.0),
                  round(box_msg.center.position.y + box_msg.size.y / 2.0))

        # draw box
        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

        # write text
        label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
        pos = (min_pt[0] + 5, min_pt[1] + 25)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image, label, pos, font,
                    1, color, 1, cv2.LINE_AA)

        return cv_image

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

            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):

                aux_msg = Detection()

                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]
                    label = aux_msg.class_name
                    if label not in self._class_to_color:
                        r = random.randint(0, 255)
                        g = random.randint(0, 255)
                        b = random.randint(0, 255)
                        self._class_to_color[label] = (r, g, b)

                    color = self._class_to_color[label]

                    cv_image = self.draw_box(cv_image, aux_msg, color)
                detections_msg.detections.append(aux_msg)

            # publish detections
            detections_msg.header = msg.header
            self._pub.publish(detections_msg)
            self.rate.sleep()
            self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image,
                                                           encoding=msg.encoding))
            


if __name__ == '__main__':
    rospy.init_node('yolov8_node')
    node = Yolov8Node()
    rospy.spin()
 




