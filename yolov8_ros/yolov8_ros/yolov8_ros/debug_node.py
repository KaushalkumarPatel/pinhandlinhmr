#!/usr/bin/env python3

import cv2
import random
import numpy as np
from typing import Tuple
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolov8_msgs.msg import BoundingBox2D, Detection, DetectionArray


class DebugNode:

    def __init__(self):

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        # params
        #self.image_reliability = rospy.get_param("~image_reliability", 2)

        # pubs
        self._dbg_pub = rospy.Publisher("dbg_image", Image, queue_size=10)

        # subs
        image_sub = message_filters.Subscriber("image_raw", Image)
        detections_sub = message_filters.Subscriber("detections", DetectionArray)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detections_sub], 50, 0.1)
        self._synchronizer.registerCallback(self.detections_cb)

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

    def detections_cb(self, img_msg: Image, detection_msg: DetectionArray) -> None:

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

        detection: Detection
        for detection in detection_msg.detections:

            # random color
            label = detection.class_name

            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)

            color = self._class_to_color[label]

            cv_image = self.draw_box(cv_image, detection, color)

        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image,
                                                           encoding=img_msg.encoding))


if __name__ == '__main__':
    rospy.init_node('debug_node')
    node = DebugNode()
    rospy.spin()