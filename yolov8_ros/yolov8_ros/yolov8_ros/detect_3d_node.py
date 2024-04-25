#!/usr/bin/env python3

import numpy as np
from typing import List, Tuple
import rospy
import message_filters
from cv_bridge import CvBridge
import tf2_ros
import pyrealsense2

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from yolov8_msgs.msg import KeyPoint3D
from yolov8_msgs.msg import KeyPoint3DArray
from yolov8_msgs.msg import BoundingBox3D


class Detect3DNode:

    def __init__(self):

         # parameters
        self.target_frame = rospy.get_param("~target_frame", "camera_depth_optical_frame")
        self.maximum_detection_threshold = rospy.get_param("~maximum_detection_threshold", 0.3)
        self.depth_image_units_divisor = rospy.get_param("~depth_image_units_divisor", 1000)

        self.depth_image_reliability = rospy.get_param("~depth_image_reliability", 2)  
        self.depth_info_reliability = rospy.get_param("~depth_info_reliability", 2)  

        # aux
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cv_bridge = CvBridge()

        # pubs
        self._pub = rospy.Publisher("detections_3d", DetectionArray, queue_size=10)

        # subs
        self.depth_sub = message_filters.Subscriber("depth_image", Image)
        self.depth_info_sub = message_filters.Subscriber("depth_info", CameraInfo)
        self.detections_sub = message_filters.Subscriber("detections", DetectionArray)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.depth_info_sub, self.detections_sub], 1000, 0.1)
        self._synchronizer.registerCallback(self.on_detections)

    def on_detections(
        self,
        depth_msg: Image,
        depth_info_msg: CameraInfo,
        detections_msg: DetectionArray,
    ) -> None:

        new_detections_msg = DetectionArray()
        new_detections_msg.header = detections_msg.header
        new_detections_msg.detections = self.process_detections(depth_msg, depth_info_msg, detections_msg)
        rate = rospy.Rate(5)
        self._pub.publish(new_detections_msg)
        rate.sleep()

    def process_detections(
        self,
        depth_msg: Image,
        depth_info_msg: CameraInfo,
        detections_msg: DetectionArray
    ) -> List[Detection]:

        # check if there are detections
        if not detections_msg.detections:
            return []

        transform = self.get_transform(depth_info_msg.header.frame_id)

        if transform is None:
            return []

        new_detections = []
        depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)

        for detection in detections_msg.detections:
            bbox3d = self.convert_bb_to_3d(
                depth_image, depth_info_msg, detection)

            if bbox3d is not None:
                new_detections.append(detection)

                bbox3d = Detect3DNode.transform_3d_box(
                    bbox3d, transform[0], transform[1])
                bbox3d.frame_id = self.target_frame
                new_detections[-1].bbox3d = bbox3d

                if detection.keypoints.data:
                    keypoints3d = self.convert_keypoints_to_3d(
                        depth_image, depth_info_msg, detection)
                    keypoints3d = Detect3DNode.transform_3d_keypoints(
                        keypoints3d, transform[0], transform[1])
                    keypoints3d.frame_id = self.target_frame
                    new_detections[-1].keypoints3d = keypoints3d

        return new_detections

    def convert_bb_to_3d(
        self,
        depth_image: np.ndarray,
        depth_info: CameraInfo,
        detection: Detection
    ) -> BoundingBox3D:

        # crop depth image by the 2d BB
        center_x = int(detection.bbox.center.position.x)
        center_y = int(detection.bbox.center.position.y)
        size_x = int(detection.bbox.size.x)
        size_y = int(detection.bbox.size.y)

        u_min = max(center_x - size_x // 2, 0)
        u_max = min(center_x + size_x // 2, depth_image.shape[1] - 1)
        v_min = max(center_y - size_y // 2, 0)
        v_max = min(center_y + size_y // 2, depth_image.shape[0] - 1)

        roi = depth_image[v_min:v_max, u_min:u_max] / self.depth_image_units_divisor  # convert to meters
        if not np.any(roi):
            return None

        # find the z coordinate on the 3D BB
        bb_center_z_coord = depth_image[int(center_y)][int(center_x)] / self.depth_image_units_divisor
        z_diff = np.abs(roi - bb_center_z_coord)
        mask_z = z_diff <= self.maximum_detection_threshold
        if not np.any(mask_z):
            return None

        roi_threshold = roi[mask_z]
        z_min, z_max = np.min(roi_threshold), np.max(roi_threshold)
        z = (z_max + z_min) / 2
        if z == 0:
            return None

        """ # project from image to world space
        _intrinsics = pyrealsense2.intrinsics()
        _intrinsics.width = depth_info.width
        _intrinsics.height = depth_info.height
        _intrinsics.ppx = depth_info.K[2]
        _intrinsics.ppy = depth_info.K[5]
        _intrinsics.fx = depth_info.K[0]
        _intrinsics.fy = depth_info.K[4]
        if depth_info.distortion_model == 'plumb_bob':
                _intrinsics.model = pyrealsense2.distortion.brown_conrady
        elif depth_info.distortion_model == 'equidistant':
                _intrinsics.model = pyrealsense2.distortion.kannala_brandt4
        _intrinsics.coeffs = [i for i in depth_info.D] """

        k = depth_info.K
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (center_x - px) / fx
        y = z * (center_y - py) / fy
        w = z * (size_x / fx)
        h = z * (size_y / fy)

        #result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [center_x, center_y], bb_center_z_coord)

        # create 3D BB
        msg = BoundingBox3D()
        msg.center.position.x = x
        msg.center.position.y = y
        msg.center.position.z = bb_center_z_coord
        msg.size.x = w
        msg.size.y = h
        msg.size.z = float(z_max - z_min)

        return msg

    def convert_keypoints_to_3d(
        self,
        depth_image: np.ndarray,
        depth_info: CameraInfo,
        detection: Detection
    ) -> KeyPoint3DArray:

        # build an array of 2d keypoints
        keypoints_2d = np.array([[p.point.x, p.point.y] for p in detection.keypoints.data], dtype=np.int16)
        u = np.array(keypoints_2d[:, 1]).clip(0, depth_info.height - 1)
        v = np.array(keypoints_2d[:, 0]).clip(0, depth_info.width - 1)

        # sample depth image and project to 3D
        z = depth_image[u, v]
        k = depth_info.K
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (v - px) / fx
        y = z * (u - py) / fy
        points_3d = np.dstack([x, y, z]).reshape(-1, 3) / self.depth_image_units_divisor  # convert to meters

        # generate message
        msg_array = KeyPoint3DArray()
        for p, d in zip(points_3d, detection.keypoints.data):
            if not np.isnan(p).any():
                msg = KeyPoint3D()
                msg.point.x = p[0]
                msg.point.y = p[1]
                msg.point.z = p[2]
                msg.id = d.id
                msg.score = d.score
                msg_array.data.append(msg)

        return msg_array

    def get_transform(self, frame_id: str) -> Tuple[np.ndarray]:
        # transform position from image frame to target_frame
        rotation = None
        translation = None

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                rospy.Time())

            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])

            rotation = np.array([transform.transform.rotation.w,
                                 transform.transform.rotation.x,
                                 transform.transform.rotation.y,
                                 transform.transform.rotation.z])

            return translation, rotation

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('FAILED TO GET TRANSFORM')
            return None

    @staticmethod
    def transform_3d_box(
        bbox: BoundingBox3D,
        translation: np.ndarray,
        rotation: np.ndarray
    ) -> BoundingBox3D:

        # position
        position = Detect3DNode.qv_mult(
            rotation,
            np.array([bbox.center.position.x,
                      bbox.center.position.y,
                      bbox.center.position.z])
        ) + translation

        bbox.center.position.x = position[0]
        bbox.center.position.y = position[1]
        bbox.center.position.z = position[2]

        # size
        size = Detect3DNode.qv_mult(
            rotation,
            np.array([bbox.size.x,
                      bbox.size.y,
                      bbox.size.z])
        )

        bbox.size.x = abs(size[0])
        bbox.size.y = abs(size[1])
        bbox.size.z = abs(size[2])

        return bbox

    @staticmethod
    def transform_3d_keypoints(
        keypoints: KeyPoint3DArray,
        translation: np.ndarray,
        rotation: np.ndarray,
    ) -> KeyPoint3DArray:

        for point in keypoints.data:
            position = Detect3DNode.qv_mult(
                rotation,
                np.array([
                    point.point.x,
                    point.point.y,
                    point.point.z
                ])
            ) + translation

            point.point.x = position[0]
            point.point.y = position[1]
            point.point.z = position[2]

        return keypoints

    @staticmethod
    def qv_mult(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        q = np.array(q, dtype=np.float64)
        v = np.array(v, dtype=np.float64)
        qvec = q[1:]
        uv = np.cross(qvec, v)
        uuv = np.cross(qvec, uv)
        return v + 2 * (uv * q[0] + uuv)


if __name__ == '__main__':
    rospy.init_node('detect_3d_node')
    
    node = Detect3DNode()
    
    rospy.spin()
    
    
