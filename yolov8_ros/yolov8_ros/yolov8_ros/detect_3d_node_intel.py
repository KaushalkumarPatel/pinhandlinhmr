#!/usr/bin/env python3

import numpy as np
from typing import List
import rospy
import message_filters
from cv_bridge import CvBridge
import tf.transformations
import tf2_ros
import tf
import open3d as o3d
from sensor_msgs import point_cloud2 as pc2  # Import the point_cloud2 module
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from yolov8_msgs.msg import BoundingBox3D

class Detect3DNode:

    def __init__(self):

        # parameters
        self.target_frame = rospy.get_param("~target_frame", "camera1_depth_optical_frame")
        #self.maximum_detection_threshold = rospy.get_param("~maximum_detection_threshold", 0.3)
        self.depth_image_units_divisor = rospy.get_param("~depth_image_units_divisor", 1000)
        self.cv_bridge = CvBridge()

        # pubs
        self._pub = rospy.Publisher("detections_3d", DetectionArray, queue_size=50)
        #self.rate = rospy.Rate(1)  # 1 Hz

        # subs
        self.depth_sub = message_filters.Subscriber("depth_image", Image)
        self.depth_info_sub = message_filters.Subscriber("depth_info", CameraInfo)
        self.detections_sub = message_filters.Subscriber("detections", DetectionArray)
        self.pointcloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloud_callback)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.depth_info_sub, self.detections_sub], queue_size=1000, slop=0.1)
        self._synchronizer.registerCallback(self.on_detections)

        self.pointcloud = None

    def pointcloud_callback(self, msg):
        self.pointcloud = msg

    def on_detections(
        self,
        depth_msg: Image,
        depth_info_msg: CameraInfo,
        detections_msg: DetectionArray,
    ) -> None:   

        new_detections_msg = DetectionArray()
        new_detections_msg.header = detections_msg.header
        new_detections_msg.detections = self.process_detections(depth_msg, depth_info_msg, detections_msg)
        self._pub.publish(new_detections_msg)
        #self.rate.sleep()

    def process_detections(
        self,
        depth_msg: Image,
        depth_info_msg: CameraInfo,
        detections_msg: DetectionArray
    ) -> List[Detection]:

        # check if there are detections
        if not detections_msg.detections:
            return []

        new_detections = []
        depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)

        for detection in detections_msg.detections:
            bbox3d = self.convert_2dbb_to_3d(
                depth_image, depth_info_msg, detection)

            if bbox3d is not None:
                new_detections.append(detection)

                # Create a unique frame ID for the detected object
                object_frame_id = f"{detection.class_name}_frame"

                # Publish the transform for the detected object
                self.publish_transform(depth_info_msg.header.frame_id, object_frame_id, bbox3d.center.position, bbox3d.center.orientation)

                bbox3d.frame_id = self.target_frame
                new_detections[-1].bbox3d = bbox3d
                
        return new_detections

    def pointcloud2_to_open3d(self, pointcloud_msg):
        # Convert ROS PointCloud2 message to Open3D point cloud
        points = []
        for p in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            distance = np.linalg.norm([p[0], p[1], p[2]])
            if distance <= 1.5:  # Filter points farther than 150 cm
                points.append([p[0], p[1], p[2]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def convert_2dbb_to_3d(
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

        # find the z coordinate on the 3D BB
        center_z_coord = depth_image[int(center_y)][int(center_x)] / self.depth_image_units_divisor
        
        k = depth_info.K
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = center_z_coord * (center_x - px) / fx
        y = center_z_coord * (center_y - py) / fy
        w = center_z_coord * (size_x / fx)
        h = center_z_coord * (size_y / fy)

        if detection.class_id in [1, 4, 6, 8, 10]:
        # create 3D BB
            msg = BoundingBox3D()
            msg.center.position.x = x
            msg.center.position.y = y-0.02
            msg.center.position.z = center_z_coord
            msg.size.x = w
            msg.size.y = h
            msg.size.z = center_z_coord

            # Calculate orientation using surface normal
            if self.pointcloud:
                pcd = self.pointcloud2_to_open3d(self.pointcloud)
                #downpcd = pcd.voxel_down_sample(voxel_size=0.02)
                pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                distances = np.linalg.norm(np.asarray(pcd.points) - np.array([x, y-0.02, center_z_coord]), axis=1)
                closest_point_idx = np.argmin(distances)
                normal = np.asarray(pcd.normals)[closest_point_idx]

                # Average the normals of the neighboring points
                kdtree = o3d.geometry.KDTreeFlann(pcd)
                [_, idx, _] = kdtree.search_knn_vector_3d(pcd.points[closest_point_idx], 50)
                neighboring_normals = np.asarray(pcd.normals)[idx]
                average_normal = np.mean(neighboring_normals, axis=0)
                average_normal = average_normal / np.linalg.norm(average_normal)

                orientation = tf.transformations.quaternion_from_euler(normal[0], normal[1], normal[2])
                msg.center.orientation.x = orientation[0]
                msg.center.orientation.y = orientation[1]
                msg.center.orientation.z = orientation[2]
                msg.center.orientation.w = orientation[3]

            return msg
        
        else:
            
            # create 3D BB
            msg = BoundingBox3D()
            msg.center.position.x = x
            msg.center.position.y = y
            msg.center.position.z = center_z_coord
            msg.size.x = w
            msg.size.y = h
            msg.size.z = center_z_coord

            # Calculate orientation using surface normal
            if self.pointcloud:
                pcd = self.pointcloud2_to_open3d(self.pointcloud)
                #downpcd = pcd.voxel_down_sample(voxel_size=0.02)
                pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                distances = np.linalg.norm(np.asarray(pcd.points) - np.array([x, y, center_z_coord]), axis=1)
                closest_point_idx = np.argmin(distances)
                normal = np.asarray(pcd.normals)[closest_point_idx]

                # Average the normals of the neighboring points
                kdtree = o3d.geometry.KDTreeFlann(pcd)
                [_, idx, _] = kdtree.search_knn_vector_3d(pcd.points[closest_point_idx], 50)
                neighboring_normals = np.asarray(pcd.normals)[idx]
                average_normal = np.mean(neighboring_normals, axis=0)
                average_normal = average_normal / np.linalg.norm(average_normal)
                    
                orientation = tf.transformations.quaternion_from_euler(normal[0], normal[1], normal[2])
                msg.center.orientation.x = orientation[0]
                msg.center.orientation.y = orientation[1]
                msg.center.orientation.z = orientation[2]
                msg.center.orientation.w = orientation[3]

            return msg

    def publish_transform(self, parent_frame_id, child_frame_id, position, orientation):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z

        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w

        br.sendTransform(t)

if __name__ == '__main__':

    rospy.init_node('detect_3d_node')
    
    node = Detect3DNode()
    
    rospy.spin()