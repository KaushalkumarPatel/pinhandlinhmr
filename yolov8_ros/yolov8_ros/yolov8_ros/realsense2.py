import numpy as np
from typing import List, Tuple
import rospy
import message_filters
from cv_bridge import CvBridge
import tf2_ros
import tf
import open3d as o3d
from sensor_msgs import point_cloud2 as pc2  # Import the point_cloud2 module
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from yolov8_msgs.msg import BoundingBox3D


class PointCloudProcessor:
    def __init__(self):
        rospy.init_node('point_cloud_processor', anonymous=True)
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        self.pointcloud = None

    def pointcloud_callback(self, msg):
        self.pointcloud = msg

    def pointcloud2_to_open3d(self, pointcloud_msg):
        # Convert ROS PointCloud2 message to Open3D point cloud
        points = []
        for point in pc2.read_points(pointcloud_msg, skip_nans=True):
            points.append([point[0], point[1], point[2]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def process_point_cloud(self, x, y, center_z_coord):
        if self.pointcloud:
            pcd = self.pointcloud2_to_open3d(self.pointcloud)
            downpcd = pcd.voxel_down_sample(voxel_size=0.05)
            downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=60))
            downpcd.orient_normals_consistent_tangent_plane(k=30)
            distances = np.linalg.norm(np.asarray(downpcd.points) - np.array([x, y, center_z_coord]), axis=1)
            closest_point_idx = np.argmin(distances)
            
            # Check if the closest point index is within bounds
            if closest_point_idx >= len(downpcd.normals):
                rospy.logerr(f"Closest point index {closest_point_idx} is out of bounds for normals array of size {len(downpcd.normals)}")
                return None

            normal = np.asarray(downpcd.normals)[closest_point_idx]
            normal = normal / np.linalg.norm(normal)
                
            quaternion = tf.transformations.quaternion_from_euler(normal[0], normal[1], normal[2])
            return quaternion
        return None

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
        center_z_coord = depth_image[int(center_y)][int(center_x)] / self.depth_image_units_divisor

        z_diff = np.abs(roi - center_z_coord)
        mask_z = z_diff <= self.maximum_detection_threshold
        if not np.any(mask_z):
            return None

        roi_threshold = roi[mask_z]
        z_min, z_max = np.min(roi_threshold), np.max(roi_threshold)
        z = (z_max + z_min) / 2
        if z == 0:
            return None 
        
        k = depth_info.K
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = (z * (center_x - px) / fx) / 2 
        y = (z * (center_y - py) / fy) / 2 
        w = z * (size_x / fx)
        h = z * (size_y / fy)

        # create 3D BB
        msg = BoundingBox3D()
        msg.center.position.x = x
        msg.center.position.y = y
        msg.center.position.z = center_z_coord
        msg.size.x = w
        msg.size.y = h
        msg.size.z = float(z_max - z_min)

        # Calculate orientation using surface normal
        if self.pointcloud:
            pcd = self.pointcloud2_to_open3d(self.pointcloud)
            downpcd = pcd.voxel_down_sample(voxel_size=0.05)
            downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=60))
            downpcd.orient_normals_consistent_tangent_plane(k=30)
            distances = np.linalg.norm(np.asarray(downpcd.points) - np.array([x, y, center_z_coord]), axis=1)
            closest_point_idx = np.argmin(distances)
            
            # Check if the closest point index is within bounds
            if closest_point_idx >= len(downpcd.normals):
                print(f"Closest point index {closest_point_idx} is out of bounds for normals array of size {len(downpcd.normals)}")
                return msg

            normal = np.asarray(downpcd.normals)[closest_point_idx]
            normal = normal / np.linalg.norm(normal)
                
            quaternion = tf.transformations.quaternion_from_euler(normal[0], normal[1], normal[2])
            msg.center.orientation.x = quaternion[0]
            msg.center.orientation.y = quaternion[1]
            msg.center.orientation.z = quaternion[2]
            msg.center.orientation.w = quaternion[3]

        return msg

if __name__ == '__main__':
    processor = PointCloudProcessor()
    rospy.spin()