#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pinhandling_pcl.msg import point_lists
from pinhandling_pathfinder.msg import contour_coefficients


class LidarContouringNode():
    def __init__(self):
        rospy.init_node("lidar_contouring")

        self.points_right = []
        self.points_left = []

        self.cluster_sub = rospy.Subscriber("/detection/edge", point_lists, self.edge_callback, queue_size=1)

        self.contour_pub = rospy.Publisher("/guidance/contour", contour_coefficients, queue_size=1)
        self.marker_pub = rospy.Publisher("/visual/contour", MarkerArray, queue_size=1)
        self.debug_pub = rospy.Publisher("/debug/guidance/contouring", Marker, queue_size=1)


    ##
    ##  Callback for the "/detection/edge" topic
    ##  Assign the respective point lists to the member variables
    ##  Visualize all received points for debug purposes
    ##
    def edge_callback(self, msg):
        self.points_right += msg.points_min
        self.points_left += msg.points_max

        self.debug_visualization([*msg.points_min, *msg.points_max])


    ##
    ##  Fitting a first order polynomial via linear regression to a provided list of data entries
    ##
    ##  @ARGUMENTS
    ##      points: List of geometry_msgs::Point() objects to which the polynomial is to be fitted
    ##
    ##  @RETURN
    ##      float, float: Describing the slope and axis intersection of a linear function, respectively
    ##
    def lin_regression_fit(self, points):       
        x, y = [], []

        for i, point in enumerate(points):
            x.append(points[i].x)
            y.append(points[i].y)

        return np.polyfit(x, y, 1)
    

    ##
    ##  Determine two points on a linear function
    ##  Only required for visualization!
    ##
    ##  @ARGUMENTS
    ##      m: float describing the slope of a linear function
    ##      b: float describing the intersection of the y-axis
    ##
    ##  @RETURN
    ##      []: List of geometry_msgs::Point() objects that are located on the given function
    ##
    def calc_points_on_line(self, m, b):
        x_min, x_max = 0, 5
        point1, point2 = Point(), Point()

        poly1d_fn = np.poly1d([m, b])

        point1.x = x_min
        point1.y = poly1d_fn(x_min)
        point1.z = self.points_right[0].z

        point2.x = x_max
        point2.y = poly1d_fn(x_max)
        point2.z = self.points_right[0].z

        return [point1, point2]


    ##
    ##  Visualization of the approximated contour via two line strips
    ##
    ##  @ARGUMENTS
    ##      points1: List of geometry_msgs::Point() objects on the approximated contour function (one side)
    ##      points2: List of geometry_msgs::Point() objects on the approximated contour function (other side)
    ##
    def visualization(self, points1, points2):
        marker_array = MarkerArray()

        marker1 = Marker()
        marker1.header.frame_id = "rslidar"
        marker1.id = 1
        marker1.type = marker1.LINE_STRIP
        marker1.scale.x = 0.05
        marker1.color.r = 0.0
        marker1.color.g = 0.0
        marker1.color.b = 1.0
        marker1.color.a = 1.0
        marker1.points = points1
        marker_array.markers.append(marker1)

        marker2 = Marker()
        marker2.header.frame_id = "rslidar"
        marker2.id = 2
        marker2.type = marker2.LINE_STRIP
        marker2.scale.x = 0.05
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        marker2.color.a = 1.0
        marker2.points = points2
        marker_array.markers.append(marker2)

        self.marker_pub.publish(marker_array)


    ##
    ##  Visualization of all received points from the edge detection
    ##  Debug purposes only!
    ##
    ##  @ARGUMENTS
    ##      points: List of geometry_msgs::Point() objects containing all received edge points
    ##
    def debug_visualization(self, points):
        marker = Marker()
        marker.header.frame_id = "rslidar"
        marker.type = marker.POINTS
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = points
        self.debug_pub.publish(marker)


    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.points_right and self.points_left:
                coeff = contour_coefficients()
                coeff.m_right, coeff.b_right = self.lin_regression_fit(self.points_right)
                coeff.m_left, coeff.b_left = self.lin_regression_fit(self.points_left)
                self.contour_pub.publish(coeff)

                self.visualization(self.calc_points_on_line(coeff.m_right, coeff.b_right), self.calc_points_on_line(coeff.m_left, coeff.b_left))

                self.points_right.clear()
                self.points_left.clear()
            else:
                rospy.logwarn("LIDAR_CONTOURING: NO POINTS RECEIVED")
            
            rate.sleep()



def main():
    node = LidarContouringNode()
    node.run()

if __name__ == "__main__":
    main()