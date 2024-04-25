#!/usr/bin/python3

import rospy
import math

from pinhandling_manipulation_services.srv import CartesianTrajectory, CartesianTrajectoryResponse

class EETrajectoryCalculatorNode:
    """
    The Pinhandling Trajectory Point Calculator Node
    """

    def __init__(self):
        rospy.init_node("trajectory_planner_bs_open_server")

        rospy.Service('cartesian_path_bs_open',    # service name
                        CartesianTrajectory,                # service type
                        self.calculate_trajectory           # function service provides
                        )

    def calculate_trajectory(self, req):
        grab_Y = req.grab_Y
        grab_Z = req.grab_Z
        pin_X = req.pin_X
        pin_Y = req.pin_Y
        pin_Z = req.pin_Z
        degree = req.degree
        points = req.points

        # Calculate radius of the grabbing coordinate according to the pin rotation center
        r = math.sqrt(grab_Y**2 + grab_Z**2)

        # First grabbing degree relative to pin center
        firstgrabdegree = math.atan(grab_Y / grab_Z)

        # First grabbing degree of the end effector
        firsteedegree = math.radians(-90.0)

        # Desired degree increment in radians
        Δdegree = math.radians(degree)

        trajectory = []

        for i in range(points):
            # Calculate Quaternions (Rotation around x axis)
            qx = math.sin((firsteedegree - i * Δdegree) / 2)
            qy = 0.0
            qz = 0.0
            qw = math.cos((firsteedegree - i * Δdegree) / 2)

            # Calculate x, y, z coordinates
            x = (pin_X)
            y = (pin_Y - r * math.cos(firstgrabdegree + i * Δdegree))
            z = pin_Z + r * math.sin(firstgrabdegree + i * Δdegree)

            # Append individual float values to the trajectory list
            trajectory.extend([qx, qy, qz, qw, x, y, z])

        return CartesianTrajectoryResponse(trajectory)
    
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            


def main():
    print("Ready to calculate cartesian trajectory for backside open of the pin")
    node = EETrajectoryCalculatorNode()
    node.run()
    
if __name__ == '__main__':
    main()

