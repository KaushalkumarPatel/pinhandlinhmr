#!/usr/bin/python3

import rospy
import sys

from pinhandling_manipulation_services.srv import CartesianTrajectory

def trajectory_planner_bs_open_client(grab_Y, grab_Z, pin_X, pin_Y, pin_Z, degree, points):
    rospy.wait_for_service('cartesian_path_bs_open')
    try:
        cartesian_path_bs_open = rospy.ServiceProxy('cartesian_path_bs_open', # service name
                                                CartesianTrajectory   # service type
                                                )
        resp1 = cartesian_path_bs_open(grab_Y, grab_Z, pin_X, pin_Y, pin_Z, degree, points)

        return resp1.trajectory
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e) 
    
def usage():
        return "Usage: rosrun pinhandling_manipulation_services cartesian_trajectory_planner_bs_open_client.py <grabbing coordinate Y relative to pin center> <grabbing coordinate Z relative to pin center> <pin center X relative to base_link> <pin center Y relative to base_link> <pin center Z relative to base_link> <circular degree> <number of points>"
    


if __name__ == '__main__':
    if len(sys.argv) == 8:
        grab_Y = float(sys.argv[1])
        grab_Z = float(sys.argv[2])
        pin_X = float(sys.argv[3])
        pin_Y = float(sys.argv[4])
        pin_Z = float(sys.argv[5])
        degree = float(sys.argv[6])
        points = int(sys.argv[7])
    else:
         print(usage())
         sys.exit(1)
    print("Requesting Cartesian Trajectory calculation for backside pin open")
    print("Result=\n", trajectory_planner_bs_open_client(grab_Y, grab_Z, pin_X, pin_Y, pin_Z, degree, points))