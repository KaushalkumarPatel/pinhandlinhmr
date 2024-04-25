#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import copy
import numpy as np

from pinhandling_manipulation_actions.msg import PinOpenCloseBsAction, PinOpenCloseBsFeedback, PinOpenCloseBsResult
from pinhandling_manipulation_services.srv import CartesianTrajectory


class PinCloseBSTrajActionServer():
    # Create messages that are used to publish feedback/result

    def __init__(self, name):

        moveit_commander.roscpp_initialize(sys.argv)

        # Action Server Initialization part
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name,      # Server name string
                                    PinOpenCloseBsAction,                # Action message type
                                    self.publish_position,              # Action Function
                                    auto_start = False 
                                    )
        
        self._as.start()

        # Instantiating a RobotCommander object. 
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiating a PlanningSceneInterface object. 
        # This provides a remote interface for getting, setting, 
        # and updating the robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiating a MoveGroupCommander object. 
        # This object is an interface to a planning group (group of joints).
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = self.move_group.get_planning_frame()
        print(planning_frame)

        # self.move_group.set_pose_reference_frame('robot_arm_base_link')

        # Service Server Initialization part
        # For 10 points
        self.grab_Y = 0.09402
        self.grab_Z = 0.08978
        self.pin_X = -0.47
        self.pin_Y = 1.060
        self.pin_Z = 0.397
        self.degree = 1.777778
        self.points = 93
        
        rospy.wait_for_service('cartesian_path_bs_close')

    def publish_position(self, goal):

        if goal:
            _feedback = PinOpenCloseBsFeedback()
            _result = PinOpenCloseBsResult()

            cartesian_path_bs_close = rospy.ServiceProxy('cartesian_path_bs_close', # service name
                                                    CartesianTrajectory   # service type
                                                    )
            resp1 = cartesian_path_bs_close(
                                    self.grab_Y, 
                                    self.grab_Z, 
                                    self.pin_X, 
                                    self.pin_Y, 
                                    self.pin_Z, 
                                    self.degree, 
                                    self.points
                                    )

            move_group = self.move_group
            
            # Planning Cartesian Path
            waypoints = []

            wpose = move_group.get_current_pose().pose
            
            _feedback.achieved_orientation_x = wpose.orientation.x
            _feedback.achieved_orientation_y = wpose.orientation.y
            _feedback.achieved_orientation_z = wpose.orientation.z
            _feedback.achieved_orientation_w = wpose.orientation.w
            _feedback.achieved_position_x = wpose.position.x
            _feedback.achieved_position_y = wpose.position.y
            _feedback.achieved_position_z = wpose.position.z

            # Publish info to the console for the user
            rospy.loginfo('\n%s: Executing, pin backside closing cartesian path planner action server, current arm configuration:\nOrientation x= %s\nOrientation y= %s\nOrientation z= %s\nOrientation w= %s\nPosition x= %s\nPosition y= %s\nPosition z= %s'
                        % (self._action_name, _feedback.achieved_orientation_x, 
                            _feedback.achieved_orientation_y, _feedback.achieved_orientation_z, _feedback.achieved_orientation_w, 
                            _feedback.achieved_position_x, _feedback.achieved_position_y, _feedback.achieved_position_z))
            
            print("\n",resp1.trajectory)
            # Orient the end-effector and move in the specified direction
            # Enter the direction and the offset amount
            for i in range(0, len(resp1.trajectory)-7, 7):
                wpose.orientation.x -= (resp1.trajectory[i+7] - resp1.trajectory[i])
                wpose.orientation.w -= (resp1.trajectory[i+10] - resp1.trajectory[i+3])
                wpose.position.y += resp1.trajectory[i+12] - resp1.trajectory[i+5]
                wpose.position.z += resp1.trajectory[i+13] - resp1.trajectory[i+6]
                waypoints.append(copy.deepcopy(wpose))

            (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,  # waypoints to follow
                                        0.01,       # eef_step
                                        0.0)        # jump_threshold
            
            print("\nFRACTION = ", fraction)
        

            # Call the planner to compute the plan and execute it
            # 'go()' returns a boolean indicating whether the plan and execute it
            success = move_group.execute(plan, wait=True)

            self._as.publish_feedback(_feedback)

            if success:
                _result.finish = True

                rospy.loginfo('%s: Operation Result: Succeeded' % self._action_name)
                self._as.set_succeeded(_result, "True")

                apose = self.move_group.get_current_pose().pose

                achieved_orientation_x = apose.orientation.x
                achieved_orientation_y = apose.orientation.y
                achieved_orientation_z = apose.orientation.z
                achieved_orientation_w = apose.orientation.w
                achieved_position_x = apose.position.x
                achieved_position_y = apose.position.y
                achieved_position_z = apose.position.z

                # Publish info to the console for the user
                rospy.loginfo('Current arm configuration:\nOrientation x= %s\nOrientation y= %s\nOrientation z= %s\nOrientation w= %s\nPosition x= %s\nPosition y= %s\nPosition z= %s'
                            % (achieved_orientation_x, 
                                achieved_orientation_y, achieved_orientation_z, achieved_orientation_w, 
                                achieved_position_x, achieved_position_y, achieved_position_z))


            else:
                _result.finish = False
                rospy.loginfo('%s: Operation Result: Failed' % self._action_name)
                self._as.set_aborted(_result, "False")

if __name__ == '__main__':
    rospy.init_node('pin_close_bs_traj_action')
    server = PinCloseBSTrajActionServer(rospy.get_name())
    rospy.spin()