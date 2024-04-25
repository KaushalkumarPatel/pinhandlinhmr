#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from pinhandling_manipulation_actions.msg import PinOpenCloseBsAction, PinOpenCloseBsFeedback, PinOpenCloseBsResult
from pinhandling_manipulation_services.srv import CartesianTrajectory

class PinOpenBSActionServer():
    # Create messages that are used to publish feedback/result

    def __init__(self, name):

        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiating a RobotCommander object. 
        # Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiating a PlanningSceneInterface object. 
        # This provides a remote interface for getting, setting, 
        # and updating the robot’s internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz.
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Instantiating a MoveGroupCommander object. 
        # This object is an interface to a planning group (group of joints).
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = self.move_group.get_planning_frame()
        print(planning_frame)

        # Service Server Initialization part
        # For 10 points
        # self.grab_Y = 0.08085
        # self.grab_Z = 0.14
        # self.pin_X = 0.04
        # self.pin_Y = 0.560
        # self.pin_Z = 0.397
        # self.degree = 10.0
        # self.points = 10
        
        # For 90 points
        self.grab_Y = 0.08085
        self.grab_Z = 0.14
        self.pin_X = -0.47
        self.pin_Y = 1.060
        self.pin_Z = 0.397
        self.degree = 1.0
        self.points = 90

        rospy.wait_for_service('cartesian_path_bs_open')
        self.move_group.set_pose_reference_frame('robot_arm_base_link')

        # Action Server Initialization part
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name,      # Server name string
                                    PinOpenCloseBsAction,                # Action message type
                                    self.publish_position,              # Action Function
                                    auto_start = False 
                                    )
        
        self._as.start()

    def publish_position(self, goal):

        if goal:
            _feedback = PinOpenCloseBsFeedback()
            _result = PinOpenCloseBsResult()
            apose = self.move_group.get_current_pose().pose

            print("Requesting Cartesian Trajectory calculation for backside pin open")
            
            cartesian_path_bs_open = rospy.ServiceProxy('cartesian_path_bs_open', # service name
                                                    CartesianTrajectory   # service type
                                                    )
            resp1 = cartesian_path_bs_open(
                                    self.grab_Y, 
                                    self.grab_Z, 
                                    self.pin_X, 
                                    self.pin_Y, 
                                    self.pin_Z, 
                                    self.degree, 
                                    self.points
                                    )
            

            _feedback.achieved_orientation_x = apose.orientation.x
            _feedback.achieved_orientation_y = apose.orientation.y
            _feedback.achieved_orientation_z = apose.orientation.z
            _feedback.achieved_orientation_w = apose.orientation.w
            _feedback.achieved_position_x = apose.position.x
            _feedback.achieved_position_y = apose.position.y
            _feedback.achieved_position_z = apose.position.z
            
            # Publish info to the console for the user
            rospy.loginfo('\n%s: Executing, pin backside opening action server, current arm configuration:\nOrientation w= %s\nOrientation x= %s\nOrientation y= %s\nOrientation z= %s\nPosition x= %s\nPosition y= %s\nPosition z= %s'
                        % (self._action_name, _feedback.achieved_orientation_w, _feedback.achieved_orientation_x, 
                            _feedback.achieved_orientation_y, _feedback.achieved_orientation_z, 
                            _feedback.achieved_position_x, _feedback.achieved_position_y, _feedback.achieved_position_z))
            
            # Setting up the reference frame of the kinematic chain

            # For using in lab environment, uncomment the line below
            # self.move_group.set_pose_reference_frame('base_link')
            # For using on mobile platform, uncomment the line below
            self.move_group.set_pose_reference_frame('robot_arm_base_link')

            # Plan a motion for this group to a desired pose for the end-effector
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = resp1.trajectory[0]
            pose_goal.orientation.y = resp1.trajectory[1]
            pose_goal.orientation.z = resp1.trajectory[2]
            pose_goal.orientation.w = resp1.trajectory[3]
            pose_goal.position.x = resp1.trajectory[4]
            pose_goal.position.y = resp1.trajectory[5]
            pose_goal.position.z = resp1.trajectory[6]

            print("\nEnd Effector sending to:\n", pose_goal)

            self.move_group.set_pose_target(pose_goal)

            # Call the planner to compute the plan and execute it
            # 'go()' returns a boolean indicating whether the plan and execute it
            success = self.move_group.go(wait=True)

            self._as.publish_feedback(_feedback)

            if success:
                _result.finish = True

                rospy.loginfo('%s: Operation Result: Succeeded' % self._action_name)
                self._as.set_succeeded(_result, "True")
                
                # Call 'stop()' to ensure that there is no residual movement
                self.move_group.stop()
                # Clear targets after planning poses
                self.move_group.clear_pose_targets()

            else:
                _result.finish = False
                rospy.loginfo('%s: Operation Result: Failed' % self._action_name)
                self._as.set_aborted(_result, "False")

                # Call 'stop()' to ensure that there is no residual movement
                self.move_group.stop()
                # Clear targets after planning poses
                self.move_group.clear_pose_targets()

if __name__ == '__main__':
    rospy.init_node('pin_open_bs_action')
    server = PinOpenBSActionServer(rospy.get_name())
    rospy.spin()