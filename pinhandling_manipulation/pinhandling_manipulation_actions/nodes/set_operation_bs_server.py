#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import actionlib

from pinhandling_manipulation_actions.msg import PinOpenCloseBsAction, PinOpenCloseBsFeedback, PinOpenCloseBsResult

class SetOperationBSPositionServer():
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

        # Instantiating a MoveGroupCommander object. 
        # This object is an interface to a planning group (group of joints).
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

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

            print("Setting the arm to the Operation position")

            _feedback.achieved_orientation_x = apose.orientation.x
            _feedback.achieved_orientation_y = apose.orientation.y
            _feedback.achieved_orientation_z = apose.orientation.z
            _feedback.achieved_orientation_w = apose.orientation.w
            _feedback.achieved_position_x = apose.position.x
            _feedback.achieved_position_y = apose.position.y
            _feedback.achieved_position_z = apose.position.z
            
            # Publish info to the console for the user
            rospy.loginfo('\n%s: Executing, set Operation position server'
                        % self._action_name)

            self.move_group.set_named_target('operation_ready_bs')

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
    rospy.init_node('set_operation_bs_position_action')
    server = SetOperationBSPositionServer(rospy.get_name())
    rospy.spin()