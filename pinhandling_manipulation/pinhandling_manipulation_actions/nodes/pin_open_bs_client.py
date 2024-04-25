#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import actionlib
import pinhandling_manipulation_actions.msg

from gazebo_ros_link_attacher.srv import Attach, AttachRequest

def pin_open_bs_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (ee_coordinateAction) to the constructor.
    client = actionlib.SimpleActionClient('pin_open_bs_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

    # Wait until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

    # Send the goal to the action server
    client.send_goal(goal)
    # Wait for the server to finish performing the action
    client.wait_for_result()
    rospy.sleep(2)

    i = 1
    while client.get_goal_status_text() != "True":
        rospy.loginfo("Couldn't find any solution at trial number: %d\nTrying again...\n", i)
        # Send the goal to the action server
        client.send_goal(goal)
        # Wait for the server to finish performing the action
        client.wait_for_result()
        i+=1

        if i==6:
            rospy.loginfo("Couldn't find any solution: Aborting...")
            break

    if client.get_goal_status_text() == "True":
        rospy.loginfo("Solution found at trial number: %d\n", i)

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
        
        attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        # Link them
        rospy.loginfo("Attaching Magnetic Gripper and Pin")
        rospy.sleep(5)
        req = AttachRequest()

        req.model_name_1 = "robot"
        req.link_name_1 = "robot_arm_wrist_3_link"
        req.model_name_2 = "table"
        req.link_name_2 = "pin_link"

        attach_srv.call(req)
    else:
        return client.get_result()
    
    rospy.loginfo("Starting Cartesian Trajectory")
    rospy.sleep(3)
    
    client2 = actionlib.SimpleActionClient('pin_open_bs_traj_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

    # Wait until the action server has started up and started listening for goals.
    client2.wait_for_server()

    # Creates a goal to send to the action server.
    goal2 = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

    # Send the goal to the action server
    client2.send_goal(goal2)
    # Wait for the server to finish performing the action
    client2.wait_for_result()
    rospy.sleep(2)

    # While loop to make the system more robust, but somehow Cartesian Planner returns "True" even if 
    # it doesn't complete the trajectory properly. So, decide later whether this loop is relevant.
    
    # p = 1
    # while client2.get_goal_status_text() != "True":
    #     rospy.loginfo("Couldn't find any solution at trial number: %d\nTrying again...\n", p)
    #     # Send the goal to the action server
    #     client2.send_goal(goal2)
    #     # Wait for the server to finish performing the action
    #     client2.wait_for_result()
    #     p+=1

    #     if p==6:
    #         rospy.loginfo("Couldn't find any solution: Aborting...")
    #         break

    if client2.get_goal_status_text() == "True" or client2.get_goal_status_text() == "False":
        # rospy.loginfo("Solution found at trial number: %d\n", p)

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
        
        attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        # Link them
        rospy.loginfo("Detaching Magnetic Gripper and Pin")
        req = AttachRequest()

        req.model_name_1 = "robot"
        req.link_name_1 = "robot_arm_wrist_3_link"
        req.model_name_2 = "table"
        req.link_name_2 = "pin_link"

        attach_srv.call(req)

    rospy.loginfo("Setting home position")
    
    client3 = actionlib.SimpleActionClient('set_home_position_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

    # Wait until the action server has started up and started listening for goals.
    client3.wait_for_server()

    # Creates a goal to send to the action server.
    goal3 = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

    # Send the goal to the action server
    client3.send_goal(goal3)
    # Wait for the server to finish performing the action
    client3.wait_for_result()
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        # Initialize a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS
        rospy.init_node('pin_open_bs_client_py')
        result = pin_open_bs_client()
        

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)