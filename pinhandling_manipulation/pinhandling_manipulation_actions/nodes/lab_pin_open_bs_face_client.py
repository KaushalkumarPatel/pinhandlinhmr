#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import actionlib
import pinhandling_manipulation_actions.msg
import socket

# Define the Modbus parameters
robot_ip = "192.168.56.1"
port = 502  # Default Modbus TCP port

def pin_open_bs_face_client():

    client_threshold = 'False'

    while client_threshold != 'True':

        # Turn off the magnetic gripper
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((robot_ip, port))

        # Construct the Modbus TCP packet to write '1' to register 128
        # The packet format is: [Transaction Identifier][Protocol Identifier][Length][Unit Identifier][Function Code][Starting Address][Number of Registers][Data]
        packet = b"\x00\x01\x00\x00\x00\x06\x00\x06\x00\x80\x00\x00"  # Write single register (coil) at address 128 with value 1

        # Send the packet to the server
        s.send(packet)

        # Close the connection
        s.close()
        rospy.sleep(1)

        rospy.loginfo("Setting arm to 'operation ready' pose")
        # Set the position to operation ready
        client = actionlib.SimpleActionClient('set_operation_bs_face_position_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

        # Wait until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

        # Send the goal to the action server
        client.send_goal(goal)
        # Wait for the server to finish performing the action
        client.wait_for_result()
        rospy.sleep(2)

        rospy.loginfo("Sending arm to grasping position to close the pin")
        # Creates the SimpleActionClient, passing the type of the action
        # (ee_coordinateAction) to the constructor.
        client1 = actionlib.SimpleActionClient('pin_open_bs_face_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

        # Wait until the action server has started up and started listening for goals.
        client1.wait_for_server()

        # Creates a goal to send to the action server.
        goal1 = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

        # Send the goal to the action server
        client1.send_goal(goal1)
        # Wait for the server to finish performing the action
        client1.wait_for_result()
        rospy.sleep(2)

        i = 1
        while client1.get_goal_status_text() != "True":
            rospy.loginfo("Couldn't find any solution at trial number: %d\nTrying again...\n", i)
            # Send the goal to the action server
            client1.send_goal(goal1)
            # Wait for the server to finish performing the action
            client1.wait_for_result()
            i+=1

            if i==6:
                rospy.loginfo("Couldn't find any solution: Aborting...")
                break

        if client1.get_goal_status_text() == "True":
            # Create a socket object and connect to the server
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((robot_ip, port))

            # Construct the Modbus TCP packet to write '1' to register 128
            # The packet format is: [Transaction Identifier][Protocol Identifier][Length][Unit Identifier][Function Code][Starting Address][Number of Registers][Data]
            packet = b"\x00\x01\x00\x00\x00\x06\x00\x06\x00\x80\x00\x01"  # Write single register (coil) at address 128 with value 1

            # Send the packet to the server
            s.send(packet)

            # Close the connection
            s.close()
        else:
            return client1.get_result()
        
        rospy.loginfo("Starting Cartesian Trajectory")
        rospy.sleep(2)
        client2 = actionlib.SimpleActionClient('pin_open_bs_face_traj_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

        # Wait until the action server has started up and started listening for goals.
        client2.wait_for_server()

        # Creates a goal to send to the action server.
        goal2 = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

        # Send the goal to the action server
        client2.send_goal(goal2)
        # Wait for the server to finish performing the action
        client2.wait_for_result()

        client_threshold = client2.get_goal_status_text()
        if client_threshold == 'False':
            rospy.loginfo("Fraction is below 0.8, restarting the process")
            
        rospy.sleep(1)

    # Create a socket object and connect to the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((robot_ip, port))

    # Construct the Modbus TCP packet to write '1' to register 128
    # The packet format is: [Transaction Identifier][Protocol Identifier][Length][Unit Identifier][Function Code][Starting Address][Number of Registers][Data]
    packet = b"\x00\x01\x00\x00\x00\x06\x00\x06\x00\x80\x00\x00"  # Write single register (coil) at address 128 with value 1

    # Send the packet to the server
    s.send(packet)

    # Close the connection
    s.close()

    # Moving ee to escape from the pin
    client3 = actionlib.SimpleActionClient('pin_open_bs_face_last_manoeuvre_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

    # Wait until the action server has started up and started listening for goals.
    client3.wait_for_server()

    # Creates a goal to send to the action server.
    goal3 = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

    # Send the goal to the action server
    client3.send_goal(goal3)
    # Wait for the server to finish performing the action
    client3.wait_for_result()
    rospy.sleep(1)

    # Set the operation ready position
    rospy.loginfo("Setting arm to 'operation ready' pose")
    client4 = actionlib.SimpleActionClient('set_operation_bs_face_position_action', pinhandling_manipulation_actions.msg.PinOpenCloseBsAction)

    # Wait until the action server has started up and started listening for goals.
    client4.wait_for_server()

    # Creates a goal to send to the action server.
    goal4 = pinhandling_manipulation_actions.msg.PinOpenCloseBsGoal(start = True)

    # Send the goal to the action server
    client4.send_goal(goal4)
    # Wait for the server to finish performing the action
    client4.wait_for_result()
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        # Initialize a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS
        rospy.init_node('pin_open_bs_face_client_py')
        result = pin_open_bs_face_client()
        

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)