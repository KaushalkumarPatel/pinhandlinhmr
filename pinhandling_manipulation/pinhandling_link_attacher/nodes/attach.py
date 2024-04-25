#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class LinkAttacherNode:
    """
    The Pinhandling Link Attacher Node
    """
    def __init__(self):
        rospy.init_node('attach_links')

    def attach_links(self):
        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
        
        attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        # Link them
        rospy.loginfo("Attaching Magnetic Gripper and Cube")
        req = AttachRequest()

        # For lab environment uncommnent below
        # req.model_name_1 = "robot"
        # req.link_name_1 = "wrist_3_link"
        # req.model_name_2 = "robot"
        # req.link_name_2 = "pin_link"

        # For mobile platform uncommnent below
        req.model_name_1 = "robot"
        req.link_name_1 = "robot_arm_wrist_3_link"
        req.model_name_2 = "table"
        req.link_name_2 = "pin_link"

        attach_srv.call(req)

def main():
    node = LinkAttacherNode()

    node.attach_links()

if __name__ == '__main__':
    main()