#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class LinkDetacherNode:
    """
    The Pinhandling Link Detacher Node
    """
    def __init__(self):
        rospy.init_node('detach_links')

    def detach_links(self):
        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
        
        attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")


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
    node = LinkDetacherNode()

    node.detach_links()


if __name__ == '__main__':
    main()