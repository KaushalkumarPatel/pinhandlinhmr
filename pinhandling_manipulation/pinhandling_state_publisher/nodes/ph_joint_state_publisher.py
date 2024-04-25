#!/usr/bin/python3

import rospy
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

REAL_COMMAND_TOPIC = '/scaled_pos_traj_controller/follow_joint_trajectory/command'
SIM_COMMAND_TOPIC = '/arm/scaled_pos_traj_controller/command'


class JointStatePublisherNode:
    """
    The Pinhandling Joint State Publisher
    """

    def __init__(self, name, mode, position, exec_time):

        rospy.init_node(name, anonymous=False)
        command_topic = None
        if not mode in {'real', 'sim'}:
            raise ValueError(f'mode {mode} is not a valid mode')
        
        self.mode = mode
        
        if mode == 'real':
            command_topic = REAL_COMMAND_TOPIC
        elif mode == 'sim':
            command_topic = SIM_COMMAND_TOPIC

        if not command_topic is None:
            self.joint_publisher = rospy.Publisher(
                command_topic, JointTrajectory, queue_size=10)
            
        if not position in {'home', 'operation'}:
            raise ValueError(f'position {position} is not a valid position')
        
        self.position = position

        if position == 'home':
            self.des_position = [
                0.0, 
                0.0, 
                0.0, 
                0.0, 
                0.0, 
                0.0]
        elif position == 'operation':
            self.des_position = [
                2.3562, 
                -2.3562, 
                0.0, 
                3.1415, 
                -1.5708, 
                0.0]
        
        self.exec_time = exec_time

    def send_joint_command(self):

        msg = JointTrajectory()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.joint_names = [
            'elbow_joint', 
            'shoulder_lift_joint', 
            'shoulder_pan_joint', 
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint']
        
        point = JointTrajectoryPoint()

        point.positions = self.des_position
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = rospy.Duration(self.exec_time) # This part of the message determines our robot's movement time from its current pos to desired pos

        msg.points.append(point)
        rospy.loginfo(msg)

        self.joint_publisher.publish(msg)
    
    def run(self):
        rate = rospy.Rate(100)
        while ~rospy.is_shutdown():
            self.send_joint_command()
            rate.sleep()


def main(argv):
    if len(argv) < 4:
        print("Usage: rosrun pinhandling_state_publisher ph_joint_state_publisher.py <mode> <position> <exec_time>")
        return

    mode = argv[1]
    position = argv[2]
    exec_time = int(argv[3])

    node = JointStatePublisherNode('ph_joint_state_publisher_node', 
                                   mode, 
                                   position, 
                                   exec_time
                                   )

    node.run()

if __name__ == '__main__':
    main(sys.argv)

