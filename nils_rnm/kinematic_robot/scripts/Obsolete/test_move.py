#!/usr/bin/python3
from math import cos

import rospy
import sys
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float64MultiArray


class RobotArm:
    def __init__(self, move_dist, command_topic):
        # There are small differences between the simulation environment and the real hardware
        # One difference is the topic name
        if "sim" in command_topic:
            msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10))
        else:
            msg = rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10))

        self.initial_position = msg.position
        self.move_dist = move_dist
        # Setup the publisher
        self.publisher = rospy.Publisher(command_topic, Float64MultiArray, queue_size=1)
        self.counter = 0

    def send_step_command(self):
        goal = np.zeros((7, 1))
        delta_angle = (1. - cos(np.pi / 5.0 * self.counter/1000.)) * self.move_dist * np.pi / 180.
        for i in range(7):
            if i == 4:
                goal[i] = self.initial_position[i] - delta_angle
            else:
                goal[i] = self.initial_position[i] + delta_angle
        self.counter += 1

        # Create the message and send it to the advertised topic
        msg = Float64MultiArray()
        msg.data = goal
        self.publisher.publish(msg)


def main(argv):
    # Initialize ROS connection
    # The script with the main function needs to be executable.
    # You can make the file executable by executing 'chmod +x /path/to/file.py'
    # in the terminal (without the "'")
    rospy.init_node("simple_trajectory_player", argv)

    # Read parameters specified in the .launch file
    # The "~" in the beginning means that it will use parameters from the node
    # e.g. /node_name/parameter_name can then be
    # read by rospy.get_param("parameter_name", default)
    move_dist = rospy.get_param("~joint_move_dist", 10)
    command_topic = rospy.get_param("~command_topic", "/default_command_topic")

    rospy.logwarn("Using topic " + str(command_topic) + " and distance " + str(move_dist))
    robot_arm = RobotArm(move_dist, command_topic)

    # Loop infinitely with a fixed frequency of 1000 hz
    rate = rospy.Rate(1000)
    while ~rospy.is_shutdown():
        robot_arm.send_step_command()
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)

