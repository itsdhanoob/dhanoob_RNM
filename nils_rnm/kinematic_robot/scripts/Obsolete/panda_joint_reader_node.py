#!/usr/bin/python3

import rospy
import sys
from sensor_msgs.msg import JointState


def callback(msg):
    pos = msg.position
    print(pos)


def main(argv):
    # Initialize ROS connection
    # The script with the main function needs to be executable.
    # You can make the file executable by executing 'chmod +x /path/to/file.py'
    # in the terminal (without the "'")
    rospy.init_node("panda_joint_reader_node", argv)

    # Read parameters specified in the .launch file
    # The "~" in the beginning means that it will use parameters from the node
    # e.g. /node_name/parameter_name can then be
    # read by rospy.get_param("parameter_name", default)
    topic_name = rospy.get_param("~topic_name", "/default_topic_name")
    queue_size = rospy.get_param("~queue_size", 10)

    # Register a callback function (a function that is called every time a new message arrives)
    rospy.Subscriber(name=topic_name,
                     data_class=JointState,
                     callback=callback,
                     queue_size=queue_size)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

