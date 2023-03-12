#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class DummyPublisher:
    def __init__(self):
        pass

    def main(self):
        
        # Init
        rospy.init_node("dummy_publisher")

        publish_joint_states = rospy.Publisher(name="/joint_states", data_class=JointState, queue_size=20)
        pub_goal_pose_js     = rospy.Publisher(name="~/goal_pose_js", data_class=Float64MultiArray, queue_size=20 )


        message = JointState()

        message.name     = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        message.position = [np.deg2rad(10), np.deg2rad(5), np.deg2rad(0), np.deg2rad(10), np.deg2rad(5), np.deg2rad(0),np.deg2rad(10)]
        message.velocity = [0, 0, 0, 0, 0, 0, 0]

        msg         = Float64MultiArray()
        msg.data    = [np.deg2rad(10), np.deg2rad(5), np.deg2rad(0), np.deg2rad(10), np.deg2rad(5), np.deg2rad(0),np.deg2rad(10)]
        

        # Loop
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
        
            publish_joint_states.publish(message)
            pub_goal_pose_js.publish(msg)

            rate.sleep()
        

if __name__ == '__main__':
    
    dummy_publisher = DummyPublisher()
    dummy_publisher.main()

