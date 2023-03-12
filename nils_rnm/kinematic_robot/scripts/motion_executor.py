#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import Float64MultiArray
import csv
import sys
import numpy as np
import time
from sensor_msgs.msg import JointState

class MotionExecutor:
    def __init__(self, command_topic, robot_kinematics, joint_state_topic):
        
        self.pub_joint_state    = rospy.Publisher(command_topic, Float64MultiArray, queue_size=1)
        self.robot_kinematics   = robot_kinematics
        self.publish_list       = []
        self.current_joint_state = [0,0,0,0,0,0,0]
        self.current_joint_state_sub    = rospy.Subscriber(joint_state_topic, JointState, self.callback_joint_states)
    
    def callback_joint_states(self, msg):
        ''' Callback function for the topic_joint_states. Stores current angles in object variable'''
        self.current_joint_state = msg.position
        return

    def run(self, filename, current_pose, MOVEMENT_SPEED):
        time.sleep(1) #FIXME time or rostime
        publish_list = self.get_joint_list(filename)
        self.move_to_start(current_pose, publish_list, MOVEMENT_SPEED)
        self.publish_joint(publish_list)
    
    def run_reversed(self, filename, current_pose, MOVEMENT_SPEED):
        time.sleep(1)
        publish_list = self.get_joint_list(filename)
        reversed_list = self.reverse_list(publish_list)
        self.move_to_start(current_pose, reversed_list, MOVEMENT_SPEED)
        self.publish_joint(reversed_list)

    def reverse_list(self, publish_list):
        reversed_list = []
        for i in range(len(publish_list)):
            reversed_list.append(publish_list[-1-i])
        return reversed_list

    def get_joint_list(self, filename):
        publish_list = []
        with open((os.path.join(os.path.dirname(__file__),filename))) as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                publish_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
        rospy.logwarn("[ME] Got Joint List")
        return publish_list

    def move_to_start(self, current_pose, list, MOVEMENT_SPEED):
        """
        Moves robot in desired start position. To get there a linear path between the current joint state and the target joint state 
        will be interpolatet. The movement speed is set and should not be above 0.02 m/s.
        """
        #FIXME mit trajectory_planner kombinieren -> Methode schreiben

        # get current cartesian robot state
        current_pose_cartesian = self.robot_kinematics.get_pose_from_angles(current_pose)

        # get target cartesian robot state
        start_pose_cartesian = self.robot_kinematics.get_pose_from_angles(list[0])     # /FIXME confirm correction and error handling if List is empty
        delta_pose_cartesian = start_pose_cartesian - current_pose_cartesian
        delta_pose = np.array(list[0]) - np.array(current_pose)
        err_tol = 1e-2
        max_delta_pose = np.abs(delta_pose).max()
        if max_delta_pose > err_tol:
            #FIXME change into control of joint movements
            #rospy.logwarn("[ME] Robot NOT at planned start pose")  
            # calculate distance between cartesian coordinates of current and start position
            steps           = int(max_delta_pose / MOVEMENT_SPEED)
            delta_joints_per_step = np.abs(delta_pose).max() / steps
            
            # set Updaterate to 1000 Hz and publish every 1ms a new joint state
            rate    = rospy.Rate(1000)
            for j in range(steps+1):

                joint = current_pose + j*delta_joints_per_step
                msg = Float64MultiArray()
                msg.data = joint
                self.pub_joint_state.publish(msg)
                rate.sleep()
            rospy.logwarn("[ME] Moved robot to planned start pose")   
        else:
            rospy.logwarn("[ME] Robot at planned start pose")
        
        return

    def publish_joint(self, list):
        """
        /TODO write comment
        """
        # publish new joint every 1ms
        rate    = rospy.Rate(1000)
        current_joint = self.current_joint_state
        rospy.logwarn("[ME] Start publishing joints.")
        for i in range(len(list)):
           
           
            msg = Float64MultiArray()
            msg.data = list[i]
            self.pub_joint_state.publish(msg)
            pose_reached = self.control_movement_err(list[i])
            while not pose_reached: 
                self.pub_joint_state.publish(msg)
                pose_reached = self.control_movement_err(list[i])
                rate.sleep()
            rate.sleep()        

        """
        TODO Joint Limit Speed Controller implementieren
            list_speed_control = self.control_joint_speed(current_joint, next_joint)
            for i in range(len(list_speed_control)):
                msg = Float64MultiArray()
                msg.data = list_speed_control[i]
                self.pub_joint_state.publish(msg)
                pose_reached = self.control_movement_err(list_speed_control[i])
                while not pose_reached: 
                    if pose_reached:
                        break
                    self.pub_joint_state.publish(msg)
                    pose_reached = self.control_movement_err(list_speed_control[i])
                
            current_joint = next_joint
        """
        rospy.logwarn("[ME] Published all Joints.")
    
    def control_movement_err(self, goal_pose, max_err=1e-2):
        
        
        diff = np.array(goal_pose)-np.array(self.current_joint_state)
        err = np.abs(diff).max() 
        if err >= max_err:
            rospy.logwarn("[ME] Can not follow published joint. Wait for robot to reach joint.")
            return False
        else:
            return True
    
    def control_joint_speed(self, current_pose, goal_pose):
        limits = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])*1e-3 # max rad/s for sampling of 1000Hz
        diff = np.absolute(np.array(goal_pose)-np.array(current_pose))
        list_speed_control = []
        if any(diff > limits):

            rospy.logwarn("[ME] reached limits. Diff:" + str(diff))
            list_speed_control.append(goal_pose)
            max_deviation = np.abs(diff-limits).max()
            steps = int(max_deviation/(np.amin(limits))+1)
            delta_joints_per_step = (np.array(goal_pose) - np.array(current_pose)) / steps
            for j in range(steps + 1):
                    sample_joint = current_pose + j * delta_joints_per_step
                    list_speed_control.append(sample_joint)
            rospy.logwarn("[ME] new list:" + str(list_speed_control))
        else:
            list_speed_control.append(goal_pose)
        return list_speed_control


    def slow_start_controller(self):
        pass
                    
# for testing purpose
def main(argv):
    rospy.init_node("variable_publisher")
    command_topic = rospy.get_param("~command_topic", "/joint_position_example_controller_sim/joint_command")
    rospy.logwarn("[ME] Using topic: " + str(command_topic))

    MotionExecution = MotionExecutor(command_topic)
    MotionExecution.run()

if __name__ == '__main__':
    main(sys.argv)