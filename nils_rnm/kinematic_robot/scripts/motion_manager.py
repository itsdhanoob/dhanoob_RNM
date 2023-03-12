#!/usr/bin/env python3
import csv
import os
from pickle import TRUE
import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from path_planner import path_planner
from robot_kinematics import robot_kinematics
from trajectory_planner_simple import trajectory_planner_simple
from motion_executor import MotionExecutor

class MotionManager:
    def __init__(self, joint_command_topic, joint_state_topic="/joint_states", err_tol=1e-3, debug=False):

        # ROS communication
        #rospy.init_node("motion_manager")
        
        # Subscriber
        self.current_joint_state = [0,0,0,0,0,0,0]
        self.current_joint_state_sub    = rospy.Subscriber(joint_state_topic, JointState, self.callback_joint_states)
        
        # Objects
        self.kinematics         = robot_kinematics()
        self.path_planner       = path_planner(self.kinematics)
        self.trajectory_planner = trajectory_planner_simple(self.kinematics)
        self.motion_executor    = MotionExecutor(joint_command_topic, self.kinematics, joint_state_topic="/joint_states")

        # File-Names
         ## Path
        self.filename_path_preinsertion_cartesian    = "Path/calculated_path_preinsertion_cartesian.csv"
        self.filename_path_insertion_cartesian       = "Path/calculated_path_insertion_cartesian.csv"
        self.filename_path_preinsertion_joint_space  = "Path/calculated_path_preinsertion_jointspace.csv"
        self.filename_path_insertion_joint_space     = "Path/calculated_path_insertion_jointspace.csv"
         ## Trajectory
        self.filename_trajectory_preinsertion    = "Trajectory/created_trajectory_to_goal_1ms.csv"
        self.filename_trajectory_insertion       = "Trajectory/created_trajectory_to_insertion_1ms.csv"



        # Constants and parameters
        """FIXME Pose2Angle error, can not create path when having different rotation in endefector
        Offset neadle!! Wir bekommen ein Target A, welches zuruckgerechnet werden muss bis zum Joint, dass ist Zielposition
        """

        #self.pos_target         = [0.37982467,  0.35923062,  0.43909587]
        #self.pos_target         = [0.31982467,  0.05923062,  0.23909587]
        
        # TODO clean up
        #self.current_theta      = self.current_joint_state
        #self.current_A          = self.kinematics.get_pose_from_angles(self.current_theta)
        #self.current_rot        = self.current_A[:9]
        #self.curr_goal_pose     = [self.current_rot[0], self.current_rot[1], self.current_rot[2], self.current_rot[3], self.current_rot[4], self.current_rot[5], self.current_rot[6], self.current_rot[7], self.current_rot[8], self.pos_target[0], self.pos_target[1], self.pos_target[2]]
        self.err_tolerance      = err_tol

        # Load Parameter

        self.max_dist_between_supports = rospy.get_param("~max_dist_between_supports", 1)

        # Debug
        #rospy.logwarn("[MM] Current A:" + str(self.current_A))

    def callback_joint_states(self, msg):
        ''' Callback function for the topic_joint_states. Stores current angles in object variable'''
        self.current_joint_state = msg.position
        return
  
    def move2goal_js(self, GoalPose, MOVEMENT_SPEED):
        """
        Move the robot's endefector to goal pose given in joint space. 
        To do so:
        1. interpolate actual to goal joint angles and sample to 1ms steps
        2. Load trajectory and execute motion
        """
        
        self.trajectory_planner.create_point_to_point_traj(self.current_joint_state, GoalPose, MOVEMENT_SPEED, self.filename_path_preinsertion_joint_space, self.filename_trajectory_preinsertion) 
        self.motion_executor.run(self.filename_trajectory_preinsertion, self.get_current_pose(), MOVEMENT_SPEED)
        return True

    def move_start2preinsertion(self, needle_goal_pose, MOVEMENT_SPEED): #FIXME rename into curent2preinsection
        self.path_planner.calculate_path(self.current_joint_state, needle_goal_pose, self.get_max_dist_between_waypoints_preinsertion(), self.get_max_dist_between_waypoints_insertion(), self.filename_path_preinsertion_cartesian, self.filename_path_insertion_cartesian, self.filename_path_preinsertion_joint_space, self.filename_path_insertion_joint_space)
        #TODO calcutlation intersectionpoint2target mit mehr waypoints
        self.trajectory_planner.create_simple_trajectory_JS(self.filename_path_preinsertion_joint_space, self.filename_trajectory_preinsertion) #FIXME Change for Cem trajectory planner
        self.motion_executor.run(self.filename_trajectory_preinsertion, self.get_current_pose(), MOVEMENT_SPEED)
        return True

    def move_preinsertion2target(self, MOVEMENT_SPEED):
        
        self.trajectory_planner.create_simple_trajectory_JS(self.filename_path_insertion_joint_space, self.filename_trajectory_insertion)
        self.motion_executor.run(self.filename_trajectory_insertion, self.get_current_pose(), MOVEMENT_SPEED)
        return True

    def move_target2init_pose(self, MOVEMENT_SPEED): # TODO need to be debugged!
        self.motion_executor.run_reversed(self.filename_trajectory_insertion, self.get_current_pose(), MOVEMENT_SPEED)
        self.motion_executor.run_reversed(self.filename_trajectory_preinsertion, self.get_current_pose(), MOVEMENT_SPEED)
        return True

    def get_max_dist_between_waypoints_preinsertion(self):
        # maximum distance between each waypoint (|x/y/z|), no rotation is taken into account
        # TODO 
        max_dist_between_supports = rospy.get_param("/max_dist_between_supports_preinsertion", 100)
        return max_dist_between_supports
    
    def get_max_dist_between_waypoints_insertion(self):
        # maximum distance between each waypoint (|x/y/z|), no rotation is taken into account
        # TODO 
        max_dist_between_supports = rospy.get_param("/max_dist_between_supports_insertion", 0.01)
        return max_dist_between_supports
    
    def get_current_pose(self):
        current_pose = self.current_joint_state
        return current_pose


if __name__ == '__main__':
    rospy.init_node("motion_manager")
    #operation_mode      = rospy.get_param("/operation_mode")
    joint_command_topic   = rospy.get_param("joint_command_topic", "/joint_position_example_controller_sim/joint_command")

    motion_manager = MotionManager(joint_command_topic)
    MOVEMENT_SPEED = 0.005/1000
    GoalPose = [-7.455726072969071e-06, -3.5540748690721102e-06, -6.046157276173858e-06, -0.7851757638374179, 4.600804249577095e-06, 1.4001585464384902e-06, 1.013981160369326e-06]
    needle_goal_pose = [7.07267526e-01, -5.96260536e-06 ,-7.06945999e-01 ,-1.09650444e-05, -1.00000000e+00 ,-2.53571628e-06 ,-7.06945999e-01 , 9.54512406e-06 ,-7.07267526e-01,  0.30874679,  0.24655161, 0.45860086]
    motion_manager.move2goal_js(GoalPose, MOVEMENT_SPEED)

    motion_manager.move_start2preinsertion(needle_goal_pose, MOVEMENT_SPEED)
        
    motion_manager.move_preinsertion2target(MOVEMENT_SPEED)