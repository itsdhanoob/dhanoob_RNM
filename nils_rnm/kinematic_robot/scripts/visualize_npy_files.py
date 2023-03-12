#!/usr/bin/env python
import os
import sys
import time
import cv2
import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from numpy.linalg import inv
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Int16, String
from sympy import true

from robot_kinematics import robot_kinematics



HM_cam2gripper = np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/backup/HM_cam2gripper.npy")
HM_gripper2cam = np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/backup/HM_gripper2cam.npy")
HM_ir2rgb = np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/backup/HM_ir2rgb.npy")
HM_rgb2ir = np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/backup/HM_rgb2ir.npy")


print("Rosbag Cam2Gripper")
print(HM_cam2gripper)
print("\n Rosbag Gripper2Cam")
print(HM_gripper2cam)

# print("Rosbag HM_ir2rgb")
# print(HM_ir2rgb)
# print("\n Rosbag HM_rgb2ir")
# print(HM_rgb2ir)


HM_cam2gripper = np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/HM_cam2gripper.npy")
HM_gripper2cam = np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/HM_gripper2cam.npy")
print("\n")
print("\nself collected data Cam2Gripper")
print(HM_cam2gripper)
print("\n self.collected data Gripper2Cam")
print(HM_gripper2cam)
