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



class CameraCalibration():

    def __init__(self):
        # Initialize ROS Specific Functions
        self.node_name = "hand_in_eye"
        rospy.init_node(self.node_name)

        # Start and Finish Flags
        self.start_task = False
        self.TASKCMD = "handeye_calibration_start"
        self.TASKFIN = "handeye_calibration__finished"

        #HomogenousMatrix Result
        self.cam2gripper_hm = []

        # Load Camera Calibration Results
        self.result_path = os.path.join(os.path.dirname(__file__),'CV_camera_calibration_results')  
        self.R_gripper2base = np.load(self.result_path + "/R_gripper2base.npy")
        self.t_gripper2base = np.load(self.result_path + "/t_gripper2base.npy")
        self.rmat_rgb = np.load(self.result_path + "/rmat_rgb.npy")
        self.tvecs_rgb = np.load(self.result_path + "/tvecs_rgb.npy")

        # Initialize ROS Specific Functions
        self.sub_task_command       = rospy.Subscriber('/task_command', String, self.callback_task_command)
        self.pub_task_finished       = rospy.Publisher('/task_finished', String, queue_size=1)

        rospy.logwarn(f"[HiE] Loaded calibration result from {self.result_path}")

    def main(self):

        # Wait until process_manager is ready
        rospy.logwarn('[HiE] Waiting for PM...')
        self.wait_for_task_command()
        rospy.logwarn('[HiE] Received task_start_command')
        
        
        self.R_cam2gripper, self.t_cam2gripper = cv.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.rmat_rgb, self.tvecs_rgb, method = cv.CALIB_HAND_EYE_PARK)
        
        # Saves result of R- t and HM in CV_camera_calibration folder
     #   self.save_result() 


        # Give Feedback to PM
        self.publish_task_finished()
        rospy.logwarn('[HiE] Main process finished')

    def rmat_tvec_to_hm(self, rmat, tvec):
        """return pose_hm"""
        temp = [0, 0, 0 ,1]

        pose_3x4 = np.c_[rmat, np.reshape(tvec, (3,1))]
        pose_hm = np.r_[pose_3x4, np.reshape(temp, (1,4))]
        return pose_hm

    # Task control
    def callback_task_command(self, msg : String):
        ''' Listenes for task command and sets self.start_task '''

        msg = msg.data
        if msg == self.TASKCMD:
            rospy.logwarn(f'[CC] Received correct start command "{msg}"')
            self.start_task = True


    def publish_task_finished(self):
        msg = String()
        msg.data = self.TASKFIN
        self.pub_task_finished.publish(msg)
        return

    def wait_for_task_command(self):
        while not self.start_task:
            pass
        time.sleep(2)


    def save_result(self):

        np.save(self.result_path + "/R_cam2gripper.npy", self.R_cam2gripper)
        np.save(self.result_path + "/t_cam2gripper.npy", self.t_cam2gripper)

        HM_cam2gripper = self.rmat_tvec_to_hm(self.R_cam2gripper, self.t_cam2gripper)
        HM_gripper2cam = inv(HM_cam2gripper)
        np.save(self.result_path + "/HM_cam2gripper.npy", HM_cam2gripper)
        np.save(self.result_path + "/HM_gripper2cam.npy", HM_gripper2cam)
        
    

def main(args):       
    try:
        hand_in_eye = CameraCalibration()
        hand_in_eye.main()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("[HiE] Shutting down.")
        cv.DestroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)