#!/usr/bin/env python
from importlib.resources import path
from ssl import ALERT_DESCRIPTION_HANDSHAKE_FAILURE
from sympy import true
import rospy
import sys
import cv2 
import cv2 as cv
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import time
from robot_kinematics import robot_kinematics

robot = robot_kinematics() # creates object needed to use get_pose_from_angles
current_joint_state = [] # stores current joint angle which can be access when frame is saved


# RGB Setup
rgb_image_counter = 0
rgb_object_point_counter = 0
objpoints_rgb = [] # 3d point in real world space
imgpoints_rgb = [] # 2d points in image plane.
image_selecter_rgb = 25
R_gripper2base_rgb = [] # stores rotational matrix after get_pose_from_angles calculation
t_gripper2base_rgb = [] # stores translation vector from get_pose_from_angles calcuclation
frameSize_rgb = (2048,1536) # Resolution width=2048, height=1536
rgb_frame = [] # acts as temporary storage for current rgb frame

# IR Setup
ir_image_counter = 0
ir_object_point_counter = 0
objpoints_ir = [] # 3d point in real world space
imgpoints_ir = [] # 2d points in image plane.
image_selecter_ir = 25
R_gripper2base_ir = [] # stores rotational matrix after get_pose_from_angles calculation
t_gripper2base_ir = [] # stores translation vector from get_pose_from_angles calcuclation
frameSize_ir = (640,576) # Resolution width=640, height=576
ir_frame = [] # acts as temporary storage for current ir frame



class image_converter():

    def __init__(self):
  
        # Creating node name 
        self.node_name = "camera_calibration"
        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth(ir-camera) topics and set
        # the appropriate callbacks
        self.rgb_frame    = rospy.Subscriber("/k4a/rgb/image_raw", Image, self.rgb_image_callback)
        self.ir_frame= rospy.Subscriber("/k4a/ir/image_raw", Image, self.ir_image_callback)
        self.joint_state = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        rospy.loginfo("Waiting for image topics...")
        
        
        old_len = 0

        cv.waitKey(1000)
        while not rospy.is_shutdown():
            print(f"RGB Frames {rgb_image_counter}")
            print(f"RGB Sampled data {rgb_object_point_counter}")
            print("-------------------------------------------------------------")
            print(f"IR Frames {ir_image_counter}")
            print(f"IR Sampled data {ir_object_point_counter}")
            print("-------------------------------------------------------------")
            self.calibration()
            rate = rospy.Rate(1) # 1 Hz





            rate.sleep() # Sleeps for 1/rate sec

        # Calibrate Camera with obtained objectpoints [defined by checkerboard and set worldframe]
        # and imagepoints [found by cv.findchessboard]
        # flag = CALIB_RATIONAL_MODEL is set to get more than 5 distortion parameters
        ret_rgb, cameraMatrix_rgb, dist_rgb, rvecs_rgb, tvecs_rgb = cv.calibrateCamera(objpoints_rgb, imgpoints_rgb, frameSize_rgb, None, None, flags=cv.CALIB_RATIONAL_MODEL)
        ret_ir, cameraMatrix_ir, dist_ir, rvecs_ir, tvecs_ir = cv.calibrateCamera(objpoints_ir, imgpoints_ir, frameSize_ir, None, None, flags=cv.CALIB_RATIONAL_MODEL)
       
       
        print(f"RGB: Return Value: {ret_rgb}")

        # RGB Calibration Result
        print(" RGB: Camera matrix:")
        print(cameraMatrix_rgb)

        print("\n RGB: Distortion coefficient:")
        print(dist_rgb)

        print(f"IR: Return Value: {ret_ir}")
        # IR Calibration Result
        print("IR: Camera matrix:")
        print(cameraMatrix_ir)

        print("\n IR:Distortion coefficient:")
        print(dist_ir)
        #print("\n Rotation Vectors:")
        #print(rvecs)

        #print("\n Translation Vectors:")
        #print(tvecs)


        # Reprojection Error
        # L2 norm give square root of squared difference of elements weighted by amount of object points
        
        for i in range(len(objpoints)):
            imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            error = cv.norm(imgpoints_rgb[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            mean_error += error
        print("The mean error is:")
        print(mean_error)


        #   print( "total error: {}".format(mean_error/len(objpoints)) )

        # Process any keyboard commands
        self.keystroke = cv.waitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")
       
    def joint_state_callback(self, joint_state):
        # save current joint states into global variable to make is accessable when frame is selected
        # for camera calibration
        global current_joint_state
        current_joint_state = joint_state.position
       
    def rgb_image_callback(self, ros_rgb_img):
        global rgb_frame
        global rgb_image_counter
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")
            rgb_image_counter += 1
            
        except CvBridgeError:
            print ("e")
        

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        rgb_frame = np.array(rgb_frame, dtype=np.uint8)

    def ir_image_callback(self, ros_ir_img):
        global ir_frame
        global ir_image_counter
        try:
            ir_frame = self.bridge.imgmsg_to_cv2(ros_ir_img, "bgr8")
            ir_image_counter += 1
            
        except CvBridgeError:
            print ("e")
            
    def calibration(self):

        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Create chessboard and generate object points
        chessboardSize = (8,5)
        # In millimeter
        square_size = 40 

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        # ngrid generates two matrices which contain the x and y value for every grid point respectively
        # objp = (40,2) -> 40 points in meshgrid with x,y coordinate
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)*square_size


        # Generate Image Points
        gray_rgb = cv.cvtColor(rgb_frame, cv.COLOR_BGR2GRAY)
        gray_ir = cv.cvtColor(ir_frame, cv.COLOR_BGR2GRAY)
        

        # Find the chess board corners
        ret_rgb, corners_rgb = cv.findChessboardCorners(gray_rgb, chessboardSize, None)
        ret_ir, corners_ir = cv.findChessboardCorners(gray_ir, chessboardSize, None)

        self.draw_chessboard_corners_to_image("RGB_Frame", rgb_frame, chessboardSize, corners_rgb, ret_rgb)
        self.draw_chessboard_corners_to_image("IR_Frame", ir_frame, chessboardSize, corners_ir, ret_ir)

                # If found all corners found, add object points, image points (after refining them)
        global image_selecter_rgb
        global rgb_image_counter
        global rgb_object_point_counter
        if ret_rgb == True:
            if rgb_image_counter >= image_selecter_rgb:

                # Append object points according to corners to global array
                objpoints_rgb.append(objp)
                rgb_object_point_counter += 1

                # Refine found corners in subpixel space and then append them to imgpoints
                corners2_rgb = cv.cornerSubPix(gray_rgb, corners_rgb, (11,11), (-1,-1), criteria)
                imgpoints_rgb.append(corners2_rgb)
                
                # Get joint angles of this frame and calculate the pose to extract
                # Rotational matrix and translation vector
                current_pose = robot.get_pose_from_angles(current_joint_state)
                current_pose = np.reshape(current_pose, (3,4))
                
                R_gtb = current_pose[0:3, 0:3]
                t_gtb = current_pose[0:3, 3:4]
                R_gripper2base_rgb.append(R_gtb)
                t_gripper2base_rgb.append(t_gtb)
                image_selecter_rgb += 25
         
        global image_selecter_ir
        global ir_object_point_counter    
        if ret_ir == True:
            if rgb_image_counter >= image_selecter_ir:
                
                # Append object points according to corners to global array
                objpoints_ir.append(objp)
                ir_object_point_counter += 1

                # Refine found corners in subpixel space and then append them to imgpoints
                corners2_ir = cv.cornerSubPix(gray_ir, corners_ir, (11,11), (-1,-1), criteria)
                imgpoints_ir.append(corners2_ir)

                # Get joint angles of this frame and calculate the pose to extract
                # Rotational matrix and translation vector
                current_pose = robot.get_pose_from_angles(current_joint_state)
                current_pose = np.reshape(current_pose, (3,4))
                R_gtb = current_pose[0:3, 0:3]
                t_gtb = current_pose[0:3, 3:4]
                R_gripper2base_ir.append(R_gtb)
                t_gripper2base_ir.append(t_gtb)

                image_selecter_ir += 25
             
    def draw_chessboard_corners_to_image(self, window_name, frame, chessboardSize, corners, ret):
        # Draw and display the corners
        frame = cv.drawChessboardCorners(frame, chessboardSize, corners, ret)

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 600, 600) 
        cv.imshow(window_name, frame)
        cv.waitKey(100)
     
    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()   





def main(args):       
    try:
        image_converter()
        ImageConverter = image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down vision node.")
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)