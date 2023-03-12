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


image_counter_rgb = 0
object_point_counter = 0
image_counter_ir = 0
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
frameSize = (2048,1536)
image_selecter = 50
robot = robot_kinematics()
current_joint_state = []
R_gripper2base = []
t_gripper2base = []




class image_converter():
    def __init__(self):
        self.node_name = "rgb_converter"
        self.node_name1 = "ir_converter"
        

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        # self.cv_window_name = self.node_name
        # cv.namedWindow(self.cv_window_name,cv2.WINDOW_NORMAL)
        # cv.moveWindow(self.cv_window_name, 25, 75)

        # self.cv_window_name1 = self.node_name1
        # cv.namedWindow(self.cv_window_name1,cv2.WINDOW_NORMAL)
        # cv.moveWindow(self.cv_window_name1, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.bridge1 = CvBridge()

        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.rgb_frame    = rospy.Subscriber("/k4a/rgb/image_raw", Image, self.rgb_image_callback)
        self.ir_frame= rospy.Subscriber("/k4a/ir/image_raw", Image, self.ir_image_callback)
        self.joint_state = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        rospy.loginfo("Waiting for image topics...")
        print("Objektpoint_test:")
        
        old_len = 0

       
        while  image_counter_rgb < 1500 and not rospy.is_shutdown():
            #print("Objektpoint_test:")
            #print(objpoints_test)
            #print("--------------------------------------------")
        
            print(f"Number of Image Frames {image_counter_rgb}")
            print(f"Number of Object points {object_point_counter}")
            rate = rospy.Rate(1) # 1 Hz
            rate.sleep() # Sleeps for 1/rate sec

        
        print("Kamera Kalibrierung")
        print(f"Len of objpoints {len(objpoints)}")
        print(f"Len of objpoints {len(imgpoints)}")
        print("------------------------------------------")
        ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
        print(f"Return Value: {ret}")



       # ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
        R_cam2gripper, t_cam2gripper = 	cv.calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, method = cv.CALIB_HAND_EYE_TSAI )
        print("WEEEE DID IT!!!! HAND IN EYE WITH SHITTY VALUES, BUT IT WORKS")
        print(R_cam2gripper)
        print(t_cam2gripper)

        # h,  w = img.shape[:2]
        # newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

        # # Undistort
        # dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

        # # crop the image
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]

        # # Undistort with Remapping
        # mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
        # dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

        # # crop the image
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]

        print(" Camera matrix:")
        print(cameraMatrix)

        print("\n Distortion coefficient:")
        print(dist)

        print("\n Rotation Vectors:")
        print(rvecs)
        
        #print(type(objpoints))
        #print(objpoints[:])
        #objpoints = np.reshape(objpoints, (-1, 3))
        #print(type(imgpoints))
        #print(objpoints)
        #print(imgpoints)
        #print(imgpoints[0])

        retval, pnp_rvecs, pnp_tvecs = cv.solvePnP(objpoints[1], imgpoints[1], cameraMatrix, dist)
        
        print("\n Rotation Vectors (pnp):")
        print(pnp_rvecs)

        print("\n Translation Vectors:")
        print(tvecs)

        print("\n Translation Vectors (pnp):")
        print(pnp_tvecs)


        # Reprojection Error
        mean_error = 0

        for i in range(len(objpoints)):
            imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            mean_error += error
        

        #   print( "total error: {}".format(mean_error/len(objpoints)) )

        # Process any keyboard commands
        self.keystroke = cv.waitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")


            
            
    def joint_state_callback(self, joint_state):
        #print(type(np.array(joint_state.position)))
       # print(robot.get_pose_from_angles(joint_state.position))
        global current_joint_state
        current_joint_state = joint_state.position
        #print(current_joint_state)

        




    def rgb_image_callback(self, ros_rgb_img):
        #global objpoints_test
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")
        except CvBridgeError:
            print ("e")

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        rgb_frame = np.array(rgb_frame, dtype=np.uint8)
    
        global image_counter_rgb
        global object_point_counter
        newImage = rgb_frame
      #  cv2.imwrite("/home/rnm/img/file" + str(image_counter_rgb) + ".png", newImage)

       ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

        chessboardSize = (8,5)
        

        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)*40
       # print(objp[:,:])

        #size_of_chessboard_squares_mm = 
       # objp = objp * size_of_chessboard_squares_mm


        # Arrays to store object points and image points from all the images.
       
        img = newImage
        #img = cv2.imread("/home/rnm/img/file" + str(image_counter_rgb) + ".png")
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        image_counter_rgb+= 1
        
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

        # If found, add object points, image points (after refining them)
        global image_selecter
        if ret == True:
            if image_counter_rgb >= image_selecter:
                object_point_counter += 1
                objpoints.append(objp)
                #print(objpoints)
                corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners)
               # print(imgpoints)
                current_pose = robot.get_pose_from_angles(current_joint_state)
                current_pose = np.reshape(current_pose, (3,4))
                print(current_pose)
                R_gtb = current_pose[0:3, 0:3]
                t_gtb = current_pose[0:3, 3:4]
                R_gripper2base.append(R_gtb)
                t_gripper2base.append(t_gtb)
                print(R_gripper2base)
                print(t_gripper2base)

                image_selecter += 50
                
        
                # Draw and display the corners
                cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
            # cv.imshow('img', img)
                cv.waitKey(10)
            # print(len(objpoints))


    def ir_image_callback(self, ros_ir_img):
        a = 0
    #     # Use cv_bridge() to convert the ROS image to OpenCV format
    #     try:
    #         ir_frame = self.bridge1.imgmsg_to_cv2(ros_ir_img, "bgr8")
    #     except CvBridgeError:
    #         print ("e")

    #     # Convert the image to a Numpy array since most cv2 functions
    #     # require Numpy arrays.
    #     ir_frame = np.array(ir_frame, dtype=np.uint8)
    #     newImage=ir_frame

    #     global image_counter_ir
    #    # cv2.imwrite("/home/rnm/img/ir_image/file" + str(image_counter_ir) + ".png", newImage)

    #    ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

    #     chessboardSize = (8,5)
    #     frameSize = (2048,1536)

    #     # termination criteria
    #     criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


    #     # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    #     objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    #     objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

    #     size_of_chessboard_squares_mm = 40
    #     objp = objp * size_of_chessboard_squares_mm


    #     # Arrays to store object points and image points from all the images.
    #     objpoints = [] # 3d point in real world space
    #     imgpoints = [] # 2d points in image plane.
    #     img = newImage
    #    # img = cv2.imread("/home/rnm/img/ir_image/file" + str(image_counter_ir) + ".png")
    #     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #     image_counter_ir+= 1
        
    #     # Find the chess board corners
    #     ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    #     # If found, add object points, image points (after refining them)
    #     if ret == True:

    #         objpoints.append(objp)
    #         corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    #         imgpoints.append(corners)

    #         # Draw and display the corners
    #         cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
    #         #cv.imshow('img', img)
    #         cv.waitKey(10)

    #         ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

    #         h,  w = img.shape[:2]
    #         newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

    #         # Undistort
    #         dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

    #         # crop the image
    #         x, y, w, h = roi
    #         dst = dst[y:y+h, x:x+w]

    #         # Undistort with Remapping
    #         mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
    #         dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

    #         # crop the image
    #         x, y, w, h = roi
    #         dst = dst[y:y+h, x:x+w]

    #         # print(" Camera matrix IR:")
    #         # print(newCameraMatrix)
 
    #         # print("\n Distortion coefficient IR:")
    #         # print(dst)
 
    #         # print("\n Rotation Vectors IR:")
    #         # print(rvecs)
            
    #         # print("\n Translation Vectors IR:")
    #         # print(tvecs)

    #         # Reprojection Error
    #         mean_error = 0

    #         for i in range(len(objpoints)):
    #             imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    #             error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    #             mean_error += error

    #         #print( "total error IR: {}".format(mean_error/len(objpoints)) )



    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()   

def main(args):       
    try:
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down vision node.")
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
