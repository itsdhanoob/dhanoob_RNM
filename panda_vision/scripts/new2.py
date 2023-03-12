#!/usr/bin/env python3

from importlib.resources import path
from ssl import ALERT_DESCRIPTION_HANDSHAKE_FAILURE

from sympy import true
import rospy
import sys
import cv2 
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
import time


image_counter_rgb = 0
image_counter_ir = 0


class image_converter():
    def __init__(self):
        self.node_name = "rgb_converter"
        self.node_name1 = "ir_converter"

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        #self.cv_window_name = self.node_name
        #cv.namedWindow(self.cv_window_name,cv2.WINDOW_NORMAL)
        #cv.moveWindow(self.cv_window_name, 25, 75)

        #self.cv_window_name1 = self.node_name1
        #cv.namedWindow(self.cv_window_name1,cv2.WINDOW_NORMAL)
        #cv.moveWindow(self.cv_window_name1, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.bridge1 = CvBridge()

        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.rgb_frame    = rospy.Subscriber("/k4a/rgb/image_raw", Image, self.rgb_image_callback)
        self.ir_frame= rospy.Subscriber("/k4a/ir/image_raw", Image, self.ir_image_callback)

        rospy.loginfo("Waiting for image topics...")

    def rgb_image_callback(self, ros_rgb_img):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(ros_rgb_img, "bgr8")
        except CvBridgeError:
            print ("e")

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        rgb_frame = np.array(rgb_frame, dtype=np.uint8)
    
        global image_counter_rgb
    
        newImage = rgb_frame
        cv2.imwrite("/home/rnm/img/file" + str(image_counter_rgb) + ".png", newImage)

       ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

        chessboardSize = (8,5)
        frameSize = (2048,1536)

        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

        size_of_chessboard_squares_mm = 40
        objp = objp * size_of_chessboard_squares_mm


        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        img = cv2.imread("/home/rnm/img/file" + str(image_counter_rgb) + ".png")
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        image_counter_rgb+= 1
        
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, chessboardSize, flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK+
                                                                              cv.CALIB_CB_NORMALIZE_IMAGE)

        # If found, add object points, image points (after refining them)
        if ret == True:

            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
            cv.imshow('rgb_img', img)
            cv.waitKey(10)

            ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, cameraMatrix=None, distCoeffs=None, rvecs=None, tvecs=None,
                                                               flags=cv.CALIB_RATIONAL_MODEL, criteria=criteria) 

            h,  w = img.shape[:2]
            newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

            # Undistort
            dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]

            # Undistort with Remapping
            mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
            dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]

            print(" Camera matrix:")
            print(cameraMatrix)
 
            print("\n Distortion coefficient:")
            print(dst)
 
            print("\n Rotation Vectors:")
            print(rvecs)
            
            print("\n Translation Vectors:")
            print(tvecs)

            # # Reprojection Error
            # mean_error = 0

            # for i in range(len(objpoints)):
            #     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            #     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            #     mean_error += error

            # print( "total error: {}".format(mean_error/len(objpoints)) )

#################################################################################################################################################################

            # alpha=cv.calibrateHandEye()

            # Process any keyboard commands
            self.keystroke = cv.waitKey(5)
            if 32 <= self.keystroke and self.keystroke < 128:
                cc = chr(self.keystroke).lower()
                if cc == 'q':
                    # The user has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit.")

    def ir_image_callback(self, ros_ir_img):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            ir_frame = self.bridge1.imgmsg_to_cv2(ros_ir_img, "bgr8")
        except CvBridgeError:
            print ("e")

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        ir_frame = np.array(ir_frame, dtype=np.uint8)
        newImage=ir_frame

        global image_counter_ir
        cv2.imwrite("/home/rnm/img/ir_image/file" + str(image_counter_ir) + ".png", newImage)

       ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

        chessboardSize = (8,5)
        frameSize = (2048,1536)

        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

        size_of_chessboard_squares_mm = 40
        objp = objp * size_of_chessboard_squares_mm


        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        img = cv2.imread("/home/rnm/img/ir_image/file" + str(image_counter_ir) + ".png")
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        image_counter_ir+= 1
        
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

        # If found, add object points, image points (after refining them)
        if ret == True:

            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
            cv.imshow('ir_img', img)
            cv.waitKey(10)

            ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

            h,  w = img.shape[:2]
            newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

            # Undistort
            dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]

            # Undistort with Remapping
            mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
            dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]

            print(" Camera matrix IR:")
            print(newCameraMatrix)
 
            print("\n Distortion coefficient IR:")
            print(dst)
 
            print("\n Rotation Vectors IR:")
            print(rvecs)
            
            print("\n Translation Vectors IR:")
            print(tvecs)

            # Reprojection Error
            mean_error = 0

            for i in range(len(objpoints)):
                imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
                error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
                mean_error += error

            print( "total error: {}".format(mean_error/len(objpoints)) )



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