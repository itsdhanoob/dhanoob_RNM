#!/usr/bin/env python3

from ssl import ALERT_DESCRIPTION_HANDSHAKE_FAILURE
import rospy
import sys
import cv2 
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter():
    def __init__(self):
        self.node_name = "rgb_converter"
        self.node_name1 = "ir_converter"

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.namedWindow(self.cv_window_name,cv2.WINDOW_NORMAL)
        cv.moveWindow(self.cv_window_name, 25, 75)

        # Create the OpenCV display window for the IR image
        self.cv_window_name1 = self.node_name1
        cv.namedWindow(self.cv_window_name1,cv2.WINDOW_NORMAL)
        cv.moveWindow(self.cv_window_name1, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.bridge1 = CvBridge()

        # Subscribe to the camera image and depth topics and set the appropriate callbacks
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
    
        # Display the image.
        cv2.imshow(self.node_name, rgb_frame)

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

        # Display the image.
        cv2.imshow(self.node_name1, ir_frame)

        # Process any keyboard commands
        self.keystroke = cv.waitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

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