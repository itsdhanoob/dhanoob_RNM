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


class CameraCalibration():
        
    def __init__(self):

        rospy.logwarn('[CC] Init started...')

        # Set To False If Calibration Shouldn't Be Visible In Window Frame
        self.visualize_calibration_process = True
        # Path For Calibration Results
        self.result_path        = os.path.join(os.path.dirname(__file__),'CV_camera_calibration_results')  
        # Path To Read-In Desired Joint List
        self.joint_list_path    = os.path.join(os.path.dirname(__file__),'CV_camera_calibration_data/dataset003/collected_joint_list.npy')  
        # Joint State Topic
        self.joint_state_topic  = rospy.get_param('/joint_state_topic', '/joint_states')
        # Image topics
        self.rgb_frame_topic    = rospy.get_param('/rgb_image_topic', "/k4a/rgb/image_raw")
        self.ir_frame_topic     = rospy.get_param('/ir_image_topic', "/k4a/ir/image_raw")

        # World Frame Object Points: same chessboard for RGB and IR camera
        self.objpoints = [] # 3d point in real world space
        
        # RGB Camera Variables
        self.imgpoints_rgb = [] # 2d points in image plane.
        self.frameSize_rgb = (2048,1536) # Resolution width=2048, height=1536
        self.rmat_rgb = []  # Rotation Matrix RGB Frame For Convertion Of Rodrigues To Matrix
        self.current_rgb_frame = []  # Current RGB Frame From Callback Function
        self.ret_rgb = []  # Ret Value From "findChessboardCorners"
        self.corners_rgb = [] # Detected Corners From "findChessboardCorners"

        # IR Camera Variables
        self.imgpoints_ir = [] # 2d points in image plane.
        self.frameSize_ir = (640,576) # Resolution width=640, height=576
        self.current_ir_frame = []  # Current IR Frame From Callback Function
        self.ret_ir = []  # Ret Value From "findChessboardCorners"
        self.corners_ir = [] # Detected Corners From "findChessboardCorners"

        # Robot Arm
        self.kinematics = robot_kinematics()
        self.R_gripper2base = [] # stores rotational matrix after get_pose_from_angles calculation
        self.t_gripper2base = [] # stores translation vector from get_pose_from_angles calcuclation
        self.current_joint_state = [] # stores current joint angle which can be access when frame is saved 


        # Setup Checkerboard Parameters
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # termination criteria
        self.chessboard_size = (8,5)  # Create chessboard and generate object points
        self.square_size = 0.04 # Checkerboard Field Size in Millimeter

        # Create Object Points in 3D Meshgrid Space And Covert Them In Desired Array Shape Like (0,0,0), (0,0,40), (0,0,80....
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.chessboard_size[0],0:self.chessboard_size[1]].T.reshape(-1,2)*self.square_size

        
   
        # Initialize ROS Specific Functions
        self.node_name = "camera_calibration"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()

        self.sub_rgb_frame          = rospy.Subscriber(self.rgb_frame_topic, Image, self.rgb_image_callback)
        self.sub_ir_frame           = rospy.Subscriber(self.ir_frame_topic, Image, self.ir_image_callback)
        self.sub_joint_state        = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)

        # Task Control and PM communication
        self.sub_goal_pose_reached  = rospy.Subscriber("/goal_pose_reached", Int16, self.callback_goal_pose_reached)
        self.pub_goal_pose_js       = rospy.Publisher("/goal_pose_js", Float64MultiArray, queue_size=1)
        
        self.sub_task_command       = rospy.Subscriber('/task_command', String, self.callback_task_command)
        self.pub_task_finished      = rospy.Publisher('/task_finished', String, queue_size=1)
        
        self.start_task     = False                         # Signals to start the task, is set from callback_task_command
        self.TASKCMD        = "camera_calibration_start"    # message to start task /TODO replace with appropriate value
        self.TASKFIN        = "camera_calibration_finished" # message to signal finished task /TODO replace with appropriate value

        self.at_desired_goal_pose = False
        self.joint_list_pos = 0

        rospy.logwarn("[CC] Waiting for image topics...")
        time.sleep(1)



    def joint_to_dk_to_hm(self, joints, inv: bool):
        """return R_mat, t_vec, pose_hm"""
        temp = [0, 0, 0 ,1]

        pose_list = self.kinematics.get_pose_from_angles(joints)
        pose_3x4 = np.reshape(pose_list, (4,3)).T
        pose_hm = np.r_[pose_3x4, np.reshape(temp, (1,4))]
        if inv == True:
            inv(pose_hm)
        else:
            R_mat = pose_hm[0:3, 0:3]
            t_vec = pose_hm[0:3, 3:4]
        return R_mat, t_vec, pose_hm

    def rmat_tvec_to_hm(self, rmat, tvec):
        """return pose_hm"""
        temp = [0, 0, 0 ,1]

        pose_3x4 = np.c_[rmat, np.reshape(tvec, (3,1))]
        pose_hm = np.r_[pose_3x4, np.reshape(temp, (1,4))]
        return pose_hm


    def joint_state_callback(self, joint_state : JointState):
        self.current_joint_state = joint_state.position

    # Store Current RGB Frame In Class Variable
    def rgb_image_callback(self, ros_rgb_img):
        a =1
    
    # Store Current IR Frame In Class Variable
    def ir_image_callback(self, ros_ir_img):
        a = 2


    def draw_image_points(self, window_name, frame, corners, ret):
        """ Function used in create_and_draw_image_points to draw detected chessboard corners in rgb and ir
            image frame"""
        # Draw And Display Corners In Frame
        frame = cv.drawChessboardCorners(frame, self.chessboard_size, corners, ret)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 600, 600) 
        cv.imshow(window_name, frame)
        cv.waitKey(250)


    def create_and_draw_image_points(self, visualization_flag):
        """ This function creates 2D image points based on OpenCV. Set visualization_flag = True to show
            corner detection process of RGB and IR Camera frame. Image points will be always calculated"""
        
        # Convert Color Image TO BGR OpenCV Convention (Necessary!)
        self.gray_rgb = cv.cvtColor(self.current_rgb_frame, cv.COLOR_BGR2GRAY)
        self.gray_ir = cv.cvtColor(self.current_ir_frame, cv.COLOR_BGR2GRAY)

        # Detect Chessboard Corners
        self.ret_rgb, self.corners_rgb = cv.findChessboardCorners(self.gray_rgb, self.chessboard_size, None)
        self.ret_ir, self.corners_ir = cv.findChessboardCorners(self.gray_ir, self.chessboard_size, None)

        # Only Proceed When Visualization Is Disired
        if visualization_flag == True:
            self.draw_image_points("RGB_Frame", self.current_rgb_frame, self.corners_rgb, self.ret_rgb)
            self.draw_image_points("IR_Frame", self.current_ir_frame, self.corners_ir, self.ret_ir)
        

    def collect_calibration_data(self):
        """ Collects the data e.g image points, object, points, joint_angles etc. if robot is in desired goal
            position"""
       
        if self.ret_rgb and self.ret_ir == True:
          
            # Append object points according to corners to global array
            self.objpoints.append(self.objp)

            # Refine found corners in subpixel space and then append them to imgpoints
            self.corner_sub_pixel_rgb = cv.cornerSubPix(self.gray_rgb, self.corners_rgb, (11,11), (-1,-1), self.criteria)
            self.imgpoints_rgb.append(self.corner_sub_pixel_rgb)
            self.corner_sub_pixel_ir = cv.cornerSubPix(self.gray_ir, self.corners_ir, (11,11), (-1,-1), self.criteria)
            self.imgpoints_ir.append(self.corner_sub_pixel_ir)
            
            # Get joint angles of this frame and calculate the pose to extract
            # Rotational matrix and translation vector

            R_gtb, t_gtb, pose_hm = self.joint_to_dk_to_hm(self.current_joint_state, False)
            self.R_gripper2base.append(R_gtb)
            self.t_gripper2base.append(t_gtb)

            rospy.logwarn(f"[CC] Collected data for frame with ID: {self.joint_list_pos}")
      

    def rgb_ir_calibration(self):
        """ This function calibrates the rgb and ir camera:
            Saves all important parameters as .npy and write new yaml file for camera driver
            It also calculates the extrinsic parameter between RGB and IR camera"""

        # Calibrate RGB Single Camera
        ret_rgb, cameraMatrix_rgb, dist_rgb, self.rvecs_rgb, self.tvecs_rgb = cv.calibrateCamera(self.objpoints, self.imgpoints_rgb, self.frameSize_rgb, None, None, flags=cv.CALIB_RATIONAL_MODEL)
        # Convert rvecs To Rotation Matrix For Hand In Eye
        for rvec in self.rvecs_rgb:
            rmat, jacobian = cv.Rodrigues(rvec)
            self.rmat_rgb.append(rmat) 
      
        # Calibrate IR Single Camera
        ret_ir, cameraMatrix_ir, dist_ir, rvecs_ir, tvecs_ir = cv.calibrateCamera(self.objpoints, self.imgpoints_ir, self.frameSize_ir, None, None, flags=cv.CALIB_RATIONAL_MODEL)

        # Calibrate Stereo Camera: extrinsic parameters between RGB and IR camera
        retStereo, cameraMatrix_stereo_rgb, dist_stereo_rgb, cameraMatrix_stereo_ir, dist_stereo_ir, self.rmat_stereo, self.tvec_stereo, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(self.objpoints, self.imgpoints_rgb, self.imgpoints_ir, cameraMatrix_rgb, dist_rgb, cameraMatrix_ir, dist_ir, None, flags = (cv.CALIB_FIX_INTRINSIC + cv.CALIB_RATIONAL_MODEL))
        
        rospy.logwarn("[CC] Calibrated RGB, IR and Stereo Camera successfully")
        self.reprojection_error("RGB Camera", self.objpoints, self.imgpoints_rgb, self.rvecs_rgb, self.tvecs_rgb, cameraMatrix_rgb, dist_rgb)
     #   self.save_calibration_results()
        

    def save_calibration_results(self):
        np.save(self.result_path + "/R_gripper2base.npy", self.R_gripper2base)
        np.save(self.result_path + "/t_gripper2base.npy", self.t_gripper2base)
        np.save(self.result_path + "/rmat_rgb.npy", self.rmat_rgb)
        np.save(self.result_path + "/tvecs_rgb.npy", self.tvecs_rgb)
        np.save(self.result_path + "/rvec_stereo.npy", self.rmat_stereo)
        np.save(self.result_path + "/tvec_stereo.npy", self.tvec_stereo)
        HM_rgb2ir = self.rmat_tvec_to_hm(self.rmat_stereo, self.tvec_stereo)
        np.save(self.result_path + "/HM_rgb2ir", HM_rgb2ir)
        HM_ir2rgb = inv(self.rmat_tvec_to_hm(self.rmat_stereo, self.tvec_stereo))
        np.save(self.result_path + "/HM_ir2rgb.npy", HM_ir2rgb)
        rospy.logwarn(f"[CC] Saved calibration result in {self.result_path}")

    # PM communication
    def publish_desired_goal_pose(self, pose_js : np.array, pose_id : int):
        ''' Publihes current desired goal pose and current pose id using the goal_pose_js_pub.
            Also sets the self.curr_pose_id to the id that is sent in the message.
            Parameters:
                pose_js (List): Current goal pose in joint space
                pose_id (int): Current goal pose ID
        '''
        pose_js = np.append(pose_js, pose_id)
        msg         = Float64MultiArray()
        msg.data    = pose_js

        self.pub_goal_pose_js.publish(msg)
       

        rospy.logwarn(f"[CC] Send new goal_pose_js with ID {pose_id}")

    def callback_goal_pose_reached(self, msg : Int16):
        ''' Checks the received confirmation ID and sets at_desired_goal_pose flag accordingly
        '''
        received_id = msg.data
        if received_id == self.joint_list_pos:
            rospy.logwarn(f"[CC] Received correct confirmation ID {received_id}")
            self.at_desired_goal_pose = True
        else:
            rospy.logwarn(f"[CC] !ERR! Received incorrect confirmation ID {received_id}")


    # Task control
    def callback_task_command(self, msg : String):
        ''' Listenes for task command and sets self.start_task '''

        msg = msg.data
        if msg == self.TASKCMD:
            rospy.logwarn(f'[CC] Received correct start command "{msg}"')
            self.start_task = True

    def is_topic_published(self, topic_name : str):
        ''' Checks the rostopic list for the given topic 
        '''
        topics  = rospy.get_published_topics()
        topics  = [topics[i][0] for i in range(len(topics))]
        return (topic_name in topics)

    def wait_for_task_command(self):
        while not self.start_task:
            pass
        time.sleep(2)

    def publish_task_finished(self):
        msg = String()
        msg.data = self.TASKFIN
        self.pub_task_finished.publish(msg)
        return

    def reprojection_error(self, name : str, objpoints, imgpoints, rvecs, tvecs, cameraMatrix, dist):
        # Reprojection Error
        mean_error = 0

        for i in range(len(objpoints)):
            imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
            mean_error += error
        rospy.logwarn(f"[CC] Mean_error (Norm L2) for {name} is: {mean_error}")



    def main_calibration(self):

        rospy.logwarn('[CC] Starting main calibration...')

        # Load Joint List For Goal Positions
        joint_list = np.load(self.joint_list_path)
        print("Len(joint_list)")
        print(len(joint_list))
        

        # Wait until process_manager is ready
        rospy.logwarn('[CC] Waiting for PM task command...')
        self.wait_for_task_command()

        # Publish First Goal Position by giving Joint List (7 Angles) and Joint List Position (Out of All)
        self.publish_desired_goal_pose(joint_list[self.joint_list_pos], self.joint_list_pos)

        while self.joint_list_pos < len(joint_list):

            # TODO Just for Rosbag pre recorded data loading
            self.at_desired_goal_pose = True
            self.current_joint_state =joint_list[self.joint_list_pos]
            self.current_rgb_frame = cv.imread("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_data/dataset003/rgb_" + str(self.joint_list_pos) + ".png")
            self.current_ir_frame = cv.imread("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_data/dataset003/ir_" + str(self.joint_list_pos) + ".png")
            ########################################################################################################################

            self.create_and_draw_image_points(self.visualize_calibration_process)
            
            if self.at_desired_goal_pose:
                self.collect_calibration_data()
                # Prepare For Next Goal Position
                self.at_desired_goal_pose = False

                self.joint_list_pos += 1
                
                if self.joint_list_pos < len(joint_list):
                    self.publish_desired_goal_pose(joint_list[self.joint_list_pos], self.joint_list_pos)
         
         
        # Start Main Calibration with collected data
        self.rgb_ir_calibration()

        # Give Feedback to PM
        self.publish_task_finished()
        rospy.logwarn('[CC] Main process finished')

    def cleanup(self):
        rospy.logwarn("[CC] Shutting down vision node.")
        cv.destroyAllWindows()   


def main(args):       
    try:
        camera_calibration = CameraCalibration()
        camera_calibration.main_calibration()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("[CC] Shutting down vision node.")
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
