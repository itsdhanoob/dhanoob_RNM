#!/usr/bin/env python
import numpy
import sys
import numpy as np 
import open3d as o3d
import copy
import rospy
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray, Int16, String
import os
import time
from robot_kinematics import robot_kinematics

class ModelRegistration():

    def __init__(self):

       # Paths To load In Data
        self.joint_list_path    = os.path.join(os.path.dirname(__file__),'CV_model_registration_data/joint_list_MR.npy')  #TODO: Create joint_list_mr with pose collector
        self.stl_path = os.path.join(os.path.dirname(__file__),'CV_model_registration_data/Skeleton_Target.stl')
        self.pcd_path = os.path.join(os.path.dirname(__file__),'CV_model_registration_data/PCD_1.pcd') #TODO: Add PCD File Name or name it like this is pose collector
        self.cropped_pcd_path = os.path.join(os.path.dirname(__file__),'CV_model_registration_data/cropped_PCD_1.ply')

        self.kinematics = robot_kinematics()

        # Initialize ROS Specific Functions
        self.node_name = "mode_registration"
        rospy.init_node(self.node_name)

        # Task Control and PM communication
        self.sub_goal_pose_reached  = rospy.Subscriber("/goal_pose_reached", Int16, self.callback_goal_pose_reached)
        self.pub_goal_pose_js       = rospy.Publisher("/goal_pose_js", Float64MultiArray, queue_size=1)
        
        self.sub_task_command       = rospy.Subscriber('/task_command', String, self.callback_task_command)
        self.pub_task_finished      = rospy.Publisher('/task_finished', String, queue_size=1)
        self.pub_needle_goal_pose      = rospy.Publisher('/needle_goal_pose', String, queue_size=1)
        
        self.start_task     = False                         # Signals to start the task, is set from callback_task_command
        self.TASKCMD        = "model_registration_start"    # message to start task /TODO replace with appropriate value
        self.TASKFIN        = "model_registration_finished" # message to signal finished task /TODO replace with appropriate value

        self.at_desired_goal_pose = False
        self.joint_list_pos = 0

        self.HM_cam2gripper= []
        self.HM_ir2rgb=[]
        self.HM_gripper2base=[]

        # Model Registration Data

        self.source = []    # For STL File
        self.target = []    # For PointCloud2
        self.transformation = [] # 4x4 HM from icp_result.transform
        self.voxel_size = 0.09
        self.trans_init =  np.asarray([[0, -1,  0 , -0.00219754],
                            [-1, 0,  0,  0.00376681 ], # moved it cw 
                             [0, 0 ,-1,  0.14037132], #change in rotation
                             [0 ,0,   0,  1]])

    
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
       

        rospy.logwarn(f"[MR] Send new goal_pose_js with ID {pose_id}")

    def callback_goal_pose_reached(self, msg : Int16):
        ''' Checks the received confirmation ID and sets at_desired_goal_pose flag accordingly
        '''
        received_id = msg.data
        if received_id == self.joint_list_pos:
            rospy.logwarn(f"[MR] Received correct confirmation ID {received_id}")
            self.at_desired_goal_pose = True
        else:
            rospy.logwarn(f"[MR] !ERR! Received incorrect confirmation ID {received_id}")

    # Task control
    def callback_task_command(self, msg : String):
        ''' Listenes for task command and sets self.start_task '''

        msg = msg.data
        if msg == self.TASKCMD:
            rospy.logwarn(f'[MR] Received correct start command "{msg}"')
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

    # PM communication
    def publish_needle_goal_pose(self, needle_point : np.array):
        ''' Publihes current desired goal pose and current pose id using the goal_pose_js_pub.
            Also sets the self.curr_pose_id to the id that is sent in the message.
            Parameters:
                pose_js (List): Current goal pose in joint space
                pose_id (int): Current goal pose ID
        '''
        
        msg         = Float64MultiArray()
        msg.data    = needle_point

        self.pub_needle_goal_pose.publish(msg)
        # TODO: Store in fila on disk
        rospy.logwarn(f"[MR] Send new needle_goal_pose")
    
    ################### generating a mesh on the skeleton ########################
    def mesh(self):
        mesh = o3d.io.read_triangle_mesh(self.stl_path)
        point_cld = mesh.sample_points_poisson_disk(100000)
        point_cld_scale = np.asarray(point_cld.points)
        point_cld_scale = point_cld_scale/1000
        pcd_skt = o3d.geometry.PointCloud()
        pcd_skt.points = o3d.utility.Vector3dVector(point_cld_scale)

        #o3d.visualization.draw_geometries([mesh])

        #o3d.visualization.draw_geometries([pcd_skt])
        return pcd_skt
   
    ################## reading image from the bag file #########################
    def pcd_maker(self):
        pcd = o3d.io.read_point_cloud(self.pcd_path)           #("/home/rnm/Desktop/pcd_data/1590496721.382030864.pcd" )  
        pcd.estimate_normals()
        pcd_ds = pcd.voxel_down_sample(voxel_size = 0.02)
        pcd_ds.remove_radius_outlier(nb_points=16, radius=0.05)
        o3d.visualization.draw_geometries_with_editing([pcd_ds])

        ################### reading the cropped image #################################

        pcd = o3d.io.read_point_cloud(self.cropped_pcd_path)     #("/home/rnm/catkin_ws/src/panda_vision/scripts/cropped_03.ply")
        pcd.remove_radius_outlier(nb_points=16, radius=0.05)
        pcd.estimate_normals()
        o3d.visualization.draw_geometries([pcd])
        return pcd
  
    ################# tranformation between current scene and target ###############
    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def preprocess_point_cloud(self, pcd):
        print(":: Downsample with a voxel size %.3f." % self.voxel_size)
        pcd_down = pcd.voxel_down_sample(self.voxel_size)

        radius_normal = self.voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))

        radius_feature = self.voxel_size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    #################### input data set for global registration #####################
    def prepare_dataset(self, voxel_size):
        print(":: Loading source and target")
        self.source = self.mesh()
        self.target = self.pcd_maker()
        self.source.estimate_normals() 
        self.target.estimate_normals()
        self.source.transform(self.trans_init)
        self.draw_registration_result(self.source, self.target, transformation = np.identity(4))

        self.source_down, self.source_fpfh = self.preprocess_point_cloud(self.source)
        self.target_down, self.target_fpfh = self.preprocess_point_cloud(self.target)

    def execute_fast_global_registration(self):

        distance_threshold = self.voxel_size * 30
        print(":: Apply fast global registration with distance threshold %.3f" \
                % distance_threshold)
        self.result_fast = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
            self.source_down, self.target_down, self.source_fpfh, self.target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        
    def refine_registration(self):
        distance_threshold = self.voxel_size * 0.5
        print(":: Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)  
        self.result_icp = o3d.pipelines.registration.registration_icp(self.source, self.target, distance_threshold, self.result_fast.transformation,
                                                o3d.pipelines.registration.TransformationEstimationPointToPlane())
    
    def select_goal_point(self):
            self.goal_point = o3d.visualization.VisualizerWithEditing()
            self.goal_point.create_window()
            self.goal_point.add_geometry(self.source.transform(self.result_icp.transformation))
            self.goal_point.run()
            self.goal_point.destroy_window()
            self.goal_pts= []
            self.goal_pts= self.goal_point.get_picked_points()  # shift+lmb to select target point

    def target_point_in_base(self):
        point= self.goal_point_in_target_frame @ self.HM_ir2rgb @ self.HM_cam2gripper @ self.HM_gripper2base
        return point


    def stl_goal_point_to_target_frame(self):

            # Source Point Cloud to Array of Points
            # Select Goal Point by Index (tgt_pts[0]) from Source.point Array
            np_src=np.asarray(self.source.points)
            tgt_p=np.reshape(np_src[self.goal_pts[0]],(3,1))
            # Reshape to Column Vector with appended value 1 for HM Transformation operation
            self.goal_point_column = np.r_[tgt_p,np.reshape([1],(1,1))]
            self.goal_point_in_target_frame = np.dot(self.result_icp.transformation, self.goal_point_column)
            print(self.goal_point_in_target_frame)

    def main_process(self):

        # Load Joint List For Goal Positions
        joint_list = np.load(self.joint_list_path)
       
      
        # Wait until process_manager is ready
        rospy.logwarn('[MR] Waiting for PM task command...')
        self.wait_for_task_command()

        # Publish First Goal Position by giving Joint List (7 Angles) and Joint List Position (Out of All)
        self.publish_desired_goal_pose(joint_list[self.joint_list_pos], self.joint_list_pos)

        # Homogenous matrix import

        self.HM_cam2gripper= np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/HM_cam2gripper.npy")
        self.HM_ir2rgb= np.load("/home/rnm/catkin_ws/src/RNM/kinematic_robot/scripts/CV_camera_calibration_results/HM_ir2rgb.npy")
        self.HM_gripper2base= self.joint_to_dk_to_hm(joint_list[self.joint_list_pos],False)


        while self.joint_list_pos < 1:
            # Desired Joint Pose Reached
            if self.at_desired_goal_pose:
                rospy.logwarn('[MR] Start Collecting Data')

                # Increment joint_list_index, after reaching desired goal pose
                self.joint_list_pos += 1

                #Collect Point Cloud in Realtime (at desired position)


                # Creating Source Target from File
                self.prepare_dataset(self.voxel_size)
                
                rospy.logwarn('[MR] Pointclouds Prepared')

                # Get self.result_fast
                self.execute_fast_global_registration()

                # Visualize Source and Target (Fast Global Registration)
                self.draw_registration_result(self.source_down, self.target_down, self.result_fast.transformation)

                # Obtain "detailed" self.result_icp
                self.refine_registration()

                # Visualize Source and Target (refined; iterative closest point)
                self.draw_registration_result(self.source, self.target, self.result_icp.transformation)
                print(self.result_icp.transformation)
                rospy.logwarn('[MR] Refined Model Registration Drawing Done')

                # Select 3D Point in STL Frame by Clicking
                self.select_goal_point()
                rospy.logwarn('[MR] Selected goal_point')


                # Convert goal point index to goal point vector
                # self.goal_point_in_target_frame
                rospy.logwarn('[MR] Try to convert goal point to target frame')
                self.stl_goal_point_to_target_frame()
                rospy.logwarn('[MR] Converted goal point to target frame')       
    
        # target_frame_point to robot base_frame TODO: Do Transformation with hand in eye and direct kinematic gripper2base
        tp_base =  self.target_point_in_base()
        print("point after transformations", tp_base)

        # publish target point on topic
        self.publish_needle_goal_pose(self.goal_point_in_target_frame)  #TODO: Replace arg. by tracker point in base frame not Pointcloud Target frame
        rospy.logwarn('[MR] Published goal point on needle_goal_pose topic')


        # Give Feedback to PM
        self.publish_task_finished()
        rospy.logwarn('[MR] Main process finished')




def main(args):       
    try:
        model_registration = ModelRegistration()
        model_registration.main_process()
        rospy.spin()

    except KeyboardInterrupt:
        rospy.logwarn("[MR] Shutting down vision node.")
    

if __name__ == '__main__':
    main(sys.argv)
