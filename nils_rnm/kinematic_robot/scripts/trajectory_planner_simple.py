#!/usr/bin/env python3
import rospy
import numpy as np
import csv
import os
from tqdm import tqdm
from robot_kinematics import robot_kinematics
from trajectory_planner_complicated import TrajectoryPlannerComplicated
# FIXME eine Zeile zu viel, dividieren durch 0 unterbinden, nan bei gleicher Position?

class trajectory_planner_simple:
    def __init__(self, robot_kinematics, max_joint_vel=2.1750/4000):
        ''' Parameters:
                movement_speed (float): Movement speed for end effector [m/s] TODO: is that correct?
        '''
        self.max_joint_vel   = max_joint_vel    # [rad/s] actual max is 2.1750
        self.limits  = {"q_pos_max" : [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973],
                        "q_pos_min" : [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],
                        "q_vel_max" : [ 2.1750,	2.1750,	2.1750,	2.1750, 2.6100, 2.6100, 2.6100],
                        "q_acc_max" : [     15,    7.5,     10,   12.5,     15,     20,     20],
                        "q_jrk_max" : [   7500,   3750,   5000,   6250,   7500,  10000,  10000],
                        "p_vel_max" :   1.7000,
                        "p_acc_max" :  13.0000,
                        "p_jrk_max" : 6500.000 }

        # Instances
        self.robot_kinematics = robot_kinematics
        self.TPC    = TrajectoryPlannerComplicated(self.limits, num_joints=7, safety_factor=0.02)
        
    def create_point_to_point_traj(self, start_pose, end_pose, MOVEMENT_SPEED, file_input_waypoints, file_output_trajectory,):
        ''' Wraps create_simple_trajectory with auto waypoint file generation, such that a simple 
            point to point trajectory can easily be realised. A directory and file name of the 
            trajectory is returned for further processing (e.g. for motion execution).
            Parameters:
                start_pose (List): Contains start pose joints
                end_pose (List): Contains end pose joints
            Returns:
                traj_dirname (String): directory and file string for generated trajectory
        '''

        # Delete content of existing file or create if not existing
        with open(os.path.join(os.path.dirname(__file__), file_input_waypoints),'w+') as file:
            file.truncate(0)

        # Write start and end pose into path file
        with open((os.path.join(os.path.dirname(__file__), file_input_waypoints)), mode="a", newline="") as f:    
            writer = csv.writer(f, delimiter=",")
            writer.writerow(start_pose)
            writer.writerow(end_pose)

        # Create point 2 point trajectory
        self.TPC.create_point_to_point_traj(start_pose, end_pose, file_output_trajectory)       # /TODO confirm working properly
        rospy.logwarn(f'[TPc] Generated padded trajectory in file {file_output_trajectory}')

        #self.create_simple_trajectory_JS(file_input_waypoints, file_output_trajectory)
        
        return

    def create_simple_trajectory(self, file_input_waypoints, file_output_trajectory, MOVEMENT_SPEED, initial_waypoint=None):
        ''' Load file with waypoints in joint-space and write file with interpolated joint-angles 
            sampled at 1kHz (Step size of 1ms). An initial waypoint can be set in case the current 
            robot pose is not the same as the first input waypoint. This is useful for hardcoded 
            paths with variable starting point.
            Paramters:
                file_input_waypoints (String): Directory and file name to input waypoints in joint-space
                file_output_trajectory (String): Directory and file name for output trajectory list in joint space
                initial_waypoint (List): An initial waypoint in joint-space can preceed the first waypoint in the input file
        '''
        # Load planned path from .csv file
        joint_list = []
        with open((os.path.join(os.path.dirname(__file__), file_input_waypoints))) as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                joint_list.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
    
        # delete content of existing file
        with open(os.path.join(os.path.dirname(__file__), file_output_trajectory),'w+') as file:
            file.truncate(0)

        # Prepend initial waypoint
        if not initial_waypoint == None:
            joint_list.insert(0, initial_waypoint) 

        # inital pose from first joint
        current_pose = self.robot_kinematics.get_pose_from_angles(joint_list[0])

        # calculate number of interpolations between each given waypoint
        for i in tqdm(range(len(joint_list) - 1), ncols=100):
            current_joint  = np.array(joint_list[i])
            next_joint     = np.array(joint_list[i + 1])

            max_delta_joint = np.abs(next_joint-current_joint).max()
            steps           = int(max_delta_joint / MOVEMENT_SPEED)
            delta_joints_per_step = (next_joint - current_joint) / steps

            # calculate each joint state for given movement speed and 1000Hz publishing rate
            with open((os.path.join(os.path.dirname(__file__), file_output_trajectory)), mode="a", newline="") as f:    
                writer = csv.writer(f, delimiter=",")
                for j in range(steps + 1):
                    sample_joint = current_joint + j * delta_joints_per_step
                    writer.writerow(sample_joint)
           
        
        rospy.logwarn(f'[TPs] Generated simple trajectory in file {file_output_trajectory}')
        return 

    def create_simple_trajectory_JS(self, file_input_waypoints, file_output_trajectory, max_joint_vel=2.1750/40000, initial_waypoint=None):
        # /FIXME max_joint_vel is too small, could be increased?
        ''' Load file with waypoints in joint-space and write file with interpolated joint-angles 
            sampled at 1kHz (Step size of 1ms). An initial waypoint can be set in case the current 
            robot pose is not the same as the first input waypoint. This is useful for hardcoded 
            paths with variable starting point.
            Paramters:
                file_input_waypoints (String): Directory and file name to input waypoints in joint-space
                file_output_trajectory (String): Directory and file name for output trajectory list in joint space
                initial_waypoint (List): An initial waypoint in joint-space can preceed the first waypoint in the input file
        '''
        # Load planned path from .csv file
        waypoints = []
        with open((os.path.join(os.path.dirname(__file__), file_input_waypoints))) as f:
            reader = csv.reader(f, delimiter=",")
            for row in reader:
                waypoints.append([float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5]),float(row[6])])
    
        # delete content of existing file
        with open(os.path.join(os.path.dirname(__file__), file_output_trajectory),'w+') as file:
            file.truncate(0)

        # Prepend initial waypoint
        if not initial_waypoint == None:
            waypoints.insert(0, initial_waypoint)

        waypoints   = np.array(waypoints)
        num_wp      = waypoints.shape[0]
        num_joints  = waypoints.shape[1]

        # calculate number of interpolations between each given waypoint
        for wp_id in tqdm(range(num_wp - 1), ncols=100):
            curr_waypoint   = waypoints[wp_id]
            next_waypoint   = waypoints[wp_id + 1]     

            delta       = next_waypoint - curr_waypoint
            max_delta   = np.abs(delta).max()
            num_steps   = int(max_delta / max_joint_vel)

            traj_points = np.zeros([num_steps, num_joints])

            for joint_id in range(num_joints):
                for step_id in range(num_steps):
                    traj_points[step_id, joint_id]  = curr_waypoint[joint_id] + delta[joint_id] / num_steps * step_id
            
            traj_points  = np.append(traj_points, [next_waypoint], axis=0)

            # calculate each joint state for given movement speed and 1000Hz publishing rate
            with open((os.path.join(os.path.dirname(__file__), file_output_trajectory)), mode="a", newline="") as f:    
                writer = csv.writer(f, delimiter=",")
                for traj_point in traj_points:
                    writer.writerow(traj_point)     # /FIXME: writing in file can be done more efficiently
        
        rospy.logwarn(f'[TPs] Generated simple trajectory in file {file_output_trajectory}')
        return 

# for test purposes
if __name__ == '__main__':
    #trajectory_calculate_simple = trajectory_planner_simple()
    #path_planner.calculate_path_list_jointspace()
    #trajectory_calculate_simple.create_path()

    # New tests
    robot_kinematic = robot_kinematics()
    traj_planner    = trajectory_planner_simple(robot_kinematic)

    # Test point 2 point trajectory generation
    p1  = [4.902633183867522e-06,-2.817378035757656e-06,3.7433388122565248e-06,-0.11962521536745907,-2.3976978260620285e-06,7.143186770974808e-07,6.714333409263418e-07]
    p2  = [0.24893705772069105,2.664877200306105,0.07457192893703553,1.9891384910981544,-0.023143228919031037,0.5593613260435555,0.16576735333757406]
    #traj_file   = traj_planner.create_point_to_point_traj(p1, p2)
    
    # Test planned path trajectory generation
    file_input_waypoints    = "Path/calculated_path_preinsertion_jointspace.csv"
    file_output_trajectory  = "Trajectory/created_trajectory_to_goal_1ms.csv"

    file_input_waypoints2    = "Path/calculated_path_insertion_jointspace.csv"
    file_output_trajectory2  = "Trajectory/created_trajectory_to_insertion_1ms.csv"
    #initial_waypoint        = [4.9026e-06,-2.8173e-06,3.7433e-06,-0.1196,-2.3976e-06,7.1431e-07,6.7143e-07]

    traj_planner.create_simple_trajectory_JS(file_input_waypoints, file_output_trajectory, 0.01/1000)
    traj_planner.create_simple_trajectory(file_input_waypoints2, file_output_trajectory2, 0.01/1000)