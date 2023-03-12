#!/usr/bin/env python3

import numpy as np
from numpy import sin, cos
import matplotlib.pyplot as plt
from DH_creator import DhCreator
import rospy

class robot_kinematics:
    def __init__(self, debug=False):
        ''' Parameters:
                debug (bool): Enables debug behavior
        '''
        # flags
        self.debug = debug                      # flag to (de)activate plots to controll divergence/ konvergence
        
        # instances
        dh_creator = DhCreator()                
        self.A_lamb = dh_creator.get_A_lamb()   # load lambdafunction of A which is calculated with the help of a DH-Matrix
        self.J_lamb = dh_creator.get_J_lamb()   # load lambdafunction of J which is calculated with the help of the lambdafunction of A

        rospy.logwarn("[DH] Loaded DH-Table.")

    def get_pose_from_angles(self, theta_target):
        ''' Get pose matrix A from joint angles. Utilizing pre-computed expressions using DH-parameters.
            
            Parameters:
                theta (List): Containing 7 joint angles

            Returns:
                A (Matrix): Pose matrix
                    Position is in A[9:12]
                    Rotation is in np.reshape(A[0:9], (3,3))
        
        cos_target  = [np.cos(q) for q in theta_target] # calculate cos(q_i) for each q1 - q7 of theta
        sin_target  = [np.sin(q) for q in theta_target] # calculate sin(q_i) for each q1 - q7 of theta
        '''
        A = np.array(self.A_lamb(theta_target))  # calculate A
        A = A[:,0]                                         # reshape A 

        return A


    def get_angles_from_pose(self, theta_init, A_target, step=0.01, err_tol=1e-4):
        ''' Get joint angles for a target pose. Utilizing pre-computed expressions using DH-parameters.
            
            Parameters:
                theta_init (List): Containing 7 joint angles used to init the iterative search
                A_target (Matrix): Matrix encoding target pose
                step (float): Step width for iteration
                err_tol (float): Error tolerance for returned angles

            Returns:
                TODO
                pos_error (float): Cartesian position error
        '''
        

        # Settings for divergence detection 
        n_last_entries  = 500                 # n past entries used for detection
        n_allowed_fails = 10000               # n consequtive iterations that are allowed before abortion


        delta_list_sqsum    = []            # Sum of squared delta_A entries
        delta_list_max      = []            # Max of delta_A matrix
        n_failed_iterations = 0             # n of failed iterations (non converging)
        
        counter = 0                         # counter for debugging purpose (iterations per completet calculation)

        A_current = self.get_pose_from_angles(theta_init)   # calculate intial A
        delta_A = (A_target - A_current)                    # calculate intial delta A
        theta   = theta_init                                # set inital theta

        while np.abs(delta_A).max() > err_tol:

            J_theta = self.get_J(theta)
            
            # multiplying by step_size to interpolate between current and target pose
            delta_theta = np.matmul(np.linalg.pinv(J_theta), delta_A * step)
            theta       = theta + delta_theta
            
            A_current   = self.get_pose_from_angles(theta)
            delta_A     = (A_target - A_current)

            # Detect divergence
            delta_sqsum = sum(delta_A**2)
            delta_max   = max(abs(delta_A))
            counter += 1
          
        pos_error   = np.linalg.norm(delta_A[9:12])
        print(counter)
        return theta, pos_error

    def get_J(self,theta):
        # current_trigo: sin and cos of actual theta's
        '''
        cos_target  = [np.cos(q) for q in theta]
        sin_target  = [np.sin(q) for q in theta]
        '''
        J = np.array(self.J_lamb(theta))
        return J


# for testing an debugging


import os

select = 1
if __name__ == '__main__' and select == 0:

    num_run     = 10
    kinematics  = robot_kinematics(debug=True)
    thetas = []
    poses = []
    err_tol = 0.0001
    
    for _ in range(num_run):
        theta   = np.random.uniform(low = -2, high=2, size=7)
        thetas.append(theta)
        poses.append(kinematics.get_pose_from_angles(theta))

    for i in range(num_run-1):
        
        angles, err = kinematics.get_angles_from_pose(thetas[i], poses[i+1])
        #print(f"poses : {poses[i+1]}")
        #print(f"angels : {angles}")
        print(f"err : {err}")



if __name__ == '__main__' and select == 1:

    profiler_on = False

    if profiler_on:
        import cProfile, pstats
        profiler = cProfile.Profile()
        profiler.enable()

    
    kinematics  = robot_kinematics(debug=True)

    theta_init  = np.array([-0.46566852410275156, -0.3110300093826501, 0.8544756305928134, -2.8423748966685567, -0.4328396366462611, 2.0623442616300496, 1.6652260407523973])    # Theta used to init iterative search (in degree)
    theta_target = np.array([-1.5936105274902141, 0.5478572209508795, 2.85868600131448, -2.1916521760233776, -2.8638610953095265, 1.2092807170550028, 0.41151600447462666])    # Delta from init theta to target theta (in degree)
   
    A_init   = kinematics.get_pose_from_angles(theta_init)
    A_target    = kinematics.get_pose_from_angles(theta_target)
    print(A_init)
   
    
    
    q_ik, error = kinematics.get_angles_from_pose(theta_init, A_target)
    print(np.rad2deg(q_ik))
    print(f"Position error: {error}")


    if profiler_on:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.print_stats(20)
