import csv
import os
from typing import List

import matplotlib.pyplot as plt
import numpy as np
#import scipy.optimize
from matplotlib.ticker import FormatStrFormatter
from numpy.polynomial import Polynomial
import math
#import rospy

# UTIL-----------------------------------------------------
def get_list_from_csv(dir_name, file_name):
    ''' Get a list of lists from a csv file
        dir_name should be relative to the location of util.py
    '''

    output  = []
    with open((os.path.join(os.path.dirname(__file__), dir_name, file_name)), newline="") as csv_file:
        reader  = csv.reader(csv_file)
        for row in reader:
            row_float = [float(entry) for entry in row]
            output.append(row_float)

    return output

def printm(message):
    print(message)



# CLASSES--------------------------------------------------
class TrajectorySegment:

    def __init__(self,  seg_id : int, poly_coef : List, start_t : float, end_t : float, 
                        start_pos : float, end_pos : float, is_waypoint : bool):
        ''' One trajectory segment, consisting of one quintic polynomial expression.
            This init should be called by the Trajectory object when registering a 
            new trajectory segment.
            Parameters: 
                seg_id (int): ID of current segment
                poly_coef (List): List of quintic polynomial coefficients
                start_t (float): Start time of joint in this segment
                end_t (float): End time of joint in this segment
                start_pos (float): Start position of joint in this segment
                end_pos (float): End position of joint in this segment
                is_waypoint (bool): start_pos is waypoint
            Returns:
                Nothing
        '''
        self.id_num     = seg_id
        self.poly_coef  = poly_coef
        self.start_t    = start_t
        self.end_t      = end_t
        self.start_pos  = start_pos
        self.end_pos    = end_pos
        self.is_waypoint= is_waypoint

    def get_values_for_window(self, derivative=0, step=0.001, sig_dec=3):
        ''' Returns x and y values for the n-th derivative of the polynomial in the window
            between start_t and end_t
            Parameters:
                derivative (int): Indicates the desired derivative of the polynomial
                step (float): x step width in seconds, must be <= 1
                sig_dec (int): Round to significant decimal place
            Returns:
                x (List): x values in seconds
                y (List): y values
        '''
        start   = math.ceil(self.start_t * 10**sig_dec) / 10**sig_dec
        end     = math.floor(self.end_t * 10**sig_dec) / 10**sig_dec
        x       = np.arange(start, end + step, step)
        poly    = Polynomial(self.poly_coef)
        y       = poly.deriv(derivative)(x - start)

        x       = np.round(x, 8)

        return x, y

    def get_y_values(self, x_values : np.array, derivative=0):
        ''' Returns positions for specified times, where the given times are global for the 
            trajectory and not local to the segment. Throws an error if the given times are
            outside of the segment window (i.e. a segment which starts at t0 and ends at t1
            will reject x values for x < t0 and x > t1
            Parameters;
                x_values (np.array): Desired times
                derivative (int): Indicates the desired derivative of the polynomial
            Returns:
                y_values (np.array): Position data corresponding to desired times
        '''

        assert x_values.min() >= self.start_t, f'x_values can not be < start_t'
        assert x_values.max() <= self.end_t, f'x_values can not be > end_t'

        poly        = Polynomial(self.poly_coef)
        y_values    = np.array(poly.deriv(derivative)(x_values - self.start_t))

        return y_values


class Trajectory:
    ''' Stores trajectory for one joint. Trajectory segments must be registered
        in order of actual progression.
        All available trajectories objects can be accesseed through the "trajectories"
        class variable
    '''

    trajectories    = []    # list of all trajectories

    def __init__(self, joint_id : int, initial_time = 0):
        ''' Parameters:
                joint_id (int): Joint ID
                initial_time (float): t0 for trajactory calculation'''
        self.joint_id       = joint_id
        self.segment_list   = []                #type: List[TrajectorySegment]
        self.time_list      = [initial_time]

        self.trajectories.append(self)
        
    def register_segment(self, poly_coef, duration, start_pos, end_pos, is_waypoint):
        ''' Register new trajectory segment
            Parameters:
                poly_coef (List): List of quintic polynomial coefficients
                duration (float): Duration of segment in sec
                start_pos (float): Start position of joint in this segment
                end_pos (float): End position of joint in this segment
                is_waypoint (bool): Indicates waypoint as given by path planning
            Returns:
                Nothing
        '''
        if duration == 0 or duration == np.inf or duration == np.nan:
            assert 0, "Duration seems wrong"

        segment = TrajectorySegment(seg_id      = len(self.segment_list),
                                    poly_coef   = poly_coef,
                                    start_t     = self.time_list[-1],
                                    end_t       = self.time_list[-1] + duration,
                                    start_pos   = start_pos,
                                    end_pos     = end_pos,
                                    is_waypoint = is_waypoint)

        self.segment_list.append(segment)
        self.time_list.append(segment.end_t)

    def get_all_segments(self, derivative=0, step=0.001, sig_dec=3):
        ''' Stitches all available segments together and returns an x and y list for all segemnts
            in the specified sampling resolution. Additionally returns an array with all 
            time_offsets, positions and waypoint indicator
            Parameters:
                derivative (int): The desired derivative of the segments
                step (float): Sampling step size for the trajectory
                sig_dec (int): Significicant decimal for rounding operations
            Returns:
                x_vals (List): All time values for sampled points
                y_vals (List): All position values for sampled points
                (self.time_list, pos, is_wp) (Tuple)
                    self.time_list (List): All time stamps for segments points
                    pos (List): All positions for segment points
                    is_wp (List): Bool to indicate waypoint (if false: control point)
        '''
        # Get clean x_vals
        max_t   = np.floor(self.time_list[-1] * 10**sig_dec) / 10**sig_dec + step
        n_steps = int(max_t / step)
        x_vals  = np.zeros(n_steps)
        x_vals  = [i * step for i in range(0, n_steps)]
        x_vals  = np.round(x_vals, sig_dec + 1)

        x_temp  = []
        x_conf  = np.zeros(x_vals.shape[0])
        y_vals  = np.zeros(x_vals.shape[0])
        pos     = []
        is_wp   = []

        # Iterate over all x_vals, get y_vals and secondary values
        for segment in self.segment_list:
            start_t = segment.start_t
            end_t   = segment.end_t

            x_indx  = np.array(np.where(np.logical_and(x_vals >= start_t, x_vals <= end_t)))
            x_temp  = x_vals[x_indx]
            x_conf[x_indx] = x_temp
            y_vals[x_indx] = segment.get_y_values(x_temp, derivative=derivative)

            pos.append(segment.start_pos)
            is_wp.append(segment.is_waypoint)

        pos.append(self.segment_list[-1].end_pos)
        is_wp.append(True)
        
        x_vals  = np.append(x_vals, max_t + step)
        x_conf  = np.append(x_conf, max_t + step)
        y_vals  = np.append(y_vals, segment.get_y_values(segment.end_t, derivative=derivative))      

        assert x_vals.shape == y_vals.shape, 'x_vals and y_vals have different length'
        assert np.array_equal(x_vals, x_conf), 'x_vals and x_conf not equal'

        return x_vals, y_vals, (self.time_list, pos, is_wp)

    def get_all_segments_old(self, derivative=0, step=0.001, sig_dec=3):
        ''' Stitches all available segments together and returns an x and y list for all segemnts
            in the specified sampling resolution. Additionally returns an array with all 
            time_offsets, positions and waypoint indicator
            Parameters:
                derivative (int): The desired derivative of the segments
                step (float): Sampling step size for the trajectory
                sig_dec (int): Significicant decimal for rounding operations
            Returns:
                x (List): All time values for sampled points
                y (List): All position values for sampled points
                (self.time_list, pos, is_wp) (Tuple)
                    self.time_list (List): All time stamps for segments points
                    pos (List): All positions for segment points
                    is_wp (List): Bool to indicate waypoint (if false: control point)
        '''
        x       = []
        y       = []
        pos     = []
        is_wp   = []
        
        for segment in self.segment_list:   
            x_new, y_new = segment.get_values_for_window(derivative, step, sig_dec)

            [x.append(x_n) for x_n in x_new]
            [y.append(y_n) for y_n in y_new]
            pos.append(segment.start_pos)
            is_wp.append(segment.is_waypoint)
        
        pos.append(self.segment_list[-1].end_pos)
        is_wp.append(True)

        return x, y, (self.time_list, pos, is_wp)

    def pad_trajectory_until_time(self, time_end : float):
        ''' Adds a constant segment to the end of the trajectory until a specified time '''

        x, y, _ = self.get_all_segments()

        last_t  = self.time_list[-1]
        last_p  = y[-1]

        assert time_end >= last_t, 'padded end time needs to be >= unpadded end time'

        poly_coef   = [last_p, 0, 0, 0, 0]
        duration    = time_end - last_t

        self.register_segment(poly_coef, duration, last_p, last_p, True)


class TrajectoryPlannerComplicated: 
    ''' Takes in a list of joint positions "waypoints" and returns a list of 1kHz joint commands.
    '''

    def __init__(self, limits_dict, num_joints, safety_factor=0.1, safety_margin=0.01, debug=False):
        ''' Parameters:
                limits_dict (dict): Joint limits for pos, vel, acc and jrk
                num_joints (int): Number of joints
                safety_factor (float): Scales vel, acc and jrk limits
                safety_margin (float): Distance to min/max pos
                debug (bool): Enables debug behavior
        '''

        # Our objects
        #self.kinematics     = robot_kinematics()

        # Constants and parameters
        self.safety_factor  = safety_factor
        self.safety_margin  = safety_margin

        self.qlimit_pos_max = np.array(limits_dict["q_pos_max"]) - safety_margin    # Joint space [rad]
        self.qlimit_pos_min = np.array(limits_dict["q_pos_min"]) + safety_margin
        self.qlimit_vel_max = np.array(limits_dict["q_vel_max"]) * safety_factor
        self.qlimit_acc_max = np.array(limits_dict["q_acc_max"]) * safety_factor
        self.qlimit_jrk_max = np.array(limits_dict["q_jrk_max"]) * safety_factor
        self.plimit_vel_max = np.array(limits_dict["p_vel_max"]) * safety_factor    # Cartesian space [m/s]
        self.plimit_acc_max = np.array(limits_dict["p_acc_max"]) * safety_factor
        self.plimit_jrk_max = np.array(limits_dict["p_jrk_max"]) * safety_factor

        # Create trajectory objects
        self.trajectories   = [Trajectory(joint_id, 0) for joint_id in range(num_joints)]    #type: List[Trajectory]
        self.num_joints     = num_joints

    def __get_quintic_params(self, t, pos, vel, acc):
        ''' Get parameters for quintic polynomial from time, position, velocity 
            and acceleration values.
            For each individual calculation, the start time is set to 0.
        '''
        poly_coefs  = []
        for i in range(1, len(t)):
            ip  = i - 1
            t2  = t[i] - t[ip]                     

            a   = np.array([[1,  0,       0,       0,        0,        0],
                            [0,  1,       0,       0,        0,        0],
                            [0,  0,       2,       0,        0,        0],
                            [1, t2,   t2**2,   t2**3,    t2**4,    t2**5],
                            [0,  1, 2*t2   , 3*t2**2,  4*t2**3,  5*t2**4],
                            [0,  0,       2, 6*t2   , 12*t2**2, 20*t2**3]])
            b   = np.array([pos[ip], vel[ip], acc[ip], pos[i], vel[i], acc[i]])
            p   = np.linalg.solve(a, b)
            poly_coefs.append(p)
        
        return poly_coefs

    # Plotting methods
    def plot_duration_vs_alpha(self, og_wp1, og_wp2, og_v1, og_v2, kin_limits, alpha_marker=None):
        ''' Plots a graph for the duration for alpha factors ranging from 0 to 1 
            Parameters:
                alpha_marker (float): Marks an alpha value and corresponding duration
        '''
        (max_vel, max_acc, max_jrk) = kin_limits

        # Settings
        alpha_vals = np.linspace(-1, 1, 1000)
        x_label    = 'alpha'
               
        # Get data       
        dur = []
        for alpha in alpha_vals:
            d  = self.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, (max_vel, max_acc * alpha, max_jrk))
            dur.append(d)
        
        # Plot data
        fig, axs = plt.subplots(1, 2)
        for n, (ymin, ymax) in enumerate(zip([-15, 0], [15, 1.2])):
            axs[n].plot(alpha_vals, dur)
            
            axs[n].set_xlabel(x_label)
            axs[n].set_ylim(ymin, ymax)
            axs[n].grid()

            if not alpha_marker == None: 
                d  = self.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, (max_vel, max_acc  * alpha_marker, max_jrk))
                a  = axs[n].axvline(alpha_marker, color='red', lw=0.5, label=f'alpha = {alpha_marker:.4f}, t = {d:.4f}')
                b  = axs[n].axhline(d, color='red', lw=0.5)
                axs[1].set_xlim(max(alpha_marker - 5 * abs(alpha_marker), -1), min(alpha_marker + 5 * abs(alpha_marker), 1))
                axs[1].set_ylim(0.8 * d, 1.2 * d)
                plt.legend(handles = [a], labels=[f'alpha = {alpha_marker:.4f}, t = {d:.4f}'], title='marker at')


        axs[0].set_ylabel('Duration [s]')
        plt.suptitle(f'Duration to traverse from wp1 to wp2 depending on alpha factor for max acc\nwp1 = {og_wp1:.2f}, wp2={og_wp2:.2f}, v1={og_v1:.2f}, v2={og_v2:.2f}')
        plt.show()

    def plot_trajectory(self, joint_id):
        
        traj        = self.trajectories[joint_id]
        assert joint_id == traj.joint_id, 'Joint id and trajectory object position in list is not the same'
        max_value   = [self.qlimit_pos_max[joint_id],  self.qlimit_vel_max[joint_id],  self.qlimit_acc_max[joint_id],  self.qlimit_jrk_max[joint_id]]
        min_value   = [self.qlimit_pos_min[joint_id], -self.qlimit_vel_max[joint_id], -self.qlimit_acc_max[joint_id], -self.qlimit_jrk_max[joint_id]]

        fig, axs    = plt.subplots(4, 1, sharex='col')
        labels_axis = [r'Pos $[deg]$', r'Vel $[deg/s]$', r'Acc $[deg/s^2]$', r'Jerk $[deg/s^3]$'] 

        for axs_n in range(4):

            x, y, (t, pos, is_wp) = traj.get_all_segments(derivative=axs_n, step=0.00001, sig_dec=5)
            wp_count    = 0
            
            # Draw additional lines and points
            for i, (t_, pos_, is_wp_) in enumerate(zip(t, pos, is_wp)):
                if(is_wp_) :
                    axs[axs_n].axvline(t_, color='black', lw=1, linestyle='-')
                    if axs_n == 0: 
                        axs[0].plot(t_, pos_, 'o', color='black', label='Waypoint', ms=5)
                        axs[0].text(t_, -20, str(wp_count))
                        wp_count = wp_count + 1
                else :
                    axs[axs_n].axvline(t_, color='gray', lw=0.5, linestyle='-')
                    if axs_n == 0: 
                        axs[0].plot(t_, pos_, 'x', color='black', label='Control Point', ms=5)

            axs[axs_n].axhline(max_value[axs_n], color='black', lw=2, linestyle='--')
            axs[axs_n].axhline(min_value[axs_n], color='black', lw=2, linestyle='--')
            axs[axs_n].axhline(0, color='gray', lw=1, linestyle='-')

            # Plot trajectory
            axs[axs_n].plot(x, y)

            # Ax settings
            axs[axs_n].set_ylabel(labels_axis[axs_n])
            if axs_n == 0: axs[0].set_ylim(min(y) - 0.2, max(y) + 0.2)
        
        axs[3].set_xlabel('t [s]')
        plt.suptitle(f'Trajectory for joint {traj.joint_id}')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

    def plot_trajectory_waypoint(self, poly_coef, time_offset, pos):

        fig, ax = plt.subplots()

        x       = np.linspace(0, time_offset[-1], 100)  # in seconds
        labels  = ['ramp up', 'cruise', 'ramp down', 'hold']

        # Plot poly lines
        for pc, t, label in zip(poly_coef, time_offset, labels) :
            poly    = Polynomial(pc)
            y       = [poly(i - t) for i in x]
            ax.plot(x, y, label=label)

        # Plot vertical lines for quintic segments
        for t in time_offset:
            ax.axvline(t, color='black', lw=1, linestyle=':')
        
        # Plot dots to show waypoints
        for i, (t, p) in enumerate(zip(time_offset, pos)):
            if i == 0 or i == len(time_offset) - 1:
                ax.plot(t, p, 'o', color='black', label='Waypoints' if i == 0 else "")
            else:
                ax.plot(t, p, 'x', color='black', label='Control Points' if i == 1 else "")


        ax.set_xlim([-0.05, time_offset[-1] + 0.05])
        ax.set_ylim([min(pos) - 5, max(pos) + 5])
        ax.set_title('Quintic poly interpolation between two waypoints')
        ax.legend()
        plt.show()

    def plot_trajectory_waypoint_limits(self, params_pos, time_offset, pos, limits, joint_id=0):

        params_vel = [Polynomial(poly).deriv(1).coef for poly in params_pos]
        params_acc = [Polynomial(poly).deriv(2).coef for poly in params_pos]
        params_jrk = [Polynomial(poly).deriv(3).coef for poly in params_pos]

        fig, axs= plt.subplots(4, 1, sharex='col')

        x               = np.ogrid[0: np.round(time_offset[-1], 4): 0.001]      # Actual sample rate
        x               = np.linspace(0, time_offset[-1], 5000)                 # in seconds
        labels_segments = ['ramp up', 'cruise', 'ramp down', 'hold']
        labels_axis     = [r'Pos $[deg]$', r'Vel $[deg/s]$', r'Acc $[deg/s^2]$', r'Jerk $[deg/s^3]$']     # /TODO: Change to radians for implementation
        poly_params     = [params_pos, params_vel, params_acc, params_jrk]
        max_value       = [limits['q_pos_max'][joint_id],  limits['q_vel_max'][joint_id],  limits['q_acc_max'][joint_id],  limits['q_jrk_max'][joint_id]]
        min_value       = [limits['q_pos_min'][joint_id], -limits['q_vel_max'][joint_id], -limits['q_acc_max'][joint_id], -limits['q_jrk_max'][joint_id]]

        for n in range(len(poly_params)):
            params  = poly_params[n]
            label_a = labels_axis[n]

            # Plot poly lines
            for p, (param, t) in enumerate(zip(params, time_offset)) :
                poly    = Polynomial(param)
                xx      = [i for i in x if i >= t and i <= time_offset[p + 1]]
                y       = [poly(i - t) for i in xx]
                axs[n].plot(xx, y, label=labels_segments[p])
            
            # Plot vertical lines for quintic segments
            for t in time_offset:
                axs[n].axvline(t, color='black', lw=1, linestyle=':')
            
            # Plot horizontal lines for limits
            axs[n].axhline(max_value[n], color='black', lw=2, linestyle='--')
            axs[n].axhline(min_value[n], color='black', lw=2, linestyle='--')

            axs[n].set_xlim([-0.05, time_offset[-1] + 0.05])
            axs[n].set_ylabel(label_a)

        # Plot dots to show waypoints and limit yaxis
        for i, (t, p) in enumerate(zip(time_offset, pos)):
            if i == 0 or i == len(time_offset) - 1:
                axs[0].plot(t, p, 'o', color='black', label='Waypoints' if i == 0 else "")
            else:
                axs[0].plot(t, p, 'x', color='black', label='Control Point' if i == 1 else "")

        axs[0].set_ylim([min(pos) - 5, max(pos) + 5])
        axs[0].set_title('4 segment quintic polynomial interpolation')
        axs[3].legend(bbox_to_anchor =(0.5,-0.8), loc='lower center', ncol=6)

        plt.tight_layout()
        plt.show()

    def plot_all_trajectories(self, derivative=0):
        ''' Plots all available trajectories '''

        fig_title   = 'Trajectories (' + [r'Pos $[rad]$', r'Vel $[rad/s]$', r'Acc $[rad/s^2]$', r'Jerk $[rad/s^3]$'][derivative] + ')'
        num_traj    = len(self.trajectories)
        fig, axs    = plt.subplots(num_traj, 1, sharex='col')
              
        for joint_id, traj in enumerate(self.trajectories):
            assert joint_id == traj.joint_id, 'Joint id and trajectory object position in list is not the same'
            
            x, y, (t, pos, is_wp) = traj.get_all_segments(derivative=derivative, step=0.00001, sig_dec=5)
            wp_count    = 0

            max_value   = [self.qlimit_pos_max[joint_id],  self.qlimit_vel_max[joint_id],  self.qlimit_acc_max[joint_id],  self.qlimit_jrk_max[joint_id]][derivative]
            min_value   = [self.qlimit_pos_min[joint_id], -self.qlimit_vel_max[joint_id], -self.qlimit_acc_max[joint_id], -self.qlimit_jrk_max[joint_id]][derivative]
            
            # Draw additional points and lines (waypoints, control points)
            for i, (t_, pos_, is_wp_) in enumerate(zip(t, pos, is_wp)):
                if(is_wp_) :
                    axs[joint_id].axvline(t_, color='black', lw=1, linestyle='-')
                    if derivative == 0: axs[joint_id].plot(t_, pos_, 'o', color='black', label='Waypoint', ms=5)
                    wp_count = wp_count + 1
                else :
                    axs[joint_id].axvline(t_, color='gray', lw=0.5, linestyle='-')
                    if derivative == 0: axs[joint_id].plot(t_, pos_, 'x', color='black', label='Control Point', ms=5)

            # Draw additional lines (min, max, 0)
            axs[joint_id].axhline(max_value, color='black', lw=2, linestyle='--')
            axs[joint_id].axhline(min_value, color='black', lw=2, linestyle='--')
            axs[joint_id].axhline(0, color='gray', lw=1, linestyle='-')

            # Plot trajectory
            axs[joint_id].plot(x, y)

            # Ax settings
            axs[joint_id].set_ylabel(f'joint {joint_id}')
            axs[joint_id].set_ylim(min(y) - 0.2, max(y) + 0.2)
            axs[joint_id].yaxis.set_major_formatter(FormatStrFormatter('% 1.2f'))
        
        axs[num_traj - 1].set_xlabel('t [s]')
        plt.suptitle(fig_title)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

    # Aux methods
    def get_initial_velocities(self, waypoints_t):
        ''' Given a waypoint matrix [joint][waypoint] a corresponding velocity matrix is returned.
            Velocities are given in the following way:
                - First and last velocities are always set to 0
                - if prev_waypoint < curr_waypoint < next_waypoint: curr_vel = + max_vel
                - if prev_waypoint > curr_waypoint > next_waypoint: curr_vel = - max_vel
                - else: curr_vel = 0
        '''
        
        joints_vel  = np.zeros(waypoints_t.shape)
        wp          = waypoints_t

        # Iterate over joints
        for j in range(wp.shape[0]):
            max_vel = self.qlimit_vel_max[j]
            for k in range(1, wp.shape[1] - 1):
                if   wp[j, k - 1] < wp[j, k] < wp[j, k + 1]: joints_vel[j, k] = max_vel
                elif wp[j, k - 1] > wp[j, k] > wp[j, k + 1]: joints_vel[j, k] = - max_vel
                else: joints_vel[j, k] = 0

        joints_vel[:, 0] = 0
        joints_vel[:,-1] = 0

        return joints_vel

    def get_sequences_alpha(self, target_dur, og_wp1, og_wp2, og_v1, og_v2, kin_limits, err_tol=1e-6, max_iter=10, debug=False):
        ''' Get alpha factor for a 4 segment quintic interpolation with given target_duration. 
            Employs a simple iterative algorithm to find alpha. max_acc and max_jrk are not set to the
            limits stored in the object to allow exploration
            Returns:
                alpha_est (float): The estimated alpha to achieve the target duration [1]
                dur_err (float): Duration error [s]
        '''
        (max_vel, max_acc, max_jrk) = kin_limits

        # /FIXME: This is a hotfix which is not investigated thorougly
        if og_v1 == og_v2:
            alpha_est   = 1
            err         = abs(target_dur - abs(og_wp1 - og_wp2) / og_v1)
            return alpha_est, err


        lower_bound = 0
        upper_bound = 1

        for _ in range(max_iter):

            alpha       = np.linspace(lower_bound, upper_bound, 10000)
            max_acc_mod = alpha * max_acc
            t_vals  = [self.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, (max_vel, max_acc_m, max_jrk)) for max_acc_m in max_acc_mod]
            t_vals  = np.array(t_vals)
            closest_idx = np.nanargmin(np.abs(t_vals - target_dur))

            alpha_est   = alpha[closest_idx]
            max_acc_m   = alpha_est * max_acc
            dur_est     = self.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, (max_vel, max_acc_m, max_jrk))
            err         = abs(dur_est - target_dur)

            if err <= err_tol: 
                break

            if closest_idx == 0:
                lower_bound = alpha[closest_idx - 0]
                upper_bound = alpha[closest_idx + 1]
            elif closest_idx == len(alpha) - 1:
                lower_bound = alpha[closest_idx - 1]
                upper_bound = alpha[closest_idx + 0]
            else:
                lower_bound = alpha[closest_idx - 1]
                upper_bound = alpha[closest_idx + 1]

        # Debug and error handling
        if debug: self.plot_duration_vs_alpha(og_wp1, og_wp2, og_v1, og_v2, kin_limits, alpha_marker=alpha_est)
        if not 0 < alpha_est <= 1:
            self.plot_duration_vs_alpha(og_wp1, og_wp2, og_v1, og_v2, kin_limits, alpha_marker=alpha_est)
            assert 0, f"alpha has invalid value: {alpha_est}"
        if err >= err_tol: printm(f"Warning: Duration error too large in alpha estimation. Returning alpha = {alpha_est:.4f} with error = {err:.4f}")

        return alpha_est, err

    def get_sequences_duration(self, og_wp1, og_wp2, og_v1, og_v2, kin_limits, debug=False, enforce_monotony=False):
        ''' Get duration for a 4 segment quintic interpolation.
            If the enforce_monotony flag is set true, the actual duration function will be altered.
            This can be useful to use iterative algorithms which rely on monotony.
                If v2 < -v1, a constant corresponding to v2 = v-1 is set
                If v2 > max_vel, a linearly decreasing function is applied
            max_acc and max_jrk are not set to the limits stored in the object to allow exploration.
        '''
        (max_vel, max_acc, max_jrk) = kin_limits

        if max_acc == 0: return np.nan

        # Setup
        wp_increase = og_wp1 < og_wp2           # Flag for swapped waypoints (in case og_wp1 > og_wp2)
        v_increase  = abs(og_v1) < abs(og_v2)   # Flag for swapped velocities (in case og_v1 > og_v2)
        dist        = abs(og_wp1 - og_wp2)
        ramp_time   = (np.pi * max_acc) / (2 * max_jrk)

        # Mirroring to fit standard interpolation pattern
        if v_increase:      # Keep velocities
            v1  = og_v1
            v2  = og_v2
        else:               # Swap velocities
            v1  = og_v2
            v2  = og_v1

        if wp_increase:     # Keep waypoints
            wp1 = og_wp1
            wp2 = og_wp2
        else:               # Swap waypoints and negate velocities
            wp1 = og_wp2
            wp2 = og_wp1
            v1  = - v1
            v2  = - v2

        if v1 == v2:
            return dist/v1

        if enforce_monotony and v2 < -v1:
           return self.get_sequences_duration(og_wp1, og_wp2, og_v1, -og_v1, kin_limits)

        vel_after_ramp_up    = v1 + (max_acc * ramp_time) / 2   # V_a
        vel_before_ramp_down = v2 - (max_acc * ramp_time) / 2   # V_b
    
        dist_ramp_up    = max_acc * (ramp_time ** 2) * (1/4 - 1/(np.pi**2)) + v1 * ramp_time
        dist_cruise     = (vel_before_ramp_down ** 2 - vel_after_ramp_up ** 2) / (2 * max_acc)
        dist_ramp_down  = max_acc * (ramp_time ** 2) * (1/4 + 1/(np.pi**2)) + vel_before_ramp_down * ramp_time
                    
        duration  =  2 * ramp_time + (vel_before_ramp_down - vel_after_ramp_up) / max_acc 
        if not v2 == 0:
            duration  = duration + (dist - dist_ramp_up - dist_cruise - dist_ramp_down) / v2
        
        if debug: printm(f"og_wp1: {og_wp1:.2f}\tog_wp2: {og_wp2:.2f}\tog_v1: {og_v1:.2f}\tog_v2: {og_v2:.2f}\tmax_acc: {max_acc:.2f}\tmax_jrk: {max_jrk:.2f}\tduration: {duration:.2f}\t")

        if enforce_monotony and v2 > max_vel:
            duration = duration - v2 / max_vel + 1
      
        return duration

    def get_sequence_limits(self, og_wp1, og_wp2, og_v1, og_v2, kin_limits, debug=False):
        ''' Returns the max v1, v2 and min duration for a sequence between two waypoints.
            If working correctly, at most one of max_v1 and max_v2 should divert from og_v1
            and og_v2.
            Parameters:
                og_wp1 (float): First waypoint in sequence 
                og_wp2 (float): Second waypoint in sequence
                og_v1 (float):  First velocity in sequence
                og_v2 (float):  Ideal second velocity in sequence
                kin_limits (tuple): kinematic limits (max_vel, max_acc, max_jrk)
            Returns:
                max_v1 (float): Maximum achievable v1 (first velocity in sequence)
                max_v2 (float): Maximum achievable v2 (second velocity in sequence)
                duration (float):  Minimum achievable duration of the sequence
        '''
        (max_vel, max_acc, max_jrk) = kin_limits
        if max_acc == 0: return np.nan

        # Setup
        wp_increase = og_wp1 < og_wp2           # Flag for swapped waypoints (in case og_wp1 > og_wp2)
        v_increase  = abs(og_v1) < abs(og_v2)   # Flag for swapped velocities (in case og_v1 > og_v2)
        dist        = abs(og_wp1 - og_wp2)
        ramp_time   = (np.pi * max_acc) / (2 * max_jrk)

        # Mirroring to fit standard interpolation pattern
        if v_increase:      # Keep velocities
            v1  = og_v1
            v2  = og_v2
        else:               # Swap velocities
            v1  = og_v2
            v2  = og_v1

        if not wp_increase:     # Swap waypoints and negate velocities
            v1  = - v1
            v2  = - v2

        v2 = max_vel

        # Loop in case v2 is too large
        while True:
            vel_after_ramp_up    = v1 + (max_acc * ramp_time) / 2   # V_a
            vel_before_ramp_down = v2 - (max_acc * ramp_time) / 2   # V_b
        
            dist_ramp_up         = max_acc * (ramp_time ** 2) * (1/4 - 1/(np.pi**2)) + v1 * ramp_time
            dist_cruise          = (vel_before_ramp_down ** 2 - vel_after_ramp_up ** 2) / (2 * max_acc)
            dist_ramp_down       = max_acc * (ramp_time ** 2) * (1/4 + 1/(np.pi**2)) + vel_before_ramp_down * ramp_time                      

            if dist_ramp_up + dist_cruise + dist_ramp_down > dist: 
                #printm(f"Warning: v2 too large, reducing v2 from {v2:.4f} to {v2 - 0.02 * max_vel:.4f}")
                v2 = v2 - 0.005 * max_vel
                assert v2 > 0, "investigate this"
            else:
                break

        # Shift velocities back
        if v_increase:      # Keep velocities
            max_v1  = v1
            max_v2  = v2
        else:               # Swap velocities
            max_v1  = v2
            max_v2  = v1

        if not wp_increase:     # Swap waypoints and negate velocities
            max_v1  = - max_v1
            max_v2  = - max_v2
        
        assert not v2 == 0, "investigate this"
        duration  =  2 * ramp_time + (vel_before_ramp_down - vel_after_ramp_up) / max_acc 
        duration  = duration + (dist - dist_ramp_up - dist_cruise - dist_ramp_down) / v2
        
        if debug: printm(f"og_wp1: {og_wp1:.2f}\tog_wp2: {og_wp2:.2f}\t|og_v1: {og_v1:.2f}\tout_v1: {max_v1:.2f}\t|og_v2: {og_v2:.2f}\tout_v2: {max_v2:.2f}\t|min_duration: {duration:.4f}\t")
    
        return max_v1, max_v2, duration

    def pad_all_trajectories_until_time(self, time_end : float):
        ''' Adds a constant segment to the end of all trajectory until a specified time '''
        for traj in self.trajectories:
            traj.pad_trajectory_until_time(time_end)
        
        return self.trajectories

    def save_trajectories_to_file(self, file_output_name : str):
        ''' Saves the stored trajectories to the specified file in 1ms steps. 
            Overrides any existing file 
        '''
        file_path   = os.path.join(os.path.dirname(__file__), file_output_name)
        positions_t = []
        times_t     = []    # For error detection
        last_pos    = []    # For error detection

        for traj in self.trajectories:
            x, y, (_, pos, _) = traj.get_all_segments(step=0.001, sig_dec=3)
            times_t.append(x)
            positions_t.append(y)
            last_pos.append(pos[-1])

        times_t     = np.array(times_t)
        positions_t = np.array(positions_t)
        positions   = np.transpose(positions_t)

        # Error checks
        assert np.array_equal(times_t[:,0], np.zeros(self.num_joints)), 'Joints do not start at time 0'
        assert np.array_equal(positions_t[:,-1], last_pos), 'Joints do not end at desired position'
        assert len(np.unique(times_t[:,-1])) == 1, 'Joints have different end times'

        # Delete content of existing file
        with open(file_path,'w+') as file:
            file.truncate(0)

        # calculate each joint state for given movement speed and 1000Hz publishing rate
        np.savetxt(file_path, positions, delimiter=',', fmt='%+.20f')


    # Main methods
    def get_trajectories(self, waypoints_t : np.array, velocities_t : np.array, alphas_t : np.array, debug=False):
        ''' Returns a trajectory object for given
            Parameters:
                waypoints (List): Waypoint positions for joints [joint_id][pos]
        '''
    
        # Check if path has invalid values (out of min max bounds)
        for joint_id, joint_pos in enumerate(waypoints_t):
            for k, wp_id in enumerate(joint_pos):
                if wp_id < self.qlimit_pos_min[joint_id] or wp_id > self.qlimit_pos_max[joint_id]:
                    print(f"Waypoint {k} of joint {joint_id} out of bounds: {wp_id:.3f}, bounds are  q_min_pos: {self.qlimit_pos_min[joint_id]:.3f}, q_max_pos: {self.qlimit_pos_max[joint_id]:.3f} ")

        ''' Notes:
            - No two consecutive waypoints can have 0 velocity
            - No two consecutive waypoints can have the same pos
            - Implicit 0 velocity crossing is possible, however, pos bound is not guaranteed
        '''
        # /TODO Check if velocities comply with notes above
        # /TODO Make safety check at the end for pos as well


        # Iterate over joints
        for joint_id, joint_pos in enumerate(waypoints_t):

            # Joint parameters
            max_vel     = self.qlimit_vel_max[joint_id]
            max_jrk     = self.qlimit_jrk_max[joint_id]
            trajectory  = self.trajectories[joint_id]

            # Iterate over waypoints
            for wp_id in range(1, len(joint_pos)):

                # Setup------------------------------------------------------------------------------------
                og_wp1  = joint_pos[wp_id - 1]      # Previous waypoint
                og_wp2  = joint_pos[wp_id]          # Current waypoint
                wp_increase = og_wp1 < og_wp2       # Flag for swapped waypoints (in case og_wp1 > og_wp2)

                og_v1   = velocities_t[joint_id][wp_id - 1]     # Previous velocity
                og_v2   = velocities_t[joint_id, wp_id]         # Current velocity
                v_increase  = abs(og_v1) < abs(og_v2)           # Flag for swapped velocities (in case og_v1 > og_v2)

                max_acc     = self.qlimit_acc_max[joint_id] * alphas_t[joint_id, wp_id]
                ramp_time   = (np.pi * max_acc) / (2 * max_jrk)    # Time for a full ramp with sinosodial acc profile

                dist    = abs(og_wp1 - og_wp2)
                full_ramp_dist  = max_acc * (ramp_time ** 2) + 2 * og_v1 * ramp_time        # /FIXME: is og_v1 correct here? should it be mirrored?

                # Mirroring to fit standard interpolation pattern
                # Normal case
                if wp_increase and v_increase:
                    wp1 = og_wp1
                    wp2 = og_wp2
                    v1  = og_v1
                    v2  = og_v2
                # Swap velocities, so we always have a rising velocity for calculations
                if wp_increase and not v_increase:
                    wp1 = og_wp1
                    wp2 = og_wp2
                    v1  = og_v2
                    v2  = og_v1
                # Swap waypoints, so we always have rising position for calculation
                if not wp_increase and v_increase:
                    wp1 = og_wp2
                    wp2 = og_wp1
                    v1  = - og_v1
                    v2  = - og_v2
                # Swap waypoints and velocities
                if not wp_increase and not v_increase:
                    wp1 = og_wp2
                    wp2 = og_wp1
                    v1  = - og_v2
                    v2  = - og_v1
                
                if debug:
                    print(f"Original: wp1: {og_wp1:.2f}\twp2: {og_wp2:.2f}\tv1: {og_v1:.2f} \tv2: {og_v2:.2f}")
                    print(f"Modified: wp1: {wp1:.2f}\twp2: {wp2:.2f}\tv1: {v1:.2f} \tv2: {v2:.2f}")
                    print(f"wp_increase: {wp_increase}, v_increase: {v_increase}")
                    print(f"Alpha factor: {alphas_t[joint_id, wp_id]:.4f}")

                # Get and register trajectory segment------------------------------------------------------
                # Case: Speed change between positions
                if v1 < v2:
                    # Make sure that there is enough distance for full acceleration ramp
                    if not dist > full_ramp_dist or not v2 - v1 > ramp_time * max_acc:
                        assert 0, "Use Acceleration Pulse, or make dist larger - not implemented yet"

                    # Loop in case v2 is too large
                    while True:
                        # Use Sustained Acceleration Pulse
                        vel_after_ramp_up    = v1 + (max_acc * ramp_time) / 2   # V_a
                        vel_before_ramp_down = v2 - (max_acc * ramp_time) / 2   # V_b
                        
                        dist_ramp_up    = max_acc * (ramp_time ** 2) * (1/4 - 1/(np.pi**2)) + v1 * ramp_time
                        dist_cruise     = (vel_before_ramp_down ** 2 - vel_after_ramp_up ** 2) / (2 * max_acc)
                        dist_ramp_down  = max_acc * (ramp_time ** 2) * (1/4 + 1/(np.pi**2)) + vel_before_ramp_down * ramp_time
                        
                        if dist_ramp_up + dist_cruise + dist_ramp_down > dist: 
                            printm(f"Warning: v2 too large, reducing v2 from {v2:.4f} to {v2 * 0.9:.4f}")
                            v2 = v2 * 0.9       # TODO: this must be given back and stored in velocities_t
                            assert 0, "Adjust velocities should have taken care of this"
                            assert v2 >= 0.001, "Something went wrong, consider lowering v2 or increasing distance"
                        else:
                            break
                    
                    # Get values for quintic control points
                    t0  = 0
                    t1  = t0 + ramp_time
                    t2  = t1 + (vel_before_ramp_down - vel_after_ramp_up) / max_acc
                    t3  = t2 + ramp_time
                    t4  = t3 + (dist - dist_ramp_up - dist_cruise - dist_ramp_down) / v2
                    
                    pos0 = wp1
                    pos1 = pos0 + dist_ramp_up
                    pos2 = pos1 + dist_cruise
                    pos3 = pos2 + dist_ramp_down
                    pos4 = wp2

                    t   = [t0, t1, t2, t3, t4]
                    pos = [pos0, pos1, pos2, pos3, pos4]
                    vel = [v1, vel_after_ramp_up, vel_before_ramp_down, v2, v2]
                    acc = [0, max_acc, max_acc, 0, 0]

                    if any(tt == np.nan or tt == np.inf for tt in t) or sum(t) == 0:
                        assert 0, "Invalid times: Something is wrong here. Check if position change makes sense with velocities."

                    poly_coef   = self.__get_quintic_params(t, pos, vel, acc) 

                # Case: No speed change between positions
                elif v1 == v2:
                    t0  = 0
                    t1  = dist / v1
                    
                    t   = [t0, t1]
                    pos = [wp1, wp2]

                    poly_coef   = [[wp1, (wp2 - wp1) / (t1 - t0), 0, 0, 0]]

                # Catch errror
                else:
                    assert 0, 'No polyonom has been computed, check velocities'

                # Mirror and shift polynoms, set new offset times and mirror pos as well
                if not v_increase:

                    l               = len(poly_coef)
                    polycoefs_old   = poly_coef.copy()
                    times_old       = t.copy()
                    pos_old         = pos.copy()
                    durations       = [times_old[k + 1] - t[k] for k in range(l)]

                    t[0] = 0
                    for i in range(l):
                        old_poly        = Polynomial(polycoefs_old[i])
                        # Mirror along y axis and shift in x direction
                        temp_coefs      = old_poly.convert(window=[1 + durations[i], - 1 + durations[i]]).coef
                        # Mirror along x axis and shift in y direction
                        temp_coefs      = -temp_coefs
                        temp_coefs[0]   = temp_coefs[0] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

                        poly_coef[l - 1 - i] = temp_coefs
                        t[i + 1]            = t[i] + durations[l - 1 - i]

                    for i in range(l + 1):
                        pos[l - i]  = -pos_old[i] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

                # Mirror over offsetted x, set new pos as well
                if not wp_increase :

                    l               = len(poly_coef)
                    polycoefs_old   = poly_coef.copy()
                    pos_old         = pos.copy()

                    for i in range(l):
                        old_poly        = Polynomial(polycoefs_old[i])
                        temp_coefs      = old_poly.coef
                        # Mirror along x axis and shift in y direction
                        temp_coefs      = -temp_coefs
                        temp_coefs[0]   = temp_coefs[0] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

                        poly_coef[i] = temp_coefs

                    for i in range(l + 1):
                        pos[i]  = - pos_old[i] + (og_wp1 + (og_wp2 - og_wp1) / 2) * 2

                # Register trajectory segments
                for i, p_param_pos in enumerate(poly_coef):
                    is_wp = (i == 0 or i == len(poly_coef))
                    trajectory.register_segment(p_param_pos, t[i + 1] - t[i], pos[i], pos[i+1], is_wp)

                # self.plot_trajectory_waypoint(poly_coef, t, pos)
                # self.plot_trajectory_waypoint_limits(poly_coef, t, pos, limits)
                a = 1

            if debug:
                self.plot_trajectory(joint_id)
                print(t)
        
        return self.trajectories

    def get_waypoint_parameters(self, waypoints_t : np.array, ideal_vel_t : np.array, sync_joints=True, debug=False):
        ''' To synchronise all joints, they must be at a waypoint at the same time. 
            To achieve this, the max acceleration for each joint at each waypoint is computed.
            The max acceleration is coded into a list containing the fractions of max_acc that
            need to be used. This factor is called alpha: used_max_acc = alpha * actual_max_acc

            For each waypoint
                For each joint

            1.  Depending on distance between waypoints, get max starting and ending velocity.
                The duration is computed which is needed to transition to the next waypoint.
                Initally alpha = 1 (such that used_max_acc = actual_max_acc)
            2.  The max. duration of all joints is set for all joints, this way the slowest
                joint will have its min duration while all other joints are slowed down
            3.  The corresponding alpha factor is computed, such that the desired delay is achieved.
                This is done numerically.           
            
            Parameters:
                ideal_vel_t (List): Ideal velocities for each joint and waypoint [joint_id][wp] (+-max_vel, 0) 
                waypoints_t (List): Waypoint positions for joints [joint_id][wp]
                sync_joints (bool): Enables step 2 and 3
            Returns:
                alphas_t (List): Alpha factors corresponding to each joint and waypoint [joint_id][wp]
                adjust_vel (List): Adjusted velocities, such that they can be reached in the given distance [joint_id][wp]
        '''
        adjust_vel_t  = ideal_vel_t                       # Adjusted vel, init with ideal vel [joint_id][wp]
        initial_dur_t = np.zeros(waypoints_t.shape)       # [joint_id][wp]
        synced_dur    = 0                                 # Scalar
        alphas_t      = np.ones(waypoints_t.shape)        # Alpha factors [joint_id][wp]

        num_joints  = waypoints_t.shape[0]
        num_wp      = waypoints_t.shape[1]

        # Kinematic limits in a list
        kin_limits_l = [(self.qlimit_vel_max[joint_id], self.qlimit_acc_max[joint_id], self.qlimit_jrk_max[joint_id]) for joint_id in range(num_joints)]

        # Iterate over waypoints
        for wp_id in range(num_wp - 1):

            # 1. Get initial durations for each waypoint transition
            for joint_id in range(num_joints):
                wp1         = waypoints_t[joint_id, wp_id]
                wp2         = waypoints_t[joint_id, wp_id + 1]
                v1          = adjust_vel_t[joint_id, wp_id]
                v2          = adjust_vel_t[joint_id, wp_id + 1]
                kin_limits  = kin_limits_l[joint_id]
                
                max_v1, max_v2, min_d             = self.get_sequence_limits(wp1, wp2, v1, v2, kin_limits, debug=debug)
                adjust_vel_t[joint_id, wp_id]     = min(min([max_v1, v1]), max([max_v1, v1]), key=abs)
                adjust_vel_t[joint_id, wp_id + 1] = min(min([max_v2, v2]), max([max_v2, v2]), key=abs)

                initial_dur_t[joint_id, wp_id]    = min_d

            # Do not continue if sync joints is disabled
            if not sync_joints:
                continue

            # 2. Synchronise duration
            synced_dur = max(initial_dur_t[:, wp_id])
            if debug: print(f"synced_dur: {synced_dur:.4f} for wp {wp_id}")

            # 4. Get synchronized alphas at waypoints
            for joint_id in range(num_joints):
                wp1         = waypoints_t[joint_id, wp_id]
                wp2         = waypoints_t[joint_id, wp_id + 1]
                v1          = adjust_vel_t[joint_id, wp_id]
                v2          = adjust_vel_t[joint_id, wp_id + 1]
                kin_limits  = kin_limits_l[joint_id]

                alpha, err                      = self.get_sequences_alpha(synced_dur, wp1, wp2, v1, v2, kin_limits, debug=debug)
                alphas_t[joint_id, wp_id]       = alpha

                # TODO here !
                #kin_limits_mod  = (self.qlimit_vel_max[joint_id], self.qlimit_acc_max[joint_id] * alpha, self.qlimit_jrk_max[joint_id])
                #
                #max_v1, max_v2, min_d           = self.get_sequence_limits(wp1, wp2, v1, v2, kin_limits_mod, debug=debug)
                #adjust_vel[joint_id, wp_id]     = min(min([max_v1, v1]), max([max_v1, v1]), key=abs)
                #adjust_vel[joint_id, wp_id + 1] = min(min([max_v2, v2]), max([max_v2, v2]), key=abs)

                if debug: print(f'dur err: {err} for joint {joint_id} wp1 {wp1} to wp2 {wp2}')

        # Dummy alphas for last waypoint
        for joint_id in range(num_joints):
            alphas_t[joint_id, -1] = 1     

        return alphas_t, adjust_vel_t, initial_dur_t
        

    # Old methods
    def plot_duration_vs_v2(self, og_wp1, og_wp2, kin_limits, v1_list : List, v2_marker=None, scaled=False):
        ''' Plots a graph for the duration with a given v1 
            Parameters:
                v1_list (List): List of v1 to be plotted
                v2_marker (float): Marks a v2 value and corresponding duration
                scaled (bool): Scales x to max_vel
                
                v1_list and v2_marker must be scaled, if scaled is true
        '''
        (max_vel, max_acc, max_jrk) = kin_limits

        # Settings
        v2_vals = np.linspace(-2 * max_vel, 2 * max_vel, 1000)
        if scaled:
            x_label         = 'v2/max_vel'
            legend_label    = 'v1/max_vel'
            v2_axis         = v2_vals / max_vel
            v1_vals         = v1_list * max_vel
        else:
            x_label         = 'v2 [ang/s]'
            legend_label    = 'v1 [ang/s]'
            v2_axis         = v2_vals
            v1_vals         = v1_list
       
        # Get data       
        dur     = [[0] * len(v2_vals) for i in range(len(v1_vals))]
        for i, og_v1 in enumerate(v1_vals):
            for j, og_v2 in enumerate(v2_vals):

                d           = self.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, kin_limits, enforce_monotony=True)
                dur[i][j]   = d
        
        # Plot data
        fig, axs = plt.subplots(1, 2)
        for n, ylim in enumerate([25, 1.25]):
            for i in range(len(v1_list)):
                axs[n].plot(v2_axis, dur[i], label=f"{v1_list[i]:.2f}")
            
            axs[n].set_xlabel(x_label)
            axs[n].set_ylim(0, ylim)
            axs[n].grid()

            # Plot v2_marker lines
            if not v2_marker == None:
                axs[n].axvline(v2_marker, color='red', lw=0.5)

                for i, og_v1 in enumerate(v1_list):
                    d_marker = (self.get_sequences_duration(og_wp1, og_wp2, og_v1, v2_marker, kin_limits, enforce_monotony=True))
                    axs[n].axhline(d_marker, color='red', lw=0.5)

        axs[0].set_ylabel('Duration [s]')
        plt.suptitle(f'Duration to traverse from wp1 to wp2 depending on v1 and v2\nwp1 = {og_wp1}, wp2={og_wp2}')
        plt.legend(title=legend_label)
        plt.show()

    def get_sequences_v2(self, target_dur, og_wp1, og_wp2, og_v1, kin_limits, err_tol=1e-6, max_iter=5, debug=False):
        ''' Get v2 for a 4 segment quintic interpolation with given target_duration. Employs 
            a simple iterative algorithm to find v2. max_acc and max_jrk are not set to the
            limits stored in the object to allow exploration
            Problems: Does not converge oftentimee and returns edge values
            Returns:
                est_v2 (float): The estimated v2 to achieve the target duration [ang/s]
                dur_err (float): Duration error [s]
        '''  
        (max_vel, max_acc, max_jrk) = kin_limits

        lower_bound = -1 * max_vel
        upper_bound =  1 * max_vel

        for _ in range(max_iter):

            v2_vals = np.linspace(lower_bound, upper_bound, 10000)
            t_vals  = [self.get_sequences_duration(og_wp1, og_wp2, og_v1, v2, kin_limits, enforce_monotony=True) for v2 in v2_vals]
            t_vals  = np.array(t_vals)
            closest_idx = (np.abs(t_vals - target_dur)).argmin()
            
            v2_est  = v2_vals[closest_idx]
            dur_est = self.get_sequences_duration(og_wp1, og_wp2, og_v1, v2_est, kin_limits, enforce_monotony=True)
            err     = abs(dur_est - target_dur)

            if err <= err_tol: break

            if closest_idx == 0:
                lower_bound = v2_vals[closest_idx - 0]
                upper_bound = v2_vals[closest_idx + 1]
            elif closest_idx == len(v2_vals) - 1:
                lower_bound = v2_vals[closest_idx - 1]
                upper_bound = v2_vals[closest_idx + 0]
            else:
                lower_bound = v2_vals[closest_idx - 1]
                upper_bound = v2_vals[closest_idx + 1]

        if debug:
            self.plot_duration_vs_v2(og_wp1, og_wp2, kin_limits, [og_v1]) 
            self.plot_duration_vs_v2(og_wp1, og_wp2, kin_limits, [og_v1], v2_marker=v2_est)

        assert abs(v2_est) <= max_vel, "Estimated v2 larger than max_vel"

        # In case no convergence, return -max_vel or +max_vel
        if err <= err_tol:
            dur_est_minus   = self.get_sequences_duration(og_wp1, og_wp2, og_v1, -max_vel, kin_limits, enforce_monotony=True)
            dur_est_plus    = self.get_sequences_duration(og_wp1, og_wp2, og_v1,  max_vel, kin_limits, enforce_monotony=True)
            if abs(dur_est_minus - target_dur) < abs(dur_est_plus - target_dur):
                v2_est  = dur_est_minus
                err     = abs(dur_est_minus - target_dur)
            else:
                v2_est  = dur_est_plus
                err     = abs(dur_est_plus - target_dur)

            printm(f"Warning: Duration error too large in v2 estimation. Returning v2_est = {v2_est:.4f}")

        return v2_est, err

    def get_sequences_v2_newton(self, target_dur, og_wp1, og_wp2, og_v1, kin_limits, err_tol=1e-6, debug=False):
        ''' Get v2 for a 4 segment quintic interpolation with given target_duration. Employs 
            Newton's Secant method to find v2. max_acc and max_jrk are not set to the limits 
            stored in the object to allow exploration
            Returns:
                est_v2 (float): The estimated v2 to achieve the target duration [ang/s]
                dur_err (float): Duration error [s]
        '''
        (max_vel, max_acc, max_jrk) = kin_limits

        def f1(og_v2):
            return self.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, kin_limits, enforce_monotony=True) - target_dur
        def f2(og_v2):
            return self.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, kin_limits, enforce_monotony=True) + target_dur

        #x0  = 1e-6 * max_vel
        #try:
        #    est_v2  = scipy.optimize.newton(f1, x0, tol=err_tol)
        #except RuntimeError:
        #    try:
        #        est_v2  = scipy.optimize.newton(f2, x0, tol=err_tol)
        #    except RuntimeError:
        #        est_v2 = self.get_sequences_v2(target_dur, og_wp1, og_wp2, og_v1, kin_limits)

        est_dur = self.get_sequences_duration(og_wp1, og_wp2, og_v1, est_v2, kin_limits, enforce_monotony=True)
        dur_err = abs(target_dur - est_dur)

        # Check if v2 is too large
        if est_v2 > max_vel:
            print('!error: Estimated v2 larger than max_vel')
            self.plot_duration_vs_v2(og_wp1, og_wp2, kin_limits, [og_v1], v2_marker=est_v2)
            assert 0, 'Estimated v2 larger than max_vel'

        if debug: self.plot_duration_vs_v2(og_wp1, og_wp2, kin_limits, og_v1, v2_marker=est_v2) 

        return est_v2, dur_err

    # Interface methods
    def create_point_to_point_traj(self, og_wp1, og_wp2, file_output_name, debug=False):
        ''' Creates a point to point trajectory, starting and ending with 0 vel. Creates an
            intermediate waypoint to adjust velocity for each joint.
        '''

        wp1     = np.array(og_wp1)
        wp2     = np.array(og_wp2)
        wp_int  = wp1 + (wp2 - wp1) / 2     # intermediate waypoint

        waypoints_t = np.transpose([wp1, wp_int, wp2])
        ideal_vel_t = self.get_initial_velocities(waypoints_t)

        num_joints  = waypoints_t.shape[0]
        num_wp      = waypoints_t.shape[1]

        # Check if waypoints are far enough apart for full ramp up and down
        for joint_id in range(num_joints):
            max_vel = self.qlimit_vel_max[joint_id]
            max_acc = self.qlimit_acc_max[joint_id]
            max_jrk = self.qlimit_jrk_max[joint_id]
            
            for wp_id in range(num_wp - 1):

                dist            = abs(waypoints_t[joint_id, wp_id] - waypoints_t[joint_id, wp_id + 1])
                ramp_time       = (np.pi * max_acc) / (2 * max_jrk)    # Time for a full ramp with sinosodial acc profile
                full_ramp_dist  = max_acc * (ramp_time ** 2) + 2 * 0 * ramp_time    # * 0 because og_v1 is always 0 

                assert dist > full_ramp_dist, f'Waypoint {wp1} and {wp2} of joint {joint_id} are too close together for full ramps'
        
        
        alphas_t, adjust_vel_t, _ = self.get_waypoint_parameters(waypoints_t, ideal_vel_t, sync_joints=False, debug=debug)
        
        # Get trajectories
        trajectories    = self.get_trajectories(waypoints_t, adjust_vel_t, alphas_t)

        # Find max trajectory time and pad all trajectores
        max_times       = [traj.time_list[-1] for traj in trajectories]
        max_time        = max(max_times)
        trajectories_p  = self.pad_all_trajectories_until_time(max_time + 1)

        if debug:
            self.plot_all_trajectories()

        self.save_trajectories_to_file(file_output_name)

        a = 1
        


# /TODO: Quintic interpolation for slower joints

if __name__ == '__main__':
    select  = 0

# Test create_point_to_point_traj
if __name__ == '__main__' and select == 3:

    # Init
    limits  = { "q_pos_max" : [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973],
                "q_pos_min" : [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],
                "q_vel_max" : [ 2.1750,	2.1750,	2.1750,	2.1750, 2.6100, 2.6100, 2.6100],
                "q_acc_max" : [     15,    7.5,     10,   12.5,     15,     20,     20],
                "q_jrk_max" : [   7500,   3750,   5000,   6250,   7500,  10000,  10000],
                "p_vel_max" :   1.7000,
                "p_acc_max" :  13.0000,
                "p_jrk_max" : 6500.000 }

    waypoints   = get_list_from_csv('', "temp_point2point_path.csv")
    wp1         = waypoints[0]
    wp2         = waypoints[1]

    trajectory_planer = TrajectoryPlannerComplicated(limits, 7)
    trajectory_planer.create_point_to_point_traj(wp1, wp2, 'test_create_p2p_traj.csv')
    trajectory_planer.plot_all_trajectories()
    a = 1

# Test get_trajectory
if __name__ == '__main__' and select == 0:

    limits  = { "q_pos_max" : [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973],
                "q_pos_min" : [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],
                "q_vel_max" : [ 2.1750,	2.1750,	2.1750,	2.1750, 2.6100, 2.6100, 2.6100],
                "q_acc_max" : [     15,    7.5,     10,   12.5,     15,     20,     20],
                "q_jrk_max" : [   7500,   3750,   5000,   6250,   7500,  10000,  10000],
                "p_vel_max" :   1.7000,
                "p_acc_max" :  13.0000,
                "p_jrk_max" : 6500.000 }

    # Waypoints here in degree (actually in radians)
    waypoints   = get_list_from_csv("", "Path/waypoints_02.csv")
    waypoints   = np.deg2rad(np.array(waypoints))

    # Convert limits to degree, for easier reading
    #keys    = ["q_pos_max", "q_pos_min", "q_vel_max", "q_acc_max" ,"q_jrk_max"]
    #for key in keys:
    #    limits[key] = np.rad2deg(limits[key])

    max_vel_l = limits['q_vel_max']

    waypoints_t         = np.transpose(waypoints)
    trajectory_planer   = TrajectoryPlannerComplicated(limits, 7, safety_factor=1, safety_margin=0)

    num_joints  = waypoints_t.shape[0]
    num_wp      = waypoints_t.shape[1]
    

    velocities_man = []
    alphas_man     = []
    for i in range(num_joints):
        max_vel = max_vel_l[i]
        #velocities_man.append([0, max_vel, max_vel, 0, -max_vel, -max_vel, 0])      # Rise and fall
        #velocities_man.append([max_vel, -max_vel, -max_vel, max_vel, -max_vel, 0])         # Oscilate
        #velocities_man.append([0, max_vel, max_vel, max_vel, 0, -max_vel, -max_vel, 0])
        velocities_man.append([0,  max_vel, max_vel, 0, -max_vel, -max_vel, 0])
        alphas_man.append([1, 1, 1, 1, 1, 1, 1, 1])
        
    velocities_man  = np.array(velocities_man)
    alphas_man      = np.array(alphas_man)

    alphas, adjust_vel, _   = trajectory_planer.get_waypoint_parameters(waypoints_t, velocities_man, sync_joints=True, debug=False)
    traj_all                = trajectory_planer.get_trajectories(waypoints_t, adjust_vel, alphas_man)
    trajectory_planer.plot_trajectory(0)

# Explore duration to og_v1 and og_v2 relationship 
if __name__ == '__main__' and select == 1:

    limits  = { "q_pos_max" : [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973],
                "q_pos_min" : [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],
                "q_vel_max" : [ 2.1750,	2.1750,	2.1750,	2.1750, 2.6100, 2.6100, 2.6100],
                "q_acc_max" : [     15,    7.5,     10,   12.5,     15,     20,     20],
                "q_jrk_max" : [   7500,   3750,   5000,   6250,   7500,  10000,  10000],
                "p_vel_max" :   1.7000,
                "p_acc_max" :  13.0000,
                "p_jrk_max" : 6500.000 }

    # Waypoints here in degree (actually in radians)
    waypoints   = get_list_from_csv("", "waypoints_02.csv")
    waypoints   = np.array(waypoints)

    # Convert limits to degree, for easier reading
    keys    = ["q_pos_max", "q_pos_min", "q_vel_max", "q_acc_max" ,"q_jrk_max"]
    for key in keys:
        limits[key] = np.rad2deg(limits[key])

    trajectory_planer   = TrajectoryPlannerComplicated(limits, safety_factor=1, safety_margin=0)

    # Settings
    waypoints_t = np.transpose(waypoints)
    joint_id    = 0
    pos1        = 0
    max_vel     = limits['q_vel_max'][joint_id]
    max_acc     = limits['q_acc_max'][joint_id]
    max_jrk     = limits['q_jrk_max'][joint_id]
    kin_limits  = (max_vel, max_acc, max_jrk)

    v1_list     = np.linspace(0, 1, 6)  * max_vel   # Factor scaling max_vel
 
    # Get data
    og_wp1  = 0 #waypoints_t[joint_id, pos1]
    og_wp2  = 10 #waypoints_t[joint_id, pos1 + 1]
    
    trajectory_planer.plot_duration_vs_v2(og_wp1, og_wp2, kin_limits, v1_list, scaled=False)

    target_dur  = 2
    og_v1       = 0
    est_v2, err = trajectory_planer.get_sequences_v2(target_dur, og_wp1, og_wp2, og_v1, kin_limits)

    a = 1

# Explore duration to og_v1 and max_vel, max_acc, max_jrk relationship (used for alpha functions)
if __name__ == '__main__' and select == 2:

    limits  = { "q_pos_max" : [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973],
                "q_pos_min" : [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],
                "q_vel_max" : [ 2.1750,	2.1750,	2.1750,	2.1750, 2.6100, 2.6100, 2.6100],
                "q_acc_max" : [     15,    7.5,     10,   12.5,     15,     20,     20],
                "q_jrk_max" : [   7500,   3750,   5000,   6250,   7500,  10000,  10000],
                "p_vel_max" :   1.7000,
                "p_acc_max" :  13.0000,
                "p_jrk_max" : 6500.000 }

    # Waypoints here in degree (actually in radians)
    waypoints   = get_list_from_csv("", "waypoints_02.csv")
    waypoints   = np.array(waypoints)

    # Convert limits to degree, for easier reading
    keys    = ["q_pos_max", "q_pos_min", "q_vel_max", "q_acc_max" ,"q_jrk_max"]
    for key in keys:
        limits[key] = np.rad2deg(limits[key])

    trajectory_planer   = TrajectoryPlannerComplicated(limits, safety_factor=1, safety_margin=0)

    # Settings
    waypoints_t = np.transpose(waypoints)
    joint_id    = 0
    pos1        = 0
    max_vel     = limits['q_vel_max'][joint_id]
    max_acc     = limits['q_acc_max'][joint_id]
    max_jrk     = limits['q_jrk_max'][joint_id]
    kin_limits  = (max_vel, max_acc, max_jrk)

    v1_list     = np.linspace(0, 1, 6)  * max_vel   # Factor scaling max_vel
 
    # Get data
    og_wp1  = 20 #waypoints_t[joint_id, pos1]
    og_wp2  = 100 #waypoints_t[joint_id, pos1 + 1]
    og_v1   = max_vel * 0.5
    og_v2   = max_vel
     
    target_dur      = 0.67
    alpha_est, err  = trajectory_planer.get_sequences_alpha(target_dur, og_wp1, og_wp2, og_v1, og_v2, kin_limits)
    achieved_dur    = trajectory_planer.get_sequences_duration(og_wp1, og_wp2, og_v1, og_v2, (max_vel, max_acc * alpha_est, max_jrk))
    print(f'Target duration: {target_dur:.4f}, achieved duration: {achieved_dur:.4f} (err: {err:.4f}) with alpha: {alpha_est:.4f}')
    trajectory_planer.plot_duration_vs_alpha(og_wp1, og_wp2, og_v1, og_v2, kin_limits, alpha_marker=alpha_est)
    


    a = 1
