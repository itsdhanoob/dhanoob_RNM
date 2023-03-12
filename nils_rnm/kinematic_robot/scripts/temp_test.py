import numpy as np

def calculate_goal_pose_from_position(goal_position, omega_x = 0, omega_y = 0, omega_z = 0):
    rot_mat_x = np.array([  [1,0,0],
                            [0,np.cos(omega_x), -np.sin(omega_x)],
                            [0,np.sin(omega_x), np.cos(omega_x)]])
    rot_mat_y = np.array([  [np.cos(omega_y),0,np.sin(omega_y)],
                            [0,1,0],
                            [-np.sin(omega_y),0,np.cos(omega_y)]])
    rot_mat_z = np.array([  [np.cos(omega_z),-np.sin(omega_z), 0],
                            [np.sin(omega_z), np.cos(omega_z), 0],
                            [0,0,1]])
    rot = np.matmul(np.matmul(rot_mat_x, rot_mat_y),rot_mat_z)
    A = np.append(rot, goal_position)
    return A


goal_pos = [2, 3, 4]
A = calculate_goal_pose_from_position(goal_pos)
print(A)