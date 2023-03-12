import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection, LineCollection


def forward_to_endeffektor(theta1, theta2):
    """
    Calculates the forward kinematics for the robot from the exercise
    :param theta1: angle of the first joint in radiant
    :param theta2: angle of the second joint in radiant
    :return: x, y position of the endeffector
    """
    l1 = 50
    l2 = 50
    x = l2 * np.cos(theta1 + theta2) + l1 * np.cos(theta1)
    y = l2 * np.sin(theta1 + theta2) + l1 * np.sin(theta1)
    return x, y


def in_hull(p, hull):
    """
    Test if points in `p` are in `hull`

    :param p: should be a `NxK` coordinates of `N` points in `K` dimensions
    :param hull: is either a scipy.spatial.Delaunay object or the `MxK` array of the
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p) >= 0


# Discretization
xSampled = np.arange(-150,150,1)
ySampled = np.arange(-150,150,1)

theta1Sampled = np.arange(-90, 90, 1) / 180 * np.pi
theta2Sampled = np.arange(-90, 90, 1) / 180 * np.pi

obstacle = np.asarray([[20, 80], [45, 50], [70, 80]])
hull = Delaunay(obstacle)


# Mark non reachable points in grid
isNotReachable = np.zeros((len(theta2Sampled), len(theta1Sampled)), np.bool)
points = np.zeros((len(theta2Sampled)*len(theta1Sampled), 2))
for t2Idx, t2 in enumerate(theta2Sampled):
    for t1Idx, t1 in enumerate(theta1Sampled):
        x, y = forward_to_endeffektor(t1, t2)
        points[t2Idx*len(theta1Sampled) + t1Idx,:] = x,y
        if in_hull((x,y), hull):
            # Dimensions are flipped here because of imshows visualization order
            isNotReachable[len(theta2Sampled) - 1 - t2Idx, t1Idx] = True

fig, (ax1, ax2) = plt.subplots(2)
ax1.imshow(isNotReachable, extent=[-90, 90, -90, 90])
ax1.set(xlabel="Theta 1")
ax1.set(ylabel="Theta 2")

ax2.scatter(points[:,0], points[:,1])
ax2.fill_between(obstacle[:, 0], 80, obstacle[:,1])
ax2.set(xlim=[xSampled[0], xSampled[-1]])
ax2.set(ylim=[ySampled[0], ySampled[-1]])
plt.show()


