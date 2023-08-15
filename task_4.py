import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

l1 = 2.0 # length of link 1 in m
l2 = 2.0 # length of link 2 in m

# here, angles q1 and q2 are both measured from the x-axis

# inverse kinematics
def inv_kin (x, y):
    theta = np.arccos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
    q1 = np.arctan2(y, x) - np.arctan2((l2 * np.sin(theta)), (l1 + l2 * np.cos(theta)))
    q2 = q1 + theta
    return q1, q2

# forward kinematics
def fwd_kin (q1, q2):
    x = l1 * np.cos(q1) + l2 * np.cos(q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q2)
    return x, y 

# preparing the plot
fig, ax = plt.subplots(figsize=(8, 8))  # Equal aspect ratio
ax.set_xlim(-4.0, 4.0)
ax.set_ylim(-4.0, 4.0)

plt.xlabel('x (m)') #label
plt.ylabel('y (m)')
plt.title('Workspace of 2R Manipulator')
plt.grid()

for q1 in range(35, 145, 1):
    for q2 in range(35, 145, 1):
        x, y = fwd_kin(math.radians(q1), math.radians(q2))
        plt.scatter(x, y, s = 1, c='b')

plt.show()
