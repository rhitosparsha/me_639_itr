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

# create planned trajectory
t_max = 5.0   # total simulation time (s)
num_steps = 50
start_point = (-3.5, 1)
end_point = (-1, 3.5)
trajectory = [(1 - i) * np.array(start_point) + i * np.array(end_point) for i in np.linspace(0, 1, num_steps)]
joint_angles = [inv_kin(x, y) for x, y in trajectory]

# preparing the plot
fig, ax = plt.subplots(figsize=(8, 8))  # Equal aspect ratio
ax.set_xlim(-4.0, 4.0)
ax.set_ylim(-4.0, 4.0)
ax.scatter(x=0, y=0, c='r')

plt.xlabel('x (m)') #label
plt.ylabel('y (m)')
plt.title('2R Manipulator following a given trajectory')
plt.grid()

link1, = plt.plot([], [], 'b-')
link2, = plt.plot([], [], 'b-')
point1, = plt.plot([], [], 'r.')
point2, = plt.plot([], [], 'r.')

end_effector_path, = plt.plot([], [], 'g--', label="End-Effector Path")

# animation function
def animate(i):
    q1, q2 = joint_angles[i]
    x, y = fwd_kin(q1, q2)

    link1.set_data([0, l1 * np.cos(q1)], [0, l1 * np.sin(q1)])
    point1.set_data(l1 * np.cos(q1), l1 * np.sin(q1))
    link2.set_data([l1 * np.cos(q1), x], [l1 * np.sin(q1), y])
    point2.set_data(x, y)
    
    end_effector_path.set_data([p[0] for p in trajectory[:i+1]], [p[1] for p in trajectory[:i+1]])

ani = FuncAnimation(fig, animate, frames=num_steps, interval=(t_max / num_steps) * 1000)

plt.show()
