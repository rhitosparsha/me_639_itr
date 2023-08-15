import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

l1 = 2.0 # length of link 1 in m
l2 = 2.0 # length of link 2 in m
k = 600 # spring constant in N/m

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
t_max = 5   # total simulation time (s)
num_steps = 50
num_frames = 100
mean_pos = (3, 1) # mean position of spring
ex_pos_1 = (3, 0)
ex_pos_2 = (3, 2)
traj_1 = [(1 - i) * np.array(ex_pos_1) + i * np.array(ex_pos_2) for i in np.linspace(0, 1, num_steps)]
traj_2 = [(1 - i) * np.array(ex_pos_1) + i * np.array(ex_pos_2) for i in np.linspace(1, 0, num_steps)]
trajectory = traj_1 + traj_2
joint_angles = [inv_kin(x, y) for x, y in trajectory]

# preparing the plot 
fig, ax = plt.subplots(figsize=(8, 8))  # Equal aspect ratio
ax.set_xlim(-5.0, 5.0)
ax.set_ylim(-5.0, 5.0)
ax.scatter(x=0, y=0, c='r')

# plotting grid lines
ticks = np.arange(-5, 5, 1)
ax.set_xticks(ticks)
ax.set_yticks(ticks)

plt.xlabel('x (m)') #label
plt.ylabel('y (m)')
plt.title('2R Manipulator acting as Spring')
plt.grid()

plt.text(3.2, 0.9, "Mean Position : (3, 1)")
plt.plot(3, 1, 'g.')
plt.text(3.2, -0.1, "Extreme Position : (3, 0)")
plt.plot(3, 0, 'g.')
plt.text(3.2, 1.9, "Extreme Position : (3, 2)")
plt.plot(3, 2, 'g.')
plt.plot([3, 3], [0, 2], 'g--')

# calculation of spring torques
def spring_torque_calc(q1, q2):
    t1s = k * (l1 * np.sin(q1) + l2 * np.sin(q2)) * l1 * np.cos(q1) - k * (l1 * np.cos(q1) + l2 * np.cos(q2)) * l1 * np.sin(q1)
    t2s = k * (l1 * np.sin(q1) + l2 * np.sin(q2)) * l2 * np.cos(q2) - k * (l1 * np.cos(q1) + l2 * np.cos(q2)) * l2 * np.sin(q2)
    return t1s, t2s

plt.text(-3.5, 3, "Spring Constant = 600 N/m")
plt.text(-3.5, 2.8, "Instantaneous spring torque on link 1 =                   Nm")
plt.text(-3.5, 2.6, "Instantaneous spring torque on link 2 =                   Nm")

link1, = plt.plot([], [], 'b-')
link2, = plt.plot([], [], 'b-')
point1, = plt.plot([], [], 'r.')
point2, = plt.plot([], [], 'r.')



def animate(i):
    q1, q2 = joint_angles[i]
    x, y = fwd_kin(q1, q2)
    t1s, t2s = spring_torque_calc(q1, q2)

    t1s_disp = plt.text(1.1, 2.8, round(t1s, 3))
    t2s_disp = plt.text(1.1, 2.6, round(t2s, 3))

    link1.set_data([0, l1 * np.cos(q1)], [0, l1 * np.sin(q1)])
    point1.set_data(l1 * np.cos(q1), l1 * np.sin(q1))
    link2.set_data([l1 * np.cos(q1), x], [l1 * np.sin(q1), y])
    point2.set_data(x, y)
    
    plt.pause(0.0001)
    t1s_disp.remove()
    t2s_disp.remove()

ani = FuncAnimation(fig, animate, frames=num_frames, interval=5)

plt.show()
