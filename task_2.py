import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

l1 = 2.0 # length of link 1 in m
l2 = 2.0 # length of link 2 in m
num_steps = 100000 # no. of divisions of the line of wall
num_frames = 50 # no. of frames in animation
N = 500 # normal force on wall in N

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

fig, ax = plt.subplots(figsize=(8, 8))  # Equal aspect ratio
ax.set_xlim(-4.0, 4.0)
ax.set_ylim(-4.0, 4.0)
ax.scatter(0, 0, c='r')

plt.xlabel('x (m)') #label
plt.ylabel('y (m)')
plt.title('2R Manipulator applying normal force on a wall')
plt.grid()

# plot the wall with intercepts at (4,0) and (0,4) which is inclined at an angle of 135deg to x-axis
wall_x_int = (4, 0)
wall_y_int = (0, 4)

wall = plt.plot(np.array(wall_x_int), np.array(wall_y_int), 'g-', linewidth = 4)

wall_points = [(1 - i) * np.array(wall_x_int) + i * np.array(wall_y_int) for i in np.linspace(0, 1, num_steps)]

# this loop iterates through the all the points of the wall equation and stops when value of angle q2 = 45deg
# i.e. it stops when it finds a point where link 2 is perpendicular to the wall
for x, y in wall_points:
        q1, q2 = inv_kin(x, y)
        if round(q2, 4) == round(np.pi / 4, 4): # rounding off up to 4 decimal places
            break

# (x, y) is the point where the end effector touches the wall

# resolving normal force into components along the individual axes
Nx = N * np.cos(np.pi / 4)
Ny = N * np.sin(np.pi / 4)

# calculation of torques
T1 = Ny * l1 * np.cos(q1) - Nx * l1 * np.sin(q1)
T2 = Ny * l2 * np.cos(q2) - Nx * l2 * np.sin(q2) # ?????

contact_point = "Point of contact : (" + str(round(x, 2)) + ", " + str(round(y, 2)) + ")"
app_t1 = "Torque applied on link 1 = " + str(round(T1, 3)) + " N"
app_t2 = "Torque applied on link 2 = " + str(round(T2, 3)) + " N"

# printing of data on to graph
plt.text(-3.5, 3, "Normal Force = 500 N")
plt.text(-3.5, 2.8, "Wall intercepts : (4, 0) and (0, 4)")
plt.text(-3.5, 2.6, contact_point)
plt.text(-3.5, 2.4, app_t1)
plt.text(-3.5, 2.2, app_t2)

# showing trajectory of robot till it reaches desired point on wall

# create planned trajectory
t_max = 5.0   # total simulation time (s)
start_point = (-1, -3.5) # point of start of end effector
end_point = (x, y)
trajectory = [(1 - i) * np.array(start_point) + i * np.array(end_point) for i in np.linspace(0, 1, num_frames)]
joint_angles = [inv_kin(a, b) for a, b in trajectory]

link1, = plt.plot([], [], 'b-')
link2, = plt.plot([], [], 'b-')
point1, = plt.plot([], [], 'r.')
point2, = plt.plot([], [], 'r.')

end_effector_path, = plt.plot([], [], 'g--', label="End-Effector Path")

def animate(i):
    q1, q2 = joint_angles[i]
    a, b = fwd_kin(q1, q2)

    link1.set_data([0, l1 * np.cos(q1)], [0, l1 * np.sin(q1)])
    point1.set_data(l1 * np.cos(q1), l1 * np.sin(q1))
    link2.set_data([l1 * np.cos(q1), a], [l1 * np.sin(q1), b])
    point2.set_data(a, b)
    
    end_effector_path.set_data([p[0] for p in trajectory[:i+1]], [p[1] for p in trajectory[:i+1]])

ani = FuncAnimation(fig, animate, frames = num_frames, interval = (t_max / num_frames) * 1000, repeat = False)

plt.show()
