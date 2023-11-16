import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.optimize import minimize

l1 = 1
l2 = 1
l3 = 1

def jaco_3(th1, th2, th3):
    J = np.array([[-l1 * np.sin(th1) - l2 * np.sin(th2 + th1)- l3 * np.sin(th3 + th2+ th1),
                   -l2 * np.sin(th2 + th1) - l3 * np.sin(th3+ th2+ th1),
                   -l3 * np.sin(th1+th2+th3)],
                  [l1 * np.cos(th1) + l2* np.cos(th2 + th1) + l3 * np.cos(th3 + th2 + th1),
                   l2 * np.cos(th2 + th1) + l3 *np.cos(th3 + th2 + th1),
                   l3 * np.cos(th1 + th2 + th3)]])
    return J

def fwd_kin(q):
    x = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1]) + l3 * np.cos(q[0] + q[1] + q[2])
    y = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1]) + l3 * np.sin(q[0] + q[1] + q[2])
    return np.array([x, y])

def animate_3r(q, xt, yt):
    fig, ax = plt.subplots()
    ax.plot(xt, yt, '--y')

    line, = ax.plot([], [], 'k', linewidth=2)
    point, = ax.plot([], [], 'ok', markersize=8)

    def init():
        line.set_data([], [])
        point.set_data([], [])
        return line, point
    
    def update(frame):
        theta1 = q[0, frame]
        theta2 = q[1, frame]
        theta3 = q[2, frame]

        H01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, l1*np.cos(theta1)],
                        [np.sin(theta1), np.cos(theta1), 0, l1*np.sin(theta1)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        H12 = np.array([[np.cos(theta2), -np.sin(theta2), 0, l2*np.cos(theta2)],
                        [np.sin(theta2), np.cos(theta2), 0, l2*np.sin(theta2)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        H23 = np.array([[np.cos(theta3), -np.sin(theta3), 0, l3*np.cos(theta3)],
                        [np.sin(theta3), np.cos(theta3), 0, l3*np.sin(theta3)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        H02 = np.dot(H01, H12)
        H03 = np.dot(H02, H23)

        P1 = H01[:2, 3]
        P2 = H02[:2, 3]
        P3 = H03[:2, 3]
        
        line.set_data([0, P1[0], P2[0], P3[0]], [0, P1[1], P2[1], P3[1]])
        point.set_data(P3[0], P3[1])

        return line, point

    ani = FuncAnimation(fig, update, frames=len(q[0]), init_func=init, blit=True, interval = 0.1)
    plt.xlim(-3.5, 3.5)
    plt.ylim(-3.5, 3.5)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.xlabel('X axis (m)')
    plt.ylabel('Y axis (m)')
    plt.show()

# Generate a circular trajectory
dt = 1/1000
t = np.arange(0, 12+dt, dt)
radius = 1.5
theta_circle = np.linspace(0, 2 * np.pi, len(t))
x_circle = radius * np.cos(theta_circle)
y_circle = radius * np.sin(theta_circle)

# Scale the circle trajectory and set the manipulator's starting position
x = x_circle
y = y_circle
dx = np.diff(x) / dt
dy = np.diff(y) / dt
v = np.array([dx, dy])

q = np.zeros((3, len(t)))

# Since the trajectory is highly dependent on the initial conditions, implement a minimisation algorithm
# that optimises the joint angles so that the manipulator starts at point (1.5, 0)

# Function to minimize
def objective_function(q, *args):
    target_point = args
    end_effector = fwd_kin(q)
    error = np.linalg.norm(target_point - end_effector)
    return error

# Initial joint angles
initial_angles = np.array([0.5, 0.5, 0.5])

# Target point
target = np.array([1.5, 0])

# BFGS minimization
result = minimize(objective_function, initial_angles, args=target, method='BFGS')

# Extract optimized joint angles
optimized_angles = result.x

print(optimized_angles)

q[:, 0] = np.array(optimized_angles)

for k in range(len(t)-1):
    th1, th2, th3 = q[:, k]
    J = jaco_3(th1, th2, th3)
    q[:, k+1] = q[:, k] + 1 * np.linalg.pinv(J) @ v[:, k] * dt

# Animation
animate_3r(q, x, y)

