import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


# =================== DEFINITION OF PARAMETERS ===================

# Robot parameters
m1, m2, m3 = 0.8, 0.8, 0.8  # Masses of links
l1, l2, l3 = 0.25, 0.25, 0.25 # Lengths of links
lc2 = 0.125  # Distance from joint 1 to the center of mass of link 2
lc3 = 0.125  # Distance from joint 2 to the center of mass of link 3
base_offset = 0.25  # Offset from the base to the first joint
g = 9.81  # Acceleration due to gravity

# Time parameters
total_time = 8.0  # Total simulation time
num_steps = 800
dt = total_time / num_steps
time_points = np.linspace(0, total_time, int(num_steps))

# Control gains
kp1, kp2, kp3 = 100, 100, 100  # Proportional gains
Kp = np.array([kp1, kp2, kp3])

# Calculate derivative gains for critical damping
kd1 = 2 * np.sqrt(kp1 * (1/2 * m1 * lc2**2 + 1/4 * m2 * l2**2 + 1/4 * m3 * l3**2 + m3 * l2**2 + m2 * l2 * l3))
kd2 = 2 * np.sqrt(kp2 * (1/4 * m2 * l2**2 + m3 * (l2**2 + lc3**2 + l2 * l3)))
kd3 = 2 * np.sqrt(kp3 * (1/3 * m3 * l3**2))
Kd = np.array([kd1, kd2, kd3])


# =================== TRAJECTORY GENERATION ===================

# Define the coordinates of points A, B, C, and D
A = np.array([0.40, 0.06, 0.1])
B = np.array([0.40, 0.01, 0.1])
C = np.array([0.35, 0.01, 0.1])
D = np.array([0.35, 0.06, 0.1])

# Inverse kinematics function
def inverse_kinematics(position):
    x, y, z = position

    q1 = np.arctan2(y, x - base_offset)
    r = np.sqrt(x**2 + y**2)
    D = (r**2 + (z - l1)**2 - l2**2 - l3**2) / (2 * l2 * l3)
    q2 = np.arctan2(np.sqrt(1 - D**2), D)
    gamma = np.arctan2(z - l1, r - base_offset)
    beta = np.arctan2(l3 * np.sin(q2), l2 + l3 * np.cos(q2))
    q3 = np.pi/2 - (gamma - beta)
    return np.array([q1, q2, q3])

# Generate joint angles trajectory
q1_traj = np.zeros(num_steps)
q2_traj = np.zeros(num_steps)
q3_traj = np.zeros(num_steps)

# Generate joint velocities trajectory
q1_dot_traj = np.zeros(num_steps)
q2_dot_traj = np.zeros(num_steps)
q3_dot_traj = np.zeros(num_steps)

# Generate joint accelerations trajectory
q1_ddot_traj = np.zeros(num_steps)
q2_ddot_traj = np.zeros(num_steps)
q3_ddot_traj = np.zeros(num_steps)

# Generate Cartesian trajectory for the entire duration
cartesian_traj = np.zeros((num_steps, 3))

# Generate trajectory for each segment
for i in range(4):
    start_index = i * int(num_steps / 4)
    end_index = (i + 1) * int(num_steps / 4)

    # Generate Cartesian trajectory for the segment
    cartesian_traj[start_index:end_index] = np.linspace([A, B, C, D][i], [A, B, C, D][(i + 1) % 4], int(num_steps / 4))

    # Convert Cartesian positions to joint angles using inverse kinematics
    for j in range(int(num_steps / 4)):
        joint_angles = inverse_kinematics(cartesian_traj[start_index + j])
        q1_traj[start_index + j], q2_traj[start_index + j], q3_traj[start_index + j] = joint_angles

# Calculate joint velocities using numerical differentiation
q1_dot_traj[:-1] = np.diff(q1_traj) / np.diff(time_points)
q2_dot_traj[:-1] = np.diff(q2_traj) / np.diff(time_points)
q3_dot_traj[:-1] = np.diff(q3_traj) / np.diff(time_points)

# Calculate joint accelerations using numerical differentiation
q1_ddot_traj[:-1] = np.diff(q1_dot_traj) / np.diff(time_points)
q2_ddot_traj[:-1] = np.diff(q2_dot_traj) / np.diff(time_points)
q3_ddot_traj[:-1] = np.diff(q3_dot_traj) / np.diff(time_points)


# =================== SOLVING OF DYNAMICS PROBLEM ===================

# Dynamics function
def dynamics(t, state):
    q1, q2, q3, q1_dot, q2_dot, q3_dot = state

    C2 = np.cos(q2)
    S2 = np.sin(q2)
    C3 = np.cos(q3)
    S3 = np.sin(q3)
    C23 = np.cos(q2 + q3)
    S23 = np.sin(q2 + q3)

    M = np.array([
        [(1/2 * m1 * lc2**2 + 1/4 * m2 * l2**2 * C2**2 + 1/4 * m3 * l3**2 * C23**2 + m3 * l2**2 * C2**2 + m2 * l2 * l3 * C2 * C23), 0, 0],
        [0, (1/4 * m2 * l2**2 + m3 * (l2**2 + lc3**2 + l2 * l3 * C3)), 0],
        [0, 0, (1/3 * m3 * l3**2)]
    ])

    C = np.array([
        [0, (-1/4 * m2 * l2**2 * S2 - m3 * l2**2 * S2 - m3 * l2 * l3 * S23 - m3 * lc2**2 * S23), (-m3 * l2 * l3 * C2 * S23 - m3 * lc3**2 * S23)],
        [0, 0, -m2 * l2 * l3 * S3],
        [0, 0, 0]
    ])

    G = np.array([
        lc2 * m2 * g * C2 + l2 * m3 * g * C2 + lc3 * m2 * g * C23,
        0,
        m3 * g * lc3 * C23
    ])

    index = min(int(t / dt), len(q1_traj) - 1)

    # Feedforward control using desired accelerations
    feedforward_term = np.array([q1_ddot_traj[index], q2_ddot_traj[index], q3_ddot_traj[index]])
    
    # Proportional control with tracking error
    error = np.array([q1_traj[index] - q1, q2_traj[index] - q2, q3_traj[index] - q3])
    proportional_term = Kp * error

    # Derivative control
    error_dot = -np.array([q1_dot - q1_dot_traj[index], q2_dot - q2_dot_traj[index], q3_dot - q3_dot_traj[index]])
    derivative_term = Kd * error_dot

    # External disturbances (small random values)
    disturbance_scale = 0.0001
    disturbances = disturbance_scale * np.random.randn(3)

    # Calculate control input
    control_input = feedforward_term + proportional_term + derivative_term + disturbances

    qdd = np.linalg.solve(M, -np.dot(C, [q1_dot, q2_dot, q3_dot]) - G + control_input)
    return [q1_dot, q2_dot, q3_dot, qdd[0], qdd[1], qdd[2]]


# Solve the initial value problem using Runge-Kutta method
sol = solve_ivp(
    fun=dynamics,
    t_span=(0, total_time),
    y0=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Joint angles and velocities
    method='DOP853',
    t_eval=np.linspace(0, total_time, num_steps),
    args=()
)

# Extract the results
time = sol.t
q = sol.y[:3, :]
qd = sol.y[3:, :]


# =================== PLOTTING OF RESULTS ===================


# Plot for Joint 1
plt.figure()
plt.plot(time, q1_traj, label='Desired Angle')
plt.plot(time, q[0, :], label='Actual Angle')
plt.xlabel('t (s)')
plt.ylabel('q1 (rad)')
plt.grid()
plt.legend()
plt.title('Plot of Joint 1 Angles (q1) vs. Time (t) for PUMA Robot')
plt.show()

# Plot for Joint 2
plt.figure()
plt.plot(time, q2_traj, label='Desired Angle')
plt.plot(time, q[1, :], label='Actual Angle')
plt.xlabel('t (s)')
plt.ylabel('q2 (rad)')
plt.grid()
plt.legend()
plt.title('Plot of Joint 2 Angles (q2) vs. Time (t) for PUMA Robot')
plt.show()

# Plot for Joint 3
plt.figure()
plt.plot(time, q3_traj, label='Desired Angle')
plt.plot(time, q[2, :], label='Actual Angle')
plt.xlabel('t (s)')
plt.ylabel('q3 (rad)')
plt.grid()
plt.legend()
plt.title('Plot of Joint 3 Angles (q3) vs. Time (t) for PUMA Robot')
plt.show()