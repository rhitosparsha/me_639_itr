import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
base_offset = 0.25
end_effector_offset = 0.1
link_lengths = [0.25, 0.25, 0.25]

# Initial joint angles (you can adjust these based on your robot configuration)
q1 = 0.0
q2 = 0.0
q3 = 0.0

# Define the coordinates of points A, B, C, and D
A = np.array([0.40, 0.06, 0.1])
B = np.array([0.40, 0.01, 0.1])
C = np.array([0.35, 0.01, 0.1])
D = np.array([0.35, 0.06, 0.1])

# Time duration for each segment
time_duration = 2.0  # seconds

# Number of points in each segment
num_points = 100

# Inverse kinematics function
def inverse_kinematics(position):
    x, y, z = position
    l1, l2, l3 = link_lengths
    q1 = np.arctan2(y, x - base_offset)
    r = np.sqrt(x**2 + y**2)
    D = (r**2 + (z - l1)**2 - l2**2 - l3**2) / (2 * l2 * l3)
    q2 = np.arctan2(np.sqrt(1 - D**2), D)
    gamma = np.arctan2(z - l1, r - base_offset)
    beta = np.arctan2(l3 * np.sin(q2), l2 + l3 * np.cos(q2))
    q3 = np.pi/2 - (gamma - beta)
    return np.array([q1, q2, q3])

# Generate joint angles trajectory
q1_traj = np.zeros(num_points * 4)
q2_traj = np.zeros(num_points * 4)
q3_traj = np.zeros(num_points * 4)

# Generate joint velocities trajectory
q1_dot_traj = np.zeros(num_points * 4)
q2_dot_traj = np.zeros(num_points * 4)
q3_dot_traj = np.zeros(num_points * 4)

# Generate joint accelerations trajectory
q1_ddot_traj = np.zeros(num_points * 4)
q2_ddot_traj = np.zeros(num_points * 4)
q3_ddot_traj = np.zeros(num_points * 4)

# Generate Cartesian trajectory
cartesian_traj = np.zeros((num_points * 4, 3))

# Time array
time_points = np.linspace(0, time_duration, num_points * 4)

# Generate trajectory for each segment
for i in range(4):
    start_index = i * num_points
    end_index = (i + 1) * num_points

    # Generate Cartesian trajectory for the segment
    cartesian_traj[start_index:end_index] = np.linspace([A, B, C, D][i], [A, B, C, D][(i + 1) % 4], num_points)

    # Convert Cartesian positions to joint angles using inverse kinematics
    for j in range(num_points):
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

# Display joint angles and Cartesian positions
for i in range(num_points * 4):
    print(f"Time: {i * (time_duration / (num_points * 4)):.3f}s")
    print(f"q1: {q1_traj[i]:.4f} rad - q2: {q2_traj[i]:.4f} rad - q3: {q3_traj[i]:.4f} rad")
    print(f"q1_dot: {q1_dot_traj[i]:.4f} rad/s - q2_dot: {q2_dot_traj[i]:.4f} rad/s - q3_dot: {q3_dot_traj[i]:.4f} rad/s")
    print(f"q1_ddot: {q1_ddot_traj[i]:.4f} rad/s^2 - q2_ddot: {q2_ddot_traj[i]:.4f} rad/s^2 - q3_ddot: {q3_ddot_traj[i]:.4f} rad/s^2")
    print(f"Cartesian Position: {cartesian_traj[i]}")
    print()

# Visualize the trajectory in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(cartesian_traj[:, 0], cartesian_traj[:, 1], cartesian_traj[:, 2], marker='o')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
plt.show()
