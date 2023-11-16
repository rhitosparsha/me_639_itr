import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Robot parameters
link_lengths = [0.25, 0.25, 0.25]
base_offset = 0.25  # Distance from the base to the manifold
end_effector_offset = 0.1  # Distance from the end effector to the top surface of the manifold

# Coordinates of points A, B, C, D
points = {
    'A': np.array([0.45, 0.075, 0.1]),
    'B': np.array([0.45, -0.075, 0.1]),
    'C': np.array([0.25, -0.075, 0.1]),
    'D': np.array([0.25, 0.075, 0.1]),
}

# Function to check if a point is within the robot workspace
def is_within_workspace(position):
    # Define joint angle ranges
    q1_range = np.linspace(0, 2 * np.pi, 100) # Joint 1 can rotate 360 degrees
    q2_range = np.linspace(0, np.pi, 50) # Joint 2 can rotate 180 degrees
    q3_range = np.linspace(0, 2 * np.pi, 100) # Joint 3 can rotate 360 degrees

    for q1 in q1_range:
        for q2 in q2_range:
            for q3 in q3_range:
                joint_angles = np.array([q1, q2, q3])
                computed_position = forward_kinematics(joint_angles)
                if np.linalg.norm(computed_position - position) < 0.005:
                    return True
    return False

# Forward kinematics function
def forward_kinematics(joint_angles):
    q1, q2, q3 = joint_angles
    x = base_offset + link_lengths[0] * np.cos(q1) + link_lengths[1] * np.cos(q1 + q2) + link_lengths[2] * np.cos(q1 + q2 + q3)
    y = link_lengths[0] * np.sin(q1) + link_lengths[1] * np.sin(q1 + q2) + link_lengths[2] * np.sin(q1 + q2 + q3)
    z = end_effector_offset
    return np.array([x, y, z])

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

# Define the optimization objective
def objective_function(joint_angles, target_position):
    current_position = forward_kinematics(joint_angles)
    return np.linalg.norm(current_position - target_position)

# Loop through target points
for point_name, target_position in points.items():
    # Check if the target point is within the workspace
    if not is_within_workspace(target_position):
        print(f"Point {point_name} is outside the robot's workspace.")
        continue  # Skip optimization for points outside the workspace

    # Compute initial joint angles using inverse kinematics
    initial_joint_angles = inverse_kinematics(target_position)

    # Minimize the objective function
    result = minimize(objective_function, initial_joint_angles, args=(target_position,), method='BFGS')

    # Extract the optimal joint angles
    optimal_joint_angles = result.x

    # Perform forward kinematics using the optimal joint angles
    optimal_position = forward_kinematics(optimal_joint_angles)

    # Output results
    print(f"Point {point_name}:")
    print(f"Target position: {target_position}")
    print(f"Is in Workspace? : {is_within_workspace(target_position)}")
    print(f"Initial joint angles: {initial_joint_angles}")
    print(f"Optimal joint angles: {optimal_joint_angles}")
    print(f"Optimal position: {optimal_position}\n")
