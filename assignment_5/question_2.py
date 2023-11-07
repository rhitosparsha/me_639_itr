import numpy as np

def calculate_joint_velocities(jacobian, end_effector_linear_velocities, end_effector_angular_velocities):
    try:
        end_effector_velocities = np.concatenate((end_effector_linear_velocities, end_effector_angular_velocities))
        jacobian_inv = np.linalg.pinv(jacobian)
        joint_velocities = np.dot(jacobian_inv, end_effector_velocities)
        return joint_velocities[:3], joint_velocities[3:]
    except np.linalg.LinAlgError:
        return None

num_links = int(input("Enter the number of links: "))

jacobian = np.empty((6, num_links))

print("\nEnter the elements of the Jacobian matrix (row by row):")
for i in range(6):
    for j in range(num_links):
        jacobian[i][j] = float(input(f"Jacobian[{i}][{j}]: "))

end_effector_linear_velocities = np.array([float(v) for v in input("\nEnter end-effector linear velocities (comma-separated) (m/s): ").split(',')])
end_effector_angular_velocities = np.array([float(v) for v in input("Enter end-effector angular velocities (comma-separated) (rad/s): ").split(',')])

# Calculate joint velocities
joint_linear_velocities, joint_angular_velocities = calculate_joint_velocities(jacobian, end_effector_linear_velocities, end_effector_angular_velocities)

if joint_linear_velocities is not None:
    print("\nJoint Linear Velocities:", joint_linear_velocities)
    print("Joint Angular Velocities:", joint_angular_velocities)
else:
    print("\nJacobian is singular. Cannot calculate joint velocities.")
