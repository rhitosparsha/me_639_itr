import numpy as np

def compute_jacobian_and_velocity(links, dh_parameters, joint_types=None):
    if joint_types is None:
        # Default assumption: All joints are revolute
        joint_types = ["R"] * len(links)

    if len(links) != len(dh_parameters) or len(links) != len(joint_types):
        raise ValueError("Input lengths do not match")

    # Initialize the transformation matrix
    T = np.eye(4)

    # Initialize the Jacobian matrix
    J = np.zeros((6, len(links)))

    # Initialize the end-effector position
    end_effector_position = np.zeros(3)

    # Initialize the end-effector velocity
    end_effector_velocity = np.zeros(6)

    for i in range(len(links)):
        a, alpha, d, theta = dh_parameters[i]
        if joint_types[i] == 'R':  # Revolute joint
            theta = links[i]

        # Calculate the transformation matrix
        A = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                      [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                      [0, np.sin(alpha), np.cos(alpha), d],
                      [0, 0, 0, 1]])

        # Update the transformation matrix
        T = np.dot(T, A)

        # Calculate the rotational part of the Jacobian
        z_i_minus_1 = T[:3, 2]
        end_effector_position = T[:3, 3]
        J[:3, i] = np.cross(z_i_minus_1, end_effector_position)

        # Calculate the translational part of the Jacobian
        J[3:, i] = z_i_minus_1

    return J, end_effector_position, end_effector_velocity

# Default values:
links = [0.1, 0.2, 0.15]
dh_parameters = [
    [0.1, 0, 0.05, 0.1],
    [0.05, 0, 0, 0.2],
    [0.03, 0, 0, 0.15]
]
joint_types = ["R", "R", "R"]

choice = int(input("Do you want to :\n1. Use default values?\n2. Enter your own values?\nEnter option no. : "))

n = int(input("\nEnter the number of links : "))

# Take inputs if user decides to input their own values
if choice == 2:
    print("\nEnter the following values row-wise, separated by spaces :")
    links =  list(map(float, input("Enter link lengths in m  : ").split()))
    e2 =  list(map(float, input("Enter values of DH Parameters (Format : a, alpha, d, theta) : ").split()))
    dh_parameters = np.array(e2).reshape(n, 4)
    joint_types =  list(map(str, input("Enter joint types (R or P) : ").split()))

print("\nLink lengths (m) :")
print(links)
print("\nDH Parameters (Format : a, alpha, d, theta) : ")
print(dh_parameters)
print("\nJoint types : ")
print(joint_types)

jacobian, end_effector_position, end_effector_velocity = compute_jacobian_and_velocity(links, dh_parameters, joint_types)

print("\nManipulator Jacobian:")
print(jacobian)
print("\nEnd-effector Position:")
print(end_effector_position)
print("\nEnd-effector Velocity:")
print(end_effector_velocity)