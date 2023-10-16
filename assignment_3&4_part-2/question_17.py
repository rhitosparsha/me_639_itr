import numpy as np

print("Inverse kinematics of Spherical Wrist manipulator :\n")

# Take input for link lengths
q1 = float(input("Enter angle of Joint 1 (deg): "))
q2 = float(input("Enter angle of Joint 2 (deg): "))
q3 = float(input("Enter angle of Joint 3 (deg): "))

c1 = np.cos(q1)
c23 = np.cos(q2 + q3)
s1 = np.sin(q1)
s23 = np.sin(q2 + q3)

# Define the rotation matrix of the current end-effector orientation
R_current = np.array([[-c1 * c23, -c1 * s23, s1],
                     [s1 * c23, -s1 * s23, -c1],
                     [s23, c23, 0]])

# Take input of desired end-effector orientation
print("\nEnter the elements of the desired end-effector orientation matrix (row by row):")
R_desired = np.empty((3,3))
for i in range(3):
    for j in range(3):
        R_desired[i][j] = float(input(f"R_desired[{i}][{j}]: "))

# Calculate the transformation matrix that transforms from the current to the desired orientation
R_d2c = np.dot(R_current.T, R_desired)

# Extract the z-y-z Euler angles from the rotation matrix
phi = np.arctan2(R_d2c[1, 2], R_d2c[0, 2])
theta = np.arccos(R_d2c[2, 2])
psi = np.arctan2(R_d2c[2, 1], -R_d2c[2, 0])

# Print the results
print("\nDesired Rotation Matrix:")
print(R_desired)
print("\nCurrent Rotation Matrix:")
print(R_current)
print("\nCalculated Z-Y-Z Euler Angles:")
print("phi = ", np.degrees(phi), " deg")
print("theta = ", np.degrees(theta), " deg")
print("psi = ", np.degrees(psi), " deg")