import numpy as np

print("Inverse kinematics of Stanford manipulator :\n")

# Take input for link lengths
l1 = float(input("Enter length of Link 1 (m): "))
l2 = float(input("Enter length of Link 2 (m): "))
l3 = float(input("Enter length of Link 3 (m): "))

# Take input for end-effector coordinates
xc = float(input("Enter x-coordinate of end-effector (m): "))
yc = float(input("Enter y-coordinate of end-effector (m): "))
zc = float(input("Enter z-coordinate of end-effector (m): "))

# Compute inverse kinematics solutions
r = np.sqrt(xc**2 + yc**2)
s = zc - l1
theta1_a = np.arctan2(xc,yc)
theta1_b = theta1_a + np.pi
theta2 = np.arctan2(r,s) + np.pi/2
d3 = np.sqrt(r**2 + s**2)

# Output the computed values
if d3 > l3:
    print("Point lies outside the workspace of the robot!")
else:
    print("\nInverse kinematics solutions :\n")
    print("Angle of link 1 ", np.rad2deg(theta1_a), " deg, ", np.rad2deg(theta1_b), " deg")
    print("Angle of link 2 ", np.rad2deg(theta2), " deg")
    print("Link extension : ", d3, " m")
