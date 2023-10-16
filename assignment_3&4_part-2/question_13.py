import numpy as np

print("Inverse kinematics of SCARA manipulator :\n")

# Take input for link lengths
l1 = float(input("Enter length of Link 1 (m): "))
l2 = float(input("Enter length of Link 2 (m): "))
l3 = float(input("Enter length of Link 3 (m): "))
d1 = float(input("Enter height at which Joint 1 is located (m): "))

# Take input for end-effector coordinates
xc = float(input("Enter x-coordinate of end-effector (m): "))
yc = float(input("Enter y-coordinate of end-effector (m): "))
zc = float(input("Enter z-coordinate of end-effector (m): "))

# Compute inverse kinematics solutions
ox = xc
oy = yc
oz = zc
c2 = (ox**2 + oy**2 - l1**2 - l2**2)/(2 * l1 * l2)
q2 = np.arctan2(c2, np.sqrt(1 - c2))
q1 = np.arctan2(ox, oy) - np.arctan2(l1 + l2 * np.cos(q2), l2 * np.sin(q2))
d3 = d1 - zc

# Output the computed values
if oz > d1:
    print("Point lies outside the workspace of the robot!")
else:
    print("\nInverse kinematics solutions :\n")
    print("Angle of link 1 ", np.rad2deg(q1), " deg, ")
    print("Angle of link 2 ", np.rad2deg(q2), " deg")
    print("Link extension : ", d3, " m")


