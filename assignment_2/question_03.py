import numpy as np

"""
Program to calculate the end effector position vector for an RRP SCARA robot.

Parameters:
q1 = Angle of Joint 1
q2 = Angle of Joint 2
d3 = Extension length of the robot arm
l1 = Length of Link 1
l2 = Length of Link 2
"""

def scara_fwd_kin(q1, q2, l1, l2, d3):
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    z = d3
    return x, y, z

l1 = float(input("Enter length of Link 1 (m): "))
l2 = float(input("Enter length of Link 2 (m): "))
q1 = np.radians(float(input("Enter angle of Joint 1 (deg): ")))
q2 = np.radians(float(input("Enter angle of Joint 2 (deg): ")))
d3 = float(input("Enter extension length (m): "))

x, y, z = scara_fwd_kin(q1, q2, l1, l2, d3)

print("\nEnd effector position :")
print(str(x) + " \033[1mi\033[0m + " + str(y) + " \033[1mj\033[0m + " + str(z) + " \033[1mk\033[0m ")

input()
