import numpy as np

"""
Program to calculate the manipulator Jacobian for an RRP SCARA robot.

Parameters:
q1 = Angle of Joint 1
q2 = Angle of Joint 2
d3 = Extension length of the robot arm
l1 = Length of Link 1
l2 = Length of Link 2
"""

def scara_jacobian(q1, q2, l1, l2, d3):
    jacobian = np.array([
                           [-l1 * np.sin(q1) - l2 * np.sin(q1 + q2), -l2 * np.sin(q1 + q2), 0],
                           [l1 * np.cos(q1) + l2 * np.cos(q1 + q2), l2 * np.cos(q1 + q2), 0],
                           [0, 0, -d3],
                           [0, 0, 0],
                           [0, 0, 0],
                           [1, 1, 1]
                        ])
    return jacobian

l1 = float(input("Enter length of Link 1 (m): "))
l2 = float(input("Enter length of Link 2 (m): "))
q1 = np.radians(float(input("Enter angle of Joint 1 (deg): ")))
q2 = np.radians(float(input("Enter angle of Joint 2 (deg): ")))
d3 = float(input("Enter extension length (m): "))

jacobian = scara_jacobian(q1, q2, l1, l2, d3)

print("\nJacobian matrix for SCARA robot:")
print(jacobian)

input()
