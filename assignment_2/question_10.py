import numpy as np

"""
Program to calculate the manipulator Jacobian for an RRR Planar robot.

Parameters:
q1 = Angle of Joint 1
q2 = Angle of Joint 2
q3 = Angle of Joint 3
l1 = Length of Link 1
l2 = Length of Link 2
l3 = Length of Link 3
"""

def rrr_planar_jacobian(q1, q2, q3, l1, l2, l3):
    jacobian = np.array([
                           [-l1 * np.sin(q1) - l2 * np.sin(q1 + q2) - l3 * np.sin(q1 + q2 + q3), -l2 * np.sin(q1 + q2) - l3 * np.sin(q1 + q2 + q3), -l3 * np.sin(q1 + q2 + q3)],
                           [l1 * np.cos(q1) + l2 * np.cos(q1 + q2) + l3 * np.cos(q1 + q2 + q3), l2 * np.cos(q1 + q2) + l3 * np.cos(q1 + q2 + q3), l3 * np.cos(q1 + q2 + q3)],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [1, 1, 1]
                        ])
    return jacobian

l1 = float(input("Enter length of Link 1 (m): "))
l2 = float(input("Enter length of Link 2 (m): "))
l3 = float(input("Enter length of Link 3 (m): "))
q1 = np.radians(float(input("Enter angle of Joint 1 (deg): ")))
q2 = np.radians(float(input("Enter angle of Joint 2 (deg): ")))
q3 = np.radians(float(input("Enter angle of Joint 3 (deg): ")))


jacobian = rrr_planar_jacobian(q1, q2, q3, l1, l2, l3)

print("\nJacobian matrix for RRR Planar robot:")
print(jacobian)

input()