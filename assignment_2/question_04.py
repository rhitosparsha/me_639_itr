import numpy as np

"""
Program to calculate the end effector position vector for an RRP Stanford robot.

Parameters:
q1 =  Angle of Joint 1
q2 =  Angle of Joint 2
l1 = Length of Link 1
l2 = Length of Link 2
l3 = Length of link 3
d3 = Linear displacement
"""

def stanford_fwd_kin(q1, q2, l1, l2, l3, d3):
    P3 = np.array([l3, 0, 0, 1])
    
    H_01 = np.array([
                        [np.cos(q1), np.sin(q1), 0, 0],
                        [-np.sin(q1), np.sin(q1), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ]) 
    
    H_12 = np.array([
                        [np.cos(q2), -np.sin(q2), 0, 0],
                        [0, 0, -1, 0],
                        [np.sin(q2), np.cos(q2), 0, l1],
                        [0, 0, 0, 1]
                    ])

    H_23 = np.array([
                        [1, 0, 0, l2 + d3],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])   
    
    P0 = np.dot(np.dot(np.dot(H_01, H_12), H_23), P3) 

    return P0

l1 = float(input("Enter length of Link 1 (m): "))
l2 = float(input("Enter length of Link 2 (m): "))
l3 = float(input("Enter length of Link 3 (m): "))
q1 = np.radians(float(input("Enter angle of Joint 1 (deg): ")))
q2 = np.radians(float(input("Enter angle of Joint 2 (deg): ")))
d3 = float(input("Enter extension length (m): "))

pos = stanford_fwd_kin(q1, q2, l1, l2, l3, d3)

print("\nEnd effector position :")
print(str(pos[0]) + " \033[1mi\033[0m + " + str(pos[1]) + " \033[1mj\033[0m + " + str(pos[2]) + " \033[1mk\033[0m ")

input()
