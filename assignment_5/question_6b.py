import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

print("Simulation of 3-DOF Articulated Robot :\n")
print("Select Control Scheme :\n1. Simple PD Control\n2. PD Control with Gravity Compensation\n3. PD Control with Feed-forward term\n4. Computed Torrque PD Control")
choice = input("\nPress a button from 1 - 4 : ")
print("You chose: ")
if choice == '1':
    print("Simple PD Control")
elif choice == '2':
    print("PD Control with Gravity Compensation")
elif choice == '3':
    print("PD Control with Feed-forward term")
elif choice == '4':
    print("PD Control with Computed Torque Control")
print("\nPlotting results...")

# Define robot parameters
m1, m2, m3 = 1.0, 1.0, 1.0  # Masses of links
l2, l3 = 1.0, 1.0  # Link lengths
lc2, lc3 = 0.5, 0.5  # Center of mass distances
g = 9.81  # Gravitational acceleration

# Define initial conditions
initial_q = np.array([0.5, -1, 1])  # Initial joint angles
initial_q_dot = np.array([0.0, 0.0, 0.0])  # Initial joint velocities
initial_q_dot_motor = np.array([0.0, 0.0, 0.0])  # Initial motor velocities
initial_I_motor = np.array([0.0, 0.0, 0.0])  # Initial motor currents

# Desired joint angles, velocities and accelerations
q_desired = np.array([2, 3, 1])
q_dot_desired = np.array([0.5, 0.1, 0.3])
q_ddot_desired = np.array([0.1, 0.05, 0.15])

# Motor dynamics parameters
Jm = np.array([1, 1.5, 0.5])  # Motor inertias
Bm = np.array([2, 3, 2.5])  # Motor damping coefficients
Rm = np.array([1, 1.5, 0.5])  # Motor resistances
Lm = np.array([0.01, 0.015, 0.005])  # Motor inductances
Kb = np.array([0.2, 0.25, 0.1])  # Motor back-emf constants
Kt = np.array([0.1, 0.3, 0.4])  # Motor torque constants

# Motor control parameters
Kp = np.array([10.0, 10.0, 10.0])  # Proportional gains
Kd = np.array([2, 2, 2])   # Derivative gain

# Initialize state variables
q = initial_q.copy()
q_dot = initial_q_dot.copy()
q_dot_motor = initial_q_dot_motor.copy()
I_motor = initial_I_motor.copy()

# Simulation parameters
t_max = 20.0
num_steps = 10000
dt = t_max / num_steps
time_values = np.linspace(0, t_max, num_steps)

# Define the combined inertial and motor dynamics function
def dynamics(state, t):
    q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_dot_motor, q2_dot_motor, q3_dot_motor, I1, I2, I3 = state
    q = np.array([q1, q2, q3])
    q_dot = np.array([q1_dot, q2_dot, q3_dot])
    q_dot_motor = np.array([q1_dot_motor, q2_dot_motor, q3_dot_motor])
    q_ddot_motor = np.zeros(3)
    I_motor = np.array([I1, I2, I3])
    I_dot_motor = np.zeros(3)
    tau_motor_desired = np.zeros(3)
    tau_motor = np.zeros(3)
    V_back = np.zeros(3)
    V_motor = np.zeros(3)
    
    C2 = np.cos(q2)
    S2 = np.sin(q2)
    C3 = np.cos(q3)
    S3 = np.sin(q3)
    C23 = np.cos(q2 + q3)
    S23 = np.sin(q2 + q3)

    M = np.array([
        [(1/2 * m1 * lc2**2 + 1/4 * m2 * l2**2 * C2**2 + 1/4 * m3 * l3**2 * C23**2 + m3 * l2**2 * C2**2 + m2 * l2 * l3 * C2 * C23), 0, 0],
        [0, (1/4 * m2 * l2**2 + m3 * (l2**2 + lc3**2 + l2 * l3 * C3)), 0],
        [0, 0, (1/3 * m3 * l3**2)]
    ])

    C = np.array([
        [0, (-1/4 * m2 * l2**2 * S2 - m3 * l2**2 * S2 - m3 * l2 * l3 * S23 - m3 * lc2**2 * S23), (-m3 * l2 * l3 * C2 * S23 - m3 * lc3**2 * S23)],
        [0, 0, -m2 * l2 * l3 * S3],
        [0, 0, 0]
    ])

    G = np.array([
        lc2 * m2 * g * C2 + l2 * m3 * g * C2 + lc3 * m2 * g * C23,
        0,
        m3 * g * lc3 * C23
    ])
    
    # Motor Control
    if choice == '1':
        # Simple PD Control
        tau_motor_desired = Kp * (q_desired - q) + Kd * (q_dot_desired - q_dot)

    elif choice == '2':
        # PD Control with Gravity Compensation
        tau_motor_desired = M.dot(q_ddot_desired) + C.dot(q_dot_desired) + G + Kp * (q_desired - q) + Kd * (q_dot_desired - q_dot)

    elif choice == '3':
        # PD Control with Feed-forward term
        feedforward_term = M.dot(q_ddot_desired)
        tau_motor_desired = feedforward_term + Kp * (q_desired - q) + Kd * (q_dot_desired - q_dot)

    elif choice == '4':
        # PD Control with Computed Torque Control
        feedforward_term = M.dot(q_ddot_desired)
        tau_desired = feedforward_term + M.dot(q_ddot_desired) + C.dot(q_dot_desired) + G
        tau_motor_desired = tau_desired + Kp * (q_desired - q) + Kd * (q_dot_desired - q_dot)

    # Motor dynamics
    V_back = Kb * q_dot_motor
    I_motor = tau_motor_desired / (Kt * Rm)
    q_ddot_motor = (tau_motor_desired - Bm * q_dot_motor - Kt * I_motor) / Jm
    q_dot_motor += q_ddot_motor * dt # Euler integration
    V_motor = I_motor * Rm
    I_dot_motor = (V_motor - Rm * I_motor - V_back) / Lm
    tau_motor = V_motor * Kt

    q_ddot = np.linalg.solve(M, (tau_motor - np.dot(C, q_dot)) - G)

    return [q_dot[0], q_dot[1], q_dot[2], q_ddot[0], q_ddot[1], q_ddot[2], q_ddot_motor[0], q_ddot_motor[1], q_ddot_motor[2], I_dot_motor[0], I_dot_motor[1], I_dot_motor[2]]


# Initial conditions for the ODE solver
state_0 = np.concatenate((q, q_dot, q_dot_motor, I_motor))

# Use odeint to integrate the ODE
solution = odeint(dynamics, state_0, time_values)

# Extract the joint angles from the solution
q1_values = solution[:, 0]
q2_values = solution[:, 1]
q3_values = solution[:, 2]

# Plot the joint angles over time
plt.figure()
#plt.suptitle('Plot of Joint Angles (q) vs. Time (t) for 3-DOF Articulated Robot')
if choice == '1':
    plt.suptitle("\nPlot of Joint Angles (q) vs. Time (t) for 3-DOF Articulated Robot with Simple PD Control")
elif choice == '2':
    plt.suptitle("\nPlot of Joint Angles (q) vs. Time (t) for 3-DOF Articulated Robot with PD Control with Gravity Compensation")
elif choice == '3':
    plt.suptitle("\nPlot of Joint Angles (q) vs. Time (t) for 3-DOF Articulated Robot with PD Control with Feed-forward term")
elif choice == '4':
    plt.suptitle("\nPlot of Joint Angles (q) vs. Time (t) for 3-DOF Articulated Robot with PD Control with Computed Torque Control")

plt.subplot(311)
plt.plot(time_values, q1_values, label='q1')
plt.xlabel('t (s)')
plt.ylabel('q1 (rad)')
plt.grid()
plt.legend()

plt.subplot(312)
plt.plot(time_values, q2_values, label='q2')
plt.xlabel('t (s)')
plt.ylabel('q2 (rad)')
plt.grid()
plt.legend()

plt.subplot(313)
plt.plot(time_values, q3_values, label='q3')
plt.xlabel('t (s)')
plt.ylabel('q3 (rad)')
plt.grid()
plt.legend()

plt.show()
