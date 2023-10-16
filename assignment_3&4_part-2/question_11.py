import sympy as sp

# Define symbols q, qd and tau for joint variables, joint velocities, and joint torques respectively
n = int(input("Enter the number of joints: "))
q = sp.Matrix(sp.symbols('q:{}'.format(n)))
qd = sp.Matrix(sp.symbols('qd:{}'.format(n)))
tau = sp.Matrix(sp.symbols('tau:{}'.format(n)))

# Define D and V matrices
D = sp.Matrix([[0] * n for _ in range(n)])
V = sp.Matrix([0] * n)

# Take input values for the D(q) and V(q) matrices
print("\nEnter the values for the D(q) matrix : ")
for i in range(n):
    for j in range(n):
        D[i, j] = sp.simplify(sp.sympify(input(f"Enter the D({i},{j})(q) expression: ")))

print("\nEnter the values for the V(q) matrix : ")
for i in range(n):
    V[i] = sp.simplify(sp.sympify(input(f"Enter the V({i})(q) expression: ")))

# Compute the Lagrangian L = 0.5*qd^T * D(q) * qd - V(q)^T * q
L = 0.5 * qd.dot(D * qd) - V.dot(q)

# Compute the equations of motion for the robot
eqns = [sp.Eq(sp.diff(L, qd[i]) - sp.diff(L, q[i]), tau[i]) for i in range(n)]

# Simplify the equations
eqns = [sp.simplify(eqn) for eqn in eqns]

# Print the matrices
print("\nD(q) matrix:")
print(D)
print("\nV(q) matrix:")
print(V)

# Print the equations
print("\nEquations of motion for the robot:")
for eqn in eqns:
    print(eqn)