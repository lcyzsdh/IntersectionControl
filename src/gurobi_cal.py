import gurobipy as gp
from gurobipy import GRB

# Define the parameters
N = 100  # Number of time steps
T = 10  # Total time horizon
dt = T / N  # Time step
v_ref = 10  # Reference speed
a_min = -5  # Minimum acceleration
a_max = 5  # Maximum acceleration

# Define the system matrices
A = [[1, dt], [0, 1]]
B = [0, dt]

# Create a new model
m = gp.Model("double_integrator_qp")

# Decision variables
s = m.addVars(2, N, name="s")  # State variables [position, velocity]
u = m.addVars(N, lb=a_min, ub=a_max, name="u")  # Control variables [acceleration]

# Objective: minimize the sum of squared deviations from the reference speed
m.setObjective(gp.quicksum((s[1, k] - v_ref) ** 2 for k in range(N)), GRB.MINIMIZE)

# Constraints: system dynamics
for k in range(N - 1):
    m.addConstr(s[0, k + 1] == A[0][0] * s[0, k] + A[0][1] * s[1, k] + B[0] * u[k])
    m.addConstr(s[1, k + 1] == A[1][0] * s[0, k] + A[1][1] * s[1, k] + B[1] * u[k])

# Initial condition
m.addConstr(s[0, 0] == 0)
m.addConstr(s[1, 0] == 0)

# Optimize the model
m.optimize()

# Print the results
for k in range(N):
    print(f"Time step {k}: Position = {s[0, k].x}, Velocity = {s[1, k].x}, Acceleration = {u[k].x}")

print('Obj:', m.objVal)
