import gurobipy as gp
from gurobipy import GRB
import yaml

config_file = open("./src/config.yaml")
cfg = yaml.safe_load(config_file)
# Define the parameters
N = cfg["time_steps"]  # Number of time steps
T = cfg["total_time"]  # Total time horizon
dt = T / N  # Time step
#v_ref = 10  # Reference speed
a_min = cfg["a_min"]  # Minimum acceleration
a_max = cfg["a_max"]  # Maximum acceleration

# Define the system matrices
A = [[1, dt], [0, 1]]
B = [0, dt]

def cal_adjustments(passing_order,velocity):
    m=gp.Model("integrator_qp")
    s={}
    u={}
    for veh in passing_order:
        s[veh[0]]=m.addVars(3,N,name="s_"+veh[0])# State variables [positionX, positionY, velocity]
        u[veh[0]]=m.addVars(N,lb=a_min,ub=a_max,name="u_"+veh[0])# Control variables [acceleration]

    # Objective: minimize the sum of squared deviations from the reference speed
    m.setObjective(gp.quicksum((s[veh[0]][2,k]-velocity[veh[0]])**2 for veh in passing_order for k in range(N)),GRB.MINIMIZE)

    for veh in passing_order:
        for k in range(N-1):
            m.addConstr(s[veh[0]][0,k+1]==s[veh[0]][0,k]+s[veh[0]][2,k]*dt+0.5*u[veh[0]][k]*dt**2)
            m.addConstr(s[veh[0]][2,k+1]==s[veh[0]][2,k]+u[veh[0]][k]*dt)
    
    #如何确定车辆的二维坐标？