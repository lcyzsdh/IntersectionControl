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

def cal_adjustments(passing_order,velocity):
    m=gp.Model("integrator_qp")
    s={}
    u={}
    for veh in passing_order:
        s[veh]=m.addVars(3,N,name="s_"+veh)# State variables [positionX, positionY, velocity]
        u[veh]=m.addVars(N,lb=a_min,ub=a_max,name="u_"+veh)# Control variables [acceleration]

    # Objective: minimize the sum of squared deviations from the reference speed
    m.setObjective(gp.quicksum((s[veh][2,k]-velocity[veh])**2 for veh in passing_order for k in range(N)),GRB.MINIMIZE)

    #System dnamics
    for veh in passing_order:
        for k in range(N-1):
            if passing_order[veh]["route"] ==0:#horizontal,right
                m.addConstr(s[veh][0,k+1]==s[veh][0,k]+dt*s[veh][2,k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k])#positionY
                m.addConstr(s[veh][2,k+1]==s[veh][2,k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
            elif passing_order[veh]["route"] ==2:#horizontal,left
                m.addConstr(s[veh][0,k+1]==s[veh][0,k]-dt*s[veh][2,k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k])#positionY
                m.addConstr(s[veh][2,k+1]==s[veh][2,k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
            elif passing_order[veh]["route"] ==4:#vertical,down
                m.addConstr(s[veh][0,k+1]==s[veh][0,k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k]-dt*s[veh][2,k])#positionY
                m.addConstr(s[veh][2,k+1]==s[veh][2,k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
            else:#vertical,up
                m.addConstr(s[veh][0,k+1]==s[veh][0,k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k]+dt*s[veh][2,k])#positionY
                m.addConstr(s[veh][2,k+1]==s[veh][2,k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
    print("init over!!!!!!!")
    for veh in passing_order:
        for veh_1 in passing_order:
            print([passing_order[veh]["route"],passing_order[veh]["route"]])
            if [passing_order[veh]["route"],passing_order[veh_1]["route"]] in [[0, 4], [0, 6],[2, 4], [2, 6]]:
                for k in range(N):
                    print("get!!!!!!")
                    m.addConstr((s[veh][0,k]-s[veh_1][0,k])**2+(s[veh][1,k]-s[veh_1][1,k])**2>=(1)**2)#distance needs tuning
                

    #initial condition
    for veh in passing_order:
        m.addConstr(s[veh][0,0]==passing_order[veh]["Position"][0])#positionX
        m.addConstr(s[veh][1,0]==passing_order[veh]["Position"][1])#positionY
        m.addConstr(s[veh][2,0]==velocity[veh])#velocity
            
    m.optimize()
    m.computeIIS()
    m.write('model.ilp')

    for k in range(N):
        for veh in passing_order:
            print(f"Time step {k}: veh_id = {veh} PositionX = {s[veh][0, k].x}, PositionY = {s[veh][1, k].X}, Velocity = {u[veh][k].X}")

    print('Obj:', m.objVal)

#testing
if __name__ == "__main__":
    passing_order = {
        "veh_0": {
            "route": 0,
            "distance": 10,
            "Position": [-15, -1.5]
        },
        "veh_1": {
            "route": 2,
            "distance": 10,
            "Position": [15, 1.5]
        },
        "veh_2": {
            "route": 4,
            "distance": 10,
            "Position": [-1.5, 15]
        },
        "veh_3": {
            "route": 6,
            "distance": 10,
            "Position": [1.5, 15]
        }
    }
    velocity = {
        "veh_0": 10,
        "veh_1": 10,
        "veh_2": 10,
        "veh_3": 10
    }
    cal_adjustments(passing_order,velocity)