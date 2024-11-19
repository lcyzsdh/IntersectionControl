import gurobipy as gp
from gurobipy import GRB
import yaml
import test_set
config_file = open("./src/config.yaml")
cfg = yaml.safe_load(config_file)
# Define the parameters
#TODO:Correct the parameters
N = 100  # Number of time steps
T = 10  # Total time horizon
dt = T / N  # Time step
#v_ref = 10  # Reference speed
a_min = cfg["a_min"]  # Minimum acceleration
a_max = cfg["a_max"]  # Maximum acceleration


import matplotlib.pyplot as plt
import matplotlib.animation as animation
# Define the function to animate the trajectories
def animate_trajectories(passing_order, s, N):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-130, 130)
    ax.set_ylim(-130, 130)
    ax.set_xlabel("Position X")
    ax.set_ylabel("Position Y")
    ax.set_title("Vehicle Trajectories")
    ax.grid(True)

    lines = {veh: ax.plot([], [], label=f"Vehicle {veh}")[0] for veh in passing_order}
    points = {veh: ax.plot([], [], 'o')[0] for veh in passing_order}

    def init():
        for line in lines.values():
            line.set_data([], [])
        for point in points.values():
            point.set_data([], [])
        return list(lines.values()) + list(points.values())

    def update(frame):
        for veh in passing_order:
            x_coords = [s[veh][0, k].x for k in range(frame + 1)]
            y_coords = [s[veh][1, k].x for k in range(frame + 1)]
            lines[veh].set_data(x_coords, y_coords)
            points[veh].set_data([x_coords[-1]], [y_coords[-1]])
        return list(lines.values()) + list(points.values())

    ani = animation.FuncAnimation(fig, update, frames=N, init_func=init, blit=True, repeat=False)
    plt.legend()
    plt.show()

def cal_adjustments(passing_order,velocity):
    m=gp.Model("integrator_qp")
    s={}
    u={}
    v={}
    for veh in passing_order:
        #TODO: correct the bound
        s[veh]=m.addVars(2,N,lb=-300,ub=300,name="s_"+veh)# State variables [positionX, positionY]
        v[veh]=m.addVars(N,lb=0,ub=15,name="v_"+veh)# Control variables [velocity]
        u[veh]=m.addVars(N,lb=-5,ub=5,name="u_"+veh)# Control variables [acceleration]

    # Objective: minimize the sum of squared deviations from the reference speed
    m.setObjective(gp.quicksum((v[veh][k]-velocity[veh])**2 for veh in passing_order for k in range(N)),GRB.MINIMIZE)

    #System dnamics
    for veh in passing_order:
        for k in range(N-1):
            if passing_order[veh]["route"] ==0:#horizontal,right
                m.addConstr(s[veh][0,k+1]==s[veh][0,k]+dt*v[veh][k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k])#positionY
                m.addConstr(v[veh][k+1]==v[veh][k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
            elif passing_order[veh]["route"] ==2:#horizontal,left
                m.addConstr(s[veh][0,k+1]==s[veh][0,k]-dt*v[veh][k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k])#positionY
                m.addConstr(v[veh][k+1]==v[veh][k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
            elif passing_order[veh]["route"] ==4:#vertical,down
                m.addConstr(s[veh][0,k+1]==s[veh][0,k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k]-dt*v[veh][k])#positionY
                m.addConstr(v[veh][k+1]==v[veh][k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
            else:#vertical,up
                m.addConstr(s[veh][0,k+1]==s[veh][0,k])#positionX
                m.addConstr(s[veh][1,k+1]==s[veh][1,k]+dt*v[veh][k])#positionY
                m.addConstr(v[veh][k+1]==v[veh][k]+dt*u[veh][k])#velocity
                #print("get!!!!!!"+veh)
    for veh in passing_order:
        for veh_1 in passing_order:
            #print([passing_order[veh]["route"],passing_order[veh]["route"]])
            if [passing_order[veh]["route"],passing_order[veh_1]["route"]] in [[0, 4], [0, 6],[2, 4], [2, 6]]:
                for k in range(N):
                    #print("get!!!!!!")
                    #TODO: correct the distance fomula
                    m.addConstr(((s[veh][0,k]-s[veh_1][0,k])**2+(s[veh][1,k]-s[veh_1][1,k])**2)>=(5)**2)#distance needs tuning
            if veh=="veh_0" and veh_1=="veh_4":
                for k in range(N):
                    m.addConstr(((s[veh][0,k]-s[veh_1][0,k])**2)>=(2)**2)

    #initial condition
    for veh in passing_order:
        m.addConstr(s[veh][0,0]==passing_order[veh]["Position"][0])#positionX
        m.addConstr(s[veh][1,0]==passing_order[veh]["Position"][1])#positionY
        m.addConstr(v[veh][0]==velocity[veh])#velocity
            
    m.optimize()
    #m.computeIIS()
    #m.write('model.ilp')

    results={}
    for k in range(N):
        results[k]={}
        for veh in passing_order:
            print(f"Time step {k}: veh_id = {veh} PositionX = {s[veh][0, k].x}, PositionY = {s[veh][1, k].X}, Velocity = {v[veh][k].X}")
            results[k][veh]={"PositionX":s[veh][0, k].x,"PositionY":s[veh][1, k].X,"Velocity":v[veh][k].X}

    print('Obj:', m.objVal)
    animate_trajectories(passing_order, s, N)

    return results

#testing
if __name__ == "__main__":
    passing_order = {
        "veh_0": {
            "route": 0,
            "distance": 10,
            "Position": [-91, -1.5]
        },
        "veh_1": {
            "route": 2,
            "distance": 10,
            "Position": [95, 1.5]
        },
        "veh_2": {
            "route": 4,
            "distance": 10,
            "Position": [-1.5, 92]
        },
        "veh_3": {
            "route": 6,
            "distance": 10,
            "Position": [1.5, -87]
        }
    }
    velocity = {
        "veh_0": 14.5,
        "veh_1": 14,
        "veh_2": 14.3,
        "veh_3": 14.1
    }#时间切分不够细
    passing_order_new,velocity_new=test_set.parse_vehicle_data("./src/test2_fcd.xml", timestep_index=45)
    cal_adjustments(passing_order_new,velocity_new)