import os
import sys
import yaml
import traci
from traci.exceptions import TraCIException
from random import randint
from vehicle import Vehicle
from passing_order import PassingOrder
from reporting import show_report

config_file = open("./src/config.yaml")
cfg=yaml.safe_load(config_file)
times=0
sumoCmd=[os.path.join(os.environ['SUMO_HOME'],'bin','sumo-gui'),
         '-c',cfg["sumo_cfg"],
         '--tripinfo-output',cfg["trip_info_out"],
            '--collision-output',cfg["collisions_out"],
         '--statistic-output',cfg["statistics_out"],
        '--step-length',cfg["simulation_step_length"]]

def get_zone_radii():
    z1= cfg["zone_control_size"]
    z2= cfg["zone_intersection_size"]

    z1_rad=z2+z1
    return z1_rad,z2

def excute_adjustments(passing_data):#passing_data is a split piece of time
    try:
        for veh in passing_data:
            if veh in traci.vehicle.getIDList():
                print(f"vehicle {veh} is slowing down")
                traci.vehicle.slowDown(veh,passing_data[veh]["speed"],0.1)#ID!!!!
                times+=1
    except TraCIException:
        pass

def main():
    depart_time={}
    traci.start(sumoCmd)
    vehicles=[]
    my_veh={}
    i=0
    passing_data_total = None
    highest_total = 0

    step=0
    while step<cfg["max_simulation_steps"]:
        traci.simulationStep()#set step length

        veh_id_list = traci.vehicle.getIDList()
        for veh_id in veh_id_list:
            if veh_id in my_veh:
                veh_id_formal = my_veh[veh_id]       #veh_id_formal是自然数序列对应的车辆id，veh_id是SUMO中自动赋予的id
            else:
                i += 1
                my_veh[veh_id] = i
                veh_id_formal = my_veh[veh_id]
                route = traci.vehicle.getRouteID(veh_id)
                veh = Vehicle(
                    i,veh_id,route,cfg["veh_state_default"]
                )
                vehicles.append(veh)
                depart_time[veh_id_formal]=traci.simulation.getTime()
            speed = traci.vehicle.getSpeed(veh_id)

        step+=1
        z1_r,z2_r=get_zone_radii()

        for veh in vehicles:
            try:
                #if veh.is_outbound():
                #    veh.set_vehicle_state(cfg["veh_state_default"])
                #else:
                    if veh.get_dist_to_intersection() < z2_r:
                        veh.set_vehicle_state(cfg["veh_state_intersection"])
                    elif veh.get_dist_to_intersection() < z1_r:
                        veh.set_vehicle_state(cfg["veh_state_control"])
                    else:
                        veh.set_vehicle_state(cfg["veh_state_out"])
            except TraCIException:
                print(f"vehicle {veh.veh_id} has left the sim at step {step}")
                vehicles.remove(veh)
        
        current_vehicles=traci.vehicle.getIDList()

        if step>=200:#firstly enter the zone
            if cfg["passing_order_mode"]==cfg["passing_order_gurobi"]:
                if (step-200)%50==0:#100 steps for a total decision
                    veh_data=vehicles[0].gather_veh_data(vehicles)
                    passing_data_total=vehicles[0].get_passing_data()
                if passing_data_total==-1:
                    print(f"step {step}: No solution")
                    continue
                excute_adjustments(passing_data_total[(step-20)%50])#split the total decision into pieces
                print(f"step {step}: passing order is executed")
                #for veh in vehicles:
                    #print(f"vehicle {veh.veh_id} position {traci.vehicle.getPosition(veh.veh_id_ac)} speed {traci.vehicle.getSpeed(veh.veh_id_ac)}")
        
    traci.close()
    #show_report
    print(f"Simulation finished,total speed adjustments: {times}")

if __name__ == "__main__":
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("You must declare environment variable SUMO_HOME")

    main()
