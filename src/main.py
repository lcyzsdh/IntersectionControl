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

sumoCmd=[os.path.join(os.environ['SUMO_HOME'],'bin','sumo'),
         '-c',cfg["sumo_cfg"],
         '--tripinfo-output',cfg["trip_info_out"],
         '--collision-output',cfg["collisions_out"],
         '--statistic-output',cfg["statistics_out"]]

def add_vehicles(num_vehicles):
    pass#waiting for implementation

def get_zone_radii():
    z1= cfg["zone_control_size"]
    z2= cfg["zone_intersection_size"]

    z1_rad=z2+z1
    return z1_rad,z2

def excute_adjustments(passing_data):#passing_data is a split piece of time
    try:
        for veh in passing_data:
            if veh in traci.vehicle.getIDList():
                traci.vehicle.slowDown(veh,passing_data[veh]["spped"],0.01)
    except TraCIException:
        pass

def main():
    traci.start(sumoCmd)
    vehicles = add_vehicles(cfg["num_vehicles"])#waiting for implementation

    passing_data_total = None
    highest_total = 0

    step=0
    while step<cfg["max_simulation_steps"]:
        traci.simulationStep()
        step+=1
        z1_r,z2_r=get_zone_radii()

        for veh in vehicles:
            try:
                if veh.is_outbound():
                    veh.set_vehicle_state(cfg["veh_state_default"])
                else:
                    if veh.get_dist_to_intersection() < z2_radius:
                        veh.set_vehicle_state(cfg["veh_state_intersection"])
                    elif veh.get_dist_to_intersection() < z1_radius:
                        veh.set_vehicle_state(cfg["veh_state_control"])
            except TraCIException:
                print(f"vehicle {veh.veh_id} has left the sim at step {step}")
                vehicles.remove(veh)
        
        current_vehicles=traci.vehicle.getIDList()

        if step==20:#firstly enter the zone
            if cfg["passing_order_mode"]==cfg["passing_order_gurobi"]:
                veh_data=vehicles[0].get_vehicle_data()
                passing_data_total=vehicles[0].get_passing_data(veh_data)
                excute_adjustments(passing_data_total[(step-20)%100])#split the total decision into pieces
        
    traci.close()
    #show_report

if __name__ == "__main__":
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("You must declare environment variable SUMO_HOME")

    main()
