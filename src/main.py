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
cfg = yaml.safe_load(config_file)

sumoCmd = [
    os.path.join(os.environ['SUMO_HOME'], 'bin', 'sumo'),
    '-c', cfg["sumo_cfg"],
    '--tripinfo-output', cfg["trip_info_out"],
    '--collision-output', cfg["collisions_out"],
    '--statistic-output', cfg["statistics_out"]
]

def add_vehicles(num_vehicles):
    vehicles = []
    for i in range(num_vehicles):
        veh_id = "veh_{}".format(i)
        route = "route_{}".format(randint(0, cfg["num_routes"] - 1))#根据配置文件中的路数量为当前车辆选择一个随机的路线

        veh = Vehicle(
            veh_id=veh_id,
            route=route,
            state=cfg["veh_state_default"]
        )
        vehicles.append(veh)

    return vehicles

def get_zone_radii():#计算控制区和交叉路口的半径
    z1 = cfg["zone_control_size"]
    z2 = cfg["zone_intersection_size"]

    z1_rad = z2 + z1

    return z1_rad, z2

def execute_adjustments(passing_order):
    try:
        for veh in passing_order.adjusted_order:
            # if veh in traci.vehicle.getIDList(): 
                if passing_order.adjusted_order[veh] == 0:
                    traci.vehicle.slowDown(veh, cfg["veh_speed_default"], 0)
                else:
                    traci.vehicle.slowDown(veh, cfg["veh_speed_adjust"], 0)#设置为0表示马上到所需速度，后续需要考虑车辆加速度
                    traci.vehicle.setColor(veh, cfg["veh_col_orange"])
                    passing_order.adjusted_order[veh] -= 0.25#这里不懂
    except TraCIException:
        pass

def main():#可以添加车辆流
    traci.start(sumoCmd)
    vehicles = add_vehicles(cfg["num_vehicles"])#添加车辆
    passing_order = None
    highest_total = 0

    step = 0
    while step < cfg["max_simulation_steps"]:
        traci.simulationStep()
        step += 1
        # print(f"Step: {step}")

        z1_radius, z2_radius = get_zone_radii()

        

        for veh in vehicles:
            try:#！需要在每次开始之前移除已经离开仿真环境的车辆
                if veh.is_outbound():
                    veh.set_vehicle_state(cfg["veh_state_default"])
                else:
                    if veh.get_dist_to_intersection() < z2_radius:
                        veh.set_vehicle_state(cfg["veh_state_intersection"])
                    elif veh.get_dist_to_intersection() < z1_radius:
                        veh.set_vehicle_state(cfg["veh_state_control"])

            except TraCIException:  # vehicle has left the sim
                print(f"vehicle {veh.veh_id} has left the sim at step {step}")
                vehicles.remove(veh)
                
        # Get the list of vehicles currently in the simulation
        current_vehicles = traci.vehicle.getIDList()
        # print(f"current vehicles: {current_vehicles}")

        if step == 20:
            if cfg["passing_order_mode"] == cfg["passing_order_fcfs"]:
                vehicles[0].gather_veh_data(vehicles)
                passing_order = vehicles[0].get_passing_order()

        if step > 25:
            # Filter out vehicles that are no longer in the simulation
            passing_order.adjusted_order = {veh: adj for veh, adj in passing_order.adjusted_order.items() if veh in current_vehicles}
            execute_adjustments(passing_order)
            
     

    traci.close()

    show_report(passing_order, highest_total)

if __name__ == "__main__":
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("You must declare environment variable SUMO_HOME")

    main()