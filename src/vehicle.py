import yaml
import traci
import numpy as np
from non_stop_passing_order import PassingOrder

config_file = open("./src/config.yaml")
cfg=yaml.safe_load(config_file)

class Vehicle:
    def __init__(self, veh_id, veh_id_ac,route, state, veh_data=None):
        if veh_data is None:
            veh_data = {}
        self.veh_id = veh_id#system id:natural number
        self.veh_id_ac = veh_id_ac#actual id:f route.number
        self.route = route
        self.state = state
        self.veh_data = veh_data

        traci.vehicle.add(veh_id_ac, route)
        self.set_vehicle_state(state)

    def set_vehicle_state(self, state):#set different states for vehicles
        self.state = state
        if state == cfg["veh_state_default"]:
            traci.vehicle.setColor(self.veh_id_ac, cfg["veh_col_grey"])
            traci.vehicle.setSpeedMode(self.veh_id_ac, cfg["veh_safe_mode"])
            traci.vehicle.setSpeed(self.veh_id_ac, cfg["veh_speed_default"])

        elif state == cfg["veh_state_control"]:
            traci.vehicle.setColor(self.veh_id_ac, cfg["veh_col_green"])

        elif state == cfg["veh_state_intersection"]:
            traci.vehicle.setColor(self.veh_id_ac, cfg["veh_col_white"])

    def is_outbound(self):
        return traci.vehicle.getRoadID(self.veh_id) == traci.vehicle.getRoute(self.veh_id)[-1]

    def get_dist_to_intersection(self, veh_id=None):#distance to the mid-point of the intersection
        if veh_id is None:
            veh_id = self.veh_id

        loc = np.array(traci.vehicle.getPosition(veh_id))
        intersection_loc = np.array(cfg["intersection_coords"])

        return np.linalg.norm(loc - intersection_loc)
    
    def gather_veh_data(self, vehicles):#gather vehicle data
        self.veh_data = {}
        for veh in vehicles:
            self.veh_data[veh.veh_id] = {
                "route": veh.route[6:],
                "edge": traci.vehicle.getLaneID(veh.veh_id_ac),
                "distance": float(self.get_dist_to_intersection(veh.veh_id_ac)),
                "speed": traci.vehicle.getSpeed(veh.veh_id_ac),
                "Posiotion":traci.vehicle.getPosition(veh.veh_id_ac),
                "State":traci.vehicle.getState(veh.veh_id_ac)
            }
        
        return self.veh_data
    
    def get_passing_data(self):
        passing_data={}
        passing_data=PassingOrder(sorted(self.veh_data.items(),key=lambda x:x[1]["distance"],reverse=True))#sort the vehicles by distance to the intersection

        passing_data.calculate_global_changes()

        return passing_data