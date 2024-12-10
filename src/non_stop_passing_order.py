import traci
import yaml
import gurobi_cal

config_file = open("./src/config.yaml")
cfg = yaml.safe_load(config_file)

class PassingOrder:
    def __init__(self, vehicle_data):#vehicle_data sorted by distance to intersection
        self.veh_data = vehicle_data
        self.adjusted_velocity = {}
        self.core_vehicle_data={}
        self.follower_vehicle_data={}
        for veh in vehicle_data:
            if vehicle_data[veh]["State"]==cfg["veh_state_control"]:
                traci.vehicle.setSpeedMode(vehicle_data[veh]["Real_ID"], 32)
                self.core_vehicle_data[veh]=vehicle_data[veh]
                self.adjusted_velocity[veh]=vehicle_data[veh]["speed"]
            else:
                traci.vehicle.setSpeedMode(vehicle_data[veh]["Real_ID"], 31)
                self.follower_vehicle_data[veh]=vehicle_data[veh]

    def calculate_global_changes(self):
        print("model start111111111")
        print(self.core_vehicle_data)
        results=gurobi_cal.cal_adjustments(self.core_vehicle_data, self.adjusted_velocity)
        other_velocity={}
        #for i, veh in enumerate(self.veh_data):
                #for v in self.veh_data[i:0:-1]:
                    #if traci.vehicle.getRoadID(veh) == traci.vehicle.getRoadID(v):#找到前车
                         #veh1_speed=traci.vehicle.getSpeed(veh)
                         #veh1_position=traci.vehicle.getPosition(veh)
                         #veh2_speed=traci.vehicle.getSpeed(v)
                         #veh2_position=traci.vehicle.getPosition(v)
                         #distance=(veh1_position[0]-veh2_position[0])**2+(veh1_position[1]-veh2_position[1])**2
                         #a_max=5
                         #s_ex=8+max(0,veh1_speed*0.01+veh1_speed*(veh1_speed-veh2_speed)/2/a_max)
                         #dv=a_max*(1-(veh1_speed/13.88)**1 -(s_ex/distance)**2)
        return results
                              
