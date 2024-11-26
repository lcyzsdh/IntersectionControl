import traci
import yaml
import gurobi_cal

config_file = open("./src/config.yaml")
cfg = yaml.safe_load(config_file)

class PassingOrder:
    def __init__(self, vehicle_data):
        self.passing_order = vehicle_data
        self.adjusted_velocity = {}

        for veh in vehicle_data:
            #veh[0]为车辆ID，veh[1]为车辆数据
            self.adjusted_velocity[veh[0]] = traci.vehicle.getSpeed(veh[0])


    def calculate_global_changes(self):
        results=gurobi_cal.cal_adjustments(self.passing_order, self.adjusted_velocity)
        other_velocity={}
        for i, veh in enumerate(self.passing_order):
                for v in self.passing_order[i:0:-1]:
                    if traci.vehicle.getRoadID(veh) == traci.vehicle.getRoadID(v):#找到前车
                         veh1_speed=traci.vehicle.getSpeed(veh)
                         veh1_position=traci.vehicle.getPosition(veh)
                         veh2_speed=traci.vehicle.getSpeed(v)
                         veh2_position=traci.vehicle.getPosition(v)
                         distance=(veh1_position[0]-veh2_position[0])**2+(veh1_position[1]-veh2_position[1])**2
                         a_max=5
                         s_ex=8+max(0,veh1_speed*0.01+veh1_speed*(veh1_speed-veh2_speed)/2/a_max)
                         dv=a_max*(1-(veh1_speed/13.88)**1 -(s_ex/distance)**2)
                              
