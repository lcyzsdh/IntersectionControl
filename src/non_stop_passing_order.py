import traci
import yaml
import gurobi_cal

config_file = open("./src/config.yaml")
cfg = yaml.safe_load(config_file)

class PassingOrder:
    def __init__(self, passing_order):
        self.passing_order = passing_order
        self.adjusted_velocity = {}

        for veh in passing_order:
            #veh[0]为车辆ID，veh[1]为车辆数据
            self.adjusted_velocity[veh[0]] = traci.vehicle.getSpeed(veh[0])


    def calculate_adjustments(self):
        results=gurobi_cal.cal_adjustments(self.passing_order, self.adjusted_velocity)