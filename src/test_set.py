import xml.etree.ElementTree as ET

def parse_vehicle_data(xml_file, timestep_index=0):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # 获取指定的timestep
    timestep = root.findall('timestep')[45]

    vehicle_data = {}
    velocity={}
    for vehicle in timestep.findall('vehicle'):
        vehicle_id = vehicle.get('id')
        if(not(vehicle_id=="veh_0" or vehicle_id=="veh_1" or vehicle_id=="veh_2" or vehicle_id=="veh_3" or vehicle_id=="veh_4")):
            continue
        x = float(vehicle.get('x'))
        y = float(vehicle.get('y'))
        speed = float(vehicle.get('speed'))
        lane = vehicle.get('lane')
        if(lane=="road_hrz_1_0"):#从左到右
            lane=0
        elif(lane=="road_ver_1_0"):#从上到下
            lane=4
        elif(lane=="road_hrz_3_0"):#从右到左
            lane=2
        else:
            lane=6
        vehicle_data[vehicle_id] = {
            "route": lane,
            "Position":[x,y]
        }
        velocity[vehicle_id]=speed

    vehicle_data={'veh_0': {'route': 0, 'Position': [-55.62, -1.6]}, 'veh_1': {'route': 4, 'Position': [-1.6, 67.34]}, 'veh_2': {'route': 2, 'Position': [55.62, 1.6]}, 'veh_3': {'route': 6, 'Position': [1.6, -67.34]}, 'veh_4': {'route': 0, 'Position': [-59.69, -1.6]}}
    velocity={'veh_0': 13.88, 'veh_1': 13.88, 'veh_2': 13.88, 'veh_3': 13.88, 'veh_4': 13.88}
    return vehicle_data,velocity

# 示例用法
xml_file = './src/test2_fcd.xml'
vehicle_data = parse_vehicle_data(xml_file, timestep_index=0)
print(vehicle_data)

vehicle_data={'veh_0': {'route': 0, 'Position': [-115.62, -1.6]}, 'veh_1': {'route': 4, 'Position': [-1.6, 127.34]}, 'veh_2': {'route': 2, 'Position': [115.62, 1.6]}, 'veh_3': {'route': 6, 'Position': [1.6, -127.34]}, 'veh_4': {'route': 0, 'Position': [-119.69, -1.6]}}
velocity={'veh_0': 13.88, 'veh_1': 13.88, 'veh_2': 13.88, 'veh_3': 13.88, 'veh_4': 13.88}