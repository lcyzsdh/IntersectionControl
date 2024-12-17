veh_data = {}
veh_data["veh_0"] = {
    "route": "1",
    "edge": 5
}
veh_data["veh_1"] = {
    "route": "22",
    "edge": 5
}
for veh in veh_data:
    print(veh)
    print(veh_data[veh])
    print(veh_data[veh]["route"])