lidar = {
    "id Vendor": 0x19a2,
    "id Product": 0x5001,
    "origin": [0,0,0]
}

command = {
    "message": 'sRN LMDscandata'
}

picar = {
    "beta": 90,
    "tire-tire width": 135,
    "lidar-back wheel": 78,
    "tire-tire length": 140,
    "tire radius": 32,
    "angular velocity": 100,         # rpm at speed=40
    "offset": 85,                   # Due to hanging USB Cable
    "safety gap": 70                # To avoid hitting obstacles during turning
}

lidar_distances = {
    "minimum distance": 20,
    "maximum distance": 4000,
    "turning min": 350,         # Safe distance from closest object to allow smooth turning for the picar
    "turning max": 700,
}