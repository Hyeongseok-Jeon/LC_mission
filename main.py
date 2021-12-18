import sys
import numpy as np
from UDP.UDP_parser import udp_parser,udp_sender
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json
from stanley import StanleyController
from paths.utils import pathReader, findLocalPath

# path = os.path.dirname(os.path.abspath( __file__ ))
path = os.getcwd()

with open(os.path.join(path,("UDP/params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]

user_ip = params["user_ip"]
host_ip = params["host_ip"]

object_port = params["object_info_dst_port"]
ego_port = params["ego_vehicle_stats_dst_port"]
TL_get_port = params["get_traffic_light_dst_port"]
TL_set_port = params["set_traffic_light_host_port"]
ego_ctrl_port = params["ego_ctrl_host_port"]
scene_port = params["scenario_load_host_port"]

sur = udp_parser(user_ip, object_port,'obj')
ego = udp_parser(user_ip, ego_port,'status')
TL_get = udp_parser(user_ip, TL_get_port,'get_traffic')
TL_set = udp_sender(host_ip, TL_set_port,'set_traffic')
ego_ctrl = udp_sender(host_ip, ego_ctrl_port, 'ctrl_cmd')
scenario_load = udp_sender(host_ip, scene_port, 'scenario')

path_tracker = StanleyController()
ego_status = dict()
sur_parse = dict()
i = 0
path_reader = pathReader()
global_path = path_reader.read('jeju_airport.json')
scenario_load.send_data(['1_LC_mission                  ', True, False, True, True, True, True, False])

while True:
    try:
        sur_data = sur.get_data()
        ego_data = ego.get_data()
        # TL_idx = TL_get.get_data()
        # TL_set.send_data([TL_idx[0], 32])
        '''여기 아래서 부터는 계속 작업중인 부분이니 고려안하셔도 됩니다'''
        ego_status['vel'] = ego_data[2]
        ego_status['x'] = ego_data[12]
        ego_status['y'] = ego_data[13]
        ego_status['heading'] = ego_data[17]

        local_path, current_waypoint = findLocalPath(global_path, ego_status['x'], ego_status['y'])

        path_tracker.set_ego_status(ego_status)
        path_tracker.set_path(local_path)
        steering_angle_in_rad = path_tracker.steering_angle()
        steering_angle = -np.rad2deg(steering_angle_in_rad) / 36.25
        print(np.rad2deg(steering_angle_in_rad))

        ctrl_mode = 2  # 2 = AutoMode / 1 = KeyBoard
        Gear = 4  # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
        cmd_type = 2  # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration
        send_velocity = 20  # cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0  # cmd_type이 3일때 원하는 가속도를 넣어준다.
        accel = 0
        brake = 0

        ego_ctrl.send_data([ctrl_mode, Gear, cmd_type, send_velocity, acceleration, accel, brake, steering_angle])

        time.sleep(0.02)
    except:
        scenario_load.send_data(['1_LC_mission                  ', True, False, True, True, True, True, False])


