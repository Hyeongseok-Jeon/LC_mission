import sys
import numpy as np
from UDP.UDP_parser import udp_parser,udp_sender
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json

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
sur = udp_parser(user_ip, object_port,'obj')
ego = udp_parser(user_ip, ego_port,'status')
TL_get = udp_parser(user_ip, TL_get_port,'get_traffic')
TL_set = udp_sender(host_ip, TL_set_port,'set_traffic')
ego_ctrl = udp_sender(host_ip, ego_ctrl_port, 'ctrl_cmd')

ego_parse = dict()
sur_parse = dict()
i = 0
while True:
    i = i + 0.01
    sur_data = sur.get_data()
    ego_data = ego.get_data()
    TL_idx = TL_get.get_data()[0]
    TL_set.send_data([TL_get.get_data()[0], 4])
    ego_parse['vel_mps'] = ego_data[2] /3.6
    ego_parse['x'] = ego_data[12]
    ego_parse['y'] = ego_data[13]
    ego_parse['heading'] = ego_data[17]

    ctrl_mode = 2  # 2 = AutoMode / 1 = KeyBoard
    Gear = 4  # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
    cmd_type = 2  # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration
    send_velocity = 5  # cmd_type이 2일때 원하는 속도를 넣어준다.
    acceleration = 0  # cmd_type이 3일때 원하는 가속도를 넣어준다.
    accel = 0
    brake = 0
    steering_angle = 20 * np.sin(2*np.pi*0.1*i)
    print(steering_angle)

    ego_ctrl.send_data([ctrl_mode, Gear, cmd_type, send_velocity, acceleration, accel, brake, steering_angle])

    time.sleep(0.01)

