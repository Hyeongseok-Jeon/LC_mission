import sys
import numpy as np
from UDP.UDP_parser import udp_parser,udp_sender
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json
from stanley import StanleyController
from paths.utils import pathReader, findLocalPath
from cruise.acc import control as acc
from lane_change.lc_main import lane_changer
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

path_reader = pathReader()
global_path, global_link, mgeos = path_reader.read('jeju_airport.json')

sur = udp_parser(user_ip, object_port,'obj',mgeos)
ego = udp_parser(user_ip, ego_port,'status',mgeos)
TL_get = udp_parser(user_ip, TL_get_port,'get_traffic',mgeos)
TL_set = udp_sender(host_ip, TL_set_port,'set_traffic')
ego_ctrl = udp_sender(host_ip, ego_ctrl_port, 'ctrl_cmd')
scenario_load = udp_sender(host_ip, scene_port, 'scenario')

LC_manager = lane_changer(mgeos, predictor='cv')
path_tracker = StanleyController()
ego_status = dict()
sur_status = dict()

scenario_load.send_data(['1_LC_mission                  ', False, True, True, True, True, True, False])
state=None

while True:
    try:
        sur_data = sur.get_data()
        sur_status['id'] = sur_data[0][0]
        sur_status['x'] = sur_data[0][2]
        sur_status['y'] = sur_data[0][3]
        sur_status['heading'] = sur_data[0][5]

        # TL_idx = TL_get.get_data()
        # TL_set.send_data([TL_idx[0], 32])

        ego_data = ego.get_data()
        ego_status['vel'] = ego_data[2]/3.6
        ego_status['x'] = ego_data[12]
        ego_status['y'] = ego_data[13]
        ego_status['heading'] = ego_data[17]
        ego_status['vel_x'] = ego_data[18]/3.6
        ego_status['vel_y'] = ego_data[19]/3.6
        ego_status['acc_x'] = ego_data[21]
        ego_status['acc_y'] = ego_data[22]
        ego_status['link_id'] = ego_data[-2]
        ego_status['link_index'] = ego_data[-1]

        LC_manager.set_ego_info(ego_status)
        target_idx = LC_manager.set_veh_info_ego_cordinate(sur_data)

        local_path, current_waypoint = findLocalPath(global_path, ego_status['x'], ego_status['y'])
        path_tracker.set_ego_status(ego_status)
        path_tracker.set_path(local_path)
        steering_angle_in_rad = path_tracker.steering_angle()
        steering_angle = -np.rad2deg(steering_angle_in_rad) / 36.25

        dist_to_target = np.sqrt((sur_status['x'] - ego_status['x'])**2 + (sur_status['y'] - ego_status['y'])**2)
        brake, gas, control, state = acc(ego_status['vel'], acceleration= 0, car_in_front= dist_to_target, cruise_speed=50/3.6, state=state)
        print(control)
        print(state['prev_setpoint'])
        ctrl_mode = 2  # 2 = AutoMode / 1 = KeyBoard
        Gear = 4  # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
        cmd_type = 1  # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration
        send_velocity = 0  # cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0  # cmd_type이 3일때 원하는 가속도를 넣어준다.
        accel_pedal = gas
        brake_pedal = brake

        ego_ctrl.send_data([ctrl_mode, Gear, cmd_type, send_velocity, acceleration, accel_pedal, brake_pedal, steering_angle])

        time.sleep(0.02)
    except:
        scenario_load.send_data(['1_LC_mission                  ', False, True, True, True, True, True, False])


