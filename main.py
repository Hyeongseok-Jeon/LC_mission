import sys
import numpy as np
from UDP.UDP_parser import udp_parser, udp_sender
from math import cos, sin, sqrt, pow, atan2, pi
import time
import threading
import os, json
from stanley import StanleyController
from paths.utils import pathReader, findLocalPath
from cruise.acc import control as acc
from cruise.acc import get_target
from lane_change.lc_main import lane_changer

# path = os.path.dirname(os.path.abspath( __file__ ))
path = os.getcwd()

with open(os.path.join(path, ("UDP/params.json")), 'r') as fp:
    params = json.load(fp)

params = params["params"]

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
# from matplotlib import pyplot as plt
# for i in range(len(mgeos)):
#     points = np.asarray(mgeos[i]['points'])
#     plt.scatter(points[:, 0], points[:, 1],color='k')
#     plt.axis('equal')

sur = udp_parser(user_ip, object_port, 'obj', mgeos)
ego = udp_parser(user_ip, ego_port, 'status', mgeos)
TL_get = udp_parser(user_ip, TL_get_port, 'get_traffic', mgeos)
TL_set = udp_sender(host_ip, TL_set_port, 'set_traffic')
ego_ctrl = udp_sender(host_ip, ego_ctrl_port, 'ctrl_cmd')
scenario_load = udp_sender(host_ip, scene_port, 'scenario')

LC_manager = lane_changer(mgeos, global_path, global_link, predictor='cv')
path_tracker = StanleyController()
ego_status = dict()
sur_status = dict()
scenario = '1_LC_mission                  '
scenario_load.send_data([scenario, False, True, True, True, True, True, False])
# scenario_load.send_data(['test                          ', False, True, True, True, True, True, True])
state = None
LC_phase = 3
LC_cnt = 0

start_point = np.asarray([[-90.04747009277344, 280.0341796875]])
end_point = np.asarray([[121.24830627441406,364.656494140625],
                        [122.84101867675781, 361.5880126953125],
                        [125.08885192871094, 358.66607666015625],
                        [127.35853576660156, 355.77191162109375],
                        [130.8572235107422, 343.28472900390625]])
pos_save = 0
scene_regen = 0



while True:
    '''
    get ego info
    '''
    ego_data = ego.get_data()
    ego_status['vel'] = ego_data[2] / 3.6
    ego_status['x'] = ego_data[12]
    ego_status['y'] = ego_data[13]
    ego_status['heading'] = ego_data[17]
    ego_status['vel_x'] = ego_data[18] / 3.6
    ego_status['vel_y'] = ego_data[19] / 3.6
    ego_status['acc_x'] = ego_data[21]
    ego_status['acc_y'] = ego_data[22]
    ego_status['link_id'] = ego_data[-2]
    ego_status['link_index'] = ego_data[-1]
    ego_pos = np.asarray([ego_status['x'], ego_status['y']])
    # print(ego_status['vel'] * 3.6)
    # print(ego_status['x'], ego_status['y'])
    # print(ego_status['link_id'])
    if np.min(np.linalg.norm(end_point - ego_pos, axis= 1)) < 3:
        scene_regen = 1
    elif np.min(np.linalg.norm(start_point - ego_pos, axis= 1)) < 3:
        scene_regen = 0
    #
    # if pos_save == 0:
    #     prev_pos = ego_pos
    # pos_save = pos_save + 1
    # if pos_save == 1000:
    #     pos_save = 0
    #
    # if scene_regen == 0:
    #     if np.linalg.norm(ego_pos - prev_pos) < 0.0001:
    #         scene_regen = 1
    #     else:
    #         scene_regen = 0
    # print(pos_save)
    # print(scene_regen)

    if scene_regen == 0:
        '''
        get surrounding info
        '''
        init_time = time.time()
        sur_data = sur.get_data().copy()
        # sur_status['id'] = sur_data[0][0]
        # sur_status['x'] = sur_data[0][2]
        # sur_status['y'] = sur_data[0][3]
        # sur_status['heading'] = sur_data[0][5]
        ego_in_sur = [0, -1, ego_data[12], ego_data[13], ego_data[14], ego_data[17], ego_data[6], ego_data[7], ego_data[8],
                      ego_data[9], ego_data[10], ego_data[11], ego_data[18], ego_data[19], ego_data[20], ego_data[21], ego_data[22], ego_data[23]]
        sur_data.append(ego_in_sur)
        '''
        get prediction
        '''
        LC_manager.set_ego_info(ego_status)
        target_idx = LC_manager.set_veh_info_ego_cordinate(sur_data)
        prediction = LC_manager.prediction()

        '''
        calculate lane change path
        '''
        if LC_cnt < 4:
            if ego_status['link_id'] == '{b40ad01a-81ba-4a82-b214-ea94d56bf98f}' and LC_phase == 3:
                LC_phase = 0

            if LC_phase == 0:
                print('here')
                LC_cnt = LC_cnt + 1
                # if LC_cnt == 1:
                #     break
                global_path, global_link, LC_phase = LC_manager.get_lc_goal_cands(LC_phase)

            elif LC_phase == 1:
                if ego_status['link_id'] in global_link:
                    dist_to_link = np.min(np.linalg.norm(np.asarray(mgeos[ego_status['link_index']]['points'])[:,:2] - np.asarray([ego_status['x'], ego_status['y']]), axis=1))
                    if dist_to_link < 0.3:
                        LC_phase = 0
        else:
            pass
        # from matplotlib import pyplot as plt
        # plt.scatter(np.asarray(global_path)[:,0], np.asarray(global_path)[:,1], s = 1)
        # plt.scatter(ego_status['x'], ego_status['y'])

        '''
        get local path
        '''
        local_path, current_waypoint = findLocalPath(global_path, ego_status['x'], ego_status['y'])
        # print(ego_status['vel'] * 3.6)
        # plt.scatter(np.asarray(local_path)[:,0], np.asarray(local_path)[:,1], color ='r', s = 0.5)

        '''
        get target velocity
        '''
        if ego_status['link_id'] == '{ea2b8531-6438-4899-adf6-2f0e314c2203}':
            target_velocity = 30/3.6
        elif ego_status['link_id'] == '{a78b6e70-6381-472a-bc5d-6827dfc83795}':
            target_velocity = 30/3.6
        elif ego_status['link_id'] == '{e46e21cc-1ecc-42af-acc6-0822ece0a223}':
            target_velocity = 40/3.6
        elif ego_status['link_id'] == '{27d09adb-30e3-4d3e-8915-f72e2289871e}':
            point = np.asarray([111.09992980957031, 337.95147705078125])
            if np.linalg.norm(point - ego_pos) < 50:
                a = ((40/3.6)**2 - (ego_status['vel'])**2)/(2*np.linalg.norm(point - ego_pos))
                target_velocity = target_velocity + a * 0.01
        else:
            target_velocity = 60/3.6
        '''
        lateral controller
        '''
        path_tracker.set_ego_status(ego_status)
        path_tracker.set_path(local_path)
        path_tracker.steering_angle()
        steering_angle_in_rad, lat_offset = path_tracker.get_output()
        steering_angle = -np.rad2deg(steering_angle_in_rad) / 36.25

        '''
        longitudinal controller
        '''
        brake, gas, control, state = acc(ego_status['vel'], car_in_front=200, cruise_speed=target_velocity, state=state)


        '''
        control command
        '''
        ctrl_mode = 2  # 2 = AutoMode / 1 = KeyBoard
        Gear = 4  # 4 1 : (P / parking ) 2 (R / reverse) 3 (N / Neutral)  4 : (D / Drive) 5 : (L)
        cmd_type = 1  # 1 : Throttle  /  2 : Velocity  /  3 : Acceleration
        send_velocity = 0  # cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = 0  # cmd_type이 3일때 원하는 가속도를 넣어준다.
        if gas > 0.5:
            accel_pedal = 0.5
        else:
            accel_pedal = gas
        brake_pedal = brake

        ego_ctrl.send_data([ctrl_mode, Gear, cmd_type, send_velocity, acceleration, accel_pedal, brake_pedal, steering_angle])
        print(time.time()-init_time)

        # print(['2', time.time() - init_time])

        # target_pos = get_target(global_link, sur_data, ego_data, mgeos)
        # print(['3', time.time() - init_time])

        # try:
        #     target_index = np.argmin(np.linalg.norm(np.asarray(global_path)[:,:2] - target_pos, axis=1))
        #     ego_index = np.argmin(np.linalg.norm(np.asarray(global_path)[:,:2] - np.asarray([ego_status['x'], ego_status['y']]) , axis=1))
        #     dist_to_target = np.sum(np.linalg.norm(np.asarray(global_path)[1:,:2] - np.asarray(global_path)[:-1,:2], axis=1)[ego_index:target_index])
        # except:
        #     dist_to_target = 500
        # print(['acc', time.time()-init_time])

        # dist_to_target = np.sqrt((sur_status['x'] - ego_status['x']) ** 2 + (sur_status['y'] - ego_status['y']) ** 2)
        # print(ego_status['vel']*3.6)
    else:
        # time.sleep(0.02)
        # print(time.time()-init_time)

        path_tracker = StanleyController()
        ego_status = dict()
        sur_status = dict()
        scenario_load.send_data([scenario, False, True, True, True, True, True, False])
        # scenario_load.send_data(['test                          ', False, True, True, True, True, True, True])
        state = None
        LC_phase = 3
        LC_cnt = 0

#
# from matplotlib import pyplot as plt
# for i in range(len(LC_manager.hist_traj)):
#     plt.plot(LC_manager.hist_traj[i,LC_manager.hist_traj[i,:,0]!=0,0], LC_manager.hist_traj[i,LC_manager.hist_traj[i,:,0]!=0,1])
#
# for i in range(len(LC_manager.fut_traj)):
#     plt.plot(LC_manager.fut_traj[i, :, 0], LC_manager.fut_traj[i, :, 1], 'b')