import socket
import threading
import time
import struct
import numpy as np

class udp_parser:
    def __init__(self, ip, port, data_type, mgeo):
        self.mgeo = mgeo
        self.data_type = data_type
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip, port)
        self.sock.bind(recv_address)
        self.data_size = 65535
        self.parsed_data = []
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True
        thread.start()

    def recv_udp_data(self):
        while True:
            init = time.time()
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)
            # print(time.time() - init)

    def data_parsing(self, raw_data):

        if self.data_type == 'status':
            header = raw_data[0:11].decode()
            data_length = struct.unpack('i', raw_data[11:15])
            if header == '#MoraiInfo$' and data_length[0] == 132:
                ctrl_mode = struct.unpack('b', raw_data[27:28])[0]
                gear = struct.unpack('b', raw_data[28:29])[0]
                signed_vel = struct.unpack('f', raw_data[29:33])[0]
                map_id = struct.unpack('i', raw_data[33:37])[0]
                accel = struct.unpack('f', raw_data[37:41])[0]
                brake = struct.unpack('f', raw_data[41:45])[0]
                size_x, size_y, size_z = struct.unpack('fff', raw_data[45:57])
                overhang, wheelbase, rear_overhang = struct.unpack('fff', raw_data[57:69])
                pose_x, pose_y, pose_z = struct.unpack('fff', raw_data[69:81])
                roll, pitch, yaw = struct.unpack('fff', raw_data[81:93])
                vel_x, vel_y, vel_z = struct.unpack('fff', raw_data[93:105])
                accel_x, accel_y, accel_z = struct.unpack('fff', raw_data[105:117])
                steer = struct.unpack('f', raw_data[117:121])[0]
                link_id = raw_data[121:159].decode()
                if '\x00' in link_id:
                    link_id = '{' + link_id[:8]
                    if '\x00' in link_id:
                        link_id = '{not_detected}'
                data_1 = ctrl_mode, gear, signed_vel, map_id, accel, brake, size_x, size_y, size_z, overhang, wheelbase, rear_overhang
                data_2 = pose_x, pose_y, pose_z, roll, pitch, yaw, vel_x, vel_y, vel_z, accel_x, accel_y, accel_z, steer, link_id[1:-1]
                unpacked_data = data_1 + data_2

                self.parsed_data = list(unpacked_data)


        elif self.data_type == 'obj':

            header = raw_data[0:14].decode()

            if header == '#MoraiObjInfo$':
                unpacked_data = []
                offset_byte = 30

                for i in range(20):
                    start_byte = i * 106
                    obj_id, obj_type = struct.unpack('hh', raw_data[start_byte + offset_byte:start_byte + offset_byte + 4])
                    pos_x, pos_y, pos_z = struct.unpack('fff', raw_data[start_byte + offset_byte + 4:start_byte + offset_byte + 16])
                    heading = struct.unpack('f', raw_data[start_byte + offset_byte + 16:start_byte + offset_byte + 20])[0]
                    size_x, size_y, size_z = struct.unpack('fff', raw_data[start_byte + offset_byte + 20:start_byte + offset_byte + 32])
                    overhang, wheelbase, rear_overhang = struct.unpack('fff', raw_data[start_byte + offset_byte + 32:start_byte + offset_byte + 44])
                    vel_x, vel_y, vel_z = struct.unpack('fff', raw_data[start_byte + offset_byte + 44:start_byte + offset_byte + 56])
                    accel_x, accel_y, accel_z = struct.unpack('fff', raw_data[start_byte + offset_byte + 56:start_byte + offset_byte + 68])
                    cur_link = raw_data[start_byte+offset_byte+68:start_byte+offset_byte+106].decode()
                    if '\x00' in cur_link:
                        cur_link = '{' + cur_link[:9]
                        if '\x00' in cur_link:
                            cur_link = '{not_detected}'
                    obj_info_list = [obj_id, obj_type, pos_x, pos_y, pos_z, heading, size_x, size_y, size_z, overhang,
                                     wheelbase, rear_overhang, vel_x, vel_y, vel_z, accel_x, accel_y, accel_z, cur_link[1:-1]]

                    if not (obj_info_list[0] == 0):
                        unpacked_data.append(obj_info_list)

                    else:
                        break

                if len(obj_info_list) != 0:
                    self.parsed_data = unpacked_data
                    # print(self.parsed_data)
                else:
                    self.parsed_data = []

        elif self.data_type == 'get_traffic':
            header = raw_data[0:14].decode()
            data_length = struct.unpack('i', raw_data[14:18])
            if header == '#TrafficLight$' and data_length[0] == 16:
                traffic_index = raw_data[30:42].decode()
                traffic_type, traffic_status = struct.unpack('2h', raw_data[42:46])
                self.parsed_data = [traffic_index, traffic_type, traffic_status]

    def get_data(self):
        return self.parsed_data

    def __del__(self):
        self.sock.close()
        print('del')

    def get_cur_link(self, pos):
        pos = np.asarray(pos)
        min_dist = []
        for i in range(len(self.mgeo)):
            points = np.asarray(self.mgeo[i]['points'])[:,:2]
            min_dist.append(np.min(np.linalg.norm(points-pos, axis=1)))
        return self.mgeo[np.argmin(min_dist)]['idx'], np.argmin(min_dist)

class udp_sender:
    def __init__(self, ip, port, data_type):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip = ip
        self.port = port
        self.data_type = data_type

        if self.data_type == 'ctrl_cmd':
            header = '#MoraiCtrlCmd$'.encode()
            data_length = struct.pack('i', 23)
            aux_data = struct.pack('iii', 0, 0, 0)
            self.upper = header + data_length + aux_data
            self.tail = '\r\n'.encode()

        elif self.data_type == 'set_traffic':
            header = '#TrafficLight$'.encode()
            data_length = struct.pack('i', 14)
            aux_data = struct.pack('iii', 0, 0, 0)
            self.upper = header + data_length + aux_data
            self.tail = '\r\n'.encode()

        elif self.data_type == 'multi_ego':
            header = '#MultiEgoSetting$'.encode()
            data_length = struct.pack('i', 920)
            aux_data = struct.pack('iii', 0, 0, 0)
            self.upper = header + data_length + aux_data
            self.tail = '\r\n'.encode()

        elif self.data_type == 'scenario':
            header = '#ScenarioLoad$'.encode()
            data_length = struct.pack('i', 37)
            aux_data = struct.pack('iii', 0, 0, 0)
            self.upper = header + data_length + aux_data
            self.tail = '\r\n'.encode()

    def send_data(self, data):

        if self.data_type == 'ctrl_cmd':
            packed_mode = struct.pack('b', data[0])
            packed_gear = struct.pack('b', data[1])
            packed_cmd_type = struct.pack('b', data[2])
            packed_velocity = struct.pack('f', data[3])
            packed_acceleration = struct.pack('f', data[4])
            packed_accel = struct.pack('f', data[5])
            packed_brake = struct.pack('f', data[6])
            packed_steering_angle = struct.pack('f', data[7])
            lower = packed_mode + packed_gear + packed_cmd_type + packed_velocity + packed_acceleration + packed_accel + packed_brake + packed_steering_angle
            send_data = self.upper + lower + self.tail
            # print(len(send_data),send_data)

        elif self.data_type == 'set_traffic':
            packed_traffic_index = data[0].encode()
            packed_traffic_status = struct.pack('h', data[1])
            lower = packed_traffic_index + packed_traffic_status
            send_data = self.upper + lower + self.tail

        elif self.data_type == 'multi_ego':
            num_of_ego = len(data)
            camera_index = 0
            packed_num_of_ego = struct.pack('i', num_of_ego)
            packed_camera_index = struct.pack('i', camera_index)
            lower = None
            for ego in range(20):
                if ego < len(data):
                    print(ego, data[ego][0], data[ego][1], data[ego][2])
                    ego_index = struct.pack('i', data[ego][0])
                    status_data = struct.pack('3dffffBB', data[ego][1], data[ego][2], data[ego][3], data[ego][4],
                                              data[ego][5], data[ego][6], data[ego][7], data[ego][8], data[ego][9])
                    pack_data = ego_index + status_data

                else:
                    ego_index = struct.pack('i', 0)
                    status_data = struct.pack('3dffffBB', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0)
                    pack_data = ego_index + status_data

                if lower == None:
                    lower = pack_data
                else:
                    lower += pack_data

            send_data = self.upper + packed_num_of_ego + packed_camera_index + lower

        elif self.data_type == 'scenario':
            packed_scenario_name = data[0].encode()
            packed_delete_all = struct.pack('b', data[1])
            packed_network = struct.pack('b', data[2])
            packed_ego_vehicle = struct.pack('b', data[3])
            packed_npc_vehicle = struct.pack('b', data[4])
            packed_pedestrian = struct.pack('b', data[5])
            packed_object = struct.pack('b', data[6])
            packed_pause= struct.pack('b', data[7])

            lower = packed_scenario_name + packed_delete_all + packed_network + packed_ego_vehicle + packed_npc_vehicle + packed_pedestrian + packed_object + packed_pause
            send_data = self.upper + lower + self.tail

        self.sock.sendto(send_data, (self.ip, self.port))