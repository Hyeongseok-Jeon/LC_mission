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
sur = udp_parser(user_ip, object_port,'obj')
ego = udp_parser(user_ip, ego_port,'status')
TL_get = udp_parser(user_ip, TL_get_port,'get_traffic')
TL_set = udp_sender(user_ip, TL_set_port,'set_traffic')

ego_parse = dict()
sur_parse = dict()
while True:
    sur_data = sur.get_data()
    ego_data = ego.get_data()
    TL_set.send_data([TL_get.get_data()[0], 1])
    ego_parse['vel_mps'] = ego_data[2] /3.6
    ego_parse['x'] = ego_data[12]
    ego_parse['y'] = ego_data[13]
    ego_parse['heading'] = ego_data[17]
    time.sleep(0.1)

