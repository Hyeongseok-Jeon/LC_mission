import sys
import numpy as np
from UDP_parser import udp_parser,udp_sender
from math import cos,sin,sqrt,pow,atan2,pi
import time
import threading
import os,json

# path = os.path.dirname(os.path.abspath( __file__ ))
path = os.getcwd()

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]

user_ip = params["user_ip"]
host_ip = params["host_ip"]

object_port =params["object_info_dst_port"]
ego_port = params["ego_vehicle_stats_dst_port"]
sur = udp_parser(user_ip, object_port,'obj')
ego = udp_parser(user_ip, ego_port,'status')
sur_data = sur.get_data()
ego_data = ego.get_data()
