import os
from math import cos,sin,sqrt,pow,atan2,pi
import json
import numpy as np

def findLocalPath(ref_path,position_x,position_y):
    out_path=[]
    current_x=position_x
    current_y=position_y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path)) :
        dx=current_x - ref_path[i][0]
        dy=current_y - ref_path[i][1]
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i


    if current_waypoint+50 > len(ref_path) :
        last_local_waypoint= len(ref_path)
    else :
        last_local_waypoint=current_waypoint+50

    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=[]
        tmp_pose.append(ref_path[i][0])
        tmp_pose.append(ref_path[i][1])
        out_path.append(tmp_pose)

    return out_path,current_waypoint ## local_path와 waypoint를 반환 ##

class pathReader:
    def __init__(self):
        self.file_path = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.normpath(os.path.join(self.file_path, '..'))

    def read(self, file_name):
        if 'txt' in file_name:
            full_file_name = self.file_path + '/paths/' + file_name
            openFile = open(full_file_name, 'r')

            out_path = []

            line = openFile.readlines()

            for i in line:
                tmp = i.split()
                read_pose = []
                read_pose.append(float(tmp[0]))
                read_pose.append(float(tmp[1]))
                read_pose.append(float(tmp[2]))
                out_path.append(read_pose)

            openFile.close()

        elif 'json' in file_name:
            full_file_name = self.file_path + '/paths/' + file_name
            link_file = full_file_name[:-5] + '_link.txt'

            openFile = open(link_file, 'r')
            links = []
            line = openFile.readlines()
            for i in line:
                tmp = i.split()
                links.append(tmp[0])
            openFile.close()

            with open(full_file_name, "r") as st_json:
                mgeos = json.load(st_json)

            out_path = []
            for i in range(len(links)):
                link_name = '{'+links[i]+'}'
                for j in range(len(mgeos)):
                    if mgeos[j]['idx'] == link_name:
                        points = mgeos[j]['points']
                if i == 0:
                    out_path = out_path + points
                else:
                    out_path = out_path + points[1:]
        return out_path, links, mgeos