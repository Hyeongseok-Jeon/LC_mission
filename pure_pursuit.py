import os
from math import cos,sin,sqrt,pow,atan2,pi
import numpy as np


class Point() :
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class purePursuit:
    def __init__(self):
        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 4
        self.lfd = 5
        self.min_lfd = 5
        self.max_lfd = 30
        self.steering = 0

    def getPath(self, path):
        self.path = path

    def getEgoStatus(self, position_x, position_y, position_z, velocity, heading):

        self.current_vel = velocity  # kph
        self.vehicle_yaw = heading / 180 * pi  # rad
        self.current_postion.x = position_x
        self.current_postion.y = position_y
        self.current_postion.z = position_z

    def steering_angle(self):
        vehicle_position = self.current_postion
        rotated_point = Point()
        self.is_look_forward_point = False

        for i in self.path:
            dx = i[0] - vehicle_position.x
            dy = i[1] - vehicle_position.y
            rotated_point.x = cos(self.vehicle_yaw) * dx + sin(self.vehicle_yaw) * dy
            rotated_point.y = sin(self.vehicle_yaw) * dx - cos(self.vehicle_yaw) * dy

            if rotated_point.x > 0:
                dis = sqrt(pow(rotated_point.x, 2) + pow(rotated_point.y, 2))

                if dis >= self.lfd:

                    self.lfd = self.current_vel * 0.3  # or 0.4
                    if self.lfd < self.min_lfd:
                        self.lfd = self.min_lfd
                    elif self.lfd > self.max_lfd:
                        self.lfd = self.max_lfd
                    # self.forward_point=i
                    self.is_look_forward_point = True

                    break

        theta = atan2(rotated_point.y, rotated_point.x)

        if self.is_look_forward_point:
            # self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi
            self.steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)
            return self.steering  # deg
        else:
            print("no found forward point")
            return 0