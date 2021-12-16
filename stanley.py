import numpy as np
from math import cos,sin, radians
import numpy.matlib as npm
class StanleyController:
    def __init__(self, k=0.5, wheelbase=2.8, polynomial_order=3):
        # control agin
        self.k = k
        self.wheelbase = wheelbase
        self.polynomial_order = polynomial_order

        self.vehicle_position = np.matrix([[0.0], [0.0]])
        self.vehicle_angle = 0.0
        self.vehicle_velocity = 0.0

        self.path = np.matrix([[0.0], [0.0]])

    def set_ego_status(self, status_msg):
        '''
        ego_status['vel'] = ego_data[2]
        ego_status['x'] = ego_data[12]
        ego_status['y'] = ego_data[13]
        ego_status['heading'] = ego_data[17]

        '''
        self.current_vel = status_msg['vel'] / 3.6  # m/s
        self.vehicle_angle = radians(status_msg['heading'])
        # rear to front wheel position
        self.vehicle_position[0] = status_msg['x'] + self.wheelbase * np.cos(self.vehicle_angle)
        self.vehicle_position[1] = status_msg['y'] + self.wheelbase * np.sin(self.vehicle_angle)

    def set_path(self, path_msg):
        x = [row[0] for row in path_msg]
        y = [row[1] for row in path_msg]
        self.path = np.array([x, y])

    def steering_angle(self):
        R = np.matrix(
            [[cos(-self.vehicle_angle), -sin(-self.vehicle_angle)],
             [sin(-self.vehicle_angle), cos(-self.vehicle_angle)]]
        )
        local_path = self.path - np.tile(self.vehicle_position, (1, self.path.shape[1]))
        rotated_local_path = R.dot(local_path)

        path_coefficients = np.polyfit(
            np.squeeze(np.array(rotated_local_path[0, :])),
            np.squeeze(np.array(rotated_local_path[1, :])),
            self.polynomial_order,
        )
        path_coefficients_derivative = np.polyder(path_coefficients)

        lateral_offset = np.polyval(path_coefficients, 0)
        theta_distance = np.arctan2(self.k * lateral_offset, self.current_vel)
        theta_heading = np.arctan(np.polyval(path_coefficients_derivative, 0))

        theta = theta_distance + theta_heading
        return theta