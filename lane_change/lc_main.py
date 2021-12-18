import numpy as np


class lane_changer:
    def __init__(self, mgeos, lateral_acc_limit = 0.5, predictor='cv'):
        self.lat_acc_limit = lateral_acc_limit
        self.predictor = predictor
        self.mgeos = mgeos
        self.target_idx = None
        self.ego_pos = None
        self.ego_link = None
        self.ego_link_index = None
        self.rot = None
        # thread = threading.Thread(target=self.recv_udp_data)
        # thread.daemon = True
        # thread.start()

    def get_lc_goal_cands(self):
        right_link = self.mgeos[self.ego_link_index]['right_lane_change_dst_link_idx']
        right_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['idx'] == right_link][0]
        right_link_to_node = self.mgeos[right_link_index]['to_node_idx']
        right_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == right_link_to_node][0]
        right_front_link = self.mgeos[right_front_link_index]['idx']
        right_front_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == self.mgeos[right_front_link_index]['to_node_idx']][0]
        right_front_front_link = self.emgeos[right_front_front_link_index]['idx']

        points = np.asarray(mgeos[right_link_index]['points'] + mgeos[right_front_link_index]['points'][1:] + mgeos[right_front_front_link_index]['points'][1:])[:,:2]
        points_t = np.matmul(rot, (points- ego_pos[:2]).T).T
        points_t = points_t[points_t[:,0]>0, :]

        max_travel_dist = self.ego_data['vel'] * 2 + 0.5 * 0.5 * 9.8 * 4
        points_t = points_t[np.linalg.norm(points_t, axis=1)<max_travel_dist,:]

        vel_heading = np.arctan2(ego_data['vel_y'], ego_data['vel_x'])
        for i in range(len(points_t)):
            x0 = 0
            x1 = ego_data['vel_x']
            x2 = ego_data['acc_x'] / 2
            x3 = (points_t[i,0] - x0 - 2*x0- 4*x2)/8

            y0 = 0
            y1 = ego_data['vel_y']
            y2 = ego_data['acc_y'] / 2
            x3 = (points_t[i,0] - x0 - 2*x0- 4*x2)/8

        y0 + y1*t + y2*t**2 * y3*t**3
        y1 + 2*y2+t + 3*y3*t**2
        2*y3 + 6*y3*t
        points_t_global = np.matmul(np.linalg.inv(rot), points_t.T).T + ego_pos[:2]

    def set_veh_info_ego_cordinate(self, data):
        sur_pos_data = np.zeros(shape=(len(data), 2))
        for i in range(len(sur_pos_data)):
            sur_pos_data[i, 0] = data[i][2]
            sur_pos_data[i, 1] = data[i][3]
        disp = sur_pos_data - self.ego_pos[:2]
        self.rot = np.asarray([[np.cos(np.deg2rad(-self.ego_pos[2])), -np.sin(np.deg2rad(-self.ego_pos[2]))], [np.sin(np.deg2rad(-self.ego_pos[2])), np.cos(np.deg2rad(-self.ego_pos[2]))]])
        sur_pos_ego_cord = np.matmul(self.rot, disp.T).T

        self.target_idx = np.where((-3.3 * 1.5 < sur_pos_ego_cord[:, 1]) & (sur_pos_ego_cord[:, 1] < 0))
        return self.target_idx

    def set_ego_info(self, data):
        self.ego_data = data
        self.ego_pos = np.asarray([data['x'], data['y'], data['heading']])
        self.ego_link = self.mgeos[data['link_index']]['idx']
        self.ego_link_index = data['link_index']
