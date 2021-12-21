import numpy as np
import threading


class lane_changer:
    def __init__(self, mgeos, global_path, global_link, lateral_acc_limit=0.5, t_lc=[2, 3, 4, 5], accs=[-0, 5, 0, 0.5], predictor='cv'):
        self.t_lc = t_lc
        self.global_link = global_link
        self.global_path = np.asarray(global_path)[:, :2]
        self.accs = accs
        self.lat_acc_limit = lateral_acc_limit
        self.predictor = predictor
        self.mgeos = mgeos
        self.target_idx = None
        self.ego_pos = None
        self.ego_link = None
        self.ego_link_index = 159
        self.rot = None

    def get_lc_goal_cands(self, phase):
        if phase == 0:
            right_link = self.mgeos[self.ego_link_index]['right_lane_change_dst_link_idx']
            if right_link != None:
                new_global_link = []
                right_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['idx'] == right_link][0]
                right_link_to_node = self.mgeos[right_link_index]['to_node_idx']

                right_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == right_link_to_node][0]
                new_global_link.append(self.mgeos[right_front_link_index]['idx'])
                try:
                    right_front_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == self.mgeos[right_front_link_index]['to_node_idx']][0]
                    new_global_link.append(self.mgeos[right_front_front_link_index]['idx'])
                except:
                    pass
                try:
                    right_front_front_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == self.mgeos[right_front_front_link_index]['to_node_idx']][0]
                    new_global_link.append(self.mgeos[right_front_front_front_link_index]['idx'])
                except:
                    pass
                try:
                    right_front_front_front_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == self.mgeos[right_front_front_front_link_index]['to_node_idx']][0]
                    new_global_link.append(self.mgeos[right_front_front_front_front_link_index]['idx'])
                except:
                    pass
                try:
                    right_front_front_front_front_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == self.mgeos[right_front_front_front_front_link_index]['to_node_idx']][0]
                    new_global_link.append(self.mgeos[right_front_front_front_front_front_link_index]['idx'])
                except:
                    pass
                try:
                    right_front_front_front_front_front_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == self.mgeos[right_front_front_front_front_front_link_index]['to_node_idx']][0]
                    new_global_link.append(self.mgeos[right_front_front_front_front_front_front_link_index]['idx'])
                except:
                    pass

                try:
                    points = np.asarray(self.mgeos[right_link_index]['points'] +
                                        self.mgeos[right_front_link_index]['points'][1:] +
                                        self.mgeos[right_front_front_link_index]['points'][1:] +
                                        self.mgeos[right_front_front_front_link_index]['points'][1:] +
                                        self.mgeos[right_front_front_front_front_link_index]['points'][1:] +
                                        self.mgeos[right_front_front_front_front_front_link_index]['points'][1:] +
                                        self.mgeos[right_front_front_front_front_front_front_link_index]['points'][1:])[:, :2]
                except:
                    try:
                        points = np.asarray(self.mgeos[right_link_index]['points'] +
                                            self.mgeos[right_front_link_index]['points'][1:] +
                                            self.mgeos[right_front_front_link_index]['points'][1:] +
                                            self.mgeos[right_front_front_front_link_index]['points'][1:] +
                                            self.mgeos[right_front_front_front_front_link_index]['points'][1:] +
                                            self.mgeos[right_front_front_front_front_front_link_index]['points'][1:])[:, :2]
                    except:
                        try:
                            points = np.asarray(self.mgeos[right_link_index]['points'] +
                                                self.mgeos[right_front_link_index]['points'][1:] +
                                                self.mgeos[right_front_front_link_index]['points'][1:] +
                                                self.mgeos[right_front_front_front_link_index]['points'][1:] +
                                                self.mgeos[right_front_front_front_front_link_index]['points'][1:])[:, :2]
                        except:
                            try:
                                points = np.asarray(self.mgeos[right_link_index]['points'] +
                                                    self.mgeos[right_front_link_index]['points'][1:] +
                                                    self.mgeos[right_front_front_link_index]['points'][1:] +
                                                    self.mgeos[right_front_front_front_link_index]['points'][1:])[:, :2]
                            except:
                                try:
                                    points = np.asarray(self.mgeos[right_link_index]['points'] +
                                                        self.mgeos[right_front_link_index]['points'][1:] +
                                                        self.mgeos[right_front_front_link_index]['points'][1:])[:, :2]
                                except:
                                    try:
                                        points = np.asarray(self.mgeos[right_link_index]['points'] +
                                                            self.mgeos[right_front_link_index]['points'][1:])[:, :2]
                                    except:
                                        points = np.asarray(self.mgeos[right_link_index]['points'])[:, :2]

                points_t = np.matmul(self.rot, (points - self.ego_pos[:2]).T).T
                points_t = points_t[points_t[:, 0] > 0, :]

                max_travel_dist = self.ego_data['vel'] * self.t_lc[2] + 0.5 * 0.5 * 9.8 * self.t_lc[2] ** 2
                points_t = points_t[np.linalg.norm(points_t, axis=1) < max_travel_dist, :]
                points_t_global = np.matmul(np.linalg.inv(self.rot), points_t.T).T + self.ego_pos[:2]

                # for i in range(len(points_t_global)):
                #     for j in range(len(self.accs)):
                #         if i == 0:
                #             head_f = np.arctan2(points_t_global[i + 1, 1] - points_t_global[i, 1], points_t_global[i + 1, 0] - points_t_global[i, 0])
                #             v_f = self.ego_data['vel'] + 2 * self.accs[j]
                #         else:
                #             head_f = np.arctan2(points_t_global[i, 1] - points_t_global[i - 1, 1], points_t_global[i, 0] - points_t_global[i - 1, 0])
                #             v_f = self.ego_data['vel'] + 2 * self.accs[j]
                #         vx_f = v_f * np.cos(head_f)
                #         vy_f = v_f * np.sin(head_f)
                #
                #         x0 = self.ego_data['x']
                #         x1 = self.ego_data['vel_x']
                #         x3 = (vx_f - points_t_global[i,0] + x0 + x1)/4
                #         x2 = (vx_f - x1 -12*x3)/4
                #
                #         y0 = self.ego_data['y']
                #         y1 = self.ego_data['vel_y']
                #         y3 = (vy_f - points_t_global[i,1] + y0 + y1)/4
                #         y2 = (vy_f - y1 -12*y3)/4
                target_point_idx = -1
                acc_idx = 1
                t_lc_idx = 2
                if target_point_idx == 0:
                    head_f = np.arctan2(points_t_global[target_point_idx + 1, 1] - points_t_global[target_point_idx, 1], points_t_global[target_point_idx + 1, 0] - points_t_global[target_point_idx, 0])
                    v_f = self.ego_data['vel'] + self.t_lc[t_lc_idx] * self.accs[acc_idx]
                else:
                    head_f = np.arctan2(points_t_global[target_point_idx, 1] - points_t_global[target_point_idx - 1, 1], points_t_global[target_point_idx, 0] - points_t_global[target_point_idx - 1, 0])
                    v_f = self.ego_data['vel'] + self.t_lc[t_lc_idx] * self.accs[acc_idx]
                vx_f = v_f * np.cos(head_f)
                vy_f = v_f * np.sin(head_f)

                nearest_idx = np.argmin(np.linalg.norm(self.global_path - np.asarray([self.ego_data['x'], self.ego_data['y']]), axis=1))
                near_pt = self.global_path[nearest_idx+20]

                x0 = near_pt[0]
                x1 = self.ego_data['vel'] * np.cos(np.deg2rad(self.ego_data['heading']))
                x2 = (3 * (points_t_global[target_point_idx, 0] - x0 - x1 * self.t_lc[t_lc_idx]) - self.t_lc[t_lc_idx] * (vx_f - x1)) / (self.t_lc[t_lc_idx] ** 2)
                x3 = (points_t_global[target_point_idx, 0] - x0 - x1 * self.t_lc[t_lc_idx] - x2 * self.t_lc[t_lc_idx] ** 2) / (self.t_lc[t_lc_idx] ** 3)

                y0 = near_pt[1]
                y1 = self.ego_data['vel'] * np.sin(np.deg2rad(self.ego_data['heading']))
                y2 = (3 * (points_t_global[target_point_idx, 1] - y0 - y1 * self.t_lc[t_lc_idx]) - self.t_lc[t_lc_idx] * (vy_f - y1)) / (self.t_lc[t_lc_idx] ** 2)
                y3 = (points_t_global[target_point_idx, 1] - y0 - y1 * self.t_lc[t_lc_idx] - y2 * self.t_lc[t_lc_idx] ** 2) / (self.t_lc[t_lc_idx] ** 3)

                x = np.poly1d([x3, x2, x1, x0])
                y = np.poly1d([y3, y2, y1, y0])
                t = np.linspace(0, self.t_lc[t_lc_idx], self.t_lc[t_lc_idx]*100+1)
                LC_path = np.asarray([x(t), y(t)]).T

                new_end = points[np.argmin(np.linalg.norm(points - LC_path[-1], axis=1)):]
                new_global_path = [list(i) for i in self.global_path[:nearest_idx+20]] + [list(i) for i in LC_path] + [list(i) for i in new_end]
                # new_global_path = 'test'
            else:
                new_global_path = self.global_path
                new_global_link = self.global_link
            self.global_path = new_global_path
            self.global_link = new_global_link
            return new_global_path, new_global_link, 1

    def set_veh_info_ego_cordinate(self, data):
        sur_pos_data = np.zeros(shape=(len(data), 2))
        for i in range(len(sur_pos_data)):
            sur_pos_data[i, 0] = data[i][2]
            sur_pos_data[i, 1] = data[i][3]
        disp = sur_pos_data - self.ego_pos[:2]
        self.rot = np.asarray([[np.cos(np.deg2rad(-self.ego_pos[2])), -np.sin(np.deg2rad(-self.ego_pos[2]))], [np.sin(np.deg2rad(-self.ego_pos[2])), np.cos(np.deg2rad(-self.ego_pos[2]))]])
        sur_pos_ego_cord = np.matmul(self.rot, disp.T).T

        self.target_idx = np.where((-3.3 * 1.5 < sur_pos_ego_cord[:, 1]) & (sur_pos_ego_cord[:, 1] < -3.3 * 0.5))
        return self.target_idx

    def set_ego_info(self, data):
        self.ego_data = data
        self.ego_pos = np.asarray([data['x'], data['y'], data['heading']])
        self.ego_link = self.mgeos[data['link_index']]['idx']
        self.ego_link_index = data['link_index']

    def get_global_path(self):
        return self.global_path

