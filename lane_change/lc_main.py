import numpy as np
import threading
import time


class lane_changer:
    def __init__(self, mgeos, global_path, global_link, lateral_acc_limit=0.5, t_lc=[2, 3, 4, 5], accs=[-0.5, 0, 0.5], predictor='cv'):
        self.t_lc = t_lc
        self.global_link = global_link
        self.global_path = np.asarray(global_path)[:, :2]
        self.accs = accs
        self.lat_acc_limit = lateral_acc_limit
        self.predictor = predictor
        self.mgeos = mgeos
        self.target_idx = 0
        self.ego_pos = None
        self.ego_link = None
        self.ego_link_index = 159
        self.rot = None
        self.last_point = [117.8864124831982, 341.979038732088]
        self.end_sig = 0
        self.sur_data = []
        self.sur_data_time = []
        self.fut_traj = np.zeros(shape=(0, 20, 2))
        self.hist_traj = np.zeros(shape=(1, 20, 5))

    def prediction(self):
        if self.predictor == 'none':
            self.fut_traj = np.zeros(shape=(len(self.sur_data[-1]), 20, 2))
            self.fut_traj = np.repeat(self.hist_traj[:, -1:, :], 20, axis = 1)

        if self.predictor == 'cv':
            cur_pos = self.hist_traj[np.intersect1d(self.hist_traj[:,0,4], self.target_idx, return_indices=True)[1],-1,:]
            self.fut_traj = np.zeros(shape=(len(cur_pos), 20, 2))
            for i in range(len(cur_pos)):
                LK_path = self.get_LK_path(cur_pos[i])
                travel_length = np.concatenate((np.linalg.norm(cur_pos[i:i+1,:2] - LK_path[0:1], axis=1), np.linalg.norm(LK_path[1:] - LK_path[:-1], axis=1)), axis=0)
                for j in range(20):
                    s = 0.1* (j+1) * cur_pos[i, 2]
                    for k in range(len(travel_length)):
                        if np.sum(travel_length[:k+1]) > s:
                            if k == 0:
                                interval_start = 0
                                interval_end = 0
                            else:
                                interval_start = k - 1
                                interval_end = k
                            break
                    if interval_start == interval_end:
                        x = cur_pos[i, 0] + s * (LK_path[0,0] - cur_pos[i, 0]) / travel_length[0]
                        y = cur_pos[i, 1] + s * (LK_path[0,1] - cur_pos[i, 1]) / travel_length[0]
                    else:
                        x = LK_path[interval_start, 0] + (s-np.sum(travel_length[:interval_end])) * (LK_path[interval_end,0] - LK_path[interval_start,0]) / travel_length[interval_end]
                        y = LK_path[interval_start, 1] + (s-np.sum(travel_length[:interval_end])) * (LK_path[interval_end,1] - LK_path[interval_start,1]) / travel_length[interval_end]
                    self.fut_traj[i, j, 0] = x
                    self.fut_traj[i, j, 1] = y

                '''
                ss = 2 * cur_pos[i, 2]
                for k in range(len(travel_length)):
                    if np.sum(travel_length[:k + 1]) > ss:
                        if k == 0:
                            interval_start = 0
                            interval_end = 0
                        else:
                            interval_start = k - 1
                            interval_end = k
                        break
                for j in range(20):
                    s = 0.1 * (j + 1) * cur_pos[i, 2]
                    if interval_start == interval_end:
                        x = cur_pos[i, 0] + s * (LK_path[0, 0] - cur_pos[i, 0]) / travel_length[0]
                        y = cur_pos[i, 1] + s * (LK_path[0, 1] - cur_pos[i, 1]) / travel_length[0]
                    else:
                        # x = LK_path[interval_start, 0] + (s-np.sum(travel_length[:interval_end])) * (LK_path[interval_end,0] - LK_path[interval_start,0]) / travel_length[interval_end]
                        # y = LK_path[interval_start, 1] + (s-np.sum(travel_length[:interval_end])) * (LK_path[interval_end,1] - LK_path[interval_start,1]) / travel_length[interval_end]
                        x = cur_pos[i, 0] + s * (LK_path[interval_end, 0] - cur_pos[i, 0]) / np.linalg.norm(
                            LK_path[interval_end] - cur_pos[i, :2])
                        y = cur_pos[i, 1] + s * (LK_path[interval_end, 1] - cur_pos[i, 1]) / np.linalg.norm(
                            LK_path[interval_end] - cur_pos[i, :2])
                    self.fut_traj[i, j, 0] = x
                    self.fut_traj[i, j, 1] = y
                '''
        return self.fut_traj

    def get_lc_goal_cands(self, phase):
        if phase == 0:
            right_link = self.mgeos[self.ego_link_index]['right_lane_change_dst_link_idx']
            new_global_link = []
            new_global_link.append(right_link)
            if right_link != None:
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

                max_travel_dist = self.ego_data['vel'] * self.t_lc[2] + 0.5 * 0.3 * 9.8 * self.t_lc[2] ** 2
                points_t = points_t[np.linalg.norm(points_t, axis=1) < max_travel_dist, :]
                points_t_global = np.matmul(np.linalg.inv(self.rot), points_t.T).T + self.ego_pos[:2]
                if len(np.where(points_t_global[:,0] == self.last_point[0])[0]) == 1:
                    target_point_idx = np.where(points_t_global[:,0] == self.last_point[0])[0][0]
                    self.end_sig = 1
                else:
                    '''
                    select target point index based on the prediction result
                    min point index = 60
                    '''
                    target_point_idx = 70

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
                acc_idx = 1
                t_lc_idx = 2
                if target_point_idx == 0:
                    head_f = np.arctan2(points_t_global[target_point_idx + 1, 1] - points_t_global[target_point_idx, 1], points_t_global[target_point_idx + 1, 0] - points_t_global[target_point_idx, 0])
                    v_f = self.ego_data['vel'] + self.t_lc[t_lc_idx] * self.accs[acc_idx]
                elif self.end_sig == 1:
                    head_f = np.arctan2(points_t_global[target_point_idx, 1] - points_t_global[target_point_idx - 1, 1], points_t_global[target_point_idx, 0] - points_t_global[target_point_idx - 1, 0])
                    s = np.linalg.norm(points_t_global[target_point_idx] - self.ego_pos[:2])
                    v_f = np.sqrt(2*self.accs[acc_idx]*s + self.ego_data['vel']**2)
                else:
                    head_f = np.arctan2(points_t_global[target_point_idx, 1] - points_t_global[target_point_idx - 1, 1], points_t_global[target_point_idx, 0] - points_t_global[target_point_idx - 1, 0])
                    v_f = self.ego_data['vel'] + self.t_lc[t_lc_idx] * self.accs[acc_idx]
                vx_f = v_f * np.cos(head_f)
                vy_f = v_f * np.sin(head_f)

                nearest_idx = np.argmin(np.linalg.norm(self.global_path - np.asarray([self.ego_data['x'], self.ego_data['y']]), axis=1))
                near_pt = self.global_path[nearest_idx+5]

                x0 = self.ego_data['x']
                x1 = 1.3*self.ego_data['vel'] * np.cos(np.deg2rad(self.ego_data['heading']))
                x2 = (3 * (points_t_global[target_point_idx, 0] - x0 - x1 * self.t_lc[t_lc_idx]) - self.t_lc[t_lc_idx] * (vx_f - x1)) / (self.t_lc[t_lc_idx] ** 2)
                x3 = (points_t_global[target_point_idx, 0] - x0 - x1 * self.t_lc[t_lc_idx] - x2 * self.t_lc[t_lc_idx] ** 2) / (self.t_lc[t_lc_idx] ** 3)

                y0 = self.ego_data['y']
                y1 = 1.3*self.ego_data['vel'] * np.sin(np.deg2rad(self.ego_data['heading']))
                y2 = (3 * (points_t_global[target_point_idx, 1] - y0 - y1 * self.t_lc[t_lc_idx]) - self.t_lc[t_lc_idx] * (vy_f - y1)) / (self.t_lc[t_lc_idx] ** 2)
                y3 = (points_t_global[target_point_idx, 1] - y0 - y1 * self.t_lc[t_lc_idx] - y2 * self.t_lc[t_lc_idx] ** 2) / (self.t_lc[t_lc_idx] ** 3)

                x = np.poly1d([x3, x2, x1, x0])
                y = np.poly1d([y3, y2, y1, y0])
                t = np.linspace(0, self.t_lc[t_lc_idx], self.t_lc[t_lc_idx]*100+1)
                LC_path = np.asarray([x(t), y(t)]).T

                new_end = points[np.argmin(np.linalg.norm(points - LC_path[-1], axis=1)):]
                new_global_path = [list(i) for i in self.global_path[:nearest_idx+5]] + [list(i) for i in LC_path] + [list(i) for i in new_end]
                # new_global_path = 'test'
            else:
                new_global_path = self.global_path
                new_global_link = self.global_link
            self.global_path = new_global_path
            self.global_link = new_global_link
            return new_global_path, new_global_link, 1

    def set_veh_info_ego_cordinate(self, data):
        if len(self.sur_data) == 0:
            self.sur_data.append(data)
            self.sur_data_time.append(time.time())
        else:
            cur_time = time.time()
            if cur_time - self.sur_data_time[-1] > 0.099:
                if len(self.sur_data) < 20:
                    self.sur_data.append(data)
                    self.sur_data_time.append(time.time())
                else:
                    self.sur_data_time = self.sur_data_time[1:]
                    self.sur_data_time.append(time.time())
                    self.sur_data = self.sur_data[1:]
                    self.sur_data.append(data)

                sur_pos_data = np.zeros(shape=(len(data), 2))
                for i in range(len(sur_pos_data)):
                    sur_pos_data[i, 0] = data[i][2]
                    sur_pos_data[i, 1] = data[i][3]
                disp = sur_pos_data - self.ego_pos[:2]
                self.rot = np.asarray([[np.cos(np.deg2rad(-self.ego_pos[2])), -np.sin(np.deg2rad(-self.ego_pos[2]))], [np.sin(np.deg2rad(-self.ego_pos[2])), np.cos(np.deg2rad(-self.ego_pos[2]))]])
                sur_pos_ego_cord = np.matmul(self.rot, disp.T).T

                index = np.where((-3.3 * 1.5 < sur_pos_ego_cord[:, 1]) & (sur_pos_ego_cord[:, 1] < -3.3 * 0.5))[0]
                self.target_idx = [data[i][0] for i in index]
                # 옆차선에 있는 차량 mgeo index기반으로 추출하는 방향으로 수정필요

                num_other_veh = 0
                self.hist_traj = np.zeros(shape=(len(self.sur_data[-1]), 20, 5))
                for k in range(len(self.sur_data[-1])):
                    id = self.sur_data[-1][k][0]
                    if self.sur_data[-1][k][1] == -1:
                        row = 0
                    else:
                        row = 1 + num_other_veh
                        num_other_veh = num_other_veh + 1
                    for i in range(len(self.sur_data)):
                        index = len(self.sur_data)-1-i
                        veh_idxs = [self.sur_data[index][j][0] for j in range(len(self.sur_data[index]))]
                        if id in veh_idxs:
                            veh_idx = veh_idxs.index(id)
                            x = self.sur_data[index][veh_idx][2]
                            y = self.sur_data[index][veh_idx][3]
                            v = np.sqrt(self.sur_data[index][veh_idx][12]**2 + self.sur_data[index][veh_idx][13]**2)
                            head = self.sur_data[index][veh_idx][5]
                            self.hist_traj[row, 19-i, 0] = x
                            self.hist_traj[row, 19-i, 1] = y
                            self.hist_traj[row, 19-i, 2] = v/3.6
                            self.hist_traj[row, 19-i, 3] = head
                            self.hist_traj[row, 19-i, 4] = id

                '''
                sur_status['x'] = sur_data[0][2]
                sur_status['y'] = sur_data[0][3]
                '''
        return self.target_idx

    def set_ego_info(self, data):
        self.ego_data = data
        self.ego_pos = np.asarray([data['x'], data['y'], data['heading']])
        self.ego_link = self.mgeos[data['link_index']]['idx']
        self.ego_link_index = data['link_index']

    def get_global_path(self):
        return self.global_path


    def get_LK_path(self, pos):
        min_dist = []
        for i in range(len(self.mgeos)):
            points = np.asarray(self.mgeos[i]['points'])[:, :2]
            min_dist.append(np.min(np.linalg.norm(points - pos[:2], axis=1)))
        LK_links = []

        cur_link = self.mgeos[np.argmin(min_dist)]['idx']
        LK_links.append(cur_link)
        cur_index = np.argmin(min_dist)
        cur_link_to_node = self.mgeos[cur_index]['to_node_idx']
        front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == cur_link_to_node][0]
        LK_links.append(self.mgeos[front_link_index]['idx'])
        front_front_link_index = [i for i in range(len(self.mgeos)) if self.mgeos[i]['from_node_idx'] == self.mgeos[front_link_index]['to_node_idx']][0]
        LK_links.append(self.mgeos[front_front_link_index]['idx'])

        LK_points = np.asarray(self.mgeos[cur_index]['points'] +
                            self.mgeos[front_link_index]['points'][1:] +
                            self.mgeos[front_front_link_index]['points'][1:])[:, :2]

        nearest_next_pos = np.argmin(np.linalg.norm(LK_points - pos[:2], axis=1))
        head_to_nearest = np.rad2deg(np.arctan2((LK_points[nearest_next_pos]- pos[:2])[1], (LK_points[nearest_next_pos]- pos[:2])[0]))
        if np.abs(head_to_nearest - pos[3]) < 90:
            nearest_index = nearest_next_pos
        else:
            nearest_index = nearest_next_pos+1
        return LK_points[nearest_index:]
