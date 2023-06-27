#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from init import *
from probability_distribution import *

def return_frontier_candidate(_map, info):
    frontier_candidate = list()
    if len(_map) != 0: #地図がちゃんとSubscribeできたとき
        around = [-1, 0, 1]
        width = info.width
        height = info.height
        for y in range(width): 
            for x in range(height):
                frontier = False
                if _map[y*height + x] == 0: #（0:空白）の周りに（-1:未知）があるインデックスを探す
                    for y_diff in around:
                        for x_diff in around:
                            y_ = y + y_diff
                            x_ = x + x_diff
                            if _map[y_*height + x_] == -1:
                                frontier = True
                
                if frontier == True:
                    frontier_candidate.append(y*height + x)
    return frontier_candidate

def divide_candidate_into_groups(frontier_candidate, info):
    grouped_idxes = [[]]
    around = [-1, 0, 1]
    # around = [-4, -3, -2, -1, 0, 1, 2, 3, 4]

    for f in range(len(frontier_candidate)):
        if f == 0:
            grouped_idxes[0].append(frontier_candidate[0])
        else:
            join = False
            target_idx = frontier_candidate[f]
            for g in range(len(grouped_idxes)):
                for idx_in_g in grouped_idxes[g]:
                    x = int(idx_in_g % info.height)
                    y = int(idx_in_g / info.height)
                    for y_diff in around:
                        for x_diff in around:
                            y_ = y + y_diff
                            x_ = x + x_diff
                            around_idx = y_*info.height + x_
                            if around_idx == target_idx:
                                join = True 
                                join_g = g
                
            if  join == True:
                grouped_idxes[join_g].append(target_idx)
            else:
                grouped_idxes.append([])
                group_num = len(grouped_idxes)
                grouped_idxes[group_num-1].append(target_idx)
    return grouped_idxes

def decide_frontier(grouped_idxes, info):
    x_list = list()
    y_list = list()

    for g in range(len(grouped_idxes)):
        x_sum = 0
        y_sum = 0
        for idx in range(len(grouped_idxes[g])):
            target_idx = grouped_idxes[g][idx]
            x, y = index2point(target_idx, info)
            x_sum += x
            y_sum += y
        group_point_num = len(grouped_idxes[g])
        if group_point_num >= 10:
            x_list.append(x_sum / group_point_num)
            y_list.append(y_sum / group_point_num)

    return x_list, y_list

def sort_frontier(xlist, ylist, map_, info):
    frontier_num = len(xlist)
    #frontierの数が0ならreturn
    if frontier_num == 0:
        paths = 0
        thetalist = list()
        return xlist, ylist, thetalist 
    
    #frontierの周りに専有された格子があればfrontierから排除
    pop_idx = list()
    for f in range(frontier_num):
        is_frontier = True
        frontier_idx = point2index(xlist[f], ylist[f], info)
        x_idx = int(frontier_idx % info.height)
        y_idx = int(frontier_idx / info.height)
        around = [-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8]
        for y_diff in around:
            for x_diff in around:
                target_x = x_idx + x_diff
                target_y = y_idx + y_diff
                target_idx = target_y*info.height + target_x
                if map_[target_idx] == 100:  #周りに障害物があるとき
                    is_frontier = False

        if is_frontier == False:
            pop_idx.append(f)
    
    tmp_xlist = list()
    tmp_ylist = list()
    for f in range(frontier_num):
        if (f in pop_idx) == False:
            tmp_xlist.append(xlist[f])
            tmp_ylist.append(ylist[f])
    xlist = tmp_xlist
    ylist = tmp_ylist
    frontier_num = len(xlist)
    #frontierから最近傍の未知領域とfrontierの角度求める
    frontier_thetas = list()
    for f in range(frontier_num):
        is_unknown = False
        frontier_idx = point2index(xlist[f], ylist[f], info)
        x_idx = int(frontier_idx % info.height)
        y_idx = int(frontier_idx / info.height)
        around = [-1, 0, 1]
        unknown_idx = 0
        while is_unknown == False:
            for y_diff in around:
                for x_diff in around:
                    # if ((x_diff == 0) and (y_diff == 0)) != False:
                    target_x = x_idx + x_diff
                    target_y = y_idx + y_diff
                    target_idx = target_y*info.height + target_x
                    if map_[target_idx] == -1:  #周りに障害物があるとき
                        is_unknown = True
                        unknown_idx = target_idx
            around_max = around[-1]
            around_min = around[0]
            around.insert(0, around_min-1)
            around.append(around_max+1)
        x_unknown, y_unknown = index2point(unknown_idx, info)
        frontier_theta = math.atan2(y_unknown-ylist[f], x_unknown-xlist[f])
        frontier_thetas.append(frontier_theta)
    # print(frontier_thetas)
    return xlist, ylist, frontier_thetas

def return_grid_points(_map, info):
    interval = 1.3
    grid_interval = interval // info.resolution
    xlist = np.arange(0, info.width, grid_interval)
    ylist = np.arange(0, info.height, grid_interval)
    grid_points_idx = list()
    if len(_map) != 0: #地図がちゃんとSubscribeできたとき
        _map = np.array(_map)
        around = [-6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6]
        # around = [-4, -3, -2, -1, 0, 1, 2, 3, 4]
        width = info.width
        height = info.height
        for y in ylist: 
            for x in xlist:
                grid_points = True
                if _map[int(y*height + x)] == 0: #（0:空白）の周りに（100:占有）がある場合、格子点から除く
                    for y_diff in around:
                        for x_diff in around:
                            y_ = y + y_diff
                            x_ = x + x_diff
                            if _map[int(y_*height + x_)] != 0:
                                grid_points = False
                else:
                    grid_points = False
                if grid_points == True:
                    grid_points_idx.append(y*height + x)
    
    grid_x_points = list()
    grid_y_points = list()
    grid_theta_points = list()
    for g in range(len(grid_points_idx)):
        gx, gy = index2point(grid_points_idx[g], info)
        grid_x_points.append(gx)
        grid_y_points.append(gy)
        random_theta = random.uniform(-math.pi, math.pi)
        grid_theta_points.append(random_theta)
    return grid_x_points, grid_y_points, grid_theta_points

class Information_Gain():
    def path_callback(self, msg):
        self.path_from_astar = list()
        for p in range(len(msg.poses)):
            self.path_from_astar.append([msg.poses[p].pose.position.x, msg.poses[p].pose.position.y, eular_from_quaternion(msg.poses[p].pose.orientation)-math.pi])

    def particle_callback(self, data):
        self.gmapping_particles = data

    def travel_distance_coord(self, paths):
        distance_list = list()
        for p in range(len(paths)):
            path_len = len(paths[p])
            target_path = paths[p]
            each_dist = 0
            for idx in range(1, path_len):
                pre = target_path[idx-1]
                now = target_path[idx]
                each_dist += math.sqrt((pre[0] - now[0])**2 + (pre[1] - now[1])**2)
            distance_list.append(each_dist)
        return distance_list

    def return_control_info_coords(self, paths, theta_list):
        point_num = len(paths)
        control_info = list()
        lenear_vel = 0.2
        angular_vel = 0.5
        for p in range(point_num):
            path_len = len(paths[p])
            target_path = paths[p]
            each_control_info = list()
            for idx in range(1, path_len):
                pre = target_path[idx-1]
                now = target_path[idx]
                distance = math.sqrt((pre[0] - now[0])**2 + (pre[1] - now[1])**2)
                theta_diff = now[2] - pre[2]

                lenear_time = distance / lenear_vel
                angular_time = abs(theta_diff) / angular_vel
                if theta_diff >= 0:
                    each_control_info.append([0, angular_vel, angular_time])
                else:
                    each_control_info.append([0, -angular_vel, angular_time])
                each_control_info.append([lenear_vel, 0, lenear_time])
            angular_time = abs(theta_list[p]) / angular_vel
            if theta_list[p] >= 0:
                each_control_info.append([0, angular_vel, angular_time])
            else:
                each_control_info.append([0, -angular_vel, angular_time])
            control_info.append(each_control_info)
        return control_info

    def bresenham(self, x1,y1,x2,y2):
        """
        this code is from https://github.com/AtsushiSakai/PythonRobotics/blob/master/Mapping/lidar_to_grid_map/lidar_to_grid_map.py
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        Bresenham's Line Algorithm
        Produces a np.array from start and end (original from roguebasin.com)
        # >>> points1 = bresenham((4, 4), (6, 10))
        # >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """
        # setup initial conditions
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        points = np.array(points)
        return points

    def get_pseudo_observation(self, x_coord, y_coord, theta_coord, map_, robot_pose):
        max_range = 5
        idx = point2index(x_coord, y_coord, map_.info)
        x = math.floor(idx / map_.info.width)
        y = int(idx % map_.info.width)
        robottheta = eular_from_quaternion(robot_pose.orientation)
        angle_min = -2.0999999046325684+math.pi/2+theta_coord
        angle_max = 2.0999999046325684+math.pi/2+theta_coord
        range_max = max_range/0.05
        sensor_num = 10
        angle_step = (int(angle_max* 180 / math.pi) - int(angle_min * 180 / math.pi)) / sensor_num
        array_theta = np.arange(int(angle_min* 180 / math.pi), int(angle_max * 180 / math.pi), angle_step)
        end_x = np.cos(np.radians(array_theta)) * range_max + x
        end_y = np.sin(np.radians(array_theta)) * range_max + y
        end_x = end_x.astype(int)
        end_y = end_y.astype(int)

        #ブレゼンハムアルゴリズムによって仮にセンサが最大値を取った場合の各方向でマッピングされるセルを返す
        f_bresenham = np.frompyfunc(self.bresenham, 4, 1)
        cell_list = f_bresenham(np.full(end_x.size, x, dtype=int), np.full(end_y.size, y, dtype=int), end_x, end_y)
        cell_list = f_bresenham(np.full(end_x.size, x, dtype=int), np.full(end_y.size, y, dtype=int), end_x, end_y)

        #地図から予測される観測を得る
        new_cell_list = list()
        for c in cell_list:
            free_list = list()
            for l in range(len(c)):
                x_cell = c[l][1]
                y_cell = c[l][0]
                if map_.data[y_cell*map_.info.height+x_cell] == 100: #専有されていたら
                    new_cell_list.append(free_list)
                    break
                else:
                    free_list.append(np.array(c[l]))
                if l == len(c)-1: #センサが届かなかった場合
                    new_cell_list.append(free_list)
                    break
        cell_list = new_cell_list
        observation = list()
        unknown_cell_num = 0
        for c in cell_list:
            observation.append(len(c) * map_.info.resolution * math.sqrt(2))   
            for l in range(len(c)):
                x_cell = c[l][1]
                y_cell = c[l][0]
                if map_.data[y_cell*map_.info.height+x_cell] == -1: #未知のセルであるならば
                    unknown_cell_num += 1

        return observation, array_theta, max_range, unknown_cell_num

    def motion_model(self, control_info, pose):
        nu = control_info[0]
        omega = control_info[1]
        time = control_info[2]
        if time <= 1e-100:
            return pose
        #4次元ガウス分布から動作モデルのノイズをサンプリング
        motion_noise_stds={"nn":0.05, "no":0.1, "on":0.05, "oo":0.1}
        c = np.diag([motion_noise_stds["nn"]**2, motion_noise_stds["no"]**2, motion_noise_stds["on"]**2, motion_noise_stds["oo"]**2])
        motion_noise_rate_pdf = multivariate_normal(cov=c)
        ns = motion_noise_rate_pdf.rvs()
        nu = nu + ns[0] * math.sqrt(abs(nu)/time) + ns[1] * math.sqrt(abs(omega)/time)
        omega = omega + ns[2] * math.sqrt(abs(nu)/time) + ns[3] * math.sqrt(abs(omega)/time)
 
        t0 = pose[2]
        if math.fabs(omega) < 1e-10:
            return_pose = np.array([pose[0] + nu * math.cos(t0), pose[1] + nu * math.sin(t0), pose[2] + omega]) * time  
        else:
            return_pose = np.array([pose[0] + nu / omega * ((math.sin(t0+omega*time)) - math.sin(t0)), pose[1] + nu / omega * ((-math.cos(t0+omega*time)) + math.cos(t0)), pose[2] + omega*time])
        return return_pose

    def ig_in_slam_each(self, xlist, ylist, thetalist, map_, particle_num, step, paths, gmapping_particles, robot_pose, IG_idx, IG_part):
        start_all_step_calc_slam_IG_time = time.time()
        f_num = len(xlist)  
        #距離コスト
        travel_dist = self.travel_distance_coord(paths)

        #自己位置のIG
        destinations = list()
        for i in range(f_num):
            destinations.append([xlist[i], ylist[i], thetalist[i]])
        control = self.return_control_info_coords(paths, thetalist)
        xl = list()
        yl = list()
        thetal = list()
        pose_IG_list = list()
        map_IG_list = list()
        for f in range(f_num):
            pseudo_obs = list()
            pseudo_angle = list()
            obs, angle, max_range, unknown_cell_num = self.get_pseudo_observation(xlist[f], ylist[f], thetalist[f], map_, robot_pose)
            sensor_num = len(obs)
            for s in range(sensor_num):
                pseudo_obs_pdf = multivariate_normal([obs[s], angle[s]], [[0.1, 0.0], [0.0, 0.1]]) #疑似観測をサンプリング
                pseudo_obs_angle = pseudo_obs_pdf.rvs()
                if pseudo_obs_angle[0] > max_range: #最大距離を超えていたらガウス分布の逆側に持ってくる
                    pseudo_obs_angle[0] = 2*obs[s] - pseudo_obs_angle[0]
                pseudo_obs.append(pseudo_obs_angle[0])
                pseudo_angle.append(pseudo_obs_angle[1])

            weights = list()
            particles = list()
            xl = list()
            yl = list()
            for i in range(particle_num):
                #動作モデルで移動   
                # pose = np.array([self.robot_pose.position.x, self.robot_pose.position.y, self.eular_from_quaternion(self.robot_pose.orientation)-math.pi])
                pose = np.array([gmapping_particles.poses[i].position.x, gmapping_particles.poses[i].position.y, gmapping_particles.poses[i].position.z-math.pi])
                for c in range(len(control[f])):
                    pose = self.motion_model(control[f][c], pose)
                #観測モデルを各パーティクルの尤度（重み）計算
                obs_p, angle_p, max_range, unknown_cells= self.get_pseudo_observation(pose[0], pose[1], pose[2], map_, robot_pose)
                
                range_max_thresh = 0.1
                weight = 1
                for s in range(sensor_num):
                    obs_model = multivariate_normal([obs_p[s], angle_p[s]], [[1.0, 0.0], [0.0, 1.0]])
                    weight *= obs_model.pdf([pseudo_obs[s], pseudo_angle[s]])
                weights.append(weight)
                particles.append(pose)
                xl.append(pose[0])
                yl.append(pose[1])
                thetal.append(pose[2])
            # self.visualize_particles(xl, yl, thetal, 1, [1, 0, 0])
            sum_weights = sum(weights)
            if sum_weights <= 1e-100:
                for p in range(particle_num):
                    weights[p] = weights[p] + 1/particle_num
                p_idxes = np.random.choice(particle_num, particle_num, p=weights)
            else:
                #重みの正規化
                weights = weights / sum_weights
                p_idxes = np.random.choice(particle_num, particle_num, p=weights)
            weights = weights * particle_num
            xl = list()
            yl = list()
            sinl = list()
            cosl = list()
            for p in range(particle_num):
                xl.append(particles[p_idxes[p]][0])
                yl.append(particles[p_idxes[p]][1])
                sinl.append(math.sin(particles[p_idxes[p]][2]))
                cosl.append(math.cos(particles[p_idxes[p]][2]))
            mean = [0, 0, 0, 0]
            for p in range(particle_num):
                mean[0] += particles[p_idxes[p]][0]
                mean[1] += particles[p_idxes[p]][1]
                mean[2] += math.sin(particles[p_idxes[p]][2])
                mean[3] += math.cos(particles[p_idxes[p]][2])
            mean[0] = mean[0] / particle_num
            mean[1] = mean[1] / particle_num
            mean[2] = mean[2] / particle_num
            mean[3] = mean[3] / particle_num
            variance = np.zeros((4, 4))
            for p in range(particle_num):
                np_diff = np.array([particles[p_idxes[p]][0] - mean[0],particles[p_idxes[p]][1] - mean[1],math.sin(particles[p_idxes[p]][2]) - mean[2],math.cos(particles[p_idxes[p]][2]) - mean[3]])
                v = [[np_diff[0]**2,         np_diff[0]*np_diff[1], np_diff[0]*np_diff[2], np_diff[0]*np_diff[3]], 
                     [np_diff[1]*np_diff[0], np_diff[1]**2,         np_diff[1]*np_diff[2], np_diff[1]*np_diff[3]], 
                     [np_diff[2]*np_diff[0], np_diff[2]*np_diff[1], np_diff[2]**2        , np_diff[2]*np_diff[3]],
                     [np_diff[3]*np_diff[0], np_diff[3]*np_diff[1], np_diff[3]*np_diff[2], np_diff[3]**2]]
                variance += v

            variance = variance / particle_num
            dest_particle_sigma = np.array(variance)

            #ロボットの自己位置のエントロピー
            g_particles = list()
            g_particle_num = len(gmapping_particles.poses)
            for p in range(g_particle_num):
                g_pose = gmapping_particles.poses[0].position
                g_particles.append([g_pose.x, g_pose.y, g_pose.z])
            mean = [0, 0, 0, 0]
            for p in range(g_particle_num):
                mean[0] += g_particles[p][0]
                mean[1] += g_particles[p][1]
                mean[2] += math.sin(g_particles[p][2])
                mean[3] += math.cos(g_particles[p][2])
            mean[0] = mean[0] / g_particle_num
            mean[1] = mean[1] / g_particle_num
            mean[2] = mean[2] / g_particle_num
            mean[3] = mean[3] / g_particle_num
            variance = np.zeros((4, 4))
            for p in range(g_particle_num):
                np_diff = np.array([g_particles[p][0] - mean[0],g_particles[p][1] - mean[1],math.sin(g_particles[p][2]) - mean[2],math.cos(g_particles[p][2]) - mean[3]])
                v = [[np_diff[0]**2,         np_diff[0]*np_diff[1], np_diff[0]*np_diff[2], np_diff[0]*np_diff[3]], 
                     [np_diff[1]*np_diff[0], np_diff[1]**2,         np_diff[1]*np_diff[2], np_diff[1]*np_diff[3]], 
                     [np_diff[2]*np_diff[0], np_diff[2]*np_diff[1], np_diff[2]**2        , np_diff[2]*np_diff[3]],
                     [np_diff[3]*np_diff[0], np_diff[3]*np_diff[1], np_diff[3]*np_diff[2], np_diff[3]**2]]
                variance += v

            variance = variance / g_particle_num
            self_particle_sigma = np.array(variance)  
            map_coefficient = 1
            if np.linalg.det(dest_particle_sigma) <= 1e-100:
                IG_pose = 0
            else:
                #Probabilistic robotic p.535
                #自己位置のパーティクルのエントロピーは0とする
                dist = travel_dist[f]
                if travel_dist[f] <= 1e-100:
                    dist = math.sqrt((xlist[f] - robot_pose.position.x)**2 + (ylist[f] - robot_pose.position.y)**2)

                if np.linalg.det(self_particle_sigma) <= 1e-100: 
                    IG_pose = ( 0 - math.log( np.linalg.det(dest_particle_sigma) ) ) / dist
                else:
                    IG_pose = math.log( np.linalg.det(self_particle_sigma) ) -( math.log( np.linalg.det(dest_particle_sigma) ) )  / dist
            IG_map = map_coefficient*unknown_cell_num
            IG_part.append([IG_pose, IG_map, IG_idx])

    def nearest_free(self, info, map_, initx, inity):
        robot_idx = point2index(initx,inity, info)
        is_free = False
        x_idx = int(robot_idx % info.height)
        y_idx = int(robot_idx / info.height)
        around = [-1, 0, 1]
        # around_ = [-3, -2, -1, 0, 1, 2, 3]
        around_ = [-1, 0, 1]
        unknown_idx = 0
        while is_free == False:
            require_free_num = len(around) * len(around)
            
            for y_diff in around:
                for x_diff in around:
                    # if ((x_diff == 0) and (y_diff == 0)) != False:
                    target_x = x_idx + x_diff
                    target_y = y_idx + y_diff
                    target_idx = target_y*info.height + target_x
                    if map_[target_idx] == 0:  #周りにfreeがあるとき
                        free_num = 0
                        for y_diff_ in around_:
                            for x_diff_ in around_:
                                if map_[int((target_y+y_diff_)*info.height + (target_x+x_diff_))] == 0:
                                    free_num += 1
                        if free_num == len(around_)*len(around_):
                            is_free = True
                            unknown_idx = target_idx
            around_max = around[-1]
            around_min = around[0]
            around.insert(0, around_min-1)
            around.append(around_max+1)
        x, y = index2point(unknown_idx, info)
        return x, y

    def return_path(self, xlist, ylist, info, map_, visualize_path, step, robot_pose):
        start_all_step_path_planning_time = time.time()
        freex, freey = self.nearest_free(info, map_, robot_pose.position.x, robot_pose.position.y)
        frontier_num = len(xlist)
        paths = list()
        for f in range(frontier_num):
            free_destx, free_desty = self.nearest_free(info, map_, xlist[f], ylist[f])
            msg = geometry_msgs.PoseStamped()
            # msg.head
            msg.header.frame_id = 'map'
            msg.pose.position.x = free_destx
            msg.pose.position.y = free_desty
            msg.pose.position.z = 0
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 0

            # msg.pose.orientation = map_to_robot.transform.rotation
            self.path_target_pub.publish(msg)

            msg = geometry_msgs.PoseWithCovarianceStamped()
            # msg.head
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = freex
            msg.pose.pose.position.y = freey
            msg.pose.pose.position.z = 0
            msg.pose.pose.orientation.x = 0
            msg.pose.pose.orientation.y = 0
            msg.pose.pose.orientation.z = 0
            msg.pose.pose.orientation.w = 0
            self.path_initial_pub.publish(msg)          
            rospy.sleep(0.2)
            paths.append(self.path_from_astar)
        self.all_step_path_planning_time = time.time() - start_all_step_path_planning_time
        return paths

    def ig_in_slam_multi_p(self, xlist, ylist, thetalist, map_, particle_num, step, robot_pose):
        rospy.loginfo('[Active-SpCoSLAM]         Calculating IG about SLAM')
        paths = self.return_path(xlist, ylist, map_.info, map_.data, False, step, robot_pose)
        start_all_step_calc_slam_IG_time = time.time()
        frontier_num = len(xlist)
        core_num = len(xlist)
        iteration = int(frontier_num / core_num)
        surplus = int(frontier_num % core_num)
        pose_IG_list = list()
        map_IG_list = list()
        # プロセスを10個生成
        for i in tqdm.tqdm(range(iteration)):
            # プロセスを管理する人。デラックスな共有メモリ
            manager = multiprocessing.Manager()
            # マネージャーからリストオブジェクトを取得
            IG_part = manager.list()
            # あとでまとめてjoin()するためのプロセスのリスト
            processes = []
            for f in range(core_num):
                # マネージャーから取得したオブジェクトを引数に渡す
                process = multiprocessing.Process(target=self.ig_in_slam_each, args=([xlist[i*core_num + f]], [ylist[i*core_num + f]], [thetalist[i*core_num + f]], map_, particle_num, step, [paths[i*core_num + f]], self.gmapping_particles, robot_pose, i*core_num + f, IG_part))
                # プロセス開始
                process.start()
                # プロセスのリストに追加
                processes.append(process)

            # プロセスのリストでループ
            for p in processes:
                # プロセスの終了待ち
                p.join()
            for p in processes:
                p.terminate()
            for p in range(len(IG_part)):
                for idx_in_list in range(len(IG_part)):
                    if IG_part[idx_in_list][2] == p:
                        target_idx = idx_in_list
                pose_IG_list.append(IG_part[target_idx][0])
                map_IG_list.append(IG_part[target_idx][1])

        # プロセスを管理する人。デラックスな共有メモリ
        manager_surplus = multiprocessing.Manager()
        # マネージャーからリストオブジェクトを取得
        IG_part_surplus = manager_surplus.list()
        # あとでまとめてjoin()するためのプロセスのリスト
        processes = []
        for f in range(surplus):
            # マネージャーから取得したオブジェクトを引数に渡す
            process = multiprocessing.Process(target=self.ig_in_slam_each, args=([xlist[iteration*core_num + f]], [ylist[iteration*core_num + f]], [thetalist[iteration*core_num + f]], map_, particle_num, step, [paths[iteration*core_num + f]], self.gmapping_particles, self.robot_pose, IG_part_surplus))
            # プロセス開始
            process.start()
            # プロセスのリストに追加
            processes.append(process)

        # プロセスのリストでループ
        for p in processes:
            # プロセスの終了待ち
            p.join()
        for p in range(int(len(IG_part_surplus)/2)):
            pose_IG_list.append(IG_part_surplus[2*p])   
            map_IG_list.append(IG_part_surplus[2*p+1])  
        print(len(pose_IG_list), len(map_IG_list), len(xlist))

        IG_list = list()
        for p in range(frontier_num):
            IG_list.append(pose_IG_coef*pose_IG_list[p] + map_IG_coef*map_IG_list[p])
        all_step_calc_slam_IG_time = time.time() - start_all_step_calc_slam_IG_time

        return IG_list, map_IG_list, pose_IG_list, all_step_calc_slam_IG_time

    def ig_in_spcoae_sequential_each(self, step, x, y, theta, map_, particle_num, pseudo_observation, word_dic, particle, count_in, count_Cn, count_Sn, count_fn, IG_idx, IG_list):
        #パーティクル毎に擬似観測（単語）をサンプリング###########
        #単語擬似観測の確率値
        word_prob_sample       = np.zeros((particle_num,len(word_dic)))
        #単語疑似観測のサンプリング数
        word_sample_count = np.zeros((particle_num,len(word_dic)))

        #画像特徴量擬似観測の確率値
        feature_prob_sample       = np.zeros((particle_num,365))
        #画像特徴量疑似観測のサンプリング数
        feature_sample_count = np.zeros((particle_num,365))

        target_pose = geometry_msgs.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = 0
        for r in range(particle_num):
            #各単語に対しての確率値を得て，確率分布を構成する．
            prob_xnCnin = prob_xn(r, target_pose, particle) * prob_Cnin(step, r, count_in, count_Cn)

            word_index = 0
            for word in word_dic.keys():
                word_prob_sample[r][word_index] = np.sum(prob_xnCnin * prob_Sn(r, word, count_Sn, word_dic))
                word_index += 1
            word_prob_sample[r]       = normalization(word_prob_sample[r])
            #上で構成した確率分布に基づいたサンプリング数単語をサンプリング
            word_sample_count[r] = np.random.multinomial(n=pseudo_observation, pvals=word_prob_sample[r]) 

            #疑似観測したSaの確率分布計算
            tmp_prob_Sn = np.full((L), 1, dtype='float')
            word_index = 0
            for word in word_dic.keys():
                if word_sample_count[r][word_index] != 0:
                    tmp_prob_Sn *= word_sample_count[r][word_index] * prob_Sn(r, word, count_Sn, word_dic)[:,0]
                word_index += 1
            prob_Sn_ = tmp_prob_Sn.reshape([L,1])

            #疑似観測したSaを元にfaをサンプリング
            for feature in range(365):
                feature_prob_sample[r][feature] = np.sum(prob_xnCnin * prob_Sn_ * prob_fn_each(r, feature, count_fn))
                word_index += 1
            
            feature_prob_sample[r]       = normalization(feature_prob_sample[r])
            #上で構成した確率分布に基づいたサンプリング数
            feature_sample_count[r] = np.random.multinomial(n=pseudo_observation, pvals=feature_prob_sample[r]) 

        #IGの計算##################################
        feature_log = np.log2(feature_prob_sample / (np.sum(feature_prob_sample, axis=0) / particle_num))
        IG = (np.sum(np.sum(feature_sample_count * feature_log, axis=1) / particle_num))
        IG_list.append([IG, IG_idx])

    def ig_in_spcoae_sequential_multi_p(self, step, xlist, ylist, theta_list, map_, particle_num, pseudo_observation, word_dic, particle, count_in, count_Cn, count_Sn, count_fn):
        start_all_step_calc_spco_IG_time = time.time()
        frontier_num = len(xlist)
        core_num = len(xlist)
        iteration = int(frontier_num / core_num)
        surplus = int(frontier_num % core_num)
        IG_list = list()
        # プロセスを10個生成
        for i in tqdm.tqdm(range(iteration)):
            # プロセスを管理する人。デラックスな共有メモリ
            manager = multiprocessing.Manager()
            # マネージャーからリストオブジェクトを取得
            IG_part = manager.list()
            # あとでまとめてjoin()するためのプロセスのリスト
            processes = []
            for f in range(core_num):
                # マネージャーから取得したオブジェクトを引数に渡す
                #ここから！！！！！！！！！！
                process = multiprocessing.Process(target=self.ig_in_spcoae_sequential_each, args=(step, xlist[i*core_num + f], ylist[i*core_num + f], theta_list[i*core_num + f], map_, particle_num, pseudo_observation, word_dic, particle, count_in, count_Cn, count_Sn, count_fn, i*core_num + f, IG_part))
                # プロセス開始
                process.start()
                # プロセスのリストに追加
                processes.append(process)

            # プロセスのリストでループ
            for p in processes:
                # プロセスの終了待ち
                p.join()
            for p in processes:
                p.terminate()
            for p in range(len(IG_part)):
                for idx_in_list in range(len(IG_part)):
                    if IG_part[idx_in_list][1] == p:
                        target_idx = idx_in_list
                IG_list.append(IG_part[target_idx][0])

        # プロセスを管理する人。デラックスな共有メモリ
        manager_surplus = multiprocessing.Manager()
        # マネージャーからリストオブジェクトを取得
        IG_part_surplus = manager_surplus.list()
        # あとでまとめてjoin()するためのプロセスのリスト
        processes = []
        for f in range(surplus):
            # マネージャーから取得したオブジェクトを引数に渡す
            process = multiprocessing.Process(target=self.ig_in_spcoae_sequential_each, args=(step, xlist[iteration*core_num + f], ylist[iteration*core_num + f], theta_list[iteration*core_num + f], map_, particle_num, pseudo_observation, word_dic, particle, count_in, count_Cn, count_Sn, count_fn, IG_part_surplus))
            # プロセス開始
            process.start()
            # プロセスのリストに追加
            processes.append(process)

        # プロセスのリストでループ
        for p in processes:
            # プロセスの終了待ち
            p.join()
        for p in range(len(IG_part_surplus)):
            IG_list.append(IG_part_surplus[p])  

        calc_spco_IG_time = time.time() - start_all_step_calc_spco_IG_time
        
        return IG_list, calc_spco_IG_time
    
    def __init__(self):
        self.path_sub = rospy.Subscriber('/nav_path', nav_msgs.Path, self.path_callback, queue_size=1)
        self.path_target_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.PoseStamped, queue_size=1)
        self.path_initial_pub = rospy.Publisher('/initialpose', geometry_msgs.PoseWithCovarianceStamped, queue_size=1)
        self.particle_sub = rospy.Subscriber("/slam_gmapping/particle_poses", geometry_msgs.PoseArray, self.particle_callback)

