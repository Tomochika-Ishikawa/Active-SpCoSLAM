#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from init import *

def draw_ellipse(explored_point_x, explored_point_y, xk_list, mu_k, sigma_k, data_num, index_list, data_path, C_or_i, step, map_info):
    save_path = data_path + '/' + C_or_i + '_and_position_distribution/' + str(step)

    #分散共分散行列描画
    fig  = plt.figure()
    ax   = fig.add_subplot(1,1,1)

    for k in range(K):
        #データ点が割り当てられていない位置分布は描画しない
        if len(xk_list[k]) == 0:
            # self.all_step_words[self.step_for_draw][k] = ''
            continue
        lmda, vec            = np.linalg.eig(sigma_k[k])
        el_width,el_height   = 2 * el_c * np.sqrt(lmda)
        el_angle             = np.rad2deg(np.arctan2(vec[1,0],vec[0,0]))
        el                   = Ellipse(xy=mu_k[k],width=el_width,height=el_height,angle=el_angle,color=colorlist[k],alpha=0.3)
        ax.add_patch(el)
        
    #データ点プロット
    data_num = len(index_list)
    if data_num >= 1:
        for xy in range(data_num):
            plt.plot(explored_point_x[xy], explored_point_y[xy], color=colorlist[index_list[int(xy)]], marker='o', markersize=1)

    #地図描画
    map_file_path    = data_path + '/maps/' + str(step) + '.pgm'
    map_image        = Image.open(map_file_path)
    resolution       = np.round(map_info.resolution,decimals=3)
    plt.imshow(map_image,extent=(map_info.origin.position.x,map_info.origin.position.x+map_info.height*resolution,map_info.origin.position.y,map_info.origin.position.y+map_info.width*resolution),cmap='gray')

    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim(-10,10)
    plt.ylim(-10,10)
    plt.savefig(save_path + '.png')
    plt.savefig(save_path + '.pdf')
    plt.cla()
    plt.clf()
    plt.close()

def draw_graph(max_r, step, particle, data_path, explored_point_x, explored_point_y, map_info):

    #各ステップ毎
    # if self.each_step_graph_draw:
    xk_list      = particle[max_r]['xk_list']
    mu_k_        = particle[max_r]['theta']['mu_k']
    sigma_k_     = particle[max_r]['theta']['sigma_k']
    data_num     = step + 1

    #各ステップで重み最大パーティクルのデータ点（C）と位置分布推定結果
    index_list   = particle[max_r]['C']
    
    draw_ellipse(explored_point_x, explored_point_y, xk_list, mu_k_, sigma_k_, data_num, index_list, data_path, "C", step, map_info)

    #各ステップで重み最大パーティクルのデータ点（i）と位置分布推定結果
    index_list   = particle[max_r]['i']
    draw_ellipse(explored_point_x, explored_point_y, xk_list, mu_k_, sigma_k_, data_num, index_list, data_path, "i", step, map_info)