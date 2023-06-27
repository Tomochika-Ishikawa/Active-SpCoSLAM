#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from init import *

def save_data_preparation(path):
    #ARI計算用のデータを保存
    fp = open(path + '/data_for_ARI.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_ARI = list()
    data_for_ARI.append('step')
    data_for_ARI.append('x')
    data_for_ARI.append('y')
    data_for_ARI.append('Ct')
    data_for_ARI.append('it')
    for r in range(particle_num):
        data_for_ARI.append(str(r) + "th particle C")
    for r in range(particle_num):    
        data_for_ARI.append(str(r) + "th particle i")
    for r in range(particle_num):    
        data_for_ARI.append(str(r) + "th particle weight")
    for r in range(particle_num):    
        data_for_ARI.append(str(r) + "th particle xk_list")
    csvWriter.writerow(data_for_ARI)

    #ARI計算用のデータを保存
    fp = open(path + '/data_for_IG.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_IG = list()
    data_for_IG.append('IG in pose')
    data_for_IG.append('IG in map')
    data_for_IG.append('IG in spcoae')
    csvWriter.writerow(data_for_IG)

    #ARI計算用のデータを保存
    fp = open(path + '/data_for_sentences.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_sentences = list()
    data_for_sentences.append('step')
    data_for_sentences.append('sentence')
    csvWriter.writerow(data_for_sentences)

    #ARI計算用のデータを保存
    fp = open(path + '/data_for_entropy.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_sentences = list()
    data_for_sentences.append('step')
    data_for_sentences.append('entropy')
    csvWriter.writerow(data_for_sentences)

    #ARI計算用のデータを保存
    fp = open(path + '/data_for_time.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_time = list()
    data_for_time.append('step')
    data_for_time.append('all')
    data_for_time.append('calc ig of slam')
    data_for_time.append('calc ig of spco')
    data_for_time.append('path planning')
    data_for_time.append('move')
    data_for_time.append('spatial concept learning')
    csvWriter.writerow(data_for_time)

    #ARI計算用のデータを保存
    fp = open(path + '/data_for_prob_Si.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_time = list()
    data_for_time.append('step')
    data_for_time.append('words')
    for k in range(K):
        data_for_time.append('prob S given ' + str(k) + 'th i')
    csvWriter.writerow(data_for_time)

def save_data_for_ARI(step, xlist, ylist, Ct_list, it_list, all_step_particle, all_step_weight, data_path):
    fp = open(data_path + '/data_for_ARI.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    #場所概念が学習されていない場合値が0
    data_for_ARI = list()
    data_for_ARI.append(step)
    data_for_ARI.append(xlist[step])
    data_for_ARI.append(ylist[step])

    data_for_ARI.append(Ct_list[step])
    data_for_ARI.append(it_list[step])
    for r in range(particle_num):
        data_for_ARI.append(all_step_particle[step][r]["C"])
    for r in range(particle_num):    
        data_for_ARI.append(all_step_particle[step][r]["i"])
    for r in range(particle_num):    
        data_for_ARI.append(all_step_weight[step][r])
    for r in range(particle_num):    
        data_for_ARI.append(all_step_particle[step][r]["xk_list"])
    csvWriter.writerow(data_for_ARI)
    fp.close()

def save_data_for_IG(step, pose_ig, map_ig, spcoae_ig, data_path):
    fp = open(data_path + '/data_for_IG.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_IG = list()
    data_for_IG.append(pose_ig[step])
    data_for_IG.append(map_ig[step])
    data_for_IG.append(spcoae_ig[step])
    csvWriter.writerow(data_for_IG)
    fp.close()

def save_data_for_sentence(step, sentences, data_path):
    fp = open(data_path + '/data_for_sentences.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_sentences = list()
    data_for_sentences.append(step)
    data_for_sentences.append(sentences[step])
    csvWriter.writerow(data_for_sentences)
    fp.close()

def save_data_for_time(step, all_step_time, all_step_calc_slam_IG_time, all_step_calc_spco_IG_time, all_step_path_planning_time, all_step_moving_time, all_step_spatial_concept_learning_time, data_path):
    fp = open(data_path + '/data_for_time.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_time = list()
    data_for_time.append(step)
    data_for_time.append(all_step_time[step])
    data_for_time.append(all_step_calc_slam_IG_time[step])
    data_for_time.append(all_step_calc_spco_IG_time[step])
    data_for_time.append(all_step_path_planning_time[step])
    data_for_time.append(all_step_moving_time[step])
    data_for_time.append(all_step_spatial_concept_learning_time[step])
    csvWriter.writerow(data_for_time)
    fp.close()

def save_data_for_draw_result(current_step, all_step_weight, all_step_particle, data_path): 
    if current_step > 0:
        os.remove(data_path + '/data_for_draw_result.csv')
    #ARI計算用のデータを保存
    fp = open(data_path + '/data_for_draw_result.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_draw = list()
    data_for_draw.append("C")
    data_for_draw.append("i")
    data_for_draw.append("pi")
    data_for_draw.append("W_l")
    data_for_draw.append("phi_l")
    data_for_draw.append("mu_k")
    data_for_draw.append("sigma_k")
    data_for_draw.append("xk_list")
    csvWriter.writerow(data_for_draw)

    max_r = np.argmax(all_step_weight[current_step])
    for step in range(current_step, -1, -1):
        if max_r == -1: #-1のときは場所概念の学習が始まっていないときなので適当なパーティクル0番目を指定
            max_r = 0
        data_for_draw = list()
        data_for_draw.append(all_step_particle[step][max_r]["C"])
        data_for_draw.append(all_step_particle[step][max_r]["i"])
        data_for_draw.append(all_step_particle[step][max_r]["theta"]['pi'])
        data_for_draw.append(all_step_particle[step][max_r]["theta"]['W_l'])
        data_for_draw.append(all_step_particle[step][max_r]["theta"]['phi_l'])
        data_for_draw.append(all_step_particle[step][max_r]["theta"]['mu_k'])
        data_for_draw.append(all_step_particle[step][max_r]["theta"]['sigma_k'])
        data_for_draw.append(all_step_particle[step][max_r]["xk_list"])
        csvWriter.writerow(data_for_draw)
        max_r = all_step_particle[step][max_r]["before_id"]
    fp.close()

# {'before_id':None, 'C':list(), 'i':list(), 'theta':{'pi':None,'W_l':None,'phi_l':None,'mu_k':None,'sigma_k':None}, 'xk_list':[[] for k in range(K)]}
def save_data_for_all_particle(current_step, particle_num, all_step_particle, data_path): 
    if current_step != 0:
        os.remove(data_path + '/data_for_all_particle.csv')
    #ARI計算用のデータを保存
    fp = open(data_path + '/data_for_all_particle.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_draw = list()
    for p in range(particle_num):
        data_for_draw.append(str(p) + "th C")
        data_for_draw.append(str(p) + "th i")
        data_for_draw.append(str(p) + "th pi")
        data_for_draw.append(str(p) + "th W_l")
        data_for_draw.append(str(p) + "th phi_l")
        data_for_draw.append(str(p) + "th theta_l")
        data_for_draw.append(str(p) + "th mu_k")
        data_for_draw.append(str(p) + "th sigma_k")
        data_for_draw.append(str(p) + "th xk_list")
    csvWriter.writerow(data_for_draw)

    for step in range(current_step, -1, -1):
        data_for_draw = list()
        for p in range(particle_num):
            data_for_draw.append(all_step_particle[step][p]["C"])
            data_for_draw.append(all_step_particle[step][p]["i"])
            data_for_draw.append(all_step_particle[step][p]["theta"]['pi'])
            data_for_draw.append(all_step_particle[step][p]["theta"]['W_l'])
            data_for_draw.append(all_step_particle[step][p]["theta"]['phi_l'])
            data_for_draw.append(all_step_particle[step][p]["theta"]['theta_l'])
            data_for_draw.append(all_step_particle[step][p]["theta"]['mu_k'])
            data_for_draw.append(all_step_particle[step][p]["theta"]['sigma_k'])
            data_for_draw.append(all_step_particle[step][p]["xk_list"])
        csvWriter.writerow(data_for_draw)
    fp.close()

def save_data_for_IG(step, pose_ig, map_ig, spcoae_ig, data_path):
    fp = open(data_path + '/data_for_IG.csv','a')
    csvWriter = csv.writer(fp, delimiter=',')
    data_for_IG = list()
    data_for_IG.append(pose_ig[step])
    data_for_IG.append(map_ig[step])
    data_for_IG.append(spcoae_ig[step])
    csvWriter.writerow(data_for_IG)
    fp.close()