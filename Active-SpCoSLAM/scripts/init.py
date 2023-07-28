#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#ROS
import rospy
import roslib.packages
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as visualization_msgs
import actionlib_msgs.msg as actionlib_msgs
import tf.transformations as transformations
import actionlib

#HSR specific libraries
import hsrb_interface

#Number
import numpy as np
import scipy.stats as ss
import math
import random
from sklearn.metrics.cluster import adjusted_rand_score
from sklearn import preprocessing
from scipy.stats import multivariate_normal, norm

#Visialization
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from PIL import Image
import cv2

#Data save
import copy
import subprocess
import csv
import time
import sys
from datetime import datetime

#Natural language processing
import nltk
from pattern.text.en import singularize

#ClipCap
import os
from torch import nn
import numpy as np
import torch
import torch.nn.functional as nnf
from typing import Tuple, List, Union, Optional
from transformers import GPT2Tokenizer, GPT2LMHeadModel
import clip
import glob
import skimage.io as io
import PIL.Image
from tqdm import trange
import argparse
import time

#Places
import torchvision.models as models
from torchvision import transforms as trn
from torch.autograd import Variable
from torch.nn import functional as F

#Others
import tqdm
import multiprocessing

#Variables for ClipCap
N = type(None)
V = np.array
ARRAY = np.ndarray
ARRAYS = Union[Tuple[ARRAY, ...], List[ARRAY]]
VS = Union[Tuple[V, ...], List[V]]
VN = Union[V, N]
VNS = Union[VS, N]
T = torch.Tensor
TS = Union[Tuple[T, ...], List[T]]
TN = Optional[T]
TNS = Union[Tuple[TN, ...], List[TN]]
TSN = Optional[TS]
TA = Union[T, ARRAY]
D = torch.device
CPU = torch.device('cpu')

def get_device(device_id: int) -> D:
    if not torch.cuda.is_available():
        return CPU
    device_id = min(torch.cuda.device_count() - 1, device_id)
    return torch.device(f'cuda:{device_id}')

#Information gain
pose_IG_coef = 1.4
map_IG_coef = 0.2

#Variables for spatial concept
learning_times = 2 #Iteration number of learning spatial concept
learning_mode = "Active-SpCoSLAM" #Name of method

angle_num = 4 #Number of images capturing one time

particle_num = 100 #Number of particles of spatial concept
L = 10 #Upper number of spatial concept
K = 10 #Upper number of position distribution

dimx = 2 #Dimension of xn（x,y）
alpha_0  = 1.0 
gamma_0  = 0.1 
beta_0   = 0.1 
m_0      = np.zeros(dimx)
kappa_0  = 0.001 
V_0      = np.identity(dimx)*1.5 
v_0      = 10.0 
chi_0 = 0.1 

#Color list for spatial concept
colorlist = ['red','green','blue','cyan','magenta','darkblue','orange','purple','yellowgreen','yellow','darkred']

#Probability of ellipse
el_prob  = 0.95
el_c     = np.sqrt(ss.chi2.ppf(el_prob, 2))

#Read parameters
def parameter_copy(map_file):
    _pi = np.loadtxt('../env/' + map_file + '/param_pi.csv', delimiter=',')
    _phi_l = np.loadtxt('../env/' + map_file + '/param_phi_l.csv', delimiter=',')
    _W_l = np.loadtxt('../env/' + map_file + '/param_W_l.csv', delimiter=',')
    _mu_k = np.loadtxt('../env/' + map_file + '/param_mu_k.csv', delimiter=',')
    _sigma_k = np.zeros((_phi_l.shape[1],2,2))
    for k in range(_phi_l.shape[1]):
        _sigma_k[k] = np.loadtxt('../env/' + map_file + '/param_sigma_k_' + str(k) + '.csv', delimiter=',')
    
    sentence_list = list()
    fp = open('../env/' + map_file + '/param_sentence.csv', 'r')
    # fp = open('../env/' + map_file + '/param_sentence_test.csv', 'r')
    fp_list = csv.reader(fp, delimiter=',')
    header = next(fp_list)
    sentence_list.append(header)
    for row in fp_list:
        sentence_list.append(row)
    return _pi, _phi_l, _W_l, _mu_k, _sigma_k, sentence_list

#Index of the position → Coordinate of the position
def index2point(index, info):
    resolution = np.round(info.resolution,decimals=3)
    array_x = int(index % info.height)
    array_y = int(index / info.height)
    x = (array_x * resolution) + info.origin.position.x
    y = (array_y * resolution) + info.origin.position.y
    return x, y

#Coordinate of the position → Index of the position
def point2index(x, y,info):
    resolution   = np.round(info.resolution,decimals=3)
    array_x      = int((x - info.origin.position.x) / resolution)
    array_y      = int((y - info.origin.position.y) / resolution)
    index        = array_y * info.height + array_x
    return index

#Convert Quaternion from Euler
def Quaternion_from_euler(orientation_z):
    radian = 0.0
    radian = orientation_z*(math.pi/180.0)
    q = transformations.quaternion_from_euler(0., 0., radian)
    return geometry_msgs.Quaternion(q[0], q[1], q[2], q[3])

#Convert Euler from Quaternion
def eular_from_quaternion(orientation):
    theta = np.arccos(orientation.w)
    theta = theta*2
    return theta

#Normalization for probability
def normalization(prob):
    return_prob = prob / np.sum(prob)
    return return_prob
