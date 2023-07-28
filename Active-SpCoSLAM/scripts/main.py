#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from init import *
from save_data import *
from robot_behavior import *
from captioning import *
from spatial_concept import *
from information_gain import *

class Active_SpCoSLAM():
    def run(self):
        start_time = time.time()
        start_all_step_time = time.time()

        rospy.loginfo('[Active-SpCoSLAM]         ' + "0th step learning start")

        #Save map and capture images
        save_map(0, self.data_path)
        for angle_idx in range(angle_num):
            save_image(0, angle_idx, self.image_data, self.data_path)
            move_to_destination(self.robot_pose.position.x, self.robot_pose.position.y, angle_idx*(math.pi/2), 0, self.move_base_act)
        self.sentences.append(get_captioning_result(0, self.data_path))

        #learn spatial concept in the initial position
        max_weight_index, self.explored_point_x, self.explored_point_y, self.count_Cn, self.count_Sn, self.count_in, self.count_fn, self.particle, self.weight, self.all_step_particle, self.all_step_weight, self.C_list, self.i_list, self.word_dic, spatial_concept_learning_time = learn_spatial_concept(0, self.sentences, self.robot_pose, self.data_path, self.stop_words, self.map_data.info, self.explored_point_x, self.explored_point_y, self.count_Cn, self.count_Sn, self.count_in, self.count_fn, self.particle, self.weight, self.all_step_particle, self.all_step_weight, self.word_dic, self.C_list, self.i_list)
        self.all_step_spatial_concept_learning_time[0] = spatial_concept_learning_time

        #Save data for evaluation
        save_data_for_time(0, self.all_step_time, self.all_step_calc_slam_IG_time, self.all_step_calc_spco_IG_time, self.all_step_path_planning_time, self.all_step_moving_time, self.all_step_spatial_concept_learning_time, self.data_path)
        save_data_for_ARI(0, self.explored_point_x, self.explored_point_x, self.C_list, self.i_list, self.all_step_particle, self.all_step_weight, self.data_path)
        save_data_for_draw_result(0, self.all_step_weight, self.all_step_particle, self.data_path)
        save_data_for_all_particle(0, particle_num, self.all_step_particle, self.data_path)
        save_data_for_sentence(0, self.sentences, self.data_path)
        self.all_step_time[0] = time.time() - start_all_step_time
        self.map_num += 1

        for step in range(1, learning_times, 1):
            start_all_step_time = time.time()

            rospy.loginfo('[Active-SpCoSLAM]         ' + str(step) + "th step learning start")

            #Calculate candidate search points
            occupancy_map = self.map_data.data
            occupancy_info = self.map_data.info
            frontier_candidate = return_frontier_candidate(occupancy_map, occupancy_info)
            grouped_indexes = divide_candidate_into_groups(frontier_candidate, occupancy_info)
            frontier_x_list, frontier_y_list = decide_frontier(grouped_indexes, occupancy_info)
            frontier_x_list, frontier_y_list, frontier_theta_list = sort_frontier(frontier_x_list, frontier_y_list, occupancy_map, occupancy_info)
            grid_x_list, grid_y_list, grid_theta_list = return_grid_points(occupancy_map, occupancy_info)
            x_list = frontier_x_list + grid_x_list
            y_list = frontier_y_list + grid_y_list
            theta_list = frontier_theta_list + grid_theta_list

            #Calculate information gain (IG) in each candidate search points
            IG_class = Information_Gain()
            slam_IG_list, map_IG_list, pose_IG_list, calc_slam_IG_time = IG_class.ig_in_slam_multi_p(x_list, y_list, theta_list, self.map_data, 30, step, self.robot_pose)
            spcoae_IG_list, calc_spcoae_IG_time = IG_class.ig_in_spcoae_sequential_multi_p(step, x_list, y_list, theta_list, self.map_data, particle_num, 10, self.word_dic, self.particle, self.count_in, self.count_Cn, self.count_Sn, self.count_fn)
            self.all_step_pose_ig.append(pose_IG_list)
            self.all_step_map_ig.append(map_IG_list)
            self.all_step_spcoae_ig.append(spcoae_IG_list)
            self.all_step_calc_spco_IG_time[step-1] =  calc_spcoae_IG_time
            self.all_step_calc_slam_IG_time[step-1] =  calc_slam_IG_time

            #Active exploration based on IG maximization
            IG_list = list()
            for ig in range(len(x_list)):
                IG_list.append(slam_IG_list[ig]+spcoae_IG_list[ig])
            max_idx = np.argmax(IG_list)
            move_status, moving_time = move_to_destination(x_list[max_idx], y_list[max_idx], 0, step, self.move_base_act)

            #Save map and capture images
            save_map(step, self.data_path)
            for angle_idx in range(angle_num):
                save_image(step, angle_idx, self.image_data, self.data_path)
                move_to_destination(self.robot_pose.position.x, self.robot_pose.position.y, angle_idx*(math.pi/2), step, self.move_base_act)
            self.sentences.append(get_captioning_result(step, self.data_path))

            #Learn spatial concept
            max_weight_index, self.explored_point_x, self.explored_point_y, self.count_Cn, self.count_Sn, self.count_in, self.count_fn, self.particle, self.weight, self.all_step_particle, self.all_step_weight, self.C_list, self.i_list, self.word_dic, spatial_concept_learning_time = learn_spatial_concept(step, self.sentences, self.robot_pose, self.data_path, self.stop_words, self.map_data.info, self.explored_point_x, self.explored_point_y, self.count_Cn, self.count_Sn, self.count_in, self.count_fn, self.particle, self.weight, self.all_step_particle, self.all_step_weight, self.word_dic, self.C_list, self.i_list)
            self.all_step_spatial_concept_learning_time[step] = spatial_concept_learning_time

            #Save data for evaluation
            save_data_for_time(step, self.all_step_time, self.all_step_calc_slam_IG_time, self.all_step_calc_spco_IG_time, self.all_step_path_planning_time, self.all_step_moving_time, self.all_step_spatial_concept_learning_time, self.data_path)
            save_data_for_ARI(step, self.explored_point_x, self.explored_point_x, self.C_list, self.i_list, self.all_step_particle, self.all_step_weight, self.data_path)
            save_data_for_draw_result(step, self.all_step_weight, self.all_step_particle, self.data_path)
            save_data_for_all_particle(step, particle_num, self.all_step_particle, self.data_path)
            save_data_for_sentence(step, self.sentences, self.data_path)
            save_data_for_IG(step-1, self.all_step_pose_ig, self.all_step_map_ig, self.all_step_spcoae_ig, self.data_path)
            self.all_step_time[step] = time.time() - start_all_step_time     

            #Terminate execution
            if step == learning_times-1:
                passed_time = time.time() - start_time
                rospy.loginfo('[Active-SpCoSLAM/main]         Passed time' + str(passed_time))
                rospy.signal_shutdown('Done')

            self.map_num += 1

    #Callback functions for ROS topic
    # def path_callback(self, msg):
    #     self.path_from_astar = list()
    #     for p in range(len(msg.poses)):
    #         self.path_from_astar.append([msg.poses[p].pose.position.x, msg.poses[p].pose.position.y, self.eular_from_quaternion(msg.poses[p].pose.orientation)-pi])

    def particle_callback(self, data):
        self.gmapping_particles = data

    def pose_callback(self, msg):  
        self.robot_pose = msg.pose

    def map_callback(self, msg): 
        self.map_data = msg

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.image_data = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except:
            pass

    def __init__(self): 
        #Define type of ROS topic
        self.map_data = nav_msgs.OccupancyGrid()
        self.robot_pose = geometry_msgs.Pose()
        self.gmapping_particles = geometry_msgs.PoseArray()
        self.image_data = sensor_msgs.Image()
        self.path_from_astar = list()

        #Prepare for ROS topic
        self.twist_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.Twist , queue_size=1)
        self.point_pub = rospy.Publisher("point_pub", visualization_msgs.MarkerArray, queue_size = 1)
        self.ig_pub = rospy.Publisher("information_gain_pub", visualization_msgs.MarkerArray, queue_size = 1)
        self.particle_pub = rospy.Publisher("particle_pub", visualization_msgs.MarkerArray, queue_size = 1)
        self.words_pub = rospy.Publisher("words_pub", visualization_msgs.MarkerArray, queue_size = 1)
        self.path_pub = rospy.Publisher("path_pub", sensor_msgs.PointCloud, queue_size = 1)
        self.obs_pub = rospy.Publisher("observation_pub", visualization_msgs.MarkerArray, queue_size = 1)
        self.path_target_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.PoseStamped, queue_size=1)
        self.path_initial_pub = rospy.Publisher('/initialpose', geometry_msgs.PoseWithCovarianceStamped, queue_size=1)

        self.map_sub = rospy.Subscriber('/map', nav_msgs.OccupancyGrid, self.map_callback, queue_size=1)  
        self.pose_sub = rospy.Subscriber('/global_pose', geometry_msgs.PoseStamped, self.pose_callback)
        self.particle_sub = rospy.Subscriber("/slam_gmapping/particle_poses", geometry_msgs.PoseArray, self.particle_callback)
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw/compressed", sensor_msgs.CompressedImage, self.image_callback)
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_color/compressed", sensor_msgs.CompressedImage, self.image_callback)       
        # self.path_sub = rospy.Subscriber('/nav_path', nav_msgs.Path, self.path_callback, queue_size=1)

        self.move_base_act   = actionlib.SimpleActionClient('/move_base/move', move_base_msgs.MoveBaseAction)

        #Prepare for natural language processing
        nltk.download('averaged_perceptron_tagger')
        nltk.download('stopwords')
        self.stop_words = nltk.corpus.stopwords.words('english')
        self.word_dic = dict() #Word dictionary
        self.sentences = list() #Save all step sentence list

        #Prepare variables for ClipCap
        self.CUDA = get_device

        #Prepare variables for GMapping
        self.gmapping_particle_num = 30
        self.map_num = 0

        #Prepare variables for spatial concept
        self.count_Cn    = np.zeros((particle_num,L),dtype=np.int64)
        self.count_in    = np.zeros((particle_num,L,K),dtype=np.int64)
        self.count_fn    = np.zeros((particle_num,L,365),dtype=np.float64)
        self.count_Sn    = np.zeros((particle_num,L,1),dtype=np.int64)
        self.weight      = np.full(particle_num, 1.0/particle_num)
        
        #Initialize particles for spatial concept
        #{particle ID:{'before_id':Particle ID of previous step, 'C':[Index of spatial concept], 'i':[Index of position distribution], 'theta':theta, 'weight':[Weight of particles], 'xk_list':[[], [], []]}, ..., particle ID{}}
        self.particle = dict()
        for r in range(particle_num):
            self.particle[r] = {'before_id':None, 'C':list(), 'i':list(), 'theta':{'pi':None,'W_l':None,'phi_l':None,'theta_l':None,'mu_k':None,'sigma_k':None}, 'xk_list':[[] for k in range(K)]}

        #Save all step particles of spatial concept
        self.all_step_particle   = dict()
        for step in range(learning_times):
            self.all_step_particle[step] = dict()
            for r in range(particle_num):
                self.all_step_particle[step][r] = {'before_id':None, 'C':list(), 'i':list(), 'theta':{'pi':None,'W_l':None,'phi_l':None,'theta_l':None,'mu_k':None,'sigma_k':None}, 'xk_list':[[] for k in range(K)]}
        self.all_step_weight     = np.zeros((learning_times,particle_num))

        #Make directory for saving data
        self.data_path = roslib.packages.get_pkg_dir('Active-SpCoSLAM') + '/data/' + str('{0:%m%d}'.format(datetime.now())) + '/' + learning_mode + '/' + str(int('{0:%H}'.format(datetime.now())) + 9) + str('{0:%M}'.format(datetime.now()))
        subprocess.Popen('rm -rf ' + self.data_path, shell=True)
        self.C_path = self.data_path + '/C_and_position_distribution'
        self.i_path = self.data_path + '/i_and_position_distribution'
        subprocess.Popen('mkdir -p ' + self.C_path, shell=True)
        subprocess.Popen('mkdir -p ' + self.i_path, shell=True)
        self.image_path = self.data_path + '/images'
        subprocess.Popen('mkdir -p ' + self.image_path, shell=True)
        self.map_path = self.data_path + '/maps'
        subprocess.Popen('mkdir -p ' + self.map_path, shell=True)
        rospy.loginfo('[Server em_spcoae/main]         Data path: %s', self.data_path)

        #Prepare variables for save data
        self.explored_point_x = list()
        self.explored_point_y = list()
        self.C_list = list()
        self.i_list = list()
        self.all_step_pose_ig = list()
        self.all_step_map_ig = list()
        self.all_step_spcoae_ig = list()
        self.all_step_spatial_concept_learning_time = np.zeros(learning_times) #Time for learning spatial concept
        self.all_step_calc_spco_IG_time = np.zeros(learning_times) #Time for calculating IG of spatial concept
        self.all_step_calc_slam_IG_time = np.zeros(learning_times) #Time for calculating IG of SLAM
        self.all_step_path_planning_time = np.zeros(learning_times) #Time for path planning by A* algorithm
        self.all_step_moving_time = np.zeros(learning_times) #Time for movement
        self.all_step_time = np.zeros(learning_times) #Time from calculating IG to learn spatial concept
        save_data_preparation(self.data_path)

        rospy.loginfo('[Active-SpCoSLAM]         Initialization completed')

        #Learning of Active-SpCoSLAM
        self.run()

if __name__ == '__main__':
    rospy.init_node('Active_SpCoSLAM')
    hoge = Active_SpCoSLAM()
