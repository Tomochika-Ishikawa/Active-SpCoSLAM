#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from init import *
from probability_distribution import *
from draw_result import *

def get_image_feature(step, data_path):
    #参考サイト : https://github.com/CSAILVision/places365/blob/master/run_placesCNN_basic.py
    image_path = data_path + "/images/"

    arch = 'resnet18'
    model_file = roslib.packages.get_pkg_dir('Active-SpCoSLAM') + "/models/model_for_places/resnet18_places365.pth.tar"

    model = models.__dict__[arch](num_classes=365)
    checkpoint = torch.load(model_file, map_location=lambda storage, loc: storage)
    state_dict = {str.replace(k,'module.',''): v for k,v in checkpoint['state_dict'].items()}
    model.load_state_dict(state_dict)
    model.eval()

    # load the image transformer
    centre_crop = trn.Compose([
            trn.Resize((256,256)),
            trn.CenterCrop(224),
            trn.ToTensor(),
            trn.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    # load the class label
    file_name = roslib.packages.get_pkg_dir('Active-SpCoSLAM') + "/models/model_for_places/categories_places365.txt"

    classes = list()
    with open(file_name) as class_file:
        for line in class_file:
            classes.append(line.strip().split(' ')[0][3:])
    classes = tuple(classes)

    file_num = 4
    image_feature = np.zeros(365)
    for f in range(file_num):
        # load the test image
        img_name = image_path + str(step) + "_" + str(f) + '.png'
        img = Image.open(img_name)
        input_img = Variable(centre_crop(img).unsqueeze(0))

        # forward pass
        logit = model.forward(input_img)
        probs = F.softmax(logit, 1).data.squeeze()

        for feat in range(365):
            image_feature[feat] += probs[feat]

    image_feature = image_feature / file_num
    return image_feature

def dic_for_BoW(sentences, word_dic, stop_words):
    Noun_list = ['NN', 'NNS']
    Verb_list = ['VBD', 'VBG', 'VBN', 'VBP', 'VBZ']

    id = len(word_dic)
    for sentence in sentences:
        for word in (sentence).split(): #<list>.splitで空白を境界としてリストを作成　例)"I am" -> ["I", "am"]
            word_list = list()
            if ',' in word: #,を排除
                word = word.replace(',', '')
            if ':' in word: #:を排除
                word = word.replace(':', '')
            if '.' in word: #.を排除
                word = word.replace('.', '')
            if '．' in word: #.を排除
                word = word.replace('．', '')
            if '-' in word:
                word_list.append(word.split('-')[0])
                word_list.append(word.split('-')[1])
            else:
                word_list.append(word)

            taglist = nltk.pos_tag(word_list)
            word_num = len(word_list)
            for w in range(word_num):
                word_list[w] = word_list[w].lower() #大文字を小文字に
                # #複数形->単数形
                if taglist[w][1] in Noun_list:
                    word_list[w] = singularize(word_list[w])
                # if taglist[w][1] in Verb_list:
                #     word_list[w] = lemmatizer.lemmatize(word_list[w], pos="v")
            
                    #単語辞書に追加
            word_num = len(word_list)
            for w in range(word_num):
                if (word_list[w] in word_dic) == False and (word_list[w] in stop_words) == False: #ストップワード排除
                    word_dic[word_list[w]] = id
                    id +=1  
    return word_dic

def sentence_to_BoW(morphemes, word_dic, stop_words):
    Noun_list = ['NN', 'NNS']
    Verb_list = ['VBD', 'VBG', 'VBN', 'VBP', 'VBZ']
    #Bowを作る
    bow_set = []     
    bow = [0] * len(word_dic) #単語の数分のリストを作成
    for word in morphemes.split():
        word_list = list()
        if ',' in word: #,を排除
            word = word.replace(',', '')
        if ':' in word: #:を排除
            word = word.replace(':', '')
        if '.' in word: #.を排除
            word = word.replace('.', '')
        if '．' in word: #.を排除
            word = word.replace('．', '')
        if '-' in word:
            word_list.append(word.split('-')[0])
            word_list.append(word.split('-')[1])
        else:
            word_list.append(word)

        taglist = nltk.pos_tag(word_list)
        word_num = len(word_list)
        for w in range(word_num):
            word_list[w] = word_list[w].lower() #大文字を小文字に
            taglist = nltk.pos_tag(word_list)
            # #複数形->単数形
            if taglist[w][1] in Noun_list:
                word_list[w] = singularize(word_list[w])
            # if taglist[w][1] in Verb_list:
            #     word_list[w] = lemmatizer.lemmatize(word_list[w], pos="v")

        try:
            if (word in stop_words) == False:
                word_num = len(word_list)
                for w in range(word_num):
                    word_id = word_dic[word_list[w]] #その単語のidを取得
                    bow[word_id] += 1 #その単語のbowの値をインクリメント
        except:
            pass
    bow_set.append(bow)
    return bow_set

def learn_spatial_concept(step, sentences, robot_pose, data_path, stop_words, map_info, explored_point_x, explored_point_y, count_Cn, count_Sn, count_in, count_fn, particle, weight, all_step_particle, all_step_weight, word_dic, C_list, i_list):
    rospy.loginfo("[Active-SpCoSLAM]         Learning spatial concept")
    start_all_step_spatial_concept_learning_time = time.time()
    pre_word_num = len(word_dic)
    word_dic = dic_for_BoW(sentences, word_dic, stop_words)
    current_word_num = len(word_dic)
    if step == 0:
        count_Sn    = np.zeros((particle_num,L,current_word_num),dtype=np.int64)
    else:
        tmp_count_Sn = np.zeros((particle_num,L,current_word_num),dtype=np.int64)
        tmp_count_Sn[:,:,0:pre_word_num] = count_Sn.copy()
        count_Sn = np.zeros((particle_num,L,current_word_num),dtype=np.int64)
        count_Sn = tmp_count_Sn

    explored_point_x.append(robot_pose.position.x)
    explored_point_y.append(robot_pose.position.y)
    x = robot_pose.position.x
    y = robot_pose.position.y

    #その探索候補点でのBag-of-Words表現を取得
    BoW = sentence_to_BoW(sentences[step], word_dic, stop_words) 
    #その探索候補点でのPlaces-CNNからの特徴量を取得
    features = get_image_feature(step, data_path)

    for f in range(365):
        features[f] = int(features[f])
    #パーティクル毎に計算(start)##################
    for r in range(particle_num):
        #Cn,inのサンプリング##########################
        prob_xnSnfnCnin_ = prob_xnSnfnCnin(step, r, sentences[step], features, robot_pose, stop_words, count_Cn, count_in, count_fn, count_Sn, particle, word_dic)
        prob_qn       = normalization(prob_xnSnfnCnin_)
        #2次元配列を１次元配列に変換．
        prob_qn_1dimx = prob_qn.reshape(1,L*K)[0]
        #Cn,inのサンプリング
        index_Cnin    = np.random.choice(a=range(L*K), size=1, p=prob_qn_1dimx)[0]
        index_Cn      = index_Cnin // K
        index_in      = index_Cnin % K

        #カウントを増やす
        count_Cn[r][index_Cn]           += 1
        count_in[r][index_Cn][index_in] += 1
        count_Sn[r][index_Cn]           += BoW[0]
        count_fn[r][index_Cn] += features
        # print(count_Sn[r][index_Cn])
        particle[r]['C'].append(index_Cn)
        particle[r]['i'].append(index_in)
        particle[r]['xk_list'][index_in].append(np.array([x, y]))
        ##########################################

        #重みの更新#################################
        #式(3.16)
        weight[r] *= np.sum(prob_xnSnfnCnin_)
        ##########################################

        #パラメータΘの期待値計算#######################
        alpha_   = alpha_0/L + count_Cn[r]
        pi       = alpha_ / (alpha_0 + np.sum(count_Cn[r]))

        beta_    = beta_0 + count_Sn[r]
        #np.sum(count_Sn[r],axis=1) : 各場所概念で出てきた単語数
        W_l      = beta_ / (beta_0 * len(word_dic) + np.sum(count_Sn[r],axis=1).reshape([L,1]))

        chi_     = chi_0 + count_fn[r]
        # #W_lと同様にthetaの処理が必要#####################
        theta_l  = chi_ / (chi_0 * 365 + np.sum(count_fn[r],axis=1).reshape([L,1]))

        gamma_   = gamma_0/K + count_in[r]
        phi_l    = gamma_ / (gamma_0 + np.sum(count_in[r],axis=1).reshape([L,1]))

        m_k_      = np.full((K,dimx), m_0)
        kappa_k_  = np.full(K, kappa_0)
        V_k_      = np.full((K,dimx,dimx), V_0)
        v_k_      = np.full(K, v_0)
        for k in range(K):
            xk_array     = np.array(particle[r]['xk_list'][k])
            nk           = xk_array.shape[0]
            if nk == 0:
                continue
            kappa_k_[k]  = nk + kappa_0
            m_k_[k]      = (np.sum(xk_array, axis=0) + kappa_0*m_0) / kappa_k_[k]
            V_k_[k]      = V_0 + np.sum([np.dot(np.array([xk_array[j]]).T,np.array([xk_array[j]])) for j in range(nk)], axis=0) + kappa_0*np.dot(np.array([m_0]).T,np.array([m_0])) - kappa_k_[k]*np.dot(np.array([m_k_[k]]).T,np.array([m_k_[k]]))
            #※xk_arrayは横ベクトルなので前方を転置（論文中の数式は後方）
            v_k_[k]      = v_0 + nk
        mu_k     = np.array([m_k_[k] for k in range(K)])
        sigma_k  = np.array([V_k_[k] / (v_k_[k] - dimx - 1) for k in range(K)])

        #パラメータΘ保存
        particle[r]['theta']['pi']      = pi
        particle[r]['theta']['W_l']     = W_l
        particle[r]['theta']['phi_l']   = phi_l
        particle[r]['theta']['theta_l'] = theta_l
        particle[r]['theta']['mu_k']    = mu_k
        particle[r]['theta']['sigma_k'] = sigma_k
        ##########################################
    #パーティクル毎に計算(finish)#################

    #重みの正規化
    weight = normalization(weight)

    for r in range(particle_num):
        all_step_particle[step][r]["before_id"] = particle[r]["before_id"]
        all_step_particle[step][r]["C"] = copy.deepcopy(particle[r]["C"])
        all_step_particle[step][r]["i"] = copy.deepcopy(particle[r]["i"])
        all_step_particle[step][r]["theta"] = particle[r]["theta"].copy()
        all_step_particle[step][r]["xk_list"] = copy.deepcopy(particle[r]["xk_list"])
    all_step_weight[step] = weight
    
    
    max_weight_index = np.argmax(weight)
    #最尤のCnとinを保存
    C_list.append(particle[max_weight_index]['C'])
    i_list.append(particle[max_weight_index]['i'])
    draw_graph(max_weight_index, step, particle, data_path, explored_point_x, explored_point_y, map_info)

    #リサンプリング###############################
    new_particle     = dict()
    new_count_Cn     = np.zeros((particle_num,L))
    new_count_in     = np.zeros((particle_num,L,K))
    new_count_Sn     = np.zeros((particle_num,L,len(word_dic)))
    new_particle_id  = np.random.choice(a=range(particle_num), size=particle_num, p=weight)
    for r in range(particle_num):
        old_particle                 = particle[new_particle_id[r]]
        new_particle[r]              = old_particle.copy()
        new_particle[r]['before_id'] = new_particle_id[r]
        new_particle[r]['C']         = copy.deepcopy(old_particle['C'])
        new_particle[r]['i']         = copy.deepcopy(old_particle['i'])
        new_particle[r]['theta']     = old_particle['theta'].copy()
        new_particle[r]['xk_list']   = copy.deepcopy(old_particle['xk_list'])
        new_count_Cn[r]              = count_Cn[new_particle_id[r]].copy()
        new_count_in[r]              = count_in[new_particle_id[r]].copy()
        new_count_Sn[r]              = count_Sn[new_particle_id[r]].copy()
    particle.update(new_particle)
    weight      = np.full(particle_num, 1.0/particle_num)
    count_Cn    = new_count_Cn.copy()
    count_in    = new_count_in.copy()
    count_Sn    = new_count_Sn.copy()
    all_step_spatial_concept_learning_time = time.time() - start_all_step_spatial_concept_learning_time
    return max_weight_index, explored_point_x, explored_point_y, count_Cn, count_Sn, count_in, count_fn, particle, weight, all_step_particle, all_step_weight, C_list, i_list, word_dic, all_step_spatial_concept_learning_time