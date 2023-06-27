#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from init import *

def multivariate_t_distribution(x, mu, Sigma, df):
    '''
    Multivariate t-student density. Returns the density
    of the function at points specified by x.

    input:
        x = parameter (n-d numpy array; will be forced to 2d)
        mu = mean (d dimensional numpy array)
        Sigma = scale matrix (dxd numpy array)
        df = degrees of freedom

    Edited from: http://stackoverflow.com/a/29804411/3521179
    '''

    x = np.atleast_2d(x) # requires x as 2d
    nD = Sigma.shape[0] # dimensionality

    numerator = math.gamma(1.0 * (nD + df) / 2.0)
    denominator = (
            math.gamma(1.0 * df / 2.0) *
            np.power(df * math.pi, 1.0 * nD / 2.0) *
            np.power(np.linalg.det(Sigma), 1.0 / 2.0) *
            np.power(
                1.0 + (1.0 / df) *
                np.diagonal(
                    np.dot( np.dot(x - mu, np.linalg.inv(Sigma)), (x - mu).T)
                ),
                1.0 * (nD + df) / 2.0
                )
            )

    return 1.0 * numerator / denominator

def prob_xn(r, robot_pose, particle):
    #xnの確率分布
    x_n = np.array([robot_pose.position.x, robot_pose.position.y])
    prob_xn = np.zeros(K)
    for k in range(K):
        xk_array     = np.array(particle[r]['xk_list'][k])
        nk           = xk_array.shape[0]
        kappa_k      = nk + kappa_0
        m_k          = (np.sum(xk_array, axis=0) + kappa_0*m_0) / kappa_k
        v_k          = v_0 + nk
        V_q          = V_0 + np.sum([np.dot(np.array([xk_array[j]]).T,np.array([xk_array[j]])) for j in range(nk)], axis=0) + kappa_0*np.dot(np.array([m_0]).T,np.array([m_0])) - kappa_k*np.dot(np.array([m_k]).T,np.array([m_k]))
        #※xk_arrayは横ベクトルなので前方を転置（論文中の数式は後方）
        #master thesisの21ページ目
        #t分布の事後パラメータ計算
        dofk         = v_k - dimx + 1
        InvSigk      = (V_q*(kappa_k+1)) / (kappa_k*dofk)
        #ｔ分布の計算
        prob_xn[k]   = multivariate_t_distribution(x_n, m_k, InvSigk, dofk)

    return prob_xn

def prob_Cnin(step, r, count_in, count_Cn):
    #Cn,inの確率分布
    prob_Cnin = ((count_in[r] + (gamma_0 / K)) / ((count_Cn[r] + gamma_0).reshape([L,1]))) * (((count_Cn[r] + (alpha_0 / L)) / (np.sum(count_Cn[r]) + alpha_0)).reshape([L,1]))

    return prob_Cnin

def prob_Sn(r, word, count_Sn, word_dic): #サイズ：10（場所概念のインデックスの個数分）のリスト,　それぞれの場所概念からwordが出てくる確率を表す
    #Snの確率分布
    prob_Sn = ((count_Sn[r,0:L,word_dic[word]] + beta_0) / (np.sum(count_Sn[r],axis=1) + len(word_dic) * beta_0)).reshape([L,1])
    return prob_Sn

def prob_fn_each(r, f, count_fn): #サイズ：10（場所概念のインデックスの個数分）のリスト,　それぞれの場所概念から画像特徴量が出てくる確率を表す
    #fnの確率分布
    feature_thresh = 0.001
    prob_fn = np.full(L, 1)        
    prob_fn = ((count_fn[r,0:L, f] + chi_0) / (np.sum(count_fn[r],axis=1) + 365 * chi_0)).reshape([L,1])

    return prob_fn 

def prob_xnSnfnCnin(step, r, sentence, features, robot_pose, stop_words, count_Cn, count_in, count_fn, count_Sn, particle, word_dic):
    Noun_list = ['NN', 'NNS']
    Verb_list = ['VBD', 'VBG', 'VBN', 'VBP', 'VBZ']
    #すべての単語に対してProb_Snを掛ける
    tmp_prob_Sn = np.full((L), 1, dtype='float')
    for word in sentence.split():
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
            
            if (word_list[w] in stop_words) == False:
                for l in range(L):
                    tmp_prob_Sn[l] *= prob_Sn(r, word_list[w], count_Sn, word_dic)[l][0]
    prob_Sn_ = tmp_prob_Sn.reshape([L, 1])

    tmp_prob_fn = np.full((L), 1, dtype='float')
    for f in range(365):
        for l in range(L):
            tmp_prob_fn[l] *= prob_fn_each(r, f, count_fn)[l][0]
        tmp_prob_fn = normalization(tmp_prob_fn)
    prob_fn = tmp_prob_fn.reshape([L, 1])
    prob_xnSnfnCnin = prob_xn(r, robot_pose, particle) * prob_Sn_ * prob_fn * prob_Cnin(step, r, count_in, count_Cn)

    return prob_xnSnfnCnin