#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import pandas as pd
import roslib.packages
import matplotlib.pyplot as plt
from physbo_ros_bridge.physbo_interface import PhysboInterface


def plot_scores():
    pkg_dir = roslib.packages.get_pkg_dir('physbo_ros_bridge')
    exp_result_dir = os.path.join(pkg_dir, 'data/test_result')  # latest files are placed here
    candidates_path = os.path.join(exp_result_dir, 'candidates.csv')

    # load policy for the order of exp
    pi = PhysboInterface(candidates_path, policy_load_dir=exp_result_dir, use_saved_policy=True)
    exp_ids = pi.chosen_actions  # define exp order. exp_id = candidate_id
    #exp_ids = [10, 14, 3, 7, 13]
    exp_num = len(exp_ids)
    exp_result_paths = []
    df_result_lists = []
    df_mean_lists = []
    df_score_lists = []

    # load candidates with scores
    for id in exp_ids:
        exp_result_paths.append(os.path.join(exp_result_dir, str(id), 'candidates_with_scores.csv'))

    # generate dataframe by merging scores
    df_compiled_means = pd.DataFrame()
    df_compiled_scores = pd.DataFrame()
    for i in range(exp_num):
        df_result_lists.append(pd.read_csv(exp_result_paths[i]))
        df_mean_lists.append(df_result_lists[i].loc[:, 'mean':'std'])
        df_score_lists.append(df_result_lists[i].loc[:, 'score_EI':'score_TS'])
        df_compiled_means = pd.concat([df_compiled_means, df_mean_lists[i]], axis=1)
        df_compiled_scores = pd.concat([df_compiled_scores, df_score_lists[i]], axis=1)
    candidate_num = len(df_compiled_means)

    # export csv
    #df_compiled_means.to_csv('mean_std.csv', index=False)
    #df_compiled_scores.to_csv('scores.csv', index=False)

    # plot
    ## adjust size. default: (6.4, 4.8) = (w,h)
    ## tight_layout: avoid overlap of labels
    fig_mean, ax = plt.subplots(exp_num, 1, sharex = "all", figsize=(6.4, 8), tight_layout=True)
    fig_score, bx = plt.subplots(exp_num, 1, sharex = "all", figsize=(6.4, 8), tight_layout=True)
    fig_mean.suptitle('mean std')
    fig_score.suptitle('Acquisition function')

    array_mean = df_compiled_means.transpose().to_numpy()
    array_score = df_compiled_scores.transpose().to_numpy()
    x = range(candidate_num)

    # mean std
    for i,pos in enumerate(range(0, len(df_compiled_means.columns), 2)):
        mean = array_mean[pos]
        std = array_mean[pos+1]
        ax[i].plot(mean, label='mean')
        ax[i].fill_between(x, (mean-std), (mean+std), color='b', alpha=.1)
        ax[i].set_title('After Exp {}'.format(exp_ids[i]))
        ax[i].set_xlabel("Candidate (action) id")
        ax[i].set_ylabel("Expected value")
        ax[i].set_xlim(0,)
        ax[i].legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

    # score
    for i,pos in enumerate(range(0, len(df_compiled_scores.columns), 3)):
        score_EI = array_score[pos]
        score_PI = array_score[pos+1]
        score_TS = array_score[pos+2]
        bx[i].plot(score_EI, label='EI')
        bx[i].plot(score_PI, label='PI')
        bx[i].plot(score_TS, label='TS')
        bx[i].set_title('After Exp {}'.format(exp_ids[i]))
        bx[i].set_xlabel("Candidate (action) id")
        bx[i].set_ylabel("Function value")
        bx[i].set_xlim(0,)
        bx[i].legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

    plt.show()


if __name__ == '__main__':
    plot_scores()
