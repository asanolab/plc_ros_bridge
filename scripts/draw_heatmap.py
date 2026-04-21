#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

argc = len(sys.argv)
if(argc != 5):
    print('ERROR: argc should be 5')
    print('Usage: ./draw_heatmap.py data_path, row_index_x, row_index_y, row_index_eval')
    exit()


def draw_heatmap(data_path, column_index_x, column_index_y, column_index_eval):
    # load data
    df_all = pd.read_csv(data_path, header=0)
    header_name_x = df_all.columns[int(column_index_x)]
    header_name_y = df_all.columns[int(column_index_y)]
    header_name_eval = df_all.columns[int(column_index_eval)]
    df_xyeval = df_all.loc[:,[header_name_x, header_name_y, header_name_eval]]
    array_heatmap = np.zeros((4, 5))

    # generate row and column for heatmap
    column_all = df_xyeval[header_name_x].to_list()
    column_list = list(dict.fromkeys(column_all))
    print('column_list:\n', column_list)

    row_all = df_xyeval[header_name_y].to_list()
    row_list = list(dict.fromkeys(row_all))
    print('row_list:\n', row_list)

    # generate array and df for heatmap style
    for r, r_item in enumerate(row_list):
        for c, c_item in enumerate(column_list):
            str = '{} == {} and {} == {}'.format(header_name_x, c_item, header_name_y, r_item)
            row_index = df_xyeval.query(str).index[0]
            array_heatmap[r][c] = df_xyeval.at[row_index, header_name_eval]

    df_heatmap = pd.DataFrame(array_heatmap, index=row_list, columns=column_list)

    # check
    print('array_heatmap:\n', array_heatmap)
    print('df_heatmap:\n', df_heatmap)

    # plot by seaborn
    fig, ax = plt.subplots()
    ax = sns.heatmap(df_heatmap)

    # set label
    ax.set_xlabel(header_name_x)
    ax.set_ylabel(header_name_y)
    ax.xaxis.tick_top()

    plt.show()
    fig.savefig('heatmap.png')


if __name__ == '__main__':
    data_path = sys.argv[1]
    column_index_x = sys.argv[2]
    column_index_y = sys.argv[3]
    column_index_eval = sys.argv[4]

    draw_heatmap(data_path, column_index_x, column_index_y, column_index_eval)
