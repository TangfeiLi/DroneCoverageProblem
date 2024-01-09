import pandas as pd
import numpy as np
import warnings
import random
import math
import os

warnings.filterwarnings('ignore')

def case_generator(data_path, horizons, speed, cover_range, numDrone = 10, numclassDrone = 1):
    f = open(data_path)
    txt = []
    for line in f:
        txt.append(line.strip())

    # print(txt[0])

    num_targets = [int(j) for j in txt[1].split('\t')][0]
    num_timepoints = [int(j) for j in txt[1].split('\t')][2]
    # print(num_targets)
    # print(num_timepoints)

    replace_list = [num_targets, numclassDrone, horizons]
    txt[1] = '\t'.join(map(str, replace_list))

    replace_list = [int(j) for j in txt[3].split('\t')]
    replace_list[1] = numDrone
    replace_list[2] = int(horizons/6*5)
    replace_list[3] = speed
    replace_list[4] = cover_range
    txt[3] = '\t'.join(map(str, replace_list))

    revisit_times = [1, 6, 12, 18, 24]
    revisit_times = [i for i in revisit_times if i<=horizons]
    data = []
    for i in range(1, num_targets+1):
        # ans = [i, np.random.randint(1, num_timepoints+1), np.random.randint(1, num_targets+1)] #id, revisit time, profit
        ans = [i, random.choice(revisit_times), 1] #id, revisit time, profit
        data.append(ans)

    with open(f'Case_101_{horizons}_{numclassDrone}_{numDrone}_{speed}_{cover_range}.txt', 'w+') as f:
        for i in txt:
            f.write(i)
            f.write('\n')
        
        for i in data:
            for j in i:
                f.write(f'{j}\t')
            f.write('\n')


base_path = "../base_case.txt"
horizons_list = [6, 12, 18, 24, 30, 36]
speed_list = [1,2]
range_list = [2,4]
for h in horizons_list:
    for s in speed_list:
        for r in range_list:
            case_generator(base_path, h, s, r)