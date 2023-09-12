import glob
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import matplotlib as mpl
import seaborn as sns


def sigmoid_time(x, mean, scale_suc, scale_fail):
    if x <= mean:
        x = -1 / (1 + math.exp(-(x-mean)/scale_suc)) + 1
    else:
        x = -1 / (1 + math.exp(-(x-mean)/scale_fail)) + 1
    return x

def sigmoid_rot(x, mean_, scale_suc):
    x = -1 / (1 + math.exp(-(x-mean_)/scale_suc)) + 1
    return x

def sigmoid_scaling(x, mean_, scale_suc):
    x = 1 / (1 + math.exp(-(x-mean_)/scale_suc))
    return x

def final_score(s1, s2):
    scaling_factor = sigmoid_scaling(s2, 0.5, 0.1)
    s = 2 * scaling_factor * ((((s1 * 100) ** 1.8*s2) / 100) / 40 + 0.2)
    clip_indeces = np.where(s < 1.0, s, 1)

    return clip_indeces

mean_time = 60
scale_suc_time = 5
scale_fail_time = 20

mean_rot = 15
scale_suc_rot = 1.3
scale_fail_rot = 8

subject_baseline_data_path_1 = "/home/kia/catkin_ws/src/data_collection_human_test/data/real_test2/subject_006/baseline"
subject_controlled_data_path_1 = "/home/kia/catkin_ws/src/data_collection_human_test/data/real_test2/subject_006/controlled"


Baseline_files = sorted(glob.glob(subject_baseline_data_path_1 + '/*'))
controlled_files = sorted(glob.glob(subject_controlled_data_path_1 + '/*'))

S_time_full = []
S_rot_full = []
S_full_full = []

baseline_task_time = []
for file in Baseline_files:
    meta_data = pd.read_csv(file + '/meta_data.csv')
    baseline_task_time.append(meta_data['task_completion_time'][0])
    # baseline_task_time.append((meta_data['end_index'][0] - meta_data['start_index'][0])/58)

mean_baseline_time = sum(baseline_task_time)/len(baseline_task_time)

for file in controlled_files:
    # for file in controlled_files:
    last_task_meta = pd.read_csv(file + '/meta_data.csv')
    task_time      = last_task_meta['task_completion_time'][0]
    task_start     = last_task_meta['start_index'][0]
    task_end       = last_task_meta['end_index'][0]

    object_imu     = pd.read_csv(file + '/object_imu.csv')
    object_imu_rot = R.from_quat(object_imu[['q1', 'q2', 'q3', 'q0']]).as_euler('xyx', degrees=True)
    absolute_rotation = object_imu_rot[:, 2] - object_imu_rot[0, 2]
    task_max_rotation = max(abs(absolute_rotation[int(task_start):int(task_end)]))

    time_increase_perc = ((task_time - mean_baseline_time) / mean_baseline_time) * 100

    S_1_time     = sigmoid_time(time_increase_perc, mean_time, scale_suc_time, scale_fail_time)
    S_2_rotation = sigmoid_rot(task_max_rotation, mean_rot, scale_suc_rot)
    Full_score   = final_score(S_1_time, S_2_rotation)

    S_time_full.append(S_1_time)
    S_rot_full.append(S_2_rotation)
    S_full_full.append(Full_score)

x = np.arange(1, 41, 2)
tick_label = [1, 2,3,4,5,6,7,8,9,10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
plt.bar(x, S_time_full, align='edge', width=-0.5, label='time score', edgecolor='black', color='blue', tick_label=tick_label)
plt.bar(x, S_rot_full, align='edge', width=0.5, label='rotation score', edgecolor='black', color='green')
x = np.arange(1.5, 41.5, 2)
plt.bar(x, S_full_full, align='edge', width=0.5, label='combined score', edgecolor='black', color='red')
plt.ylabel("Score value", fontsize=18, labelpad=8)
plt.xlabel("Trial number", fontsize=18, labelpad=8)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.legend(fontsize=14)
plt.figure()

plt.show()
