import glob
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import matplotlib as mpl
import seaborn as sns


familiarization = False
name = "subject_024"

def sigmoid_time(x, mean, scale_suc, scale_fail):
    if x <= mean:
        x = -1 / (1 + math.exp(-(x-mean)/scale_suc)) + 1
    else:
        x = -1 / (1 + math.exp(-(x-mean)/scale_fail)) + 1
    return x


mean_time = 0.95
scale_suc_time = 0.3
scale_fail_time = 0.3

# sigmoid_time_vec = np.vectorize(sigmoid_time)
# x_vec = np.linspace(-0.2, 2, 100)

# ax = plt.axes()
# ax.plot(x_vec, sigmoid_time_vec(x_vec[:], mean_time, scale_suc_time, scale_fail_time), linewidth=2, c='blue')
# plt.show()

if familiarization:
    subject_baseline_data_path = "/home/kia/catkin_ws/src/data_collection_human_test/data/real_test2/" + name + "/baseline_familiarization"
else:
    subject_baseline_data_path = "/home/kia/catkin_ws/src/data_collection_human_test/data/real_test2/" + name + "/baseline"

Baseline_files = sorted(glob.glob(subject_baseline_data_path + '/*'))
task_time = pd.read_csv(Baseline_files[-1] + '/meta_data.csv')['task_completion_time'][0]
print(task_time)

# calculate task completion time socre:
# score bands: 
# < 72%   increase  : Excellent
# 72-80%  increase  : Fast
# 80-113%  increase  : Good
# >113%    increase  : Slow

if task_time <= 0.64:
    subject_time_score = 4
elif 0.64 < task_time <= 0.96:
    subject_time_score = 3
elif 0.96 < task_time <= 1.2:
    subject_time_score = 2
elif task_time > 1.2:
    subject_time_score = 1

S_1_time = sigmoid_time(task_time, mean_time, scale_suc_time, scale_fail_time)

time_bar_color = ['red', 'yellow', 'limegreen', 'forestgreen']

gridspec = dict(hspace=0.0, width_ratios=[4,1, 1, 1])

fig, ax = plt.subplots(1, 4, figsize=(9,5.5), facecolor='lightskyblue', gridspec_kw=gridspec)
fig.tight_layout(pad=8.0)
ax[0].bar(['Time score'], subject_time_score, color=time_bar_color[subject_time_score-1], width=0.04)
ax[0].set_yticks([1,2,3,4])
ax[0].set_yticklabels(['Slow',  'Normal', 'Fast', 'Excellent'], fontsize=20)
ax[0].set_xticklabels(['Time score'], fontsize=20)
ax[0].set_title("Time score", fontsize=30, pad=40)
ax2 = ax[0].twinx()
ax2.axhline(y=S_1_time, ls='-', color='black', linewidth=2)
ax2.set_yticks([])
ax[0].set_ylim(0, 4);

ax[1].axis('off')
ax[2].axis('off')
ax[3].axis('off')

plt.show()
