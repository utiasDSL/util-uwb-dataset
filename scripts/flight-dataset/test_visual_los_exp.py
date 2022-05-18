import os, sys
sys.path.append("../")
import pandas as pd
import argparse
import rosbag
import numpy as np
import matplotlib
from matplotlib import pyplot as plt

from utility.praser import extract_gt

# LaTeX font
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.sans-serif": ["Helvetica"]})

FONTSIZE = 21;   TICK_SIZE = 21
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 


if __name__ == "__main__":

    # access the survey results
    cwd = os.path.dirname(__file__)
    anchor_npz = '/home/wenda/dsl__projects__uwbDataset/dataset/flight-dataset/survey-results/anchor_const4.npz'
    anchor_survey = np.load(os.path.join(cwd, anchor_npz))
    anchor_pos = anchor_survey['an_pos']
    
    # access csv
    csv_file = '/home/wenda/dsl__projects__uwbDataset/dataset/flight-dataset/csv-data/const1/const1-trial1-tdoa2.csv'

    # print out
    df = pd.read_csv(csv_file)
    gt_pose = extract_gt(df)

        # trajectory
    fig = plt.figure(figsize=(10, 8))
    ax_t = fig.add_subplot(111, projection = '3d')
    # make the panes transparent
    ax_t.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax_t.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax_t.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines 
    ax_t.xaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax_t.yaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax_t.zaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)

    ax_t.plot(gt_pose[:,1],gt_pose[:,2],gt_pose[:,3],color='steelblue',linewidth=3.0, alpha=0.9)
    # ax_t.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2], color='red', s = 100, alpha = 0.5, label = 'anchors')
    ax_t.set_xlim([-2.5,2.5])
    ax_t.set_ylim([-2.5,2.5])
    ax_t.set_zlim([0.0,3.0])
    ax_t.set_xlabel(r'X [m]', fontsize = FONTSIZE, labelpad=12)
    ax_t.set_ylabel(r'Y [m]', fontsize = FONTSIZE, labelpad=10)
    ax_t.set_zlabel(r'Z [m]', fontsize = FONTSIZE, labelpad=10)
    ticks = np.arange(0.0, 5.0, 1.0)
    ax_t.set_zticks(ticks)
    # ax_t.legend(loc='best', bbox_to_anchor=(0.5,0.92))
    ax_t.view_init(27, -57)
    # plt.legend(['Trajectory','Anchor position'], fontsize=FONTSIZE)
    ax_t.set_box_aspect((1, 1, 0.5))  # xy aspect ratio is 1:1, but change z axis
    plt.savefig('los-traj1.pdf')
    plt.show()