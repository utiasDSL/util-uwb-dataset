'''
    visual nlos obs
'''
import os, sys
sys.path.append("../")
import argparse
import numpy as np
from numpy import linalg
import pandas as pd
from scipy import stats
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter

from utility.praser import deleteNAN

# LaTeX font
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.sans-serif": ["Helvetica"]})

FONTSIZE = 18;   TICK_SIZE = 16
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 


def plot_obs(ob_x,obstacle):
    for i in range(4):
        for j in range(i,4):
            ob_x.plot([obstacle[i,0], obstacle[j,0]], [obstacle[i,1], obstacle[j,1]], [obstacle[i,2], obstacle[j,2]], linewidth=1, label='_nolegend_', color='k')
    
    for i in range(4,8):
        for j in range(i,8):
            ob_x.plot([obstacle[i,0], obstacle[j,0]], [obstacle[i,1], obstacle[j,1]], [obstacle[i,2], obstacle[j,2]], linewidth=1, label='_nolegend_', color='k')
    
    for i in range(4):
        ob_x.plot([obstacle[i,0], obstacle[i+4,0]], [obstacle[i,1], obstacle[i+4,1]], [obstacle[i,2], obstacle[i+4,2]], linewidth=1, label='_nolegend_', color='k')
        
if __name__ == "__main__":

    folder = '/home/wenda/dsl__projects__uwbDataset/dataset/static-dataset/nlos/anTag/metal/data3'
    # search for txt files in the folder
    for file in os.listdir(folder):
        if file.endswith(".txt"):
            pose_txt = os.path.join(folder, file)

    f = open(pose_txt,"r")
    pos=[];    quat=[]       # position and quaternion
    for line in f:
        x = line.split(",")
        if len(x) == 4:
            arr_x = [float(x[1]), float(x[2]), float(x[3])]
            pos.append(arr_x)
        if len(x) == 5:
            q_x = [float(x[1]), float(x[2]), float(x[3]), float(x[4])]
            quat.append(q_x)

    pos = np.array(pos)     # [an1_p, an2_p, tag_p]
    an1_p = pos[0,:]; an2_p = pos[1,:]; tag_p = pos[2,:]
    obs_up = pos[3:,:]
    obs_bt = np.copy(obs_up);  obs_bt[:,2] = 0 
    obstacle = np.concatenate((obs_up, obs_bt), axis=0)

    # search for csv files in the folder
    for file in os.listdir(folder):
        if file.endswith(".csv"):
            data_csv = os.path.join(folder, file)
            
    df = pd.read_csv(data_csv)

    # extract data
    tdoa12 = deleteNAN(np.array(df['tdoa12']))

    # compute gt
    gt_d_12 = linalg.norm(an2_p - tag_p) - linalg.norm(an1_p - tag_p)
    gt_an = linalg.norm(an1_p - an2_p)
    # compute the tdoa12 err
    err12 = tdoa12 - gt_d_12

    # visualization
    # visualize the anchor, tag and obstacle
    fig_ob = plt.figure(figsize=(9, 9))
    ob_x = fig_ob.add_subplot(111, projection = '3d')
    # make the panes transparent
    ob_x.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ob_x.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ob_x.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines 
    ob_x.xaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ob_x.yaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ob_x.zaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    if len(obstacle)!=0:
        ob_x.scatter(obstacle[:,0], obstacle[:,1], obstacle[:,2], s = 100, marker='o',color='teal', label = 'Obstacle')

    # plot lines among obstacle vertices
    plot_obs(ob_x,obstacle)

    tag_dot = ob_x.scatter(tag_p[0],   tag_p[1],  tag_p[2], s = 100, marker='o',color='navy', label = 'Tag position')
    an1_dot = ob_x.scatter(an1_p[0],   an1_p[1],  an1_p[2], s = 100, marker='o',color='red', label = 'Anchor position')
    an2_dot = ob_x.scatter(an2_p[0],   an2_p[1],  an2_p[2], s = 100, marker='o',color='red', label = '_nolegend_')
    # plot the line segement
    ob_x.plot([tag_p[0], an1_p[0]], [tag_p[1], an1_p[1]], [tag_p[2], an1_p[2]], linestyle ='--', color='steelblue', label='_nolegend_')
    ob_x.plot([tag_p[0], an2_p[0]], [tag_p[1], an2_p[1]], [tag_p[2], an2_p[2]], linestyle ='--', color='steelblue', label='_nolegend_')
    ob_x.plot([an1_p[0], an2_p[0]], [an1_p[1], an2_p[1]], [an1_p[2], an2_p[2]], linestyle ='--', color='orangered', label='_nolegend_')
    ob_x.set_xlim3d(-3.5, 3.5)  
    ob_x.set_ylim3d(-3.5, 3.5)  
    ob_x.set_zlim3d(0.0, 3.0)  
    ob_x.set_xlabel(r'X [m]', fontsize = FONTSIZE, labelpad=10)
    ob_x.set_ylabel(r'Y [m]', fontsize = FONTSIZE, labelpad=10)
    ob_x.set_zlabel(r'Z [m]', fontsize = FONTSIZE, labelpad=10)
    ob_x.set_box_aspect((1, 1, 0.5))               # xy aspect ratio is 1:1, but change z axis
    plt.legend(bbox_to_anchor=(0.53,0.8), loc='center', ncol=3,  fontsize=FONTSIZE)
    ob_x.view_init( 21, -147)
    plt.savefig('nlos_plot.pdf')
    plt.show()
