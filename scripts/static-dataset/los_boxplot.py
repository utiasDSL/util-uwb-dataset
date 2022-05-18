'''
    boxplots for los tests
    python3 los_boxplot.py ../../dataset/static-dataset/los/
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

XY_FONTSIZE = 25;        TICK_SIZE = 16
TITLE_FONTSIZE = 35; 

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

def extract_err(folder):
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
    quat = np.array(quat)   # [an1_quat, an2_quat, tag_quat]
    an1_p = pos[0,:]; an2_p = pos[1,:]; tag_p = pos[2,:]

    # search for csv files in the folder
    for file in os.listdir(folder):
        if file.endswith(".csv"):
            data_csv = os.path.join(folder, file)

    df = pd.read_csv(data_csv)
    # extract data
    tdoa12 = deleteNAN(np.array(df['tdoa12']))
    # compute gt
    gt_d_12 = linalg.norm(an2_p - tag_p) - linalg.norm(an1_p - tag_p)
    # compute the tdoa12 err
    err12 = tdoa12 - gt_d_12

    return err12


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("csv_folder")
    args = parser.parse_args()

    # access csv file
    folder = args.csv_folder
    # folder = "/home/wenda/dsl__projects__uwbDataset/dataset/static-dataset/los"
    dist_folder  = os.path.join(folder, 'distTest')
    angle_folder = os.path.join(folder, 'angleTest')

    meas_err_dist  = {}
    meas_err_angle = {}

    # list over
    for file in os.listdir(dist_folder):
        base_path = os.path.join(dist_folder, file)
        # extract the measurement error
        meas_err_dist[file] = extract_err(base_path)

    for file in os.listdir(angle_folder):
        base_path = os.path.join(angle_folder, file)
        # extract the measurement error
        meas_err_angle[file] = extract_err(base_path)

    dist_label  =  [r'dT1', r'dT2', r'dT3', r'dT4',  r'dT5',  r'dT6',\
                    r'dT7', r'dT8', r'dT9', r'dT10', r'dT11', r'dT12']

    angle_label =  [r'aT1', r'aT2', r'aT3', r'aT4',  r'aT5',  r'aT6',\
                    r'aT7', r'aT8', r'aT9', r'aT10', r'aT11', r'aT12']

    # visualize
    fig = plt.figure(figsize=(12, 5))
    ax = fig.add_subplot(111)
    ax.set_ylabel(r'RMSE~[m]', fontsize = XY_FONTSIZE)
    # Creating plot
    ax.boxplot([meas_err_dist['distT1'], meas_err_dist['distT2'],  meas_err_dist['distT3'],  meas_err_dist['distT4'],\
                meas_err_dist['distT5'], meas_err_dist['distT6'],  meas_err_dist['distT7'],  meas_err_dist['distT8'],\
                meas_err_dist['distT9'], meas_err_dist['distT10'], meas_err_dist['distT11'], meas_err_dist['distT12']],\
                labels = dist_label, showfliers=False)

    plt.xticks(fontsize=XY_FONTSIZE)
    plt.yticks(np.arange(-0.2, 0.21, 0.1),fontsize=XY_FONTSIZE)
    plt.tight_layout()
    # plt.savefig('distT.pdf')
    fig1 = plt.figure(figsize=(12, 5))
    bx = fig1.add_subplot(111)
    bx.set_ylabel(r'RMSE~[m]', fontsize = XY_FONTSIZE)
    # Creating plot
    bx.boxplot([meas_err_angle['angleT1'], meas_err_angle['angleT2'],  meas_err_angle['angleT3'],  meas_err_angle['angleT4'],\
                meas_err_angle['angleT5'], meas_err_angle['angleT6'],  meas_err_angle['angleT7'],  meas_err_angle['angleT8'],\
                meas_err_angle['angleT9'], meas_err_angle['angleT10'], meas_err_angle['angleT11'], meas_err_angle['angleT12']],\
                labels = angle_label, showfliers=False)

    plt.xticks(fontsize=XY_FONTSIZE)
    plt.yticks(np.arange(-0.2, 0.21, 0.1),fontsize=XY_FONTSIZE)
    plt.tight_layout()
    # plt.savefig('angleT.pdf')
    plt.show()




