'''
    visualize the nlos error distribution with one type of obstacle
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

XY_FONTSIZE = 25;        LABEL_SIZE = 30
TITLE_FONTSIZE = 35; 

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

matplotlib.rc('xtick', labelsize=XY_FONTSIZE) 
matplotlib.rc('ytick', labelsize=XY_FONTSIZE) 


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

    meas_err = {}
    nlos_err = np.empty(0)

    for file in os.listdir(folder):
        base_path = os.path.join(folder, file)
        # extract the measurement error
        meas_err[file] = extract_err(base_path)
        # 
        nlos_err = np.hstack((nlos_err, meas_err[file]))

    nlos_err = -1.0 * nlos_err
    # visualize
    fig = plt.figure(figsize=(9, 9))
    mu=0;  sigma=0
    ax = plt.subplot(111)
    (mu, sigma) = stats.norm.fit(nlos_err)
    print("mean0: ", mu, "std0: ", sigma)
    print("\n")
    yhist, xhist, patches = plt.hist(nlos_err, bins=200,color='royalblue',alpha=0.5, histtype='bar', ec='white',density=True)
    plt.xlabel('NLOS error [m]', fontsize=LABEL_SIZE)
    plt.ylabel('Probability Density', fontsize=LABEL_SIZE)
    ax.set_xlim([-1.0, 1.0]) 
    plt.xticks(fontsize=XY_FONTSIZE)
    plt.yticks(fontsize=XY_FONTSIZE)
    plt.tight_layout()
    plt.savefig('nlos_dist.pdf')
    plt.show()