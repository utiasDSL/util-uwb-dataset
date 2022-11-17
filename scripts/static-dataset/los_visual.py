'''
    Visualize static los signal testing data. Read the data from csv
    python3 los_visual.py -i [csv data folder] 
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

FONTSIZE = 18;   TICK_SIZE = 16
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# set the labelsize of xtick and ytick
matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_folder")
    args = parser.parse_args()

    # access csv file
    folder = args.csv_folder
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
    tdoa21 = deleteNAN(np.array(df['tdoa21']))

    snr_an1 = deleteNAN(np.array(df['snr_an1']))
    power_dif_an1 = deleteNAN(np.array(df['power_dif_an1']))
    snr_an2 = deleteNAN(np.array(df['snr_an2']))
    power_dif_an2 = deleteNAN(np.array(df['power_dif_an2']))


    an1_rx_snr = deleteNAN(np.array(df['an1_rx_snr']) )
    an1_rx_powerdif = deleteNAN(np.array(df['an1_rx_powerdif']))
    an1_tof = deleteNAN(np.array(df['an1_tof']))

    an2_rx_snr = deleteNAN(np.array(df['an2_rx_snr']) )
    an2_rx_powerdif = deleteNAN(np.array(df['an2_rx_powerdif']))
    an2_tof = deleteNAN(np.array(df['an2_tof']))

    # compute gt
    gt_d_12 = linalg.norm(an2_p - tag_p) - linalg.norm(an1_p - tag_p)
    gt_an = linalg.norm(an1_p - an2_p)
    # compute the tdoa12 err
    err12 = tdoa12 - gt_d_12

    # visualization
    # visualize the anchor-tag positions
    fig_p = plt.figure(figsize=(16, 9))
    px = fig_p.add_subplot(111, projection = '3d')
    # make the panes transparent
    px.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    px.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    px.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines 
    px.xaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    px.yaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    px.zaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)

    px.scatter(tag_p[0],   tag_p[1],  tag_p[2], s = 100, marker='o',color='navy', label = 'Tag position')
    px.scatter(an1_p[0],   an1_p[1],  an1_p[2], s = 100, marker='o',color='red', label = 'Anchor position')
    px.scatter(an2_p[0],   an2_p[1],  an2_p[2], s = 100, marker='o',color='red', label='_nolegend_')
    # plot the line segement
    px.plot([tag_p[0], an1_p[0]], [tag_p[1], an1_p[1]], [tag_p[2], an1_p[2]],linestyle='--',color='steelblue',label='_nolegend_')
    px.plot([tag_p[0], an2_p[0]], [tag_p[1], an2_p[1]], [tag_p[2], an2_p[2]],linestyle='--',color='steelblue',label='_nolegend_')
    px.set_xlim3d(-3.5, 3.5)  
    px.set_ylim3d(-3.5, 3.5)  
    px.set_zlim3d(0.0, 3.0)  
    px.set_xlabel(r'X [m]', fontsize = FONTSIZE)
    px.set_ylabel(r'Y [m]', fontsize = FONTSIZE)
    px.set_zlabel(r'Z [m]', fontsize = FONTSIZE)
    px.set_box_aspect((1, 1, 0.35))               # xy aspect ratio is 1:1, but change z axis
    plt.legend(loc='best', fontsize=FONTSIZE)
    px.view_init(24, -58)

    fig = plt.figure(figsize=(16, 9))
    mu=0;  sigma=0
    ax = plt.subplot(111)
    (mu, sigma) = stats.norm.fit(err12)
    print("mean0: %.4f," % mu, "std0: %.4f" % sigma)
    yhist, xhist, patches = plt.hist(err12, bins=48,color='steelblue',alpha=0.75, density=True)
    plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'red')
    plt.axvline(x=0.0, alpha=1.0, linestyle ='--', color = 'black')
    plt.legend(['mean','zero'], fontsize=20)
    plt.xlabel('los error [m]', fontsize=20)
    plt.ylabel('probability density', fontsize=20)
    ax.set_xlim([-0.5, 0.5]) 

    # visualize power in tag side
    fig1 = plt.figure(figsize=(16, 9))
    ax1 = plt.subplot(2,2,1)
    (mu_snr1, sigma_snr1) = stats.norm.fit(snr_an1)
    print("SNR of anchor 1 mean: %.4f," % mu_snr1, "std: %.4f", sigma_snr1)
    yhist, xhist, patches = plt.hist(snr_an1, bins=48,color='steelblue',alpha=0.75, density=True)
    ax1.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_snr1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('SNR of An1', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    bx1 = plt.subplot(2,2,2)
    (mu_power1, sigma_power1) = stats.norm.fit(power_dif_an1)
    print("power difference of anchor 1 mean: %.4f," % mu_power1, "std: %.4f" % sigma_power1)
    yhist, xhist, patches = plt.hist(power_dif_an1, bins=48,color='steelblue',alpha=0.75, density=True)
    bx1.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_power1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('power difference of An1', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    cx1 = plt.subplot(2,2,3)
    (mu_snr2, sigma_snr2) = stats.norm.fit(snr_an2)
    print("SNR of anchor 2 mean: %.4f," % mu_snr2, "std: %.4f" % sigma_snr2)
    yhist, xhist, patches = plt.hist(snr_an2, bins=48,color='steelblue',alpha=0.75, density=True)
    cx1.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_snr2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('SNR of An2', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    dx1 = plt.subplot(2,2,4)
    (mu_power2, sigma_power2) = stats.norm.fit(power_dif_an2)
    print("power difference of anchor 2, mean: %.4f," % mu_power2, "std: %.4f" % sigma_power2)
    yhist, xhist, patches = plt.hist(power_dif_an2, bins=48,color='steelblue',alpha=0.75, density=True)
    dx1.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_power2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('power difference of An2', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    # visualize power between anchors
    fig2 = plt.figure(figsize=(18, 9))
    px1 = fig2.add_subplot(2,3,1)
    (mu_snr_rc_an1, sigma_snr_rc_an1) = stats.norm.fit(an1_rx_snr)
    print("SNR received by an1, mean: %.4f," % mu_snr_rc_an1, "std: %.4f" % sigma_snr_rc_an1)
    yhist, xhist, patches = plt.hist(an1_rx_snr, bins=48,color='steelblue',alpha=0.75, density=True)
    px1.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_snr_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('SNR received by an1', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    px2 = fig2.add_subplot(2,3,2)
    (mu_snr_rc_an2, sigma_snr_rc_an2) = stats.norm.fit(an2_rx_snr)
    print("SNR received by an2, mean: %.4f," % mu_snr_rc_an2, "std: %.4f" % sigma_snr_rc_an2)
    yhist, xhist, patches = plt.hist(an2_rx_snr, bins=48,color='steelblue',alpha=0.75, density=True)
    px2.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_snr_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('SNR received by an2', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    px3 = fig2.add_subplot(2,3,3)
    (mu_powerdif_rc_an1, sigma_powerdif_rc_an1) = stats.norm.fit(an1_rx_powerdif)
    print("power difference received by an1, mean: %.4f," % mu_powerdif_rc_an1, "std: %.4f" % sigma_powerdif_rc_an1)
    yhist, xhist, patches = plt.hist(an1_rx_powerdif, bins=48, color='steelblue', alpha=0.75, density=True)
    px3.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_powerdif_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('power difference received by an1', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    px4 = fig2.add_subplot(2,3,4)
    (mu_powerdif_rc_an2, sigma_powerdif_rc_an2) = stats.norm.fit(an2_rx_powerdif)
    print("power difference received by an2, mean: %.4f," % mu_powerdif_rc_an2, "std: %.4f" % sigma_powerdif_rc_an2)
    yhist, xhist, patches = plt.hist(an2_rx_powerdif, bins=48, color='steelblue', alpha=0.75, density=True)
    px4.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.axvline(x=mu_powerdif_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('power difference received by an2', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    px5 = fig2.add_subplot(2,3,5)
    (mu_tof_rc_an1, sigma_tof_rc_an1) = stats.norm.fit(an1_tof)
    print("Tof received by an1, mean: %.4f," % mu_tof_rc_an1, "std: %.4f" % sigma_tof_rc_an1)
    yhist, xhist, patches = plt.hist(an1_tof, bins=20, color='steelblue', alpha=0.75, density=True)
    plt.axvline(x=mu_tof_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('tof received by an1', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    px6 = fig2.add_subplot(2,3,6)
    (mu_tof_rc_an2, sigma_tof_rc_an2) = stats.norm.fit(an2_tof)
    print("Tof received by an1, mean: %.4f," % mu_tof_rc_an2, "std: %.4f" % sigma_tof_rc_an2)
    yhist, xhist, patches = plt.hist(an2_tof, bins=20, color='steelblue', alpha=0.75, density=True)
    plt.axvline(x=mu_tof_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('tof received by an1', fontsize = FONTSIZE)
    plt.ylabel('probability density', fontsize = FONTSIZE)

    plt.show()








