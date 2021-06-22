'''
visualize static los signal testing data
read the data from csv
'''

import os, sys
import argparse
import numpy as np
from numpy import linalg
from mpl_toolkits.mplot3d import Axes3D
import csv
import pandas as pd
from scipy import stats
from matplotlib import pyplot as plt

XY_FONTSIZE = 7;   LABEL_SIZE = 12
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
VISUAL = True

def deleteNAN(array):
    nan_array = np.isnan(array)
    not_nan = ~ nan_array
    new_array = array[not_nan]
    return new_array

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=1)
    args = parser.parse_args()

    # access csv file
    folder = args.i[0]
    data_name = os.path.split(sys.argv[-1])[1]
    print("Visualize data: " + data_name + "\n")
    
    pose_txt = folder + '/' + data_name + '_pose.txt'
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
    obstacle = pos[3:,:]

    data_csv = folder + '/' + data_name + '_data.csv'
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

    if VISUAL:
        # visualize the anchor, tag and obstacle
        fig_ob = plt.figure()
        ob_x = fig_ob.add_subplot(111, projection = '3d')
        ob_x.scatter(obstacle[:,0], obstacle[:,1], obstacle[:,2], marker='o',color='navy')
        ob_x.scatter(tag_p[0],   tag_p[1],  tag_p[2], marker='o',color='green')
        ob_x.scatter(an1_p[0],   an1_p[1],  an1_p[2], marker='o',color='red')
        ob_x.scatter(an2_p[0],   an2_p[1],  an2_p[2], marker='o',color='red')
        # plot the line segement
        ob_x.plot([tag_p[0], an1_p[0]], [tag_p[1], an1_p[1]], [tag_p[2], an1_p[2]])
        ob_x.plot([tag_p[0], an2_p[0]], [tag_p[1], an2_p[1]], [tag_p[2], an2_p[2]])
        ob_x.plot([an1_p[0], an2_p[0]], [an1_p[1], an2_p[1]], [an1_p[2], an2_p[2]])
        ob_x.set_xlim3d(-3.5, 3.5)  
        ob_x.set_ylim3d(-3.5, 3.5)  
        ob_x.set_zlim3d(0.0, 3.0)  
        ob_x.set_xlabel(r'X [m]')
        ob_x.set_ylabel(r'Y [m]')
        ob_x.set_zlabel(r'Z [m]')
        plt.legend(['Obstacle','Tag position','Anchor position'])

        fig = plt.figure(facecolor="white")
        mu=0;  sigma=0
        ax = plt.subplot(111)
        (mu, sigma) = stats.norm.fit(err12)
        print("mean0: ", mu, "std0: ", sigma)
        print("\n")
        yhist, xhist, patches = plt.hist(err12, bins=48,color='steelblue',alpha=0.75, density=True)
        plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'red')
        plt.axvline(x=0.0, alpha=1.0, linestyle ='--', color = 'black')
        plt.legend(['mean','zero'], fontsize=20)
        plt.xlabel('los error [m]', fontsize=20)
        plt.ylabel('Percent of Total Frequency', fontsize=20)
        plt.xticks(fontsize = XY_FONTSIZE)
        plt.yticks(fontsize = XY_FONTSIZE)
        ax.set_xlim([-0.5, 0.5]) 

        # visualize power in tag side
        fig1 = plt.figure(facecolor="white")
        ax1 = plt.subplot(2,2,1)
        (mu_snr1, sigma_snr1) = stats.norm.fit(snr_an1)
        print("SNR of anchor 1 mean: ", mu_snr1, "std: ", sigma_snr1)
        print("\n")
        yhist, xhist, patches = plt.hist(snr_an1, bins=48,color='steelblue',alpha=0.75, density=True)
        plt.axvline(x=mu_snr1, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('SNR An1', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)
        plt.xticks(fontsize = XY_FONTSIZE)
        plt.yticks(fontsize = XY_FONTSIZE)

        bx1 = plt.subplot(2,2,2)
        (mu_power1, sigma_power1) = stats.norm.fit(power_dif_an1)
        print("Power difference of anchor 1 mean: ", mu_power1, "std: ", sigma_power1)
        print("\n")
        yhist, xhist, patches = plt.hist(power_dif_an1, bins=48,color='steelblue',alpha=0.75, density=True)
        plt.axvline(x=mu_power1, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('Power difference An1', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)
        plt.xticks(fontsize = XY_FONTSIZE)
        plt.yticks(fontsize = XY_FONTSIZE)

        cx1 = plt.subplot(2,2,3)
        (mu_snr2, sigma_snr2) = stats.norm.fit(snr_an2)
        print("SNR of anchor 2 mean: ", mu_snr2, "std: ", sigma_snr2)
        print("\n")
        yhist, xhist, patches = plt.hist(snr_an2, bins=48,color='steelblue',alpha=0.75, density=True)
        plt.axvline(x=mu_snr2, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('SNR An2', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)
        plt.xticks(fontsize = XY_FONTSIZE)
        plt.yticks(fontsize = XY_FONTSIZE)

        dx1 = plt.subplot(2,2,4)
        (mu_power2, sigma_power2) = stats.norm.fit(power_dif_an2)
        print("Power difference of anchor 2, mean: ", mu_power2, "std: ", sigma_power2)
        print("\n")
        yhist, xhist, patches = plt.hist(power_dif_an2, bins=48,color='steelblue',alpha=0.75, density=True)
        plt.axvline(x=mu_power2, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('Power difference An2', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)

        # visualize power between anchors
        fig2 = plt.figure()
        px1 = fig2.add_subplot(3,2,1)
        (mu_snr_rc_an1, sigma_snr_rc_an1) = stats.norm.fit(an1_rx_snr)
        print("SNR received by an1, mean: ", mu_snr_rc_an1, "std: ",sigma_snr_rc_an1)
        print("\n")
        yhist, xhist, patches = plt.hist(an1_rx_snr, bins=48,color='steelblue',alpha=0.75, density=True)
        plt.axvline(x=mu_snr_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('SNR received by an1', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)

        px2 = fig2.add_subplot(3,2,2)
        (mu_snr_rc_an2, sigma_snr_rc_an2) = stats.norm.fit(an2_rx_snr)
        print("SNR received by an2, mean: ", mu_snr_rc_an2, "std: ",sigma_snr_rc_an2)
        print("\n")
        yhist, xhist, patches = plt.hist(an2_rx_snr, bins=48,color='steelblue',alpha=0.75, density=True)
        plt.axvline(x=mu_snr_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('SNR received by an2', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)

        px3 = fig2.add_subplot(3,2,3)
        (mu_powerdif_rc_an1, sigma_powerdif_rc_an1) = stats.norm.fit(an1_rx_powerdif)
        print("Power difference received by an1, mean: ", mu_powerdif_rc_an1, "std: ", sigma_powerdif_rc_an1)
        print("\n")
        yhist, xhist, patches = plt.hist(an1_rx_powerdif, bins=48, color='steelblue', alpha=0.75, density=True)
        plt.axvline(x=mu_powerdif_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('Power difference received by an1', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)

        px4 = fig2.add_subplot(3,2,4)
        (mu_powerdif_rc_an2, sigma_powerdif_rc_an2) = stats.norm.fit(an2_rx_powerdif)
        print("Power difference received by an2, mean: ", mu_powerdif_rc_an2, "std: ", sigma_powerdif_rc_an2)
        print("\n")
        yhist, xhist, patches = plt.hist(an2_rx_powerdif, bins=48, color='steelblue', alpha=0.75, density=True)
        plt.axvline(x=mu_powerdif_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('Power difference received by an2', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)

        px5 = fig2.add_subplot(3,2,5)
        (mu_tof_rc_an1, sigma_tof_rc_an1) = stats.norm.fit(an1_tof)
        print("Tof received by an1, mean: ", mu_tof_rc_an1, "std: ", sigma_tof_rc_an1)
        print("\n")
        yhist, xhist, patches = plt.hist(an1_tof, bins=20, color='steelblue', alpha=0.75, density=True)
        plt.axvline(x=mu_tof_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('tof received by an1', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)

        px6 = fig2.add_subplot(3,2,6)

        (mu_tof_rc_an2, sigma_tof_rc_an2) = stats.norm.fit(an2_tof)
        print("Tof received by an1, mean: ", mu_tof_rc_an2, "std: ", sigma_tof_rc_an2)
        print("\n")
        yhist, xhist, patches = plt.hist(an2_tof, bins=20, color='steelblue', alpha=0.75, density=True)
        plt.axvline(x=mu_tof_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
        plt.xlabel('tof received by an1', fontsize = LABEL_SIZE)
        plt.ylabel('PDF', fontsize = LABEL_SIZE)

        plt.show()
