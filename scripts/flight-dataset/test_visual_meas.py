'''
    visual UWB meas. w.r.t. ground truth
'''
import os, sys
sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
from numpy import linalg
from pyquaternion import Quaternion
import matplotlib
from matplotlib import pyplot as plt

from utility.praser import extract_gt, extract_tdoa

FONTSIZE = 34;     TICK_SIZE = 34

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

# translation vector from the quadcopter to UWB tag
t_uv = np.array([-0.01245, 0.00127, 0.0908]).reshape(-1,1)  


if __name__ == "__main__":

    # access the survey results
    cwd = os.path.dirname(__file__)
    anchor_npz = '/home/wenda/dsl__projects__uwbDataset/dataset/flight-dataset/survey-results/anchor_const4.npz'
    anchor_survey = np.load(os.path.join(cwd, anchor_npz))
    anchor_pos = anchor_survey['an_pos']
    
    # access csv
    csv_file = '/home/wenda/dsl__projects__uwbDataset/dataset/flight-dataset/csv-data/const4/const4-trial5-tdoa2-traj1.csv'
    df = pd.read_csv(csv_file)
    csv_name = os.path.split(csv_file)[1]
    # print out
    print("visualizing csv: " + str(csv_name) + "\n")

    # --------------- extract csv file --------------- #
    gt_pose = extract_gt(df)
    tdoa    = extract_tdoa(df)

    # external calibration: convert the gt_position to UWB antenna center
    uwb_p = np.zeros((len(gt_pose), 3))
    for idx in range(len(gt_pose)):
        q_cf =Quaternion([gt_pose[idx,7], gt_pose[idx,4], gt_pose[idx,5], gt_pose[idx,6]])    # [q_w, q_x, q_y, q_z]
        C_iv = q_cf.rotation_matrix       # rotation matrix from vehicle body frame to inertial frame

        uwb_ac = C_iv.dot(t_uv) + gt_pose[idx,1:4].reshape(-1,1)
        uwb_p[idx,:] = uwb_ac.reshape(1,-1)     # gt of uwb tag

    # get the id for tdoa_ij measurements
    tdoa_70 = np.where((tdoa[:,1]==[7])&(tdoa[:,2]==[0]))
    tdoa_meas_70 = np.squeeze(tdoa[tdoa_70, :])

    tdoa_01 = np.where((tdoa[:,1]==[0])&(tdoa[:,2]==[1]))
    tdoa_meas_01 = np.squeeze(tdoa[tdoa_01, :])

    tdoa_12 = np.where((tdoa[:,1]==[1])&(tdoa[:,2]==[2]))
    tdoa_meas_12 = np.squeeze(tdoa[tdoa_12, :])

    tdoa_23 = np.where((tdoa[:,1]==[2])&(tdoa[:,2]==[3]))
    tdoa_meas_23 = np.squeeze(tdoa[tdoa_23, :])

    tdoa_34 = np.where((tdoa[:,1]==[3])&(tdoa[:,2]==[4]))
    tdoa_meas_34 = np.squeeze(tdoa[tdoa_34, :])

    tdoa_45 = np.where((tdoa[:,1]==[4])&(tdoa[:,2]==[5]))
    tdoa_meas_45 = np.squeeze(tdoa[tdoa_45, :])

    tdoa_56 = np.where((tdoa[:,1]==[5])&(tdoa[:,2]==[6]))
    tdoa_meas_56 = np.squeeze(tdoa[tdoa_56, :])

    tdoa_67 = np.where((tdoa[:,1]==[6])&(tdoa[:,2]==[7]))
    tdoa_meas_67 = np.squeeze(tdoa[tdoa_67, :])

    # compute the ground truth for tdoa_ij
    d = []
    for i in range(8):
        d.append(linalg.norm(anchor_pos[i,:].reshape(1,-1) - uwb_p, axis = 1))
    
    # shape of d is 8 x n
    d = np.array(d)

    # measurement model
    d_70 = d[0,:] - d[7,:]
    d_01 = d[1,:] - d[0,:]
    d_12 = d[2,:] - d[1,:]
    d_23 = d[3,:] - d[2,:]

    d_34 = d[4,:] - d[3,:]
    d_45 = d[5,:] - d[4,:]
    d_56 = d[6,:] - d[5,:]
    d_67 = d[7,:] - d[6,:]
    
    # visualization 
    # UWB TDOA
    # fig1 = plt.figure(figsize=(12, 6))
    fig1 = plt.figure(figsize=(15, 12))
    ax1 = fig1.add_subplot(111)
    ax1.plot(gt_pose[:,0], d_23, color='red',linewidth=3.5, alpha = 1.0, label = "Ground truth")
    ax1.scatter(tdoa_meas_23[:,0], tdoa_meas_23[:,3], color = "steelblue", s = 55, alpha = 0.8, label = "TDOA measurements")
    # ax1.legend(loc='best')
    ax1.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    ax1.set_ylabel(r'TDOA meas. [m]',fontsize = FONTSIZE) 
    # plt.title(r"UWB TDOA measurements, (An7, An0)", fontsize=FONTSIZE, fontweight=0, color='black')
    plt.legend(bbox_to_anchor=(0.6,1.2), loc='center', ncol=2,  fontsize=FONTSIZE)
    ax1.set_xlim([0.0,127.0])
    fig1.tight_layout()
    plt.savefig('uwb_meas_trial5.pdf')
    plt.show()

