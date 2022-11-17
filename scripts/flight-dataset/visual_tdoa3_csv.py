'''
    Visualize the UWB TDOA2 and other sensor measurements in csv
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

from utility.praser import extract_baro, extract_gt, extract_tdoa, extract_tof, extract_flow

FONTSIZE = 18;     TICK_SIZE = 16

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

# translation vector from the quadcopter to UWB tag
t_uv = np.array([-0.01245, 0.00127, 0.0908]).reshape(-1,1)  
# translation vector from the quadcopter to laser-ranging sensor 
t_lv = np.array([0.0, 0.0, -0.0012]).reshape(-1,1)

if __name__ == "__main__":
    # ------------- access anchor survey and csv file ------------- #
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=2)
    args = parser.parse_args()
    
    # access the survey results
    anchor_npz = args.i[0]
    anchor_survey = np.load(anchor_npz)
    anchor_pos = anchor_survey['an_pos']
    anchor_qaut = anchor_survey['an_quat']
    # print out
    anchor_file = os.path.split(sys.argv[-2])[1]
    print("loading anchor survey results: " + str(anchor_file) + "\n")

    # access csv
    csv_file = args.i[1]
    df = pd.read_csv(csv_file)
    csv_name = os.path.split(sys.argv[-1])[1]
    # print out
    print("visualizing csv: " + str(csv_name) + "\n")

    # --------------- extract csv file --------------- #
    gt_pose = extract_gt(df)
    tdoa    = extract_tdoa(df)
    tof     = extract_tof(df)
    flow    = extract_flow(df)
    baro    = extract_baro(df)

    # external calibration: convert the gt_position to UWB antenna center
    uwb_p = np.zeros((len(gt_pose), 3))
    for idx in range(len(gt_pose)):
        q_cf =Quaternion([gt_pose[idx,7], gt_pose[idx,4], gt_pose[idx,5], gt_pose[idx,6]])    # [q_w, q_x, q_y, q_z]
        C_iv = q_cf.rotation_matrix       # rotation matrix from vehicle body frame to inertial frame

        uwb_ac = C_iv.dot(t_uv) + gt_pose[idx,1:4].reshape(-1,1)
        uwb_p[idx,:] = uwb_ac.reshape(1,-1)     # gt of uwb tag

    # ------------ select the anchor pair for visualization---------- #
    # possible anchor ID = [0,1,2,3,4,5,6,7] 
    an_i = 7;     an_j = 0
    # --------------------------------------------------------------- #
    # get the id for tdoa_ij measurements
    tdoa_id = np.where((tdoa[:,1]==[an_i])&(tdoa[:,2]==[an_j]))
    tdoa_meas = np.squeeze(tdoa[tdoa_id, :])

    # compute the ground truth for tdoa_ij
    an_pos_i = anchor_pos[an_i,:].reshape(1,-1)
    an_pos_j = anchor_pos[an_j,:].reshape(1,-1)

    # ground truth for the distance between anchor and tag
    d_i = np.asarray(linalg.norm(an_pos_i - uwb_p, axis = 1))
    d_j = np.asarray(linalg.norm(an_pos_j - uwb_p, axis = 1))
    # measurement model
    d_ij = d_j - d_i

    # visualization 
    # UWB TDOA
    fig1 = plt.figure(figsize=(12, 8))
    ax1 = fig1.add_subplot(111)
    ax1.plot(gt_pose[:,0], d_ij, color='red',linewidth=1.5, alpha = 0.9, label = "Vicon ground truth")
    ax1.scatter(tdoa_meas[:,0], tdoa_meas[:,3], color = "steelblue", s = 5.0, alpha = 0.6, label = "tdoa measurements")
    ax1.legend(loc='best',fontsize = FONTSIZE)
    ax1.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    ax1.set_ylabel(r'TDoA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(an_i, an_j), fontsize=FONTSIZE, fontweight=0, color='black')

    # Z-range ToF
    fig2 = plt.figure(figsize=(10, 8))
    ax2 = fig2.add_subplot(111)
    ax2.scatter(tof[:,0], tof[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "tof measurements")
    ax2.legend(loc='best',fontsize = FONTSIZE)
    ax2.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    ax2.set_ylabel(r'ToF measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"Z-range measurements", fontsize=FONTSIZE, fontweight=0, color='black')

    # flow pixel
    fig3 = plt.figure(figsize=(10, 8))
    ax3 = fig3.add_subplot(211)
    plt.title(r"Optical flow measurements", fontsize=FONTSIZE, fontweight=0, color='black')
    ax3.scatter(flow[:,0], flow[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel x")
    ax3.set_ylabel(r'motion delta x',fontsize = FONTSIZE) 
    bx3 = fig3.add_subplot(212)
    bx3.scatter(flow[:,0], flow[:,2], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel y")
    bx3.set_ylabel(r'motion delta y',fontsize = FONTSIZE) 
    bx3.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    plt.legend(loc='best',fontsize = FONTSIZE)

    # barometer
    fig4 = plt.figure(figsize=(10, 8))
    ax4 = fig4.add_subplot(111)
    plt.title(r"Baro measurements", fontsize=FONTSIZE, fontweight=0, color='black')
    ax4.scatter(baro[:,0], baro[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "baro asl")
    ax4.set_ylabel(r'asl',fontsize = FONTSIZE) 
    ax4.set_xlabel(r'Time [s]', fontsize=FONTSIZE)
    plt.legend(loc='best',fontsize = FONTSIZE)

    # trajectory
    fig5 = plt.figure(figsize=(10, 8))
    ax_t = fig5.add_subplot(111, projection = '3d')
    # make the panes transparent
    ax_t.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax_t.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax_t.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines 
    ax_t.xaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax_t.yaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax_t.zaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)

    ax_t.plot(gt_pose[:,1],gt_pose[:,2],gt_pose[:,3],color='royalblue',linewidth=2.0, alpha=0.9)
    ax_t.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2], color='Teal', s = 100, alpha = 0.5, label = 'anchors')
    ax_t.set_xlim([-3.5,3.5])
    ax_t.set_ylim([-3.9,3.9])
    ax_t.set_zlim([-0.0,3.0])
    ax_t.set_xlabel(r'X [m]',fontsize = FONTSIZE)
    ax_t.set_ylabel(r'Y [m]',fontsize = FONTSIZE)
    ax_t.set_zlabel(r'Z [m]',fontsize = FONTSIZE)
    ax_t.legend(loc='best', bbox_to_anchor=(0.5,0.92))
    plt.legend(['Trajectory','Anchor position'], fontsize=FONTSIZE)
    ax_t.set_box_aspect((1, 1, 0.5))  # xy aspect ratio is 1:1, but change z axis
 
    # plot separate x,y,z
    fig6 = plt.figure(figsize=(10, 8))
    a_x = fig6.add_subplot(311)
    plt.title(r"Ground truth of the trajectory", fontsize=FONTSIZE, fontweight=0, color='black', style='italic', y=1.02 )
    a_x.plot(gt_pose[:,0],gt_pose[:,1],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt x")
    a_x.legend(loc='best', fontsize = FONTSIZE)
    a_x.set_ylabel(r'X [m]', fontsize = FONTSIZE) 
    a_y = fig6.add_subplot(312)
    a_y.plot(gt_pose[:,0],gt_pose[:,2],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt y")
    a_y.legend(loc='best', fontsize = FONTSIZE)
    a_y.set_ylabel(r'Y [m]', fontsize = FONTSIZE) 
    a_z = fig6.add_subplot(313)
    a_z.plot(gt_pose[:,0],gt_pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt z")
    a_z.legend(loc='best', fontsize = FONTSIZE)
    a_z.set_ylabel(r'Z [m]', fontsize = FONTSIZE)
    a_z.set_xlabel(r'Time [s]', fontsize = FONTSIZE)

    plt.show()
