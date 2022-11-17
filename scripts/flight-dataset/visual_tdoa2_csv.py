'''
    Visualize the UWB TDOA2 and other sensor measurements in csv
'''
import os, sys
sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
from numpy import linalg
import math
from pyquaternion import Quaternion
import matplotlib
from matplotlib import pyplot as plt

from utility.praser import extract_baro, extract_gt, extract_tdoa, extract_tof, extract_flow, extract_acc, extract_gyro

FONTSIZE = 18;     TICK_SIZE = 16

# constant: convert the IMU data from the UTIL dataset to m/s^2 and rad/s
GRAVITY_MAGNITUDE = 9.81
DEG_TO_RAD  = math.pi/180.0

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
    acc     = extract_acc(df)
    gyr     = extract_gyro(df)

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
    fig1 = plt.figure(figsize=(10, 8))
    ax1 = fig1.add_subplot(221)
    ax1.scatter(tdoa_meas_70[:,0], tdoa_meas_70[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    ax1.plot(gt_pose[:,0], d_70, color='red',linewidth=1.5, label = "Vicon ground truth")
    ax1.legend(loc='best')
    ax1.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    ax1.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An7, An0)", fontsize=FONTSIZE, fontweight=0, color='black')

    bx1 = fig1.add_subplot(222)
    bx1.scatter(tdoa_meas_01[:,0], tdoa_meas_01[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    bx1.plot(gt_pose[:,0], d_01, color='red',linewidth=1.5, label = "Vicon ground truth")
    bx1.legend(loc='best')
    bx1.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    bx1.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An0, An1)", fontsize=FONTSIZE, fontweight=0, color='black')

    cx1 = fig1.add_subplot(223)
    cx1.scatter(tdoa_meas_12[:,0], tdoa_meas_12[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    cx1.plot(gt_pose[:,0], d_12, color='red',linewidth=1.5, label = "Vicon ground truth")
    cx1.legend(loc='best')
    cx1.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    cx1.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An1, An2)", fontsize=FONTSIZE, fontweight=0, color='black')


    dx1 = fig1.add_subplot(224)
    dx1.scatter(tdoa_meas_23[:,0], tdoa_meas_23[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    dx1.plot(gt_pose[:,0], d_23, color='red',linewidth=1.5, label = "Vicon ground truth")
    dx1.legend(loc='best')
    dx1.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    dx1.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An2, An3)", fontsize=FONTSIZE, fontweight=0, color='black')

    fig2 = plt.figure(figsize=(10, 8))
    ax2 = fig2.add_subplot(221)
    ax2.scatter(tdoa_meas_34[:,0], tdoa_meas_34[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    ax2.plot(gt_pose[:,0], d_34, color='red',linewidth=1.5, label = "Vicon ground truth")
    ax2.legend(loc='best')
    ax2.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    ax2.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An3, An4)", fontsize=FONTSIZE, fontweight=0, color='black')

    bx2 = fig2.add_subplot(222)
    bx2.scatter(tdoa_meas_45[:,0], tdoa_meas_45[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    bx2.plot(gt_pose[:,0], d_45, color='red',linewidth=1.5, label = "Vicon ground truth")
    bx2.legend(loc='best')
    bx2.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    bx2.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An4, An5)", fontsize=FONTSIZE, fontweight=0, color='black')

    cx2 = fig2.add_subplot(223)
    cx2.scatter(tdoa_meas_56[:,0], tdoa_meas_56[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    cx2.plot(gt_pose[:,0], d_56, color='red',linewidth=1.5, label = "Vicon ground truth")
    cx2.legend(loc='best')
    cx2.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    cx2.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An5, An6)", fontsize=FONTSIZE, fontweight=0, color='black')

    dx2 = fig2.add_subplot(224)
    dx2.scatter(tdoa_meas_67[:,0], tdoa_meas_67[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    dx2.plot(gt_pose[:,0], d_67, color='red',linewidth=1.5, label = "Vicon ground truth")
    dx2.legend(loc='best')
    dx2.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    dx2.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An6, An7)", fontsize=FONTSIZE, fontweight=0, color='black')

    # laser-ranging ToF
    fig3 = plt.figure(figsize=(10, 8))
    ax3 = fig3.add_subplot(111)
    ax3.scatter(tof[:,0], tof[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "tof measurements")
    ax3.legend(loc='best')
    ax3.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    ax3.set_ylabel(r'ToF measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"Laser-ranging measurements", fontsize=FONTSIZE, fontweight=0, color='black')

    # optical flow
    fig4 = plt.figure(figsize=(10, 8))
    ax4 = fig4.add_subplot(211)
    plt.title(r"Optical flow measurements", fontsize=FONTSIZE, fontweight=0, color='black')
    ax4.scatter(flow[:,0], flow[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel x")
    ax4.set_ylabel(r'motion delta x',fontsize = FONTSIZE) 
    bx4 = fig4.add_subplot(212)
    bx4.scatter(flow[:,0], flow[:,2], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel y")
    bx4.set_ylabel(r'motion delta y',fontsize = FONTSIZE) 
    bx4.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    plt.legend(loc='best')

    # barometer
    fig5 = plt.figure(figsize=(10, 8))
    ax5 = fig5.add_subplot(111)
    plt.title(r"Baro measurements", fontsize=FONTSIZE, fontweight=0, color='black')
    ax5.scatter(baro[:,0], baro[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "baro asl")
    ax5.set_ylabel(r'asl', fontsize=FONTSIZE) 
    ax5.set_xlabel(r'Time [s]', fontsize=FONTSIZE)
    plt.legend(loc='best', fontsize=FONTSIZE)

    # trajectory
    fig6 = plt.figure(figsize=(10, 8))
    ax_t = fig6.add_subplot(111, projection = '3d')
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
    fig7 = plt.figure(figsize=(10, 8))
    a_x = fig7.add_subplot(311)
    plt.title(r"Ground truth of the trajectory", fontsize=FONTSIZE, fontweight=0, color='black', style='italic', y=1.02)
    a_x.plot(gt_pose[:,0],gt_pose[:,1],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt x")
    a_x.legend(loc='best')
    a_x.set_ylabel(r'X [m]',fontsize = FONTSIZE) 
    a_y = fig7.add_subplot(312)
    a_y.plot(gt_pose[:,0],gt_pose[:,2],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt y")
    a_y.legend(loc='best')
    a_y.set_ylabel(r'Y [m]',fontsize = FONTSIZE) 
    a_z = fig7.add_subplot(313)
    a_z.plot(gt_pose[:,0],gt_pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt z")
    a_z.legend(loc='best')
    a_z.set_ylabel(r'Z [m]',fontsize = FONTSIZE)
    a_z.set_xlabel(r'Time [s]',fontsize = FONTSIZE)


    # plot acc and gyro
    fig8 = plt.figure(figsize=(10, 8))
    ac_x = fig8.add_subplot(311)
    plt.title(r"IMU", fontsize=FONTSIZE, fontweight=0, color='black', style='italic', y=1.02)
    ac_x.plot(acc[:,0], acc[:,1] * GRAVITY_MAGNITUDE, color='steelblue',linewidth=1.9, alpha=0.9, label = "acc_x")
    ac_x.legend(loc='best')
    ac_x.set_ylabel(r'acc_x',fontsize = FONTSIZE) 
    ac_y = fig8.add_subplot(312)
    ac_y.plot(acc[:,0], acc[:,2] * GRAVITY_MAGNITUDE, color='steelblue',linewidth=1.9, alpha=0.9, label = "acc_y")
    ac_y.legend(loc='best')
    ac_y.set_ylabel(r'acc_y',fontsize = FONTSIZE) 
    ac_z = fig8.add_subplot(313)
    ac_z.plot(acc[:,0], acc[:,3] * GRAVITY_MAGNITUDE, color='steelblue',linewidth=1.9, alpha=0.9, label = "acc_z")
    ac_z.legend(loc='best')
    ac_z.set_ylabel(r'acc_z',fontsize = FONTSIZE)
    ac_z.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    #
    fig9 = plt.figure(figsize=(10, 8))
    gy_x = fig9.add_subplot(311)
    plt.title(r"gyro", fontsize=FONTSIZE, fontweight=0, color='black', style='italic', y=1.02)
    gy_x.plot(gyr[:,0], gyr[:,1] * DEG_TO_RAD, color='steelblue',linewidth=1.9, alpha=0.9, label = "gyro_x")
    gy_x.legend(loc='best')
    gy_x.set_ylabel(r'gyro_x ',fontsize = FONTSIZE) 
    gy_y = fig9.add_subplot(312)
    gy_y.plot(gyr[:,0], gyr[:,2] * DEG_TO_RAD, color='steelblue',linewidth=1.9, alpha=0.9, label = "gyro_y")
    gy_y.legend(loc='best')
    gy_y.set_ylabel(r'gyro_y',fontsize = FONTSIZE) 
    gy_z = fig9.add_subplot(313)
    gy_z.plot(gyr[:,0], gyr[:,3] * DEG_TO_RAD, color='steelblue',linewidth=1.9, alpha=0.9, label = "gyro_z")
    gy_z.legend(loc='best')
    gy_z.set_ylabel(r'gyro_z',fontsize = FONTSIZE)
    gy_z.set_xlabel(r'Time [s]',fontsize = FONTSIZE)

    plt.show()