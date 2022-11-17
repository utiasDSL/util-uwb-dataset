'''
    Visualize the TDOA measurement biases in rosbag
'''
import os, sys
sys.path.append("../")
import argparse
import numpy as np
from numpy import linalg
from pyquaternion import Quaternion
import matplotlib
from matplotlib import pyplot as plt
import rosbag

from utility.praser import sync_pos

FONTSIZE = 18;   TICK_SIZE = 16
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

# translation vector from the quadrotor to UWB tag
t_uv = np.array([-0.01245, 0.00127, 0.0908]).reshape(-1,1)  


if __name__ == "__main__":
    # ---------------- access anchor survey and rosbag ---------------- #
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

    # access rosbag
    ros_bag = args.i[1]
    bag = rosbag.Bag(ros_bag)
    bag_name = os.path.split(sys.argv[-1])[1]
    # print out
    print("visualizing rosbag: " + str(bag_name) + "\n")

    # -------------------- extract the rosbag ----------------------------- #
    tdoa = [];      gt_pose = [] 

    for topic, msg, t in bag.read_messages(['/tdoa_data', '/pose_data']):
        if topic == '/tdoa_data':
            tdoa.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.idA, msg.idB, msg.data])
        if topic == "/pose_data":
            gt_pose.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                            msg.pose.pose.position.x,    msg.pose.pose.position.y,    msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
            
    # convert to numpy array
    tdoa = np.array(tdoa);    gt_pose = np.array(gt_pose)

    # external calibration: convert the gt_position to UWB antenna center
    uwb_p = np.zeros((len(gt_pose), 3))
    for idx in range(len(gt_pose)):
        q_cf =Quaternion([gt_pose[idx,7], gt_pose[idx,4], gt_pose[idx,5], gt_pose[idx,6]])    # [q_w, q_x, q_y, q_z]
        C_iv = q_cf.rotation_matrix       # rotation matrix from vehicle body frame to inertial frame

        uwb_ac = C_iv.dot(t_uv) + gt_pose[idx,1:4].reshape(-1,1)
        uwb_p[idx,:] = uwb_ac.reshape(1,-1)     # gt of uwb tag

    # select the anchor pair for visualization
    # possible anchor pair IDs 
    # TDOA2: 7-0, 0-1, 1-2, 2-3, 3-4, 4-5, 5-6, 6-7
    # TDOA3: i,j \in {0,1,2,3,4,5,6,7} 
    an_i = 0;     an_j = 1

    # get the id for tdoa_ij measurements
    tdoa_id = np.where((tdoa[:,1]==[an_i])&(tdoa[:,2]==[an_j]))
    tdoa_meas = np.squeeze(tdoa[tdoa_id, :])
    # compute the ground truth for tdoa_ij
    an_pos_i = anchor_pos[an_i,:].reshape(1,-1)
    an_pos_j = anchor_pos[an_j,:].reshape(1,-1)

    # interpolate vicon measurement and synchronized position values to uwb time
    pos_syn = sync_pos(gt_pose[:,0], uwb_p, tdoa_meas[:,0])

    d_i = np.asarray(linalg.norm(an_pos_i - pos_syn, axis = 1))
    d_j = np.asarray(linalg.norm(an_pos_j - pos_syn, axis = 1))
    # measurement model
    d_ij = d_j - d_i

    # bias = tdoa - gt
    bias_ij = tdoa_meas[:,3] - d_ij

    # visualization 
    # UWB TDOA
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111)
    ax.scatter(tdoa_meas[:,0], tdoa_meas[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    ax.plot(tdoa_meas[:,0], d_ij, color='red',linewidth=1.5, label = "Vicon ground truth")
    ax.legend(loc='best',fontsize = FONTSIZE)
    ax.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    ax.set_ylabel(r'TDOA measurement [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA measurements, (An{0}, An{1})".format(an_i, an_j), fontsize=FONTSIZE, fontweight=0, color='black')

    fig1 = plt.figure(figsize=(10, 8))
    bx = fig1.add_subplot(111)
    bx.scatter(tdoa_meas[:,0], bias_ij, color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa biases")
    bx.legend(loc='best',fontsize = FONTSIZE)
    bx.set_xlabel(r'Time [s]',fontsize = FONTSIZE)
    bx.set_ylabel(r'TDOA bias [m]',fontsize = FONTSIZE) 
    plt.title(r"UWB TDOA biases, (An{0}, An{1})".format(an_i, an_j), fontsize=FONTSIZE, fontweight=0, color='black')

    plt.show()





