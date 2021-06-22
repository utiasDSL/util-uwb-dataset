'''
Visualize the TDOA measurement biases in rosbag
'''
import os, sys
import argparse
import numpy as np
from numpy import linalg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import matplotlib.style as style
import rosbag
from scipy import stats, interpolate
# select the matplotlib plotting style
style.use('ggplot')
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

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
    bag_file = os.path.split(sys.argv[-1])[1]

    # print out
    bag_name = os.path.splitext(bag_file)[0]
    print("visualizing rosbag: " + str(bag_file) + "\n")


    # -------------------- extract the rosbag ----------------------------- #
    acc = [];   gyro = [];  flow = [];  tdoa = [];  tof = [];  baro = [] 
    gt_pose = [] 

    for topic, msg, t in bag.read_messages(['/accel_data', '/gyro_data', '/flow_data', '/tdoa_data', '/tof_data', '/baro_data', '/pose_data']):
        if topic == '/accel_data':
            acc.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.x, msg.y, msg.z])
        if topic == '/gyro_data':
            gyro.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.x, msg.y, msg.z])
        if topic == '/flow_data':
            flow.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.deltaX, msg.deltaY])
        if topic == '/tdoa_data':
            tdoa.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.idA, msg.idB, msg.data])
        if topic == "/tof_data":
            tof.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.zrange])
        if topic == "/baro_data":
            baro.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.asl])
        if topic == "/pose_data":
            gt_pose.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                            msg.pose.pose.position.x,    msg.pose.pose.position.y,    msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
            
    # convert to numpy array
    acc = np.array(acc);     gyro = np.array(gyro)
    flow = np.array(flow);   tdoa = np.array(tdoa)
    tof = np.array(tof);     baro = np.array(baro)
    gt_pose = np.array(gt_pose)

    # select the anchor pair for visualization
    # possible anchor ID = [0,1,2,3,4,5,6,7] 
    an_i = 7;     an_j = 0

    # get the id for tdoa_ij measurements
    tdoa_id = np.where((tdoa[:,1]==[an_i])&(tdoa[:,2]==[an_j]))
    tdoa_meas = np.squeeze(tdoa[tdoa_id, :])
    # compute the ground truth for tdoa_ij
    an_pos_i = anchor_pos[an_i,:].reshape(1,-1)
    an_pos_j = anchor_pos[an_j,:].reshape(1,-1)
    # cf position from vicon measurement
    # To compute the bias, we need to interpolate the vicon measurements. 
    # interpolate the vicon measurements to compute error of tdoa uwb measurements
    f_x = interpolate.splrep(gt_pose[:,0], gt_pose[:,1], s = 0.5)  
    f_y = interpolate.splrep(gt_pose[:,0], gt_pose[:,2], s = 0.5)
    f_z = interpolate.splrep(gt_pose[:,0], gt_pose[:,3], s = 0.5) 

    # synchronized position
    x_interp = interpolate.splev(tdoa_meas[:,0], f_x, der = 0).reshape(-1,1)
    y_interp = interpolate.splev(tdoa_meas[:,0], f_y, der = 0).reshape(-1,1)
    z_interp = interpolate.splev(tdoa_meas[:,0], f_z, der = 0).reshape(-1,1)
    pos_syn = np.concatenate((x_interp, y_interp, z_interp), axis = 1)

    d_i = np.asarray(linalg.norm(an_pos_i - pos_syn, axis = 1))
    d_j = np.asarray(linalg.norm(an_pos_j - pos_syn, axis = 1))
    # measurement model
    d_ij = d_j - d_i

    # bias = tdoa - gt
    bias_ij = tdoa_meas[:,3] - d_ij

    # visualization 
    # UWB TDOA
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(tdoa_meas[:,0], tdoa_meas[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    ax.plot(tdoa_meas[:,0], d_ij, color='red',linewidth=1.5, label = "Vicon ground truth")
    ax.legend(loc='best')
    ax.set_xlabel(r'Time [s]')
    ax.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(an_i, an_j), fontsize=13, fontweight=0, color='black')

    fig1 = plt.figure()
    bx = fig1.add_subplot(111)
    bx.scatter(tdoa_meas[:,0], bias_ij, color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa biases")
    bx.legend(loc='best')
    bx.set_xlabel(r'Time [s]')
    bx.set_ylabel(r'TDoA bias [m]') 
    plt.title(r"UWB tdoa biases, (An{0}, An{1})".format(an_i, an_j), fontsize=13, fontweight=0, color='black')

    plt.show()





