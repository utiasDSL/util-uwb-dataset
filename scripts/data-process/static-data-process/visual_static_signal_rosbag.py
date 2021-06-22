'''
visualize static signal testing data
'''
import os, sys
import argparse
import numpy as np
from numpy import linalg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import matplotlib.style as style
import rosbag
from scipy import stats
# select the matplotlib plotting style
style.use('ggplot')
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'


# ------------ parameter ----------- #
XY_FONTSIZE = 7;   LABEL_SIZE = 12
SAVE_NUMPY = False
# # for 4K screen distplay
# import matplotlib as mpl
# mpl.rcParams['figure.dpi'] = 300
# ---------------------------------- #

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=1)
    args = parser.parse_args()

    # access rosbag
    ros_bag = args.i[0]
    bag = rosbag.Bag(ros_bag)
    bag_file = os.path.split(sys.argv[-1])[1]

    # print out
    bag_name = os.path.splitext(bag_file)[0]
    print("visualizing rosbag: " + str(bag_file) + "\n")


    # ------ access bag --------- #
    logData1 = [];  t_logData1 = []
    logData2 = [];  t_logData2 = []  
    # anchor and tag positions (static)
    cf_uwb  = [];    anchor1 = [];    anchor2 = []
    cf_quat = [];   an1_quat = [];   an2_quat = []
    obstacle = []
    for topic, msg, t in bag.read_messages(['/cf2/log1', '/cf2/log2', '/vicon/dsl_cf_uwb/dsl_cf_uwb', 
                                            '/vicon/dsl_anchor1/dsl_anchor1', '/vicon/dsl_anchor2/dsl_anchor2',
                                            '/vicon/markers']):
        if topic == '/cf2/log1':
            logData1.append(msg.values)
            t_logData1.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
        if topic == '/cf2/log2':
            logData2.append(msg.values)
            t_logData2.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
        if topic == '/vicon/dsl_cf_uwb/dsl_cf_uwb':
            cf_uwb.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            cf_quat.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        if topic == '/vicon/dsl_anchor1/dsl_anchor1':
            anchor1.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            an1_quat.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        if topic == '/vicon/dsl_anchor2/dsl_anchor2':
            anchor2.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            an2_quat.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        if topic == '/vicon/markers' and obstacle == []:
            idx=[]
            for i in range(len(msg.markers)):
                if msg.markers[i].marker_name == '':     # four markers on the obstacle are without name
                    idx.append(i)  
            obstacle.append([[msg.markers[idx[0]].translation.x, msg.markers[idx[0]].translation.y, msg.markers[idx[0]].translation.z], 
                            [msg.markers[idx[1]].translation.x, msg.markers[idx[1]].translation.y, msg.markers[idx[1]].translation.z],
                            [msg.markers[idx[2]].translation.x, msg.markers[idx[2]].translation.y, msg.markers[idx[2]].translation.z],
                            [msg.markers[idx[3]].translation.x, msg.markers[idx[3]].translation.y, msg.markers[idx[3]].translation.z],
                            [msg.markers[idx[0]].translation.x, msg.markers[idx[0]].translation.y, 0.0], 
                            [msg.markers[idx[1]].translation.x, msg.markers[idx[1]].translation.y, 0.0],
                            [msg.markers[idx[2]].translation.x, msg.markers[idx[2]].translation.y, 0.0],
                            [msg.markers[idx[3]].translation.x, msg.markers[idx[3]].translation.y, 0.0],
                            ])


    min_t = min(t_logData1 + t_logData2)

    # convert to numpy array
    t_logData1 = np.array(t_logData1);    t_logData2 = np.array(t_logData2)
    logData1   = np.array(logData1);      logData2   = np.array(logData2)
    obstacle = np.squeeze(np.array(obstacle)) / 1000.0
    # obstacle = obstacle.reshape(-1,3)

    # reset ROS time base
    t_1 = (t_logData1 - min_t).reshape(-1,1)
    t_2 = (t_logData2 - min_t).reshape(-1,1)

    # compute ground truth
    cf_uwb = np.array(cf_uwb);   anchor1 = np.array(anchor1);   anchor2 = np.array(anchor2)
    cf_uwb = np.mean(cf_uwb, 0); anchor1 = np.mean(anchor1, 0); anchor2 = np.mean(anchor2, 0)

    gt_d_12 = linalg.norm(anchor2 - cf_uwb) - linalg.norm(anchor1 - cf_uwb)
    gt_an = linalg.norm(anchor1 - anchor2)
    # tdoa meas
    tdoa12 = logData1[:,0]
    err12 = tdoa12 - gt_d_12

    tdoa21 = logData1[:,1]

    # extract data
    snr_an1 = logData1[:,2];     power_dif_an1 = logData1[:,3]
    snr_an2 = logData1[:,4];     power_dif_an2 = logData1[:,5]

    an1_rx_snr = logData2[:,0];  an1_rx_powerdif = logData2[:,1];  an1_tof = logData2[:,2]
    an2_rx_snr = logData2[:,3];  an2_rx_powerdif = logData2[:,4];  an2_tof = logData2[:,5]


    # POWER difference data contains -inf values, need to remove these outliers
    outlier_an1 = [];   outlier_an2 = [];   outlier_rx_an1 = [];   outlier_rx_an2 = []
    for i in range(len(t_logData1)):
        if np.isinf(power_dif_an1[i]):
            outlier_an1.append(i)
        if np.isinf(power_dif_an2[i]):
            outlier_an2.append(i)
            
    for i in range(len(t_logData2)):
        if np.isinf(an1_rx_powerdif[i]):
            outlier_rx_an1.append(i)
        if np.isinf(an2_rx_powerdif[i]):
            outlier_rx_an2.append(i)


    power_dif_an1 = np.delete(power_dif_an1, outlier_an1)
    power_dif_an2 = np.delete(power_dif_an2, outlier_an2)

    an1_rx_powerdif = np.delete(an1_rx_powerdif, outlier_rx_an1)
    an2_rx_powerdif = np.delete(an2_rx_powerdif, outlier_rx_an2)

    if obstacle.size > 0:
        # visualize the anchor, tag and obstacle
        fig_ob = plt.figure()
        ob_x = fig_ob.add_subplot(111, projection = '3d')
        ob_x.scatter(obstacle[:,0], obstacle[:,1], obstacle[:,2], marker='o',color='navy')
        ob_x.scatter(cf_uwb[0],   cf_uwb[1],  cf_uwb[2], marker='o',color='green')
        ob_x.scatter(anchor1[0], anchor1[1], anchor1[2], marker='o',color='red')
        ob_x.scatter(anchor2[0], anchor2[1], anchor2[2], marker='o',color='red')
        # plot the line segement
        ob_x.plot([cf_uwb[0], anchor1[0]], [cf_uwb[1], anchor1[1]], [cf_uwb[2], anchor1[2]])
        ob_x.plot([cf_uwb[0], anchor2[0]], [cf_uwb[1], anchor2[1]], [cf_uwb[2], anchor2[2]])
        ob_x.plot([anchor1[0], anchor2[0]], [anchor1[1], anchor2[1]], [anchor1[2], anchor2[2]])
        ob_x.set_xlim3d(-3.5, 3.5)  
        ob_x.set_ylim3d(-3.5, 3.5)  
        ob_x.set_zlim3d(0.0, 3.0)  
        ob_x.set_xlabel(r'X [m]')
        ob_x.set_ylabel(r'Y [m]')
        ob_x.set_zlabel(r'Z [m]')
        plt.legend(['Obstacle','Tag position','Anchor position'])
        plt.show()


    # save the meas. error from nlos signal testing 
    if SAVE_NUMPY:
        np.save('/nlos_testing/'+ bag_name +'_err.npy', err12)
        # np.save(curr+'/los_testing/'+ bag_name +'_err.npy', err12)


    # visualize tdoa and error
    fig = plt.figure()
    ax = fig.add_subplot(2,1,1)
    ax.set_ylim([-3.5, 3.5])
    ax.plot(t_1, tdoa12)
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('TDOA meas. [m]', fontsize=12)
    ax_1 = fig.add_subplot(2,1,2)
    ax_1.plot(t_1, err12)
    ax_1.set_ylim([-1, 1])
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('meas. error [m]', fontsize=12)

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
    # plot(t_1, snr_an1)
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
    # plot(t_1, power_dif_an1)
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
    # plot(t_1, snr_an2)
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
    # plot(t_1, power_dif_an2)
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
    # plot(t_2, an1_rx_snr)
    (mu_snr_rc_an1, sigma_snr_rc_an1) = stats.norm.fit(an1_rx_snr)
    print("SNR received by an1, mean: ", mu_snr_rc_an1, "std: ",sigma_snr_rc_an1)
    print("\n")
    yhist, xhist, patches = plt.hist(an1_rx_snr, bins=48,color='steelblue',alpha=0.75, density=True)
    plt.axvline(x=mu_snr_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('SNR received by an1', fontsize = LABEL_SIZE)
    plt.ylabel('PDF', fontsize = LABEL_SIZE)

    px2 = fig2.add_subplot(3,2,2)
    # plot(t_2, an2_rx_snr)
    (mu_snr_rc_an2, sigma_snr_rc_an2) = stats.norm.fit(an2_rx_snr)
    print("SNR received by an2, mean: ", mu_snr_rc_an2, "std: ",sigma_snr_rc_an2)
    print("\n")
    yhist, xhist, patches = plt.hist(an2_rx_snr, bins=48,color='steelblue',alpha=0.75, density=True)
    plt.axvline(x=mu_snr_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('SNR received by an2', fontsize = LABEL_SIZE)
    plt.ylabel('PDF', fontsize = LABEL_SIZE)

    px3 = fig2.add_subplot(3,2,3)
    # plot(t_2, an1_rx_powerdif)
    (mu_powerdif_rc_an1, sigma_powerdif_rc_an1) = stats.norm.fit(an1_rx_powerdif)
    print("Power difference received by an1, mean: ", mu_powerdif_rc_an1, "std: ", sigma_powerdif_rc_an1)
    print("\n")
    yhist, xhist, patches = plt.hist(an1_rx_powerdif, bins=48, color='steelblue', alpha=0.75, density=True)
    plt.axvline(x=mu_powerdif_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('Power difference received by an1', fontsize = LABEL_SIZE)
    plt.ylabel('PDF', fontsize = LABEL_SIZE)

    px4 = fig2.add_subplot(3,2,4)
    # plot(t_2, an2_rx_powerdif)
    (mu_powerdif_rc_an2, sigma_powerdif_rc_an2) = stats.norm.fit(an2_rx_powerdif)
    print("Power difference received by an2, mean: ", mu_powerdif_rc_an2, "std: ", sigma_powerdif_rc_an2)
    print("\n")
    yhist, xhist, patches = plt.hist(an2_rx_powerdif, bins=48, color='steelblue', alpha=0.75, density=True)
    plt.axvline(x=mu_powerdif_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('Power difference received by an2', fontsize = LABEL_SIZE)
    plt.ylabel('PDF', fontsize = LABEL_SIZE)

    px5 = fig2.add_subplot(3,2,5)
    # plot(t_2, an1_tof)
    (mu_tof_rc_an1, sigma_tof_rc_an1) = stats.norm.fit(an1_tof)
    print("Tof received by an1, mean: ", mu_tof_rc_an1, "std: ", sigma_tof_rc_an1)
    print("\n")
    yhist, xhist, patches = plt.hist(an1_tof, bins=20, color='steelblue', alpha=0.75, density=True)
    plt.axvline(x=mu_tof_rc_an1, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('tof received by an1', fontsize = LABEL_SIZE)
    plt.ylabel('PDF', fontsize = LABEL_SIZE)

    px6 = fig2.add_subplot(3,2,6)
    # plot(t_2, an2_tof)
    (mu_tof_rc_an2, sigma_tof_rc_an2) = stats.norm.fit(an2_tof)
    print("Tof received by an1, mean: ", mu_tof_rc_an2, "std: ", sigma_tof_rc_an2)
    print("\n")
    yhist, xhist, patches = plt.hist(an2_tof, bins=20, color='steelblue', alpha=0.75, density=True)
    plt.axvline(x=mu_tof_rc_an2, alpha=1.0, linestyle ='--', color = 'red')
    plt.xlabel('tof received by an1', fontsize = LABEL_SIZE)
    plt.ylabel('PDF', fontsize = LABEL_SIZE)

    plt.show()



