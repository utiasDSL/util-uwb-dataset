'''
Visualize the nlos data on 0505 experiments
genericLogTopic_log1_Variables: ["tdoa3.d1-2", "tdoa3.d2-1", "tdoa3.d2-1", "tdoa3.snr_1", "tdoa3.powerdiff_1", "tdoa3.snr_2", "tdoa3.powerdiff_2"]
genericLogTopic_log2_Variables: ["tdoa3.an1_rx_snr", "tdoa3.an1_rx_powerdif", "tdoa3.an1_tof",
                                 "tdoa3.an2_rx_snr", "tdoa3.an2_rx_powerdif", "tdoa3.an2_tof"]
                                 
For an_tag nlos metal trial 9, the bias is around -3.9 [m], and the SNR of anchor1 is ~51.09, power_diff is ~17.29
For other metal trials, the bias is around -0.2 ~ -0.5 [m], and the SNR of anchor1 is ~23, power_diff is ~21.89.
The reason is that, in trial 9, it is likely the metal cabenit totally block the los signal. UWB tag receives a multipath signal from anchor 1 (likely reflected from the roof). Therefore, in this trial, the signal quality is better, yet the meas. errors are larger. 
'''
import os, sys
import numpy as np
from numpy import linalg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import matplotlib.style as style
import rosbag
from scipy import stats
from tkinter.filedialog import askopenfilename
# select the matplotlib plotting style
style.use('ggplot')
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# current path of the script
curr = os.path.dirname(sys.argv[0])
# access rosbag
bag_path = os.path.abspath(curr+'/../../2_data/rosbag/0505-data')

bagFile = askopenfilename(initialdir = bag_path, title = "Select rosbag")
bag = rosbag.Bag(bagFile)
base = os.path.basename(bagFile)
selected_bag = os.path.splitext(base)[0]

# ------------ parameter ----------- #
XY_FONTSIZE = 7;   LABEL_SIZE = 12
VISUAL = True;    SAVE_NUMPY = False

# ---------------------------------- #

logData1 = [];  t_logData1 = []
logData2 = [];  t_logData2 = []  
# anchor and tag positions (static)
cf_uwb = [];    anchor1 = [];    anchor2 = []

for topic, msg, t in bag.read_messages(['/cf2/log1', '/cf2/log2', '/vicon/dsl_cf_uwb/dsl_cf_uwb', '/vicon/dsl_anchor1/dsl_anchor1', '/vicon/dsl_anchor2/dsl_anchor2']):
    if topic == '/cf2/log1':
        logData1.append(msg.values)
        t_logData1.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
    if topic == '/cf2/log2':
        logData2.append(msg.values)
        t_logData2.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
    if topic == '/vicon/dsl_cf_uwb/dsl_cf_uwb':
        cf_uwb.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
    if topic == '/vicon/dsl_anchor1/dsl_anchor1':
        anchor1.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
    if topic == '/vicon/dsl_anchor2/dsl_anchor2':
        anchor2.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])

min_t = min(t_logData1 + t_logData2)

# convert to numpy array
t_logData1 = np.array(t_logData1);    t_logData2 = np.array(t_logData2)
logData1   = np.array(logData1);      logData2   = np.array(logData2)
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
snr_an1 = logData1[:,2];     power_diff_an1 = logData1[:,3]
snr_an2 = logData1[:,4];     power_diff_an2 = logData1[:,5]


an1_rx_snr = logData2[:,0];  an1_rx_powerdif = logData2[:,1];  an1_tof = logData2[:,2]
an2_rx_snr = logData2[:,3];  an2_rx_powerdif = logData2[:,4];  an2_tof = logData2[:,5]


# POWER difference data contains -inf values, need to remove these outliers
outlier_an1 = [];   outlier_an2 = [];   outlier_rx_an1 = [];   outlier_rx_an2 = []
for i in range(len(t_logData1)):
    if np.isinf(power_diff_an1[i]):
        outlier_an1.append(i)
    if np.isinf(power_diff_an2[i]):
        outlier_an2.append(i)
        
for i in range(len(t_logData2)):
    if np.isinf(an1_rx_powerdif[i]):
        outlier_rx_an1.append(i)
    if np.isinf(an2_rx_powerdif[i]):
        outlier_rx_an2.append(i)


power_diff_an1 = np.delete(power_diff_an1, outlier_an1)
power_diff_an2 = np.delete(power_diff_an2, outlier_an2)

an1_rx_powerdif = np.delete(an1_rx_powerdif, outlier_rx_an1)
an2_rx_powerdif = np.delete(an2_rx_powerdif, outlier_rx_an2)


# save the meas. error from nlos signal testing 
if SAVE_NUMPY:
    np.save(curr+'/nlos_error/'+selected_bag+'_err.npy', err12)

if VISUAL:
    # visualize tdoa and error
    fig = plt.figure()
    ax = fig.add_subplot(2,1,1)
    ax.set_ylim([-1, 1])
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
    # plot(t_1, power_diff_an1)
    (mu_power1, sigma_power1) = stats.norm.fit(power_diff_an1)
    print("Power diff of anchor 1 mean: ", mu_power1, "std: ", sigma_power1)
    print("\n")
    yhist, xhist, patches = plt.hist(power_diff_an1, bins=48,color='steelblue',alpha=0.75, density=True)
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
    # plot(t_1, power_diff_an2)
    (mu_power2, sigma_power2) = stats.norm.fit(power_diff_an2)
    print("Power diff of anchor 2, mean: ", mu_power2, "std: ", sigma_power2)
    print("\n")
    yhist, xhist, patches = plt.hist(power_diff_an2, bins=48,color='steelblue',alpha=0.75, density=True)
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


