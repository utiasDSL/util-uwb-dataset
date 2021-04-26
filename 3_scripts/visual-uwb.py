'''
Visualize the UWB TDOA meas.
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
# load the anchor pose
anchor_pos = np.load(curr+'/survey/AnchorPos_0415.npy')
anchor_qaut = np.load(curr+'/survey/AnchorQuat_0415.npy')
# access rosbag
bag_path = os.path.abspath(curr+'/../2_data/rosbag/')
bagFile = askopenfilename(initialdir = bag_path, title = "Select rosbag")
bag = rosbag.Bag(bagFile)
base = os.path.basename(bagFile)
selected_bag = os.path.splitext(base)[0]
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

# tdoa id
tdoa_70 = np.where((tdoa[:,1]==7)&(tdoa[:,2]==0))
tdoa_01 = np.where((tdoa[:,1]==0)&(tdoa[:,2]==1))
tdoa_12 = np.where((tdoa[:,1]==1)&(tdoa[:,2]==2))
tdoa_23 = np.where((tdoa[:,1]==2)&(tdoa[:,2]==3))

tdoa_34 = np.where((tdoa[:,1]==3)&(tdoa[:,2]==4))
tdoa_45 = np.where((tdoa[:,1]==4)&(tdoa[:,2]==5))
tdoa_56 = np.where((tdoa[:,1]==5)&(tdoa[:,2]==6))
tdoa_67 = np.where((tdoa[:,1]==6)&(tdoa[:,2]==7))


r_70 = np.squeeze(tdoa[tdoa_70, :])
r_01 = np.squeeze(tdoa[tdoa_01, :])
r_12 = np.squeeze(tdoa[tdoa_12, :])
r_23 = np.squeeze(tdoa[tdoa_23, :])

r_34 = np.squeeze(tdoa[tdoa_34, :])
r_45 = np.squeeze(tdoa[tdoa_45, :])
r_56 = np.squeeze(tdoa[tdoa_56, :])
r_67 = np.squeeze(tdoa[tdoa_67, :])

# visualization 
# UWB TDOA
fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
ax1.scatter(r_70[:,0], r_70[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax1.legend(loc='best')
ax1.set_xlabel(r'Time [s]')
ax1.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(7, 0), fontsize=13, fontweight=0, color='black')

fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
ax2.scatter(r_01[:,0], r_01[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax2.legend(loc='best')
ax2.set_xlabel(r'Time [s]')
ax2.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(0, 1), fontsize=13, fontweight=0, color='black')

fig3 = plt.figure()
ax3 = fig3.add_subplot(111)
ax3.scatter(r_12[:,0], r_12[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax3.legend(loc='best')
ax3.set_xlabel(r'Time [s]')
ax3.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(1, 2), fontsize=13, fontweight=0, color='black')

fig4 = plt.figure()
ax4 = fig4.add_subplot(111)
ax4.scatter(r_23[:,0], r_23[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax4.legend(loc='best')
ax4.set_xlabel(r'Time [s]')
ax4.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(2, 3), fontsize=13, fontweight=0, color='black')

fig5 = plt.figure()
ax5 = fig5.add_subplot(111)
ax5.scatter(r_34[:,0], r_34[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax5.legend(loc='best')
ax5.set_xlabel(r'Time [s]')
ax5.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(3, 4), fontsize=13, fontweight=0, color='black')

fig6 = plt.figure()
ax6 = fig6.add_subplot(111)
ax6.scatter(r_45[:,0], r_45[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax6.legend(loc='best')
ax6.set_xlabel(r'Time [s]')
ax6.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(4, 5), fontsize=13, fontweight=0, color='black')

fig7 = plt.figure()
ax7 = fig7.add_subplot(111)
ax7.scatter(r_56[:,0], r_56[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax7.legend(loc='best')
ax7.set_xlabel(r'Time [s]')
ax7.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(5, 6), fontsize=13, fontweight=0, color='black')

fig8 = plt.figure()
ax8 = fig8.add_subplot(111)
ax8.scatter(r_67[:,0], r_67[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax8.legend(loc='best')
ax8.set_xlabel(r'Time [s]')
ax8.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(6, 7), fontsize=13, fontweight=0, color='black')


plt.show()


