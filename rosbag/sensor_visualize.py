'''
Visualize the sensor measurements
'''
import os, sys
import numpy as np
from numpy import linalg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import rosbag
from scipy import stats
from tkinter.filedialog import askopenfilename

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# current path of the script
pathname = os.path.dirname(sys.argv[0])
bagFile = askopenfilename(initialdir = pathname, title = "Select rosbag")
# access rosbag 
bag = rosbag.Bag(bagFile)
base = os.path.basename(bagFile)
selected_bag = os.path.splitext(base)[0]

# -------------------- extract the rosbag ----------------------------- #
acc = [];   gyro = [];  flow = [];  tdoa = [];  tof = []  
gt_pose = [] 

for topic, msg, t in bag.read_messages(['/accel_data', '/gyro_data', '/flow_data', '/tdoa_data', '/tof_data', '/pose_data']):
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
    if topic == "/pose_data":
        gt_pose.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                        msg.pose.pose.position.x,    msg.pose.pose.position.y,    msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])
        
# convert to numpy array
acc = np.array(acc);     gyro = np.array(gyro)
flow = np.array(flow);   tdoa = np.array(tdoa)
tof = np.array(tof);     gt_pose = np.array(gt_pose)

# select the anchor pair for visualization
# possible anchor ID = [0,1,2,3,4,5,6,7] 
an_i = 0;     an_j = 1

# get the id for tdoa_ij measurements
tdoa_id = np.where((tdoa[:,1]==[an_i])&(tdoa[:,2]==[an_j]))
tdoa_meas = np.squeeze(tdoa[tdoa_id, :])

# visualization 
# UWB TDOA
fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
ax1.scatter(tdoa_meas[:,0], tdoa_meas[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
ax1.legend(loc='best')
ax1.set_xlabel(r'Time [s]')
ax1.set_ylabel(r'TDoA measurement [m]') 
plt.title(r"UWB tdoa measurements, (An{0}, An{1})".format(an_i, an_j), fontsize=13, fontweight=0, color='black')
# Z-range ToF
fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
ax2.scatter(tof[:,0], tof[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "tof measurements")
plt.title(r"Z-range measurements", fontsize=13, fontweight=0, color='black')
# flow pixel
fig3 = plt.figure()
ax3 = fig3.add_subplot(211)
ax3.scatter(flow[:,0], flow[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel x")
bx3 = fig3.add_subplot(212)
bx3.scatter(flow[:,0], flow[:,2], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel y")
plt.title(r"Optical flow measurements", fontsize=13, fontweight=0, color='black')

fig4 = plt.figure()
ax_t = fig4.add_subplot(111, projection = '3d')
ax_t.plot(gt_pose[:,1],gt_pose[:,2],gt_pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9)
# use LaTeX fonts in the plot
ax_t.set_xlabel(r'X [m]')
ax_t.set_ylabel(r'Y [m]')
ax_t.set_zlabel(r'Z [m]')
plt.title(r"Trajectory of the experiment", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )

plt.show()
