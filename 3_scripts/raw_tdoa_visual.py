'''
Test the raw tdoa measurements
The time values are not correct now.
'''
import os, sys
import numpy as np
from numpy import linalg
from matplotlib import pyplot as plt
import rosbag
from scipy import stats
from tkinter.filedialog import askopenfilename

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# extract the rosbag 
os.chdir("/home/william/dsl__projects__uwbDataset/2_data")
curr = os.getcwd()
bagFile = askopenfilename(initialdir = curr, title = "Select rosbag")
# access rosbag 
bag = rosbag.Bag(bagFile)
base = os.path.basename(bagFile)
selected_bag = os.path.splitext(base)[0]

logData1 = [];  t_logData1 = []
logData2 = [];  t_logData2 = []  
logData3 = [];  t_logData3 = []

for topic, msg, t in bag.read_messages(['/cf1/log1', '/cf1/log2', '/cf1/log3']):
    if topic == '/cf1/log1':
        logData1.append(msg.values)
        t_logData1.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
    if topic == '/cf1/log2':
        logData2.append(msg.values)
        t_logData2.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
    if topic == '/cf1/log3':
        logData3.append(msg.values)
        t_logData3.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)

min_t = min(t_logData1 + t_logData2 + t_logData3)

# convert to numpy array
t_logData1 = np.array(t_logData1);     t_logData2 = np.array(t_logData2);   t_logData3 = np.array(t_logData3)
logData1 = np.array(logData1);         logData2 = np.array(logData2);       logData3   = np.array(logData3)

# reset ROS time base
t_logData1 = (t_logData1 - min_t).reshape(-1,1)
t_logData2 = (t_logData2 - min_t).reshape(-1,1)
t_logData3 = (t_logData3 - min_t).reshape(-1,1)

tdoa12 = logData1[:,0]         # tdoa meas. 

tdoa_in_clT = logData1[:,1]    # tdoa in clock T

locodeckTsFreq = 499.2e6 * 128
SPEED_OF_LIGHT = 299792458.0

# calc_tdoa = (SPEED_OF_LIGHT * tdoa_in_clT) / locodeckTsFreq

t_an_r = logData3[:,0]
t_an_t = logData3[:,1]
t1     = logData3[:,2]
t2     = logData3[:,3]
tof    = logData3[:,4]
alpha  = logData3[:,5]

calc_tof = (SPEED_OF_LIGHT * tof) / locodeckTsFreq

# should be identically to tdoa_in_clT
t_tdoa_in_clT = (t2-t1) - alpha*(tof + t_an_t - t_an_r)  

fig = plt.figure(facecolor="white")
ax = plt.subplot(211)
plt.plot(np.squeeze(t_logData1), tdoa12, label = 'tdoa meas')
plt.plot(t_logData3, calc_tof, color = "green", label = 'computed tdoa')
bx = plt.subplot(212)
plt.plot(np.squeeze(t_logData1), tdoa_in_clT, color='blue', label = 'tdoa_in_clT')
plt.scatter(t_logData3, t_tdoa_in_clT, color='red', label = 't_tdoa_in_clT')

fig1 = plt.figure()
bx1 = plt.subplot(3,2,1)
plt.plot(t_logData3, t_an_r, label = 't_an_r')
bx1.legend(loc='best')

bx2 = plt.subplot(3,2,2)
plt.plot(t_logData3, t_an_t, label = 't_an_t')
bx2.legend(loc='best')

bx3 = plt.subplot(3,2,3)
plt.plot(t_logData3, t1, label = 't1')
bx3.legend(loc='best')

bx4 = plt.subplot(3,2,4)
plt.plot(t_logData3, t2, label = 't2')
bx4.legend(loc='best')

bx5 = plt.subplot(3,2,5)
plt.plot(t_logData3, tof, label = 'tof')
bx5.legend(loc='best')

bx6 = plt.subplot(3,2,6)
plt.plot(t_logData3, alpha, label = 'alpha')
bx6.legend(loc='best')
plt.show()



