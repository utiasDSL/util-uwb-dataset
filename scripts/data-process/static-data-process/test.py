'''
test code
'''
import os, sys
import argparse
import numpy as np
from numpy import linalg
import csv
import rosbag
from scipy import stats

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=2)
    args = parser.parse_args()

    # access rosbag
    ros_bag = "/home/wenda/dsl__projects__uwbDataset/UWB_Dataset/rosbag/static-signal-testing/los-signal-testing/lineTest/line_t12.bag"
    bag = rosbag.Bag(ros_bag)
    # bag_file = os.path.split(sys.argv[-1])[1]

    # print out
    # bag_name = os.path.splitext(bag_file)[0]
    # print("converting rosbag: " + str(bag_file) + " to csv \n")


    # ------ access bag --------- #
    logData1 = [];  t_log1 = []  
    logData2 = [];  t_log2 = []
    # anchor and tag positions (static)
    cf_uwb  = [];   an1_p = [];      an2_p = []
    cf_quat = [];   an1_quat = [];   an2_quat = []

    for topic, msg, t in bag.read_messages(['/cf2/log1', '/cf2/log2', '/vicon/dsl_cf_uwb/dsl_cf_uwb', 
                                            '/vicon/dsl_anchor1/dsl_anchor1', '/vicon/dsl_anchor2/dsl_anchor2']):
        if topic == '/cf2/log1':
            logData1.append(msg.values)
            t_log1.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
        if topic == '/cf2/log2':
            logData2.append(msg.values)
            t_log2.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
        if topic == '/vicon/dsl_cf_uwb/dsl_cf_uwb':
            cf_uwb.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            cf_quat.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        if topic == '/vicon/dsl_anchor1/dsl_anchor1':
            an1_p.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            an1_quat.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        if topic == '/vicon/dsl_anchor2/dsl_anchor2':
            an2_p.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            an2_quat.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
    
    min_t = min(t_log1 + t_log2)
    
    # convert to numpy array
    logData1 = np.array(logData1);      logData2 = np.array(logData2)
    t_log1 = np.array(t_log1);          t_log2 = np.array(t_log2)
    # reset ROS time base
    t_1 = (t_log1 - min_t).reshape(-1,1)
    t_2 = (t_log2 - min_t).reshape(-1,1)
    
    cf_uwb = np.array(cf_uwb)            
    uwb_p = np.mean(cf_uwb, 0)     # position of uwb tag
    an1_p = np.array(an1_p);   an2_p = np.array(an2_p)
    an1_p = np.mean(an1_p, 0); an2_p = np.mean(an2_p, 0)  # positions of anchor 1 and 2
    
    gt_d_12 = linalg.norm(an2_p - uwb_p) - linalg.norm(an1_p - uwb_p)
    gt_an = linalg.norm(an1_p - an2_p)
    

    d_an1_t = linalg.norm(an1_p - uwb_p)
    d_an2_t = linalg.norm(an2_p - uwb_p)

    

    txt_file = '/home/wenda/dsl__projects__uwbDataset/UWB_Dataset/csv/pose1.txt' 
    with open(txt_file,"w") as f:
        f.write('an1_p,'+ str(an1_p[0])+','+str(an1_p[1])+','+ str(an1_p[0]))




