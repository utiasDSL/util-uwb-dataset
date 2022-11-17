'''
    Convert the static nlos signal testing data into CSV files
    Provide the position of the obstacles
'''
import os, sys
import argparse
import numpy as np
from numpy import linalg
import csv
import rosbag
from itertools import zip_longest

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=2)
    args = parser.parse_args()

    # access rosbag
    ros_bag = args.i[0]
    bag = rosbag.Bag(ros_bag)
    bag_file = os.path.split(sys.argv[-2])[1]
    # print out
    print("converting rosbag: " + str(bag_file) + " to csv \n")

    # the folder to save data
    data_name = os.path.splitext(bag_file)[0]
    folder = args.i[1]

    # ------ access bag --------- #
    logData1 = [];  t_log1 = []  
    logData2 = [];  t_log2 = []
    # anchor and tag positions (static)
    cf_uwb  = [];   an1_p = [];      an2_p = []
    cf_quat = [];   an1_quat = [];   an2_quat = []
    obstacle = []
    for topic, msg, t in bag.read_messages(['/cf2/log1', '/cf2/log2', '/vicon/dsl_cf_uwb/dsl_cf_uwb', 
                                            '/vicon/dsl_anchor1/dsl_anchor1', '/vicon/dsl_anchor2/dsl_anchor2',
                                            '/vicon/markers']):
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
        if topic == '/vicon/markers' and obstacle == []:
            idx=[]
            for i in range(len(msg.markers)):
                if msg.markers[i].marker_name == '':     # four markers on the obstacle are without name
                    idx.append(i)  
            if len(idx) == 4:
                obstacle.append([[msg.markers[idx[0]].translation.x, msg.markers[idx[0]].translation.y, msg.markers[idx[0]].translation.z], 
                                [msg.markers[idx[1]].translation.x, msg.markers[idx[1]].translation.y, msg.markers[idx[1]].translation.z],
                                [msg.markers[idx[2]].translation.x, msg.markers[idx[2]].translation.y, msg.markers[idx[2]].translation.z],
                                [msg.markers[idx[3]].translation.x, msg.markers[idx[3]].translation.y, msg.markers[idx[3]].translation.z]
                                ])

    min_t = min(t_log1 + t_log2)
    
    # convert to numpy array
    logData1 = np.array(logData1);      logData2 = np.array(logData2)
    t_log1 = np.array(t_log1);          t_log2 = np.array(t_log2)
    # obstacle
    obstacle = np.squeeze(np.array(obstacle)) / 1000.0
    obstacle = np.round(obstacle, 5)

    # reset ROS time base
    t_1 = (t_log1 - min_t).reshape(-1,1)
    t_2 = (t_log2 - min_t).reshape(-1,1)
    
    cf_uwb = np.array(cf_uwb);        cf_quat = np.array(cf_quat)
    tag_p = np.mean(cf_uwb, 0);       tag_quat = np.mean(cf_quat, 0)
    
    an1_p = np.array(an1_p);          an2_p = np.array(an2_p)
    an1_p = np.mean(an1_p, 0);        an2_p = np.mean(an2_p, 0)  
    
    an1_quat = np.array(an1_quat);    an2_quat = np.array(an2_quat)
    an1_quat = np.mean(an1_quat, 0);  an2_quat = np.mean(an2_quat, 0)

    # round the pose to 5 decimal
    tag_p = np.round(tag_p, 5);       tag_quat = np.round(tag_quat, 5)
    an1_p = np.round(an1_p, 5);       an2_p = np.round(an2_p, 5)
    an1_quat = np.round(an1_quat, 5); an2_quat = np.round(an2_quat, 5)
    

    gt_d_12 = linalg.norm(an2_p - tag_p) - linalg.norm(an1_p - tag_p)
    gt_an = linalg.norm(an1_p - an2_p)
    
    # extract data
    tdoa12 = logData1[:,0]
    # compute the tdoa12 err
    err12 = tdoa12 - gt_d_12

    tdoa21 = logData1[:,1]
    
    snr_an1 = logData1[:,2];     power_dif_an1 = logData1[:,3]
    snr_an2 = logData1[:,4];     power_dif_an2 = logData1[:,5]

    an1_rx_snr = logData2[:,0];  an1_rx_powerdif = logData2[:,1];  an1_tof = logData2[:,2]
    an2_rx_snr = logData2[:,3];  an2_rx_powerdif = logData2[:,4];  an2_tof = logData2[:,5]
    
    # POWER difference data contains -inf values, need to remove these outliers
    outlier_an1 = [];   outlier_an2 = [];   outlier_rx_an1 = [];   outlier_rx_an2 = []
    for i in range(len(logData1)):
        if np.isinf(power_dif_an1[i]):
            outlier_an1.append(i)
        if np.isinf(power_dif_an2[i]):
            outlier_an2.append(i)
            
    for i in range(len(logData2)):
        if np.isinf(an1_rx_powerdif[i]):
            outlier_rx_an1.append(i)
        if np.isinf(an2_rx_powerdif[i]):
            outlier_rx_an2.append(i)
    # remove inf values        
    power_dif_an1 = np.delete(power_dif_an1, outlier_an1)
    power_dif_an2 = np.delete(power_dif_an2, outlier_an2)

    an1_rx_powerdif = np.delete(an1_rx_powerdif, outlier_rx_an1)
    an2_rx_powerdif = np.delete(an2_rx_powerdif, outlier_rx_an2)
    
    # save the data into csv files
    header = ['tdoa12',     'tdoa21',          'snr_an1',  'power_dif_an1',  'snr_an2',         'power_dif_an2', 
              'an1_rx_snr', 'an1_rx_powerdif', 'an1_tof',  'an2_rx_snr',     'an2_rx_powerdif', 'an2_tof' ]
    row = zip_longest(tdoa12, tdoa21, snr_an1, power_dif_an1, snr_an2, power_dif_an2,
                      an1_rx_snr, an1_rx_powerdif, an1_tof, an2_rx_snr, an2_rx_powerdif, an2_tof,
                      fillvalue='')
    
    csv_file = folder + '/' + data_name +'_data.csv'
    with open(csv_file,"w") as f:
        csv.writer(f).writerow(header)
        csv.writer(f).writerows(row)

    # save the pose of the 2 anchors and the tag into txt file
    txt_file = folder + '/' + data_name +'_pose.txt'
    with open(txt_file,"w") as f:
        f.write('an1_p,' + str(an1_p[0]) + ',' + str(an1_p[1]) + ',' + str(an1_p[2])+'\n')
        f.write('an2_p,' + str(an2_p[0]) + ',' + str(an2_p[1]) + ',' + str(an2_p[2])+'\n' )
        f.write('tag_p,' + str(tag_p[0]) + ',' + str(tag_p[1]) + ',' + str(tag_p[2])+'\n')
        f.write('an1_quat,' + str(an1_quat[0]) + ',' + str(an1_quat[1]) + ',' + 
                              str(an1_quat[2]) + ',' + str(an1_quat[3])+'\n')

        f.write('an2_quat,' + str(an2_quat[0]) + ',' + str(an2_quat[1]) + ',' + 
                              str(an2_quat[2]) + ',' + str(an2_quat[3])+'\n')

        f.write('tag_quat,' + str(tag_quat[0]) + ',' + str(tag_quat[1]) + ',' + 
                              str(tag_quat[2]) + ',' + str(tag_quat[3])+'\n')
        if len(obstacle) != 0:
            f.write('obs_m1,' + str(obstacle[0,0]) + ',' + str(obstacle[0,1]) + ',' +str(obstacle[0,2]) +'\n')
            f.write('obs_m2,' + str(obstacle[1,0]) + ',' + str(obstacle[1,1]) + ',' +str(obstacle[1,2]) +'\n')
            f.write('obs_m3,' + str(obstacle[2,0]) + ',' + str(obstacle[2,1]) + ',' +str(obstacle[2,2]) +'\n')
            f.write('obs_m4,' + str(obstacle[3,0]) + ',' + str(obstacle[3,1]) + ',' +str(obstacle[3,2]) +'\n')
                             
































