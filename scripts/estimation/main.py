'''
main file for eskf estimation
'''
#!/usr/bin/env python3
import argparse
import os, sys
import rosbag
from cf_msgs.msg import Accel, Gyro, Flow, Tdoa, Tof 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg
import math
from pyquaternion import Quaternion
from scipy import interpolate            
from sklearn.metrics import mean_squared_error

from eskf_class import ESKF
from plot_util import plot_pos, plot_pos_err, plot_traj

'''help function for timestamp'''
def isin(t_np,t_k):
    # check if t_k is in the numpy array t_np. 
    # If t_k is in t_np, return the index and bool = Ture.
    # else return 0 and bool = False
    if t_k in t_np:
        res = np.where(t_np == t_k)
        b = True
        return res[0][0], b
    b = False
    return 0, b


if __name__ == "__main__":
    # load data
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=2)
    args = parser.parse_args()
    
    # access the survey results
    anchor_npz = args.i[0]
    anchor_survey = np.load(anchor_npz)
    # select anchor constellations
    anchor_position = anchor_survey['an_pos']
    # print out
    anchor_file = os.path.split(sys.argv[-2])[1]
    print("\nselecting anchor constellation " + str(anchor_file) + "\n")

    # access rosbag file
    ros_bag = args.i[1]
    bag = rosbag.Bag(ros_bag)
    bagFile = os.path.split(sys.argv[-1])[1]
    # print out
    bag_name = os.path.splitext(bagFile)[0]
    print("visualizing rosbag: " + str(bagFile) + "\n")

    # -------------------- start extract the rosbag ------------------------ #
    pos_vicon=[];      t_vicon=[]; 
    uwb=[];            t_uwb=[]; 
    imu=[];            t_imu=[]; 
    for topic, msg, t in bag.read_messages(['/pose_data', '/tdoa_data', '/imu_data']):
        if topic == '/pose_data':
            pos_vicon.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            t_vicon.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
        if topic == '/tdoa_data':
            uwb.append([msg.idA, msg.idB, msg.data])
            t_uwb.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
        if topic == '/imu_data':
            imu.append([msg.linear_acceleration.x, msg.linear_acceleration.y,  msg.linear_acceleration.z,\
                        msg.angular_velocity.x,    msg.angular_velocity.y,     msg.angular_velocity.z     ])
            t_imu.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)


    min_t = min(t_uwb + t_imu + t_vicon)
    # get the vicon information from min_t
    t_vicon = np.array(t_vicon);              
    idx = np.argwhere(t_vicon > min_t);     
    t_vicon = t_vicon[idx]; 
    pos_vicon = np.squeeze(np.array(pos_vicon)[idx,:])

    # sensor
    t_imu = np.array(t_imu);       imu = np.array(imu);  
    t_uwb = np.array(t_uwb);       uwb = np.array(uwb);     

    # reset ROS time base
    t_vicon = (t_vicon - min_t).reshape(-1,1)
    t_imu = (t_imu - min_t).reshape(-1,1)
    t_uwb = (t_uwb - min_t).reshape(-1,1)

    # ----------------------- INITIALIZATION OF EKF -------------------------#
    # Create a compound vector t with a sorted merge of all the sensor time bases
    time = np.sort(np.concatenate((t_imu, t_uwb)))
    t = np.unique(time)
    K = t.shape[0]

    # create the object of ESKF
    eskf = ESKF(K)

    print('timestep: %f' % K)
    print('\nStart state estimation')
    for k in range(1,K):               # k = 1 ~ K-1
        # Find what measurements are available at the current time (help function: isin() )
        imu_k,  imu_check  = isin(t_imu,   t[k-1])
        uwb_k,  uwb_check  = isin(t_uwb,   t[k-1])
        dt = t[k]-t[k-1]

        # ESKF Prediction
        eskf.predict(imu[imu_k,:], dt, imu_check, k)
        
        # ESKF Correction
        if uwb_check:            # if we have UWB measurement
            eskf.UWB_correct(uwb[uwb_k,:], anchor_position, k)

    print('Finish the state estimation\n')

    ## compute the error    
    # interpolate Vicon measurements
    f_x = interpolate.splrep(t_vicon, pos_vicon[:,0], s = 0)
    f_y = interpolate.splrep(t_vicon, pos_vicon[:,1], s = 0)
    f_z = interpolate.splrep(t_vicon, pos_vicon[:,2], s = 0)
    x_interp = interpolate.splev(t, f_x, der = 0)
    y_interp = interpolate.splev(t, f_y, der = 0)
    z_interp = interpolate.splev(t, f_z, der = 0)

    x_error = eskf.Xpo[:,0] - x_interp
    y_error = eskf.Xpo[:,1] - y_interp
    z_error = eskf.Xpo[:,2] - z_interp

    pos_error = np.concatenate((x_error.reshape(-1,1), y_error.reshape(-1,1), z_error.reshape(-1,1)), axis = 1)

    rms_x = math.sqrt(mean_squared_error(x_interp, eskf.Xpo[:,0]))
    rms_y = math.sqrt(mean_squared_error(y_interp, eskf.Xpo[:,1]))
    rms_z = math.sqrt(mean_squared_error(z_interp, eskf.Xpo[:,2]))
    print('The RMS error for position x is %f [m]' % rms_x)
    print('The RMS error for position y is %f [m]' % rms_y)
    print('The RMS error for position z is %f [m]' % rms_z)

    RMS_all = math.sqrt(rms_x**2 + rms_y**2 + rms_z**2)          
    print('The overall RMS error of position estimation is %f [m]\n' % RMS_all)

    # visualization
    plot_pos(t, eskf.Xpo, t_vicon, pos_vicon)
    plot_pos_err(t, pos_error, eskf.Ppo)
    plot_traj(pos_vicon, eskf.Xpo, anchor_position)
    plt.show()