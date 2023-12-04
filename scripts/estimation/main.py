'''
    The main file for eskf estimation
'''
#!/usr/bin/env python3
import argparse
import os, sys
import math
import numpy as np
import pandas as pd
from pyquaternion import Quaternion
from scipy import interpolate            
from sklearn.metrics import mean_squared_error

from eskf_class import ESKF
import matplotlib.pyplot as plt
cwd = os.path.dirname(__file__)
sys.path.append(os.path.join(cwd, './../'))
from utility.praser import extract_gt, extract_tdoa, extract_acc, extract_gyro, interp_meas, extract_tdoa_meas
from plot_util import plot_pos, plot_pos_err, plot_traj


def isin(t_np,t_k):
    '''
        help function for timestamp
    '''
    # check if t_k is in the numpy array t_np. 
    # If t_k is in t_np, return the index and bool = Ture.
    # else return 0 and bool = False
    if t_k in t_np:
        res = np.where(t_np == t_k)
        b = True
        return res[0][0], b
    b = False
    return 0, b

def downsamp(data):
    '''
        down-sample uwb data
    '''
    data_ds = data[0::2,:]              # downsample by half
    
    data_ds = data_ds[0::2,:]           # downsample by half

    data_ds = data_ds[0::2,:]           # downsample by half
    
    # data_ds = data_ds[0::2,:]           # downsample by half

    return data_ds


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

    # access csv
    csv_file = args.i[1]
    df = pd.read_csv(csv_file)
    csv_name = os.path.split(sys.argv[-1])[1] 
    print("ESKF estimation with: " + str(csv_name) + "\n")

    TDOA2 = True

    # --------------- extract csv file --------------- #
    gt_pose = extract_gt(df)
    tdoa    = extract_tdoa(df)
    acc     = extract_acc(df)
    gyr     = extract_gyro(df)
    #
    t_vicon = gt_pose[:,0];    pos_vicon = gt_pose[:,1:4]
    # t_tdoa = tdoa[:,0];        uwb_tdoa  = tdoa[:,1:] 

    t_imu = acc[:,0]
    gyr_x_syn = interp_meas(gyr[:,0], gyr[:,1], t_imu).reshape(-1,1)
    gyr_y_syn = interp_meas(gyr[:,0], gyr[:,2], t_imu).reshape(-1,1)
    gyr_z_syn = interp_meas(gyr[:,0], gyr[:,3], t_imu).reshape(-1,1)

    imu = np.concatenate((acc[:,1:], gyr_x_syn, gyr_y_syn, gyr_z_syn), axis = 1)

    min_t = min(tdoa[0,0], t_imu[0], t_vicon[0])
    # get the vicon information from min_t
    t_vicon = np.array(t_vicon);              
    idx = np.argwhere(t_vicon > min_t);     
    t_vicon = t_vicon[idx]; 
    pos_vicon = np.squeeze(np.array(pos_vicon)[idx,:])

    # reset time base
    t_vicon = (t_vicon - min_t).reshape(-1,1)
    t_imu = (t_imu - min_t).reshape(-1,1)
    tdoa[:,0] = tdoa[:,0] - min_t

    # ------ downsample the raw data
    t_imu = downsamp(t_imu)
    imu   = downsamp(imu)

    if TDOA2:
        # extract tdoa meas.
        tdoa_70, tdoa_01, tdoa_12, tdoa_23, tdoa_34, tdoa_45, tdoa_56, tdoa_67 = extract_tdoa_meas(tdoa[:,0], tdoa[:,1:4]) 
        # downsample uwb tdoa data
        tdoa_70_ds = downsamp(tdoa_70);    tdoa_01_ds = downsamp(tdoa_01)
        tdoa_12_ds = downsamp(tdoa_12);    tdoa_23_ds = downsamp(tdoa_23)
        tdoa_34_ds = downsamp(tdoa_34);    tdoa_45_ds = downsamp(tdoa_45)
        tdoa_56_ds = downsamp(tdoa_56);    tdoa_67_ds = downsamp(tdoa_67)
        # convert back to tdoa
        tdoa_c = np.concatenate((tdoa_70_ds, tdoa_01_ds, tdoa_12_ds, tdoa_23_ds,\
                                tdoa_34_ds, tdoa_45_ds, tdoa_56_ds, tdoa_67_ds), axis = 0)
    else:
        tdoa_c = downsamp(tdoa)

    sort_id=np.argsort(tdoa_c[:,0])
    t_uwb = tdoa_c[sort_id, 0].reshape(-1,1)
    uwb   = tdoa_c[sort_id, 1:4]

    # ----------------------- INITIALIZATION OF EKF -------------------------#
    # Create a compound vector t with a sorted merge of all the sensor time bases
    time = np.sort(np.concatenate((t_imu, t_uwb)))
    t = np.unique(time)
    K = t.shape[0]
    # Initial estimate for the state vector
    X0 = np.zeros((6,1))        
    X0[0] = 1.25;  X0[1] = 0.0;  X0[2] = 0.07
        
    q0 = Quaternion([1,0,0,0])  # initial quaternion
    std_xy0 = 0.1;       std_z0 = 0.1;      std_vel0 = 0.1
    std_rp0 = 0.1;       std_yaw0 = 0.1
    # Initial posterior covariance
    P0 = np.diag([std_xy0**2,  std_xy0**2,  std_z0**2,\
                std_vel0**2, std_vel0**2, std_vel0**2,\
                std_rp0**2,  std_rp0**2,  std_yaw0**2 ])

    # create the object of ESKF
    eskf = ESKF(X0, q0, P0, K)

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
    f_x = interpolate.splrep(t_vicon, pos_vicon[:,0], s = 0.5)
    f_y = interpolate.splrep(t_vicon, pos_vicon[:,1], s = 0.5)
    f_z = interpolate.splrep(t_vicon, pos_vicon[:,2], s = 0.5)
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