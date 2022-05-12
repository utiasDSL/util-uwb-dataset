'''
    help functions for data process
'''
import numpy as np

def deleteNAN(array):
    nan_array = np.isnan(array)
    not_nan = ~ nan_array
    new_array = array[not_nan]
    return new_array
    
def interp_meas(t1, meas1, t2):
    # synchronized meas 1 w.r.t. t2
    t1 = np.squeeze(t1)
    t2 = np.squeeze(t2)
    meas1 = np.squeeze(meas1)
    syn_m1 = np.interp(t2, t1, meas1)
    return syn_m1

def sync_pos(t_1, pos_1, t_2):
    # sync. the position 
    x_s  = interp_meas(t_1, pos_1[:,0], t_2).reshape(-1,1)
    y_s  = interp_meas(t_1, pos_1[:,1], t_2).reshape(-1,1)
    z_s  = interp_meas(t_1, pos_1[:,2], t_2).reshape(-1,1)
    pos_s = np.concatenate((x_s, y_s, z_s), axis = 1)
    return pos_s


# helper functions for csv files
# extract ground truth
def extract_gt(df):
    gt_t = deleteNAN(np.array(df['t_pose'])).reshape(-1,1)
    gt_x = deleteNAN(np.array(df['pose_x'])).reshape(-1,1)
    gt_y = deleteNAN(np.array(df['pose_y'])).reshape(-1,1)
    gt_z = deleteNAN(np.array(df['pose_z'])).reshape(-1,1)
    gt_qx = deleteNAN(np.array(df['pose_qx'])).reshape(-1,1)
    gt_qy = deleteNAN(np.array(df['pose_qy'])).reshape(-1,1)
    gt_qz = deleteNAN(np.array(df['pose_qz'])).reshape(-1,1)
    gt_qw = deleteNAN(np.array(df['pose_qw'])).reshape(-1,1)
    gt_pose = np.hstack((gt_t, gt_x, gt_y, gt_z, gt_qx, gt_qy, gt_qz, gt_qw))
    return gt_pose

# extract tdoa measurements
def extract_tdoa(df):
    t_tdoa = deleteNAN(np.array(df['t_tdoa'])).reshape(-1,1)
    idA = deleteNAN(np.array(df['idA'])).reshape(-1,1)
    idB = deleteNAN(np.array(df['idB'])).reshape(-1,1)
    tdoa_meas = deleteNAN(np.array(df['tdoa_meas'])).reshape(-1,1)
    tdoa = np.hstack((t_tdoa, idA, idB, tdoa_meas))
    return tdoa

# extract acc measurements
def extract_acc(df):
    t_acc = deleteNAN(np.array(df['t_acc'])).reshape(-1,1)
    acc_x = deleteNAN(np.array(df['acc_x'])).reshape(-1,1)
    acc_y = deleteNAN(np.array(df['acc_y'])).reshape(-1,1)
    acc_z = deleteNAN(np.array(df['acc_z'])).reshape(-1,1)
    acc = np.hstack((t_acc, acc_x, acc_y, acc_z))
    return acc 

# extract gyro measurements
def extract_gyro(df):
    t_gyro = deleteNAN(np.array(df['t_gyro'])).reshape(-1,1)
    gyro_x = deleteNAN(np.array(df['gyro_x'])).reshape(-1,1)
    gyro_y = deleteNAN(np.array(df['gyro_y'])).reshape(-1,1)
    gyro_z = deleteNAN(np.array(df['gyro_z'])).reshape(-1,1)
    gyro = np.hstack((t_gyro, gyro_x, gyro_y, gyro_z))
    return gyro

# extract tof measurements
def extract_tof(df):
    t_tof = deleteNAN(np.array(df['t_tof'])).reshape(-1,1)
    tof_meas = deleteNAN(np.array(df['tof'])).reshape(-1,1)
    tof = np.hstack((t_tof, tof_meas))
    return tof

# extract flow measurements
def extract_flow(df):
    t_flow = deleteNAN(np.array(df['t_flow'])).reshape(-1,1)
    dx = deleteNAN(np.array(df['deltaX'])).reshape(-1,1)
    dy = deleteNAN(np.array(df['deltaY'])).reshape(-1,1)
    flow = np.hstack((t_flow, dx, dy))
    return flow

# extract baro measurements
def extract_baro(df):
    t_baro = deleteNAN(np.array(df['t_baro'])).reshape(-1,1)
    baro_meas = deleteNAN(np.array(df['baro'])).reshape(-1,1)
    baro = np.hstack((t_baro, baro_meas))
    return baro






