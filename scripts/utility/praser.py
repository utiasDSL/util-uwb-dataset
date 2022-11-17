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
    ''' 
        synchronized meas 1 w.r.t. t2
    '''
    t1 = np.squeeze(t1)
    t2 = np.squeeze(t2)
    meas1 = np.squeeze(meas1)
    syn_m1 = np.interp(t2, t1, meas1)
    return syn_m1

def sync_pos(t_1, pos_1, t_2):
    ''' 
        sync. the position 
    '''
    x_s  = interp_meas(t_1, pos_1[:,0], t_2).reshape(-1,1)
    y_s  = interp_meas(t_1, pos_1[:,1], t_2).reshape(-1,1)
    z_s  = interp_meas(t_1, pos_1[:,2], t_2).reshape(-1,1)
    pos_s = np.concatenate((x_s, y_s, z_s), axis = 1)
    return pos_s


def extract_gt(df):
    '''
        helper functions for csv files
        extract ground truth
    '''
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

def extract_tdoa(df):
    '''
        extract tdoa measurements
    '''
    t_tdoa = deleteNAN(np.array(df['t_tdoa'])).reshape(-1,1)
    idA = deleteNAN(np.array(df['idA'])).reshape(-1,1)
    idB = deleteNAN(np.array(df['idB'])).reshape(-1,1)
    tdoa_meas = deleteNAN(np.array(df['tdoa_meas'])).reshape(-1,1)
    tdoa = np.hstack((t_tdoa, idA, idB, tdoa_meas))
    return tdoa

def extract_acc(df):
    '''
        extract acc measurements
    '''
    t_acc = deleteNAN(np.array(df['t_acc'])).reshape(-1,1)
    acc_x = deleteNAN(np.array(df['acc_x'])).reshape(-1,1)
    acc_y = deleteNAN(np.array(df['acc_y'])).reshape(-1,1)
    acc_z = deleteNAN(np.array(df['acc_z'])).reshape(-1,1)
    acc = np.hstack((t_acc, acc_x, acc_y, acc_z))
    return acc 

def extract_gyro(df):
    '''
        extract gyro measurements
    '''
    t_gyro = deleteNAN(np.array(df['t_gyro'])).reshape(-1,1)
    gyro_x = deleteNAN(np.array(df['gyro_x'])).reshape(-1,1)
    gyro_y = deleteNAN(np.array(df['gyro_y'])).reshape(-1,1)
    gyro_z = deleteNAN(np.array(df['gyro_z'])).reshape(-1,1)
    gyro = np.hstack((t_gyro, gyro_x, gyro_y, gyro_z))
    return gyro

def extract_tof(df):
    '''
        extract tof measurements
    '''
    t_tof = deleteNAN(np.array(df['t_tof'])).reshape(-1,1)
    tof_meas = deleteNAN(np.array(df['tof'])).reshape(-1,1)
    tof = np.hstack((t_tof, tof_meas))
    return tof

def extract_flow(df):
    '''
        extract flow measurements
    '''
    t_flow = deleteNAN(np.array(df['t_flow'])).reshape(-1,1)
    dx = deleteNAN(np.array(df['deltaX'])).reshape(-1,1)
    dy = deleteNAN(np.array(df['deltaY'])).reshape(-1,1)
    flow = np.hstack((t_flow, dx, dy))
    return flow

def extract_baro(df):
    '''
        extract baro measurements
    '''
    t_baro = deleteNAN(np.array(df['t_baro'])).reshape(-1,1)
    baro_meas = deleteNAN(np.array(df['baro'])).reshape(-1,1)
    baro = np.hstack((t_baro, baro_meas))
    return baro

def extract_tdoa_meas(t_tdoa, tdoa_data):
    '''
        inputs:
            tdoa_data: [uwb_time, an_i, an_j, uwb_tdoa_meas] 
    '''
    t_tdoa = t_tdoa.reshape(-1,1)
    # combine t_tdoa and tdoa
    tdoa = np.concatenate((t_tdoa, tdoa_data), axis = 1)
    # get the id for tdoa_ij measurements
    idx_70 = np.where((tdoa[:,1]==[7])&(tdoa[:,2]==[0]))
    tdoa_70 = np.squeeze(tdoa[idx_70, :])

    idx_01 = np.where((tdoa[:,1]==[0])&(tdoa[:,2]==[1]))
    tdoa_01 = np.squeeze(tdoa[idx_01, :])

    idx_12 = np.where((tdoa[:,1]==[1])&(tdoa[:,2]==[2]))
    tdoa_12 = np.squeeze(tdoa[idx_12, :])

    idx_23 = np.where((tdoa[:,1]==[2])&(tdoa[:,2]==[3]))
    tdoa_23 = np.squeeze(tdoa[idx_23, :])

    idx_34 = np.where((tdoa[:,1]==[3])&(tdoa[:,2]==[4]))
    tdoa_34 = np.squeeze(tdoa[idx_34, :])

    idx_45 = np.where((tdoa[:,1]==[4])&(tdoa[:,2]==[5]))
    tdoa_45 = np.squeeze(tdoa[idx_45, :])

    idx_56 = np.where((tdoa[:,1]==[5])&(tdoa[:,2]==[6]))
    tdoa_56 = np.squeeze(tdoa[idx_56, :])

    idx_67 = np.where((tdoa[:,1]==[6])&(tdoa[:,2]==[7]))
    tdoa_67 = np.squeeze(tdoa[idx_67, :])

    return tdoa_70, tdoa_01, tdoa_12, tdoa_23, tdoa_34, tdoa_45, tdoa_56, tdoa_67

