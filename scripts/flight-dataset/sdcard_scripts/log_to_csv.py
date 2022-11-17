# -*- coding: utf-8 -*-
'''
    convert the binary file to csv and visualize the raw data 
    help function cfusdlog.py
'''
from log_to_bag import writeCombIMU
from numpy.lib.function_base import extract
import cfusdlog
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.style as style
import argparse
import numpy as np
import csv
from itertools import zip_longest

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

def extractTDOA(data, start_time):
    tdoa_data = []
    for (t,idA, idB, diff) in zip(data['timestamp'], 
        data['idA'], data['idB'], data['distanceDiff']):
        stamp = (t-start_time) / 1000
        tdoa_data.append([stamp, idA, idB, diff])
    return np.array(tdoa_data)

def extractAccel(data, start_time):
    accel_data = []
    for (t, x, y, z) in zip(data['timestamp'],
        data['acc.x'], data['acc.y'], data['acc.z']):
        stamp = (t-start_time) / 1000
        accel_data.append([stamp, x, y, z])
    return np.array(accel_data)

def extractGyro(data, start_time):
    gyro_data = []
    for (t, x, y, z) in zip(data['timestamp'], 
        data['gyro.x'], data['gyro.y'], data['gyro.z']):
        stamp = (t-start_time) / 1000
        gyro_data.append([stamp, x, y, z])
    return np.array(gyro_data)

def extractTof(data, start_time):
    tof_data = []
    for (t,zrange) in zip(data['timestamp'], data['range.zrange']):
        stamp = (t-start_time) / 1000
        zrange = zrange*0.001; # mm to m
        tof_data.append([stamp, zrange])
    return np.array(tof_data)

def extractFlow(data, start_time):
    flow_data = []
    for (t, dx, dy) in zip(data['timestamp'], data['motion.deltaX'], data['motion.deltaY']):
        stamp = (t-start_time) / 1000
        flow_data.append([stamp, dx, dy])
    return np.array(flow_data)

def extractBaro(data, start_time):
    baro_data = []
    for (t, asl) in zip(data['timestamp'], data['baro.asl']):
        stamp = (t-start_time) / 1000
        baro_data.append([stamp, asl])
    return np.array(baro_data)

def extractPose(data, start_time):
    pose_data = []
    for (t, x, y, z, qx, qy, qz, qw) in zip(data['timestamp'], 
        data['locSrv.x'],  data['locSrv.y'],  data['locSrv.z'],
        data['locSrv.qx'], data['locSrv.qy'], data['locSrv.qz'],
        data['locSrv.qw']):
        stamp = (t - start_time) / 1000    
        pose_data.append([stamp, x, y, z, qx, qy, qz, qw])
    return np.array(pose_data)

def writeCombIMU(gyro_data, accel_data):
    ax_int = np.interp(gyro_data[:,0], accel_data[:, 0], accel_data[:, 1]);
    ay_int = np.interp(gyro_data[:,0], accel_data[:, 0], accel_data[:, 2]);
    az_int = np.interp(gyro_data[:,0], accel_data[:, 0], accel_data[:, 3]);
    imu_data = []
    for (t, gx, gy, gz, ax, ay, az) in zip(gyro_data[:,0], gyro_data[:,1], gyro_data[:,2], gyro_data[:,3], ax_int, ay_int, az_int):
        imu_data.append([t, ax, ay, az, gx, gy, gz])
    return np.array(imu_data)

def dataVisual(tdoa, tof, flow, baro, pose):
    # get the id for tdoa_ij measurements
    tdoa_70 = np.where((tdoa[:,1]==[7])&(tdoa[:,2]==[0]))
    tdoa_meas_70 = np.squeeze(tdoa[tdoa_70, :])

    tdoa_01 = np.where((tdoa[:,1]==[0])&(tdoa[:,2]==[1]))
    tdoa_meas_01 = np.squeeze(tdoa[tdoa_01, :])

    tdoa_12 = np.where((tdoa[:,1]==[1])&(tdoa[:,2]==[2]))
    tdoa_meas_12 = np.squeeze(tdoa[tdoa_12, :])

    tdoa_23 = np.where((tdoa[:,1]==[2])&(tdoa[:,2]==[3]))
    tdoa_meas_23 = np.squeeze(tdoa[tdoa_23, :])

    tdoa_34 = np.where((tdoa[:,1]==[3])&(tdoa[:,2]==[4]))
    tdoa_meas_34 = np.squeeze(tdoa[tdoa_34, :])

    tdoa_45 = np.where((tdoa[:,1]==[4])&(tdoa[:,2]==[5]))
    tdoa_meas_45 = np.squeeze(tdoa[tdoa_45, :])

    tdoa_56 = np.where((tdoa[:,1]==[5])&(tdoa[:,2]==[6]))
    tdoa_meas_56 = np.squeeze(tdoa[tdoa_56, :])

    tdoa_67 = np.where((tdoa[:,1]==[6])&(tdoa[:,2]==[7]))
    tdoa_meas_67 = np.squeeze(tdoa[tdoa_67, :])


    # visualization 
    # UWB TDOA
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(221)
    ax1.scatter(tdoa_meas_70[:,0], tdoa_meas_70[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    ax1.legend(loc='best')
    ax1.set_xlabel(r'Time [s]')
    ax1.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An7, An0)", fontsize=13, fontweight=0, color='black')

    bx1 = fig1.add_subplot(222)
    bx1.scatter(tdoa_meas_01[:,0], tdoa_meas_01[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    bx1.legend(loc='best')
    bx1.set_xlabel(r'Time [s]')
    bx1.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An0, An1)", fontsize=13, fontweight=0, color='black')


    cx1 = fig1.add_subplot(223)
    cx1.scatter(tdoa_meas_12[:,0], tdoa_meas_12[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    cx1.legend(loc='best')
    cx1.set_xlabel(r'Time [s]')
    cx1.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An1, An2)", fontsize=13, fontweight=0, color='black')

    dx1 = fig1.add_subplot(224)
    dx1.scatter(tdoa_meas_23[:,0], tdoa_meas_23[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    dx1.legend(loc='best')
    dx1.set_xlabel(r'Time [s]')
    dx1.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An2, An3)", fontsize=13, fontweight=0, color='black')


    fig2 = plt.figure()
    ax2 = fig2.add_subplot(221)
    ax2.scatter(tdoa_meas_34[:,0], tdoa_meas_34[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    ax2.legend(loc='best')
    ax2.set_xlabel(r'Time [s]')
    ax2.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An3, An4)", fontsize=13, fontweight=0, color='black')

    bx2 = fig2.add_subplot(222)
    bx2.scatter(tdoa_meas_45[:,0], tdoa_meas_45[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    bx2.legend(loc='best')
    bx2.set_xlabel(r'Time [s]')
    bx2.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An4, An5)", fontsize=13, fontweight=0, color='black')

    cx2 = fig2.add_subplot(223)
    cx2.scatter(tdoa_meas_56[:,0], tdoa_meas_56[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    cx2.legend(loc='best')
    cx2.set_xlabel(r'Time [s]')
    cx2.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An5, An6)", fontsize=13, fontweight=0, color='black')

    dx2 = fig2.add_subplot(224)
    dx2.scatter(tdoa_meas_67[:,0], tdoa_meas_67[:,3], color = "steelblue", s = 2.5, alpha = 0.9, label = "tdoa measurements")
    dx2.legend(loc='best')
    dx2.set_xlabel(r'Time [s]')
    dx2.set_ylabel(r'TDoA measurement [m]') 
    plt.title(r"UWB tdoa measurements, (An6, An7)", fontsize=13, fontweight=0, color='black')

    # Z-range ToF
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.scatter(tof[:,0], tof[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "tof measurements")
    ax3.legend(loc='best')
    ax3.set_xlabel(r'Time [s]')
    ax3.set_ylabel(r'ToF measurement [m]') 
    plt.title(r"Z-range measurements", fontsize=13, fontweight=0, color='black')

    # flow pixel
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(211)
    plt.title(r"Optical flow measurements", fontsize=13, fontweight=0, color='black')
    ax4.scatter(flow[:,0], flow[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel x")
    ax4.set_ylabel(r'motion delta x') 
    bx4 = fig4.add_subplot(212)
    bx4.scatter(flow[:,0], flow[:,2], color = "steelblue", s = 2.5, alpha = 0.9, label = "flow dpixel y")
    bx4.set_ylabel(r'motion delta y') 
    bx4.set_xlabel(r'Time [s]')
    plt.legend(loc='best')

    # baremeter
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    plt.title(r"Baro measurements", fontsize=13, fontweight=0, color='black')
    ax4.scatter(baro[:,0], baro[:,1], color = "steelblue", s = 2.5, alpha = 0.9, label = "baro asl")
    ax4.set_ylabel(r'asl') 
    plt.legend(loc='best')

    # trajectory
    fig5 = plt.figure()
    ax_t = fig5.add_subplot(111, projection = '3d')
    ax_t.plot(pose[:,1],pose[:,2],pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9)
    ax_t.set_xlim3d(-3.5, 3.5)  
    ax_t.set_ylim3d(-3.5, 3.5)  
    ax_t.set_zlim3d( 0.0, 3.0)  
    ax_t.set_xlabel(r'X [m]')
    ax_t.set_ylabel(r'Y [m]')
    ax_t.set_zlabel(r'Z [m]')
    plt.legend(['Trajectory'])
    plt.title(r"Trajectory of the experiment", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )

    # plot separate x,y,z
    fig6 = plt.figure()
    a_x = fig6.add_subplot(311)
    plt.title(r"Ground truth of the experiment", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )
    a_x.plot(pose[:,0],pose[:,1],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt x")
    a_x.legend(loc='best')
    a_x.set_ylabel(r'X [m]') 
    a_y = fig6.add_subplot(312)
    a_y.plot(pose[:,0],pose[:,2],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt y")
    a_y.legend(loc='best')
    a_y.set_ylabel(r'Y [m]') 
    a_z = fig6.add_subplot(313)
    a_z.plot(pose[:,0],pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9, label = "Vicon gt z")
    a_z.legend(loc='best')
    a_z.set_ylabel(r'Z [m]')
    a_z.set_xlabel(r'Time [s]')
    

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    args = parser.parse_args()
    
    # decode binary log data
    data_usd = cfusdlog.decode(args.file_usd)
    # find start time
    start_time = None
    for k, (event_name, data) in enumerate(data_usd.items()):
        if start_time is None:
            start_time = data['timestamp'][0]
        else:
            start_time = min(start_time, data['timestamp'][0])

    # extract data
    for k, (event_name, data) in enumerate(data_usd.items()):
        if event_name == "estTDOA":
            print("Extract TDOA ")
            tdoa = extractTDOA(data, start_time)
        if event_name == "estAcceleration":
            print("Extract accel ")
            accel = extractAccel(data, start_time)
        if event_name == "estGyroscope":
            print("Extract gyro ")
            gyro = extractGyro(data, start_time)
        if event_name == "estTOF":
            print("Extract tof ")
            tof  = extractTof(data, start_time)
        if event_name == "estFlow":
            print("Extract flow ")
            flow = extractFlow(data, start_time)
        if event_name == "estBarometer":
            print("Extract baro ")
            baro = extractBaro(data, start_time)
        if event_name == "estPose":
            print("Extract ground truth pose ")
            pose = extractPose(data, start_time)

    if len(gyro) > 0 and len(accel) > 0:
        print("Writing combined IMU data")
        imu = writeCombIMU(gyro, accel)

    # save the data into csv files 
    header = ['t_tdoa',     'idA',     'idB',     'tdoa_meas', 't_acc',   'acc_x',   'acc_y',   'acc_z',
              't_gyro',     'gyro_x',  'gyro_y',  'gyro_z',    't_tof',   'tof',     't_flow',  'deltaX',  'deltaY',
              't_baro',     'baro',    't_pose',  'pose_x',    'pose_y',  'pose_z',  'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw']

    row = zip_longest(tdoa[:,0],  tdoa[:,1],  tdoa[:,2],  tdoa[:,3],  accel[:,0],  accel[:,1],  accel[:,2],  accel[:,3],
                      gyro[:,0],  gyro[:,1],  gyro[:,2],  gyro[:,3],  tof[:,0],    tof[:,1],    flow[:,0],   flow[:,1],  flow[:,2],
                      baro[:,0],  baro[:,1],  pose[:,0],  pose[:,1],  pose[:,2],   pose[:,3],   pose[:,4],   pose[:,5],  pose[:,6],  pose[:,7],  
                      fillvalue='')
    csv_file = args.file_usd +'.csv'
    with open(csv_file,"w") as f:
        csv.writer(f).writerow(header)
        csv.writer(f).writerows(row)
    # visualize the raw data
    print("Visualizing the raw data")
    dataVisual(tdoa, tof, flow, baro, pose)

