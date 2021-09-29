'''
An error state Kalman filter with IMU, UWB and flowdeck measurements
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

from eskf_util import isin, cross, zeta, computeG_grad
from plot_util import plot_pos, plot_pos_err, plot_traj



if __name__ == "__main__":
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
    print("selecting anchor constellation " + str(anchor_file) + "\n")

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

    # ---------------------- Parameter ---------------------- #
    USE_IMU  = True;    USE_UWB_tdoa = True; 

    std_xy0 = 0.1;    std_z0 = 0.1;      std_vel0 = 0.1
    std_rp0 = 0.1;    std_yaw0 = 0.1
    # Process noise
    w_accxyz = 2.0;      w_gyro_rpy = 0.1    # rad/sec
    w_vel = 0;           w_pos = 0;          w_att = 0;        
    # Constants
    GRAVITY_MAGNITUE = 9.81
    DEG_TO_RAD  = math.pi/180.0
    e3 = np.array([0, 0, 1]).reshape(-1,1)     

    # Standard devirations of each sensor (tuning parameter)
    # UWB measurements: sigma**2 = 0.05 
    std_uwb_tdoa = np.sqrt(0.05)
    std_flow = 0.1;          std_tof = 0.0001

    # external calibration: translation vector from the quadcopter to UWB tag
    t_uv = np.array([-0.01245, 0.00127, 0.0908]).reshape(-1,1)  

    # ----------------------- INITIALIZATION OF EKF -------------------------#
    # Create a compound vector t with a sorted merge of all the sensor time bases
    time = np.sort(np.concatenate((t_imu, t_uwb)))
    t = np.unique(time)
    K=t.shape[0]
    # Initial states/inputs
    f_k = np.zeros((3,1))       # Initial accelerometer input
    f = np.zeros((K, 3))
    f[0] = f_k.transpose()
    omega0 = np.zeros((3,1))    # Initial angular velocity input
    omega = np.zeros((K,3))
    omega[0] = omega0.transpose()
    # nominal-state: x, y, z, vx, vy, vz, q = [qw, qx, qy, qz]
    # error state vector: [dx, dy, dz, dvx, dvy, dvz, \delta pi_1, \delta pi_2, \delta pi_3]
    # [\delta pi_1, \delta pi_2, \delta pi_3] is the 3 dimension rotational correction vector, 
    # which can be used to correct the rotational matrix

    # --------------------- Initial position ----------------------- #
    X0 = np.zeros((6,1))        # Initial estimate for the state vector
    X0[0] = 1.25                  
    X0[1] = 0.0
    X0[2] = 0.07
    q0 = Quaternion([1,0,0,0])  # initial quaternion
    R = q0.rotation_matrix
    q_list = np.zeros((K,4))    # quaternion list
    q_list[0,:] = np.array([q0.w, q0.x, q0.y, q0.z])
    R_list = np.zeros((K,3,3))  # Rotation matrix list (from body frame to inertial frame) 
    # Initial posterior covariance
    P0 = np.diag([std_xy0**2,  std_xy0**2,  std_z0**2,\
                std_vel0**2, std_vel0**2, std_vel0**2,\
                std_rp0**2,  std_rp0**2,  std_yaw0**2 ])
    # nominal-state X = [x, y, z, vx, vy, vz]
    Xpr = np.zeros((K,6));    Xpo = np.zeros((K,6))
    Xpr[0] = X0.transpose();  Xpo[0] = X0.transpose()

    Ppo = np.zeros((K, 9, 9));  Ppr = np.zeros((K, 9, 9))
    Ppr[0] = P0;                Ppo[0] = P0

    # ----------------------- MAIN EKF LOOP ---------------------#
    print('timestep: %f' % len(t))
    print('Start state estimation')
    for k in range(len(t)-1):                 # k = 0 ~ N-1
        k=k+1                                 # k = 1 ~ N
        # Find what measurements are available at the current time (help function: isin() )
        imu_k,  imu_check  = isin(t_imu,   t[k-1])
        uwb_k,  uwb_check  = isin(t_uwb,   t[k-1])
        dt = t[k]-t[k-1]
        # # Process noise
        Fi = np.block([
            [np.zeros((3,3)),   np.zeros((3,3))],
            [np.eye(3),         np.zeros((3,3))],
            [np.zeros((3,3)),   np.eye(3)      ]
        ])
        Vi = (w_accxyz**2)*(dt**2)*np.eye(3)
        Thetai = (w_gyro_rpy**2)*(dt**2)*np.eye(3)
        Qi = np.block([
            [Vi,               np.zeros((3,3)) ],
            [np.zeros((3,3)),  Thetai          ]
        ])

        if imu_check and USE_IMU:
            # We have a new IMU measurement
            # update the prior Xpr based on accelerometer and gyroscope data
            omega_k = imu[imu_k,3:] * DEG_TO_RAD  # careful about the index
            omega[k] = omega_k
            Vpo = Xpo[k-1,3:6]
            # Acc: G --> m/s^2
            f_k = imu[imu_k,0:3] * GRAVITY_MAGNITUE
            f[k] = f_k
            dw = omega_k * dt                      # Attitude error
            # nominal state motion model
            # position prediction 
            Xpr[k,0:3] = Xpo[k-1, 0:3] + Vpo.T*dt + 0.5 * np.squeeze(R.dot(f_k.reshape(-1,1)) - GRAVITY_MAGNITUE*e3) * dt**2
            # velocity prediction
            Xpr[k,3:6] = Xpo[k-1, 3:6] + np.squeeze(R.dot(f_k.reshape(-1,1)) - GRAVITY_MAGNITUE*e3) * dt
            # if CF is on the ground
            if Xpr[k, 2] < 0:  
                Xpr[k, 2:6] = np.zeros((1,4))    
            # quaternion update
            qk_1 = Quaternion(q_list[k-1,:])
            dqk  = Quaternion(zeta(dw))                # convert incremental rotation vector to quaternion
            q_pr = qk_1 * dqk                          # compute quaternion multiplication with package
            q_list[k,:] = np.array([q_pr.w, q_pr.x, q_pr.y, q_pr.z])  # save quaternion in q_list
            R_list[k]   = q_pr.rotation_matrix                        # save rotation prediction to R_list
            # error state covariance matrix 
            # use the rotation matrix from timestep k-1
            R = qk_1.rotation_matrix          
            # Jacobian matrix
            Fx = np.block([
                [np.eye(3),         dt*np.eye(3),      -0.5*dt**2*R.dot(cross(f_k))],
                [np.zeros((3,3)),   np.eye(3),         -dt*R.dot(cross(f_k))       ],
                [np.zeros((3,3)),   np.zeros((3,3)),   linalg.expm(cross(dw)).T    ]            
            ])
            # Process noise matrix Fi, Qi are defined above
            Ppr[k] = Fx.dot(Ppo[k-1]).dot(Fx.T) + Fi.dot(Qi).dot(Fi.T) 
            # Enforce symmetry
            Ppr[k] = 0.5*(Ppr[k] + Ppr[k].T)   
        else:
            # if we don't have IMU data
            Ppr[k] = Ppo[k-1] + Fi.dot(Qi).dot(Fi.T)
            # Enforce symmetry
            Ppr[k] = 0.5*(Ppr[k] + Ppr[k].T)  
            
            omega[k] = omega[k-1]
            f[k] = f[k-1]
            dw = omega[k] * dt                      # Attitude error
            # nominal state motion model
            # position prediction 
            Vpo = Xpo[k-1,3:6]
            Xpr[k,0:3] = Xpo[k-1, 0:3] + Vpo.T*dt + 0.5 * np.squeeze(R.dot(f_k.reshape(-1,1)) - GRAVITY_MAGNITUE*e3) * dt**2
            # velocity prediction
            Xpr[k,3:6] = Xpo[k-1, 3:6] + np.squeeze(R.dot(f_k.reshape(-1,1)) - GRAVITY_MAGNITUE*e3) * dt
            # if CF is on the ground
            # if Xpr[k, 2] < 0:  
            #     Xpr[k, 2:6] = np.zeros((1,4))    
            # quaternion update
            qk_1 = Quaternion(q_list[k-1,:])
            dqk  = Quaternion(zeta(dw))       # convert incremental rotation vector to quaternion
            q_pr = qk_1 * dqk                 # compute quaternion multiplication with package
            q_list[k] = np.array([q_pr.w, q_pr.x, q_pr.y, q_pr.z])    # save quaternion in q_list
            R_list[k]   = q_pr.rotation_matrix                        # save rotation prediction to R_list

        # End of Prediction

        # Initially take our posterior estimates as the prior estimates
        # These are updated if we have sensor measurements (UWB)
        Xpo[k] = Xpr[k]
        Ppo[k] = Ppr[k]


        # Update with UWB tdoa measurements       
        if uwb_check and USE_UWB_tdoa:
            an_A = anchor_position[int(uwb[uwb_k,0]),:]   # idA
            an_B = anchor_position[int(uwb[uwb_k,1]),:]   # idB

            qk_pr = Quaternion(q_list[k])
            C_iv = qk_pr.rotation_matrix

            # measurement model: L2 norm between anchor and uwb tag position 
            p_uwb = C_iv.dot(t_uv) + Xpr[k,0:3].reshape(-1,1)    # uwb tag position            
            d_A = linalg.norm(an_A - np.squeeze(p_uwb)) 
            d_B = linalg.norm(an_B - np.squeeze(p_uwb))

            predicted = d_B - d_A
            err_uwb = uwb[uwb_k,2] - predicted

            # compute the gradient of measurement model
            # G is 1 x 9
            G = computeG_grad(an_A, an_B, t_uv, Xpr[k,0:3], q_list[k])

            # uwb covariance
            Q = std_uwb_tdoa**2
            M = np.squeeze(G.dot(Ppr[k]).dot(G.T) + Q)     # scalar 
            d_m = math.sqrt(err_uwb**2/M)

            # -------------------- Statistical Validation -------------------- #
            if d_m < 5:
                # Kk is 9 x 1
                Kk = (Ppr[k].dot(G.T) / M).reshape(-1,1)           # in scalar case
                # update the posterios covariance matrix for error states
                Ppo[k]= (np.eye(9) - Kk.dot(G.reshape(1,-1))).dot(Ppr[k])
                # enforce symmetry
                Ppo[k] = 0.5 * (Ppo[k] + Ppo[k].T)
                derror = Kk.dot(err_uwb)             
                # update nominal states 
                Xpo[k] = Xpr[k] +  np.squeeze(derror[0:6])
                dq_k = Quaternion(zeta(np.squeeze(derror[6:])))
                #update quaternion: q_list
                qk_po = qk_pr * dq_k
                q_list[k] = np.array([qk_po.w, qk_po.x, qk_po.y, qk_po.z])
            else:
                # keep the previous state
                Xpo[k] = Xpr[k]
                Ppo[k] = Ppr[k]
                # keep the previous quaterion  q_list[k]

    print('Finish the state estimation\n')

    ## compute the error    
    # interpolate Vicon measurements
    f_x = interpolate.splrep(t_vicon, pos_vicon[:,0], s = 0)
    f_y = interpolate.splrep(t_vicon, pos_vicon[:,1], s = 0)
    f_z = interpolate.splrep(t_vicon, pos_vicon[:,2], s = 0)
    x_interp = interpolate.splev(t, f_x, der = 0)
    y_interp = interpolate.splev(t, f_y, der = 0)
    z_interp = interpolate.splev(t, f_z, der = 0)

    x_error = Xpo[:,0] - x_interp
    y_error = Xpo[:,1] - y_interp
    z_error = Xpo[:,2] - z_interp

    pos_error = np.concatenate((x_error.reshape(-1,1), y_error.reshape(-1,1), z_error.reshape(-1,1)), axis = 1)

    rms_x = math.sqrt(mean_squared_error(x_interp, Xpo[:,0]))
    rms_y = math.sqrt(mean_squared_error(y_interp, Xpo[:,1]))
    rms_z = math.sqrt(mean_squared_error(z_interp, Xpo[:,2]))
    print('The RMS error for position x is %f [m]' % rms_x)
    print('The RMS error for position y is %f [m]' % rms_y)
    print('The RMS error for position z is %f [m]' % rms_z)

    RMS_all = math.sqrt(rms_x**2 + rms_y**2 + rms_z**2)          
    print('The overall RMS error of position estimation is %f [m]\n' % RMS_all)

    # visualization
    plot_pos(t,Xpo,t_vicon,pos_vicon)
    plot_pos_err(t, pos_error, Ppo)
    plot_traj(pos_vicon, Xpo, anchor_position)
    plt.show()