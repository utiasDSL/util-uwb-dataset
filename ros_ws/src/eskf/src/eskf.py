#!/usr/bin/env python3
import roslib
import rospy
from cf_msgs.msg import Accel, Gyro, Flow, Tdoa, Tof 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg
import math
from pyquaternion import Quaternion

CONST = 1; 
if CONST == 1:
    # const #1
    anchor_position = np.array([ [-2.41747187, -4.020796,    0.18179047],
                                [-2.82049006,  3.52503733,  2.587424  ],
                                [ 3.48193225,  3.30503995,  0.15447011],
                                [ 3.45072467, -3.71811457,  2.66932012],
                                [-3.27761604, -3.86896865,  2.67389717],
                                [ 3.26547393, -3.6510796,   0.17524745],
                                [ 3.83212931,  3.65208485,  2.62492732],
                                [-2.72277241,  3.21907986,  0.15829415]])

if CONST == 2:
    anchor_position = np.array([[-3.03389889, -4.03508883,  0.20945086],
                                [-0.36968246,  3.8258778,   2.59264305],
                                [ 3.68361254,  3.71360933,  0.17473471],
                                [ 4.06473605, -0.83435362,  2.61468839],
                                [-3.51497526, -1.00105345,  2.61189996],
                                [ 3.92656653, -3.78577938,  0.19692183],
                                [ 1.15007376, -4.23238585,  2.60606765],
                                [-3.16221909,  3.34867602,  0.17978704]])

if CONST == 3:
    anchor_position = np.array([[-3.04007399, -4.02987861,  0.20863744],
                                [-3.23226042,  3.75605646,  2.94668963],
                                [ 3.6766828,   3.715609,    0.17403476],
                                [ 4.10407725, -4.27446929,  3.20206427],
                                [-3.04151099, -4.41045031,  3.04305804],
                                [ 3.91777233, -3.78620114,  0.19753353],
                                [ 4.06150938,  3.85486278,  3.14097298],
                                [-3.16867006,  3.35218259,  0.17908062]])


class ESKF_Node(object):
    def __init__(self):
        # Publisher
        self.pose_pub = rospy.Publisher("/rob_pose", PoseWithCovarianceStamped, queue_size=10)
        self.vicon_pub = rospy.Publisher("/vicon_gt", PoseWithCovarianceStamped, queue_size=10)
        # Subscribers
        # ground truth data
        self.gt_pose_sub = rospy.Subscriber("/pose_data", PoseWithCovarianceStamped, self.gt_pose, queue_size=10, tcp_nodelay=True)
        # sensor data
        self.imu_sub = rospy.Subscriber("/imu_data", Imu, self.imu_cb, queue_size=10, tcp_nodelay=True)
        self.flow_sub = rospy.Subscriber("/flow_data", Flow, self.flow_cb, queue_size=10, tcp_nodelay=True)
        self.tdoa_sub = rospy.Subscriber("/tdoa_data", Tdoa, self.tdoa_cb, queue_size=10, tcp_nodelay=True)
        self.tof_sub = rospy.Subscriber("/tof_data", Tof, self.tof_cb, queue_size=10, tcp_nodelay=True)

        self.imu_data = []
        self.flow_data = []
        self.tof_data = []
        self.tdoa_data = []
        self.gt_pose_data = []
        self.DEG_TO_RAD = math.pi/180.0
        self.GRAVITY_MAGNITUE = 9.81
        self.e3 = np.array([0, 0, 1]).reshape(-1,1)
        # eskf related parameters
        # nominal-state: x, y, z, vx, vy, vz, q = [qw, qx, qy, qz]
        # error state vector: [dx, dy, dz, dvx, dvy, dvz, d_pi_1, d_pi_2, d_pi_3]
        # [d_pi_1, \d_pi_2, \d_pi_3] is the 3 dimension rotational correction vector, 
        # which can be used to correct the rotational matrix
        self.X = np.zeros((6,1), dtype = float)
        self.X[0] = 1.5                           # initial position
        self.q = Quaternion([1,0,0,0])            # initial quaterion
        self.P = np.eye(9, dtype=float) * 0.01    # initial covariance 
        
        # TODO: improve the noise parameter structure
        # Process noise
        self.Fi = np.block([
                [np.zeros((3,3)),  np.zeros((3,3))],
                [np.eye(3),        np.zeros((3,3))],
                [np.zeros((3,3)),  np.eye(3)      ]
            ])
        self.w_accxyz = 2.0
        self.w_gyro_rpy = 0.1    # rad/sec
        self.std_uwb_tdoa = 0.05
        
        self.predict_time = 0.0
        self.prev_predict_time = -1
        self.correct_time = 0.0 
        
    
    def eskf_predict(self):
        # prediction step
        if len(self.imu_data)>0:
            imu = self.imu_data[0]
            # propogate state forward
            if self.prev_predict_time < 0:
                self.prev_predict_time = imu.header.stamp.to_sec()
                
            dt = imu.header.stamp.to_sec() - self.prev_predict_time 
            
            acc = np.array([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]).reshape(-1,1)
            gyro = np.array([imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]).reshape(-1,1)
            
            f = acc * self.GRAVITY_MAGNITUE
            omega = gyro * self.DEG_TO_RAD
            dw = omega * dt
            R = self.q.rotation_matrix                 # compute the rotation matrix from q posterior
            # propogate state forward
            # position prediction
            self.X[0:3] = self.X[0:3] + self.X[3:6] * dt +0.5 * (R.dot(f.reshape(-1,1)) - self.GRAVITY_MAGNITUE * self.e3) * dt ** 2
            # velocity prediction
            self.X[3:6] = self.X[3:6] + (R.dot(f.reshape(-1,1)) - self.GRAVITY_MAGNITUE * self.e3) * dt
            # quaternion prediction
            dqk = Quaternion(self.zeta(dw))            # convert incremental rotation vector to quaternion
            self.q = self.q * dqk                      # compute quaternion multiplication with Quaternion package
            
            # TODO: check the following covariance update
            # error state covariance matrix
            Fx = np.block([
            [np.eye(3),         dt*np.eye(3),      -0.5*dt**2*R.dot(self.cross(f))  ],
            [np.zeros((3,3)),   np.eye(3),         -dt*R.dot(self.cross(f))         ],
            [np.zeros((3,3)),   np.zeros((3,3)),    linalg.expm(self.cross(dw)).T ]            
            ])
            Vi = (self.w_accxyz**2)*(dt**2)*np.eye(3)
            Thetai = (self.w_gyro_rpy**2)*(dt**2)*np.eye(3)
            Qi = np.block([
                [Vi,              np.zeros((3,3)) ],
                [np.zeros((3,3)), Thetai          ]
            ])
            Ppr = Fx.dot(self.P).dot(Fx.T) + self.Fi.dot(Qi).dot(self.Fi.T)
            # enforce symmetry
            self.P = 0.5 * (Ppr + Ppr.T)
            
            self.predict_time = imu.header.stamp.to_sec()
            self.prev_predict_time = self.predict_time
            self.imu_data.pop(0)
        return
    
    def zeta(self, phi):
        # angles are in rad
        phi_norm = np.linalg.norm(phi)
        if phi_norm == 0:
            # no angle changes
            dq = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            dq_xyz = (phi*(math.sin(0.5*phi_norm)))/phi_norm
            dq = np.array([math.cos(0.5*phi_norm), dq_xyz[0], dq_xyz[1], dq_xyz[2]])
        return dq
    
    def cross(self, v):
        # input: 3x1 vector, output: 3x3 matrix
        vx = np.array([
            [0,     -v[2],    v[1]],
            [v[2],      0,   -v[0]],
            [-v[1],  v[0],      0 ]
        ])
        return vx
    
    def eskf_correct_flow(self):
        # eskf correct step with flow data
        pass
    
    def eskf_correct_tof(self):
        pass
    
    def eskf_correct_tdoa(self):
        # eskf correct step with tdoa data
        if len(self.tdoa_data)>0 and self.predict_time > 0:
            tdoa = self.tdoa_data[0]
            if tdoa.header.stamp.to_sec() > self.predict_time:
                return
            else:
                an_A = anchor_position[int(tdoa.idA),:]
                an_B = anchor_position[int(tdoa.idB),:]
                dA_x = self.X[0,0] - an_A[0];   dB_x = self.X[0,0] - an_B[0]
                dA_y = self.X[1,0] - an_A[1];   dB_y = self.X[1,0] - an_B[1]
                dA_z = self.X[2,0] - an_A[2];   dB_z = self.X[2,0] - an_B[2]
                # L2 norm between anchor and prior state (shape (3,1)).
                
                d_A = linalg.norm(an_A - np.squeeze(self.X[0:3])) 
                d_B = linalg.norm(an_B - np.squeeze(self.X[0:3]))
                predicted = d_B - d_A
                err_uwb = tdoa.data - predicted

                # G is 1 x 9
                G = np.append(np.array([dB_x/d_B - dA_x/d_A,  
                                        dB_y/d_B - dA_y/d_A, 
                                        dB_z/d_B - dA_z/d_A]).reshape(1,-1), np.zeros((1,6)))
                # G = G.reshape(1,-1)
                # print("G is: "); print(G)
                # uwb covariance
                Q = self.std_uwb_tdoa**2
                # print("covariance matrix is: ")
                # print(self.P[0:3,0:3])
                # print("----------- G.dot(self.P).dot(G.T) is -------------"); print(G.dot(self.P).dot(G.T)) 
                M = np.squeeze(G.dot(self.P).dot(G.T) + Q)     # scalar 
                # print("----------- M is {0} -------------".format(M))
                d_m = math.sqrt(err_uwb**2/M)
                # print("----------- d_m is {0} -------------".format(d_m)) 
                # Kk is 9 x 1
                Kk = (self.P.dot(G.T) / M).reshape(-1,1)           # in scalar case
                # print("----------- Kk is -------------"); print(Kk) 
                # print("---------Kk.dot(G.reshape(1,-1)) is --------------"); print(Kk.dot(G.reshape(1,-1)))
                # print("------------- np.eye(9) - Kk.dot(G.reshape(1,-1)) is "); print(np.eye(9) - Kk.dot(G.reshape(1,-1)))
                # update the posterios covariance matrix for error states
                Ppo = (np.eye(9) - Kk.dot(G.reshape(1,-1))).dot(self.P)
                # enforce symmetry
                self.P = 0.5 * (Ppo + Ppo.T)
                derror = Kk.dot(err_uwb)             
                # update nominal states 
                # print("self.X is "); print(self.X)
                # print("----------- derror[0:6].reshape(-1,1) is -------------"); print(derror[0:6].reshape(-1,1))
                self.X = self.X + derror[0:6].reshape(-1,1)
                dq_k = Quaternion(self.zeta(np.squeeze(derror[6:])))
                # update quaternion
                self.q = self.q * dq_k
                
                self.correct_time = tdoa.header.stamp.to_sec()
                self.tdoa_data.pop(0)
                print("publishing estimation resutls\n")
                self.publish_data()
                return
                
                # # -------------------- Statistical Validation -------------------- #
                # d_m = math.sqrt(err_uwb**2/np.squeeze(M))
                # print("----------- d_m is {0} -------------".format(d_m)) 
                # print("The variance of x, y, z are [{0}, {1}, {2}]".format(self.P[0,0], self.P[1,1], self.P[2,2]))
                # if d_m < 5:
                #     # Kk is 9 x 1
                #     Kk = (self.P.dot(G.T) / M).reshape(-1,1)           # in scalar case
                #     # update the posterios covariance matrix for error states
                #     Ppo = (np.eye(9) - Kk.dot(G.reshape(1,-1))).dot(self.P)
                #     # enforce symmetry
                #     self.P = 0.5 * (Ppo + Ppo.T)
                #     derror = Kk.dot(err_uwb)             
                #     # update nominal states 
                #     self.X = self.X + derror[0:6].reshape(-1,1)
                #     dq_k = Quaternion(self.zeta(np.squeeze(derror[6:])))
                #     # update quaternion
                #     self.q = self.q * dq_k
                    
                #     self.correct_time = tdoa.header.stamp.to_sec()
                #     self.tdoa_data.pop(0)
                #     self.publish_data()
                #     return
                # else:
                #     # keep the previous state
                #     self.X = self.X
                #     self.q = self.q
                #     self.P = self.P
                    
                #     self.correct_time = tdoa.header.stamp.to_sec()
                #     self.tdoa_data.pop(0)
                #     self.publish_data()
                #     print("TODA measurement outliers")
                #     return
            
    def imu_cb(self, msg):
        self.imu_data.append(msg)
        self.eskf_predict()
        return
     
    def flow_cb(self, msg):
        self.flow_data.append(msg)
        self.eskf_correct_flow()
        return
    
    def tof_cb(self, msg):
        self.tof_data.append(msg)
        self.eskf_correct_tof()
        return
    
    def tdoa_cb(self, msg):
        self.tdoa_data.append(msg)
        self.eskf_correct_tdoa()
        return
    
    def gt_pose(self, msg):
        self.gt_pose_data.append(msg)
        self.publish_vicon()
        return
    
    def publish_vicon(self):
        vicon_msg = PoseWithCovarianceStamped()
        vicon_msg.header.stamp = rospy.Time.now()
        vicon_msg.header.frame_id = "world"
        
        # publish vicon ground truth
        vicon_msg.pose.pose.position.x = self.gt_pose_data[-1].pose.pose.position.x
        vicon_msg.pose.pose.position.y = self.gt_pose_data[-1].pose.pose.position.y
        vicon_msg.pose.pose.position.z = self.gt_pose_data[-1].pose.pose.position.z
        vicon_msg.pose.pose.orientation.x = self.gt_pose_data[-1].pose.pose.orientation.x
        vicon_msg.pose.pose.orientation.y = self.gt_pose_data[-1].pose.pose.orientation.y
        vicon_msg.pose.pose.orientation.z = self.gt_pose_data[-1].pose.pose.orientation.z
        vicon_msg.pose.pose.orientation.w = self.gt_pose_data[-1].pose.pose.orientation.w
        
        # print('Publishing message:x:[%f] y:[%f] z[%f]'%(self.X[0,0], self.X[1,0], self.X[2,0]))
        # print('Publishing message:x:[%f] y:[%f] z[%f]'%(self.gt_pose_data[-1].pose.pose.position.x, self.gt_pose_data[-1].pose.pose.position.y, self.gt_pose_data[-1].pose.pose.position.z))
        if self.vicon_pub:
            self.vicon_pub.publish(vicon_msg)
        return
    def publish_data(self):
        out_msg = PoseWithCovarianceStamped()
        out_msg.header.stamp = rospy.Time.now()
        out_msg.header.frame_id = "world"
        # publish the estimated states
        out_msg.pose.pose.position.x = self.X[0,0]
        out_msg.pose.pose.position.y = self.X[1,0]
        out_msg.pose.pose.position.z = self.X[2,0]
        out_msg.pose.pose.orientation.x = self.q.x
        out_msg.pose.pose.orientation.y = self.q.y
        out_msg.pose.pose.orientation.z = self.q.z
        out_msg.pose.pose.orientation.w = self.q.w
        
        print('Publishing message: x:[%f] y:[%f] z:[%f]'%(self.X[0,0], self.X[1,0], self.X[2,0]))
    
        if self.pose_pub:
            self.pose_pub.publish(out_msg)
        return
        
                
    
    