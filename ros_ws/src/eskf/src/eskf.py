#!/usr/bin/env python
import roslib
import rospy
from cf_msgs.msg import Accel, Gyro, Flow, Tdoa, Tof 
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
import math
from pyquaternion import Quaternion


class ESKF_Node(object):
    def __init__(self):
        # Subscribers
        # ground truth data
        self.gt_pose_sub = rospy.Subscriber("/pose_data", PoseWithCovarianceStamped, self.gt_pose, queue_size=10)
        # sensor data
        self.imu_sub = rospy.Subscriber("/imu_data", Imu, self.imu_cb, queue_size=10)
        self.flow_sub = rospy.Subscriber("/flow_data", Flow, self.flow_cb, queue_size=10)
        self.tdoa_sub = rospy.Subscriber("/tdoa_data", Tdoa, self.tdoa_cb, queue_size=10)
        self.tof_sub = rospy.Subscriber("/tof_data", Tof, self.tof_cb, queue_size=10)

        self.imu_data = []
        self.flow_data = []
        self.tof_data = []
        self.tdoa_data = []
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
            R = self.q.rotation_matrix
            # propogate state forward
            # position prediction
            self.X[0:3, 0] = self.X[0:3, 0] + self.X[3:6,0] * dt +0.5 * (R.dot(f.reshape(-1,1)) - self.GRAVITY_MAGNITUE * self.e3) * dt ** 2
            # velocity prediction
            self.X[3:6, 0] = self.X[3:6, 0] + (R.dot(f.reshape(-1,1)) - self.GRAVITY_MAGNITUE * self.e3) * dt
            # quaternion prediction
            dqk = Quaternion(self.zeta(dw))            # convert incremental rotation vector to quaternion
            self.q = self.q * dqk                      # compute quaternion multiplication with Quaternion package
            
            # TODO: check the following covariance update
            # error state covariance matrix
            Fx = np.block([
            [np.eye(3),         dt*np.eye(3),      -0.5*dt**2*R.dot(self.cross(f))  ],
            [np.zeros((3,3)),   np.eye(3),         -dt*R.dot(self.cross(f))         ],
            [np.zeros((3,3)),   np.zeros((3,3)),   np.linalg.expm(self.cross(dw)).T ]            
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
        return True
    
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
        if len(self.flow_data)>0:
            pass
    def eskf_correct_tof(self):
        # eskf correct step with tof data
        if len(self.tof_data)>0:
            pass
    def eskf_correct_tdoa(self):
        # eskf correct step with tdoa data
        if len(self.tdoa_data)>0:
            pass
        
    def imu_cb(self, msg):
        self.imu_data.append(msg)
        self.eskf_predict()
        return
     
    def flow_cb(self, msg):
        self.flow_data.append()
        self.eskf_correct_flow()
        return
    
    def tof_cb(self, msg):
        self.tof_data.append()
        self.eskf_correct_tof()
        return
    
    def tdoa_cb(self, msg):
        self.tdoa_data.append()
        self.eskf_correct_tdoa()
        return
    
    
    
    
    