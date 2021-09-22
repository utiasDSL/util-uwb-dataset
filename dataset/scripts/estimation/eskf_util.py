'''
some helpling functions for the ekf
'''
import numpy as np
from numpy import linalg
from numpy.core.fromnumeric import reshape
from numpy.linalg import inv
import math
from pyquaternion import Quaternion

from scipy.linalg import block_diag

def isin(t_np,t_k):
    # check if t_k is in the numpy array t_np. If t_k is in t_np, return the index and bool = Ture.
    # else return 0 and bool = False
    if t_k in t_np:
        res = np.where(t_np == t_k)
        b = True
        return res[0][0], b
    b = False
    return 0, b

def cross(v):    # input: 3x1 vector, output: 3x3 matrix
    v = np.squeeze(v)
    vx = np.array([
        [ 0,    -v[2], v[1]],
        [ v[2],  0,   -v[0]],
        [-v[1],  v[0], 0 ] 
    ])
    return vx

def update_state(Ppr, Xpr, H, error, R):
    # Calculate Kalman Gain for the incoming sensor measurement
    Xpr = Xpr.reshape(-1,1)                      #  to column vector
    error = error.reshape(-1,1)                  #  to column vector
    M = H.dot(Ppr).dot(H.transpose()) + R
    Kk = Ppr.dot(H.transpose()).dot(inv(M))
    
    # Update estimate and covariance posteriors
    Xpo = Xpr + Kk.dot(error)
    Ppo = (np.eye(9) - Kk.dot(H)).dot(Ppr)
    # Enforce symmetry of covariance matrix
    Ppo = 0.5 * (Ppo + Ppo.transpose())
    # print('Xpo:',Xpo)
    # print('Ppo:',Ppo)
    return Xpo.reshape(1,-1), Ppo     # return Xpo as a row , will change later

def error_state_update(Ppr, Xpr, H, error, R):
    # update step for error state kalman filter
    Xpr = Xpr.reshape(-1,1)                      #  to column vector
    error = error.reshape(-1,1)                  #  to column vector
    M = H.dot(Ppr).dot(H.T) + R
    Kk = Ppr.dot(H.T).dot(inv(M))
    # Update posterior covariance matrix for error states
    Ppo = (np.eye(9) - Kk.dot(H)).dot(Ppr)
    # Enforce symmetry of covariance matrix
    Ppo = 0.5 * (Ppo + Ppo.T)
    derror = Kk.dot(error)
    return derror, Ppo       # return d error states and posterior covariance matrix

# compute roll, pitch, yaw from quaternion
def calculateRPY(q):
    RPY = np.empty([1,3])
    # yaw
    RPY[0, 2] = np.arctan2(2*(q[1]*q[2]+q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3] )
    # pitch
    RPY[0, 1] = np.arcsin(-2*(q[1]*q[3] - q[0]*q[2]))
    # roll
    RPY[0, 0] = np.arctan2(2*(q[2]*q[3]+q[0]*q[1]) , q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])
    return RPY
    
def zeta(phi):
    phi_norm = np.linalg.norm(phi)
    if phi_norm == 0:
        # check here
        # print("error in zeta")
        # phi_norm = 0.001
        # dq_xyz = (phi*(math.sin(0.5*phi_norm)))/phi_norm
        dq = np.array([1, 0, 0, 0])
    else:
        dq_xyz = (phi*(math.sin(0.5*phi_norm)))/phi_norm
        dq = np.array([math.cos(0.5*phi_norm), dq_xyz[0], dq_xyz[1], dq_xyz[2]])
    return dq

def computeG_grad(an_A, an_B, t_uv, Xpr, q_k):
    qk_pr = Quaternion(q_k)
    C_iv = qk_pr.rotation_matrix
    # uwb tag position
    p_uwb = C_iv.dot(t_uv) + Xpr.reshape(-1,1)
    d_A = linalg.norm(np.squeeze(p_uwb) - an_A)
    d_B = linalg.norm(np.squeeze(p_uwb) - an_B)
    g_p = ((np.squeeze(p_uwb) - an_B)/d_B).reshape(1,-1) - ((np.squeeze(p_uwb) - an_A)/d_A).reshape(1,-1)
    g_v = np.zeros((1,3))

    # q_k = [q_w, q_x, q_y, q_z] = [q_w, q_v]
    q_w = q_k[0];  q_v = np.array([ q_k[1], q_k[2], q_k[3] ])
    
    # d_RVq = 2[q_w t_uv + q_v x t_uv, q_v^T t_uv I(3) + q_v t_uv^T - t_uv q_v - q_w[t_uv]x]
    d_vec = q_w*t_uv + cross(q_v).dot(t_uv).reshape(-1,1)   # 3 x 1 vector
    d_mat = q_v.reshape(1,-1).dot(t_uv) * np.eye(3) + q_v.reshape(-1,1).dot(np.transpose(t_uv)) - t_uv.dot(q_v.reshape(1,-1)) - q_w * cross(t_uv)

    d_RVq = 2*np.concatenate((d_vec, d_mat), axis=1)

    g_q = ((np.squeeze(p_uwb) - an_B)/d_B).reshape(1,-1).dot(d_RVq) - ((np.squeeze(p_uwb) - an_A)/d_A).reshape(1,-1).dot(d_RVq)
    G_x = np.concatenate((g_p, g_v, g_q), axis=1)

    Q_dtheta = 0.5*np.array([
        [-q_k[1], -q_k[2], -q_k[3]],
        [ q_w,    -q_k[3],  q_k[2]],
        [ q_k[3],  q_w,    -q_k[1]],
        [-q_k[2],  q_k[1],  q_w]
    ])
    G_dx = block_diag(np.eye(6), Q_dtheta)
    G = G_x.dot(G_dx)
    return G

