'''
2021 Survey code. Convert the Total Station survey results into an inertial frame (Vicon frame).
Using 6 vicon markers with known positions, the survey results are converted through a point cloud alignment. 
'''
import time, string, os, sys
import numpy as np
from numpy import linalg 
from scipy import stats
from pyquaternion import Quaternion      # package for quaternion
from tkinter.filedialog import askopenfilename
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.style as style

from survey_util import pointcloud_alignment, getRMSE, plot_survey

style.use('ggplot')

# current path of the script
curr = os.path.dirname(sys.argv[0])
pathname = os.path.abspath(curr+'/../../2_data/survey-results/')

txt_name = askopenfilename(initialdir = pathname, title = "Select txt file")
f1 = open(txt_name,"r")
pos=[]
for line in f1:
    x = line.split(",")
    # print(x[0])
    if len(x) > 3:
        arr_x = [float(x[1]), float(x[2]), float(x[3])]
        pos.append(arr_x)
    
pos = np.array(pos)
NUM_vicon = 6
vicon_frame = pos[0:NUM_vicon,:]                  # 6 points in Vicon frame
anchor_marker = pos[NUM_vicon:,:]                 # uwb anchor pose 

# # 0415
# vicon_marks = np.array([[-6.1015,    1520.0397,    5.1068],
#                         [9.6667,       8.8408,     5.1986],
#                         [1492.5110,    42.1710,    8.9143],
#                         [27.3472,   -1001.0138,    9.2206],
#                         [-989.8257,   -10.6660,    3.9086],
#                         [-945.6361, -2006.8177,   13.0570]
#                         ]) / 1000.0
# 0425
vicon_marks = np.array([[6.4833,     9.5857,     5.4701],
                        [996.7297,   20.6892,    6.5530],
                        [1493.6611,  30.2870,    8.0861],
                        [1483.5196,  530.8999,   7.4837],
                        [502.6914,   516.7251,   5.6232],
                        [11.2639,    522.5133,   5.0284]
                        ]) / 1000.0

dest_point = vicon_marks
src_point = vicon_frame
# compute the C and r from src_point to dest_point
(C_est, r_est) = pointcloud_alignment(src_point, dest_point)
print('The result of point cloud alignment:\n')
print(C_est); print(r_est)
print('\n')


# vicon markers (Total Station Survey results)
vicon_marker = np.empty((0,3))
for idx in range(NUM_vicon):
    vf = C_est.dot(vicon_frame[idx,:].reshape(-1,1)) + r_est.reshape(-1,1)
    vf = vf.reshape(1,-1)
    vicon_marker = np.vstack((vicon_marker, vf))
    
vicon_marker = np.asarray(vicon_marker)

# testing the survey accuracy (vicon_marker, dest_points)
err = vicon_marks - vicon_marker
RMSE = getRMSE(err)
print("The RMS error of Total Station survey results is {0} [m], ({1} [mm])".format(RMSE, RMSE*1000))

Marker_pos = np.empty([3, 0])
for i in range(len(anchor_marker)):
    an_i = C_est.dot(anchor_marker[i,:].reshape(-1,1)) + r_est.reshape(-1,1)
    Marker_pos = np.concatenate((Marker_pos,an_i), axis=1)
    
Marker_pos = Marker_pos.T

plot_survey(Marker_pos, vicon_marker)
plt.show()

# -------------------- Data Structure ------------------- #
# An0_M0: x, y, z, 
# An0_M1: x, y, z,
# An0_M2: x, y, z, 　
# An0_p:  x, y, z,
An_num = 8
anchor_position = np.zeros((An_num,3))
anchor_quaterion = np.zeros((An_num,4))
counter = 0
idx = 0
for i in range(An_num):
    # three markers in the inertial frame
    dest_points = Marker_pos[idx:idx+3,:]
    # 3 markers in the Anchor frame. Order: [left, up, forward]
    src_points = np.array([[110.0,   0.0,   0.0],
                           [0.0,   121.0,   0.0],
                           [0.0,     0.0,  80.0]]) / 1000.0 
    # compute the rotation from body to inertial frame
    (C_survey, t_est) = pointcloud_alignment(src_points, dest_points)   
    # save the quaternion from anchor body frame to inertial frame
    q_survey = Quaternion(matrix=C_survey)
    anchor_quaterion[counter,:] = np.array([q_survey.w, q_survey.x, q_survey.y,q_survey.z])
    # save the anchor position
    anchor_position[counter,:] = Marker_pos[idx+3,:]
    counter += 1
    idx += 4 


print(np.round(anchor_position,3))

print(np.round(anchor_quaterion,3))

np.save('AnchorPos_0425.npy', anchor_position)
np.save('AnchorQuat_0425.npy', anchor_quaterion)
