'''
2021 Survey code. Convert the Total Station survey results into an inertial frame (Vicon frame).
Using 6 vicon markers with known positions, the survey results are converted through a point cloud alignment. 
'''
import time, string, os, sys
import numpy as np
from numpy import linalg 
from scipy import stats
from tkinter.filedialog import askopenfilename
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.style as style

from util import pointcloud_alignment, getRMSE

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
vicon_marks_survey = pos[0:NUM_vicon,:]    # 6 points in Vicon frame
Anchor = pos[NUM_vicon:,:]                 # uwb anchor positions 


vicon_marks = np.array([[-6.8879,    1510.2328,    8.8522],
                        [13.6219,       1.1207,    5.0343],
                        [1498.2597,    39.4949,    7.5240],
                        [34.1612,   -1008.3287,    6.9241],
                        [-989.8104,   -17.2266,    1.0972],
                        [-934.7311, -2009.1396,    6.5468]
                        ]) / 1000.0

dest_point = vicon_marks
src_point = vicon_marks_survey
# compute the C and r from src_point to dest_point
(C_est, r_est) = pointcloud_alignment(src_point, dest_point)
print('The result of point cloud alignment:\n')
print(C_est); print(r_est)
print('\n')

## ----- anchor markers for validation ----- ## 
# 210303_uwb.txt

# anchor_gt = np.array([[1889.1328,     -756.339,   1577.8877],
#                       [1886.5869,     -651.9183,  1474.5235],
#                       [1888.4631,     -691.0937,  1476.3487],
#                       [1824.2627,     -769.071,   1475.3125]
#                       ]) / 1000.0
# anchor_survey1 = np.empty((0,3))

# for idx in range(4):
#     vf1 = C_est.dot(Anchor[idx,:].reshape(-1,1)) + r_est.reshape(-1,1)
#     vf1 = vf1.reshape(1,-1)
#     anchor_survey1 = np.vstack((anchor_survey1, vf1))

# anchor_survey1 = np.asarray(anchor_survey1)
# err1 = anchor_survey1 - anchor_gt
# RMSE1 = getRMSE(err1)
# print("rmse1 is {0}".format(RMSE1))

# vicon markers (Total Station Survey results)
vicon_marker = np.empty((0,3))
for idx in range(NUM_vicon):
    vf = C_est.dot(vicon_marks_survey[idx,:].reshape(-1,1)) + r_est.reshape(-1,1)
    vf = vf.reshape(1,-1)
    vicon_marker = np.vstack((vicon_marker, vf))
    
vicon_marker = np.asarray(vicon_marker)

# testing the survey accuracy (vicon_marker, dest_points)
err = vicon_marks - vicon_marker
RMSE = getRMSE(err)
print("The RMS error of Total Station survey results is {0} [m], ({1} [mm])".format(RMSE, RMSE*1000))

Anchor_pos = np.empty([3, 0])
for i in range(len(Anchor)):
    an_i = C_est.dot(Anchor[i,:].reshape(-1,1)) + r_est.reshape(-1,1)
    Anchor_pos = np.concatenate((Anchor_pos,an_i), axis=1)
    
Anchor_pos = Anchor_pos.T
#Anchor_pos =[[anchor1],
#             [anchor2], ... ]
print(Anchor_pos)


VIS =False
if VIS:
    fig_traj = plt.figure(facecolor = "white")
    ax_t = fig_traj.add_subplot(111, projection = '3d')
    ax_t.scatter(anchor_gt[:,0], anchor_gt[:,1], anchor_gt[:,2], color='blue',label='marker ground truth', linewidth=1.9, alpha=0.9)
    # ax_t.plot(T_hist_w[:,0,3],T_hist_w[:,1,3],T_hist_w[:,2,3],color='orange',label='VO trajectory',linewidth=1.9, alpha=0.9)
    ax_t.set_xlabel(r'X [m]')
    ax_t.set_ylabel(r'Y [m]')
    ax_t.set_zlabel(r'Z [m]')
    ax_t.legend()
    ax_t.set_xlim3d(-3.5, 3.5)
    ax_t.set_ylim3d(-3.5, 3.5)
    ax_t.set_zlim3d(0.0, 4.0)
    plt.title(r"Trajectory of the vehicle", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )
    plt.show()

