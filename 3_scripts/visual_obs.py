'''
Visualize the obstacles
'''
import os, sys
import numpy as np
from numpy import linalg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import matplotlib.style as style
import rosbag
from scipy import stats
from tkinter.filedialog import askopenfilename
# select the matplotlib plotting style
style.use('ggplot')
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# current path of the script
curr = os.path.dirname(sys.argv[0])
# load the anchor pose
anchor_pos = np.load(curr+'/survey/0425-numpy/AnchorPos_0425.npy')

woodshelf = np.array([[1892.22,  2587.42,  2026.51],
                      [1710.66,  2403.10,  2025.84],
                      [2264.10,  1852.96,  2024.60],
                      [2449.43,  2034.15,  2026.06]]) / 1000.0

plasticbox = np.array([[-2290.30, 1919.69, 844.55],
                       [-1956.36, 2326.76, 845.48],
                       [-2014.29, 1688.40, 845.39],
                       [-1678.54, 2094.41, 846.06]
                       ]) / 1000.0 

cardboard = np.array([[2297.77,  -2405.36,  1157.05],
                      [2593.35,  -1960.48,  1141.96],
                      [2751.64,  -2700.92,  1160.56],
                      [3058.63,  -2266.65,  1141.71]
                      ]) / 1000.0

metal = np.array([[-1231.57,  -2963.78,  1142.45],
                  [-1925.50,  -2364.54,  1137.66],
                  [-2243.56,  -2735.56,  1136.37],
                  [-1554.55,  -3334.40,  1141.94]
                  ]) / 1000.0


fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
ax.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2], marker='o',color='red')
ax.scatter(metal[:,0], metal[:,1], metal[:,2], marker='o',color='navy')
ax.scatter(woodshelf[:,0], woodshelf[:,1], woodshelf[:,2], marker='o',color='yellow')
ax.scatter(plasticbox[:,0], plasticbox[:,1], plasticbox[:,2], marker='o',color='teal')
ax.scatter(cardboard[:,0], cardboard[:,1], cardboard[:,2], marker='o',color='brown')

ax.set_xlim3d(np.amin(anchor_pos[:,0])-0.5, np.amax(anchor_pos[:,0])+0.5)  
ax.set_ylim3d(np.amin(anchor_pos[:,1])-0.5, np.amax(anchor_pos[:,1])+0.5)  
ax.set_zlim3d(np.amin(anchor_pos[:,2])-0.1, np.amax(anchor_pos[:,2])+0.3)  
ax.set_xlabel(r'X [m]')
ax.set_ylabel(r'Y [m]')
ax.set_zlabel(r'Z [m]')
plt.legend(['Trajectory','Anchor position'])
plt.title(r"Trajectory of the experiment", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )

plt.show()




