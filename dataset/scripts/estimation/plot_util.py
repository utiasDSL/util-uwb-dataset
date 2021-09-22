'''
plotting functions
'''
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm
import matplotlib.mlab as mlab
import math
import matplotlib
import pandas as pd

FONTSIZE = 18;   TICK_SIZE = 16

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

def plot_pos(t,Xpo,t_vicon,pos_vicon):
    fig = plt.figure(facecolor="white",figsize=(10, 8))
    ax = fig.add_subplot(311)
    ax.plot(t_vicon, pos_vicon[:,0], color='orangered',linewidth=1.9,alpha=1.0)
    ax.plot(t, Xpo[:,0], color='steelblue',linewidth=1.9, alpha=0.8)
    ax.set_ylabel(r'X [m]',fontsize=FONTSIZE)
    plt.legend(['Vicon ground truth','Estimate'])
    plt.title(r"Estimation results", fontsize=FONTSIZE,  color='black')

    ax = fig.add_subplot(312)
    ax.plot(t_vicon, pos_vicon[:,1], color='orangered',linewidth=1.9,alpha=1.0)
    ax.plot(t, Xpo[:,1], color='steelblue',linewidth=1.9, alpha=0.8)
    ax.set_ylabel(r'Y [m]',fontsize=FONTSIZE)

    ax = fig.add_subplot(313)
    ax.plot(t_vicon, pos_vicon[:,2], color='orangered',linewidth=1.9,alpha=1.0)
    ax.plot(t, Xpo[:,2], color='steelblue',linewidth=1.9, alpha=0.8)
    ax.set_xlabel(r'time [s]',fontsize=FONTSIZE)
    ax.set_ylabel(r'Z [m]',fontsize=FONTSIZE)
    
def plot_pos_err(t,pos_error, Ppo=np.zeros((0, 9, 9))):     
    fig = plt.figure(facecolor="white",figsize=(10, 8))
    ax = fig.add_subplot(311)
    plt.title(r"Estimation Error", fontsize=FONTSIZE, fontweight=0, color='black')
    SHOW_VAR = Ppo.shape[0]        # when no variance send inside, do not show the variance
    # extract the variance
    D = Ppo.shape[0]       
    delta_x = np.zeros([D,1])
    delta_y = np.zeros([D,1])
    delta_z = np.zeros([D,1])
    for i in range(D):
        delta_x[i,0] = math.sqrt(Ppo[i,0,0])
        delta_y[i,0] = math.sqrt(Ppo[i,1,1])
        delta_z[i,0] = math.sqrt(Ppo[i,2,2])
    ax.plot(t, pos_error[:,0], color='steelblue',linewidth=1.9, alpha=0.9)
    if SHOW_VAR:
        ax.plot(t, 3*delta_x, color='orangered',linewidth=1.9,alpha=0.9)
        ax.plot(t, -3*delta_x, color='orangered',linewidth=1.9,alpha=0.9)
    ax.set_ylabel(r'error x [m]',fontsize=FONTSIZE)

    ax = fig.add_subplot(312)
    ax.plot(t, pos_error[:,1], color='steelblue',linewidth=1.9, alpha=0.9)
    if SHOW_VAR:
        ax.plot(t, 3*delta_y, color='orangered',linewidth=1.9,alpha=0.9)
        ax.plot(t, -3*delta_y, color='orangered',linewidth=1.9,alpha=0.9)
    ax.set_ylabel(r'error y [m]',fontsize=FONTSIZE)

    ax = fig.add_subplot(313)
    ax.plot(t, pos_error[:,2], color='steelblue',linewidth=1.9, alpha=0.9)
    if SHOW_VAR:
        ax.plot(t, 3*delta_z, color='orangered',linewidth=1.9,alpha=0.9)
        ax.plot(t, -3*delta_z, color='orangered',linewidth=1.9,alpha=0.9)
    ax.set_xlabel(r'time [s]',fontsize=FONTSIZE)
    ax.set_ylabel(r'error z [m]',fontsize=FONTSIZE)


       
def plot_traj(pos_vicon, Xpo, anchor_pos):
    fig_traj = plt.figure(facecolor = "white",figsize=(10, 8))
    ax_t = fig_traj.add_subplot(111, projection = '3d')
    ax_t.plot(pos_vicon[:,0],pos_vicon[:,1],pos_vicon[:,2],color='steelblue',linewidth=1.9, alpha=0.9)
    ax_t.scatter( Xpo[:,0], Xpo[:,1], Xpo[:,2],color='darkblue',s=0.5, alpha=0.9,linestyle='--')
    ax_t.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2], marker='o',color='red')
    ax_t.set_xlim3d(np.amin(anchor_pos[:,0])-0.5, np.amax(anchor_pos[:,0])+0.5)  
    ax_t.set_ylim3d(np.amin(anchor_pos[:,1])-0.5, np.amax(anchor_pos[:,1])+0.5)  
    ax_t.set_zlim3d(np.amin(pos_vicon[:,2])-0.5, np.amax(pos_vicon[:,2])+0.5)  
    # use LaTeX fonts in the plot
    ax_t.set_xlabel(r'X [m]',fontsize=FONTSIZE)
    ax_t.set_ylabel(r'Y [m]',fontsize=FONTSIZE)
    ax_t.set_zlabel(r'Z [m]',fontsize=FONTSIZE)
    plt.title(r"Trajectory of the experiment", fontsize=FONTSIZE, fontweight=0, color='black', style='italic', y=1.02 )
    

