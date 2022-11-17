'''
    Plotting functions
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
    ax.plot(t_vicon, pos_vicon[:,0], color='orangered',linewidth=2.5,alpha=0.8)
    ax.plot(t, Xpo[:,0], color='royalblue',linewidth=2.5, alpha=1.0)
    ax.set_ylabel(r'X [m]',fontsize=FONTSIZE)
    plt.legend(['Vicon ground truth','Estimate'])
    plt.title(r"Estimation results", fontsize=FONTSIZE,  color='black')
    plt.xlim(0, max(t))

    ax = fig.add_subplot(312)
    ax.plot(t_vicon, pos_vicon[:,1], color='orangered',linewidth=2.5,alpha=0.8)
    ax.plot(t, Xpo[:,1], color='royalblue',linewidth=2.5, alpha=1.0)
    ax.set_ylabel(r'Y [m]',fontsize=FONTSIZE)
    plt.xlim(0, max(t))

    ax = fig.add_subplot(313)
    ax.plot(t_vicon, pos_vicon[:,2], color='orangered',linewidth=2.5,alpha=0.8)
    ax.plot(t, Xpo[:,2], color='royalblue',linewidth=2.5, alpha=1.0)
    ax.set_xlabel(r'time [s]',fontsize=FONTSIZE)
    ax.set_ylabel(r'Z [m]',fontsize=FONTSIZE)
    plt.xlim(0, max(t))

def plot_pos_err(t,pos_error, Ppo=np.zeros((0, 9, 9))):   
    # extract the variance
    D = Ppo.shape[0]       
    delta_x = np.zeros([D,1])
    delta_y = np.zeros([D,1])
    delta_z = np.zeros([D,1])
    for i in range(D):
        delta_x[i,0] = math.sqrt(Ppo[i,0,0])
        delta_y[i,0] = math.sqrt(Ppo[i,1,1])
        delta_z[i,0] = math.sqrt(Ppo[i,2,2])

    fig = plt.figure(facecolor="white",figsize=(10, 8))
    ax = fig.add_subplot(311)
    plt.title(r"Estimation Error", fontsize=FONTSIZE, fontweight=0, color='black')
    ax.plot(t, pos_error[:,0], color='steelblue',linewidth=2.0, alpha=1.0)
    # plot variance
    plt.fill_between(t,-3*delta_x[:,0], 3*delta_x[:,0],facecolor="teal",alpha=0.3)
    ax.set_ylabel(r'error x [m]',fontsize=FONTSIZE)
    plt.xlim(0, max(t))
    plt.ylim(-0.35,0.35)

    ax = fig.add_subplot(312)
    ax.plot(t, pos_error[:,1], color='steelblue',linewidth=2.0, alpha=1.0)
    # plot variance
    plt.fill_between(t,-3*delta_y[:,0], 3*delta_y[:,0],facecolor="teal",alpha=0.3)
    ax.set_ylabel(r'error y [m]',fontsize=FONTSIZE)
    plt.xlim(0, max(t))
    plt.ylim(-0.35,0.35)

    ax = fig.add_subplot(313)
    ax.plot(t, pos_error[:,2], color='steelblue',linewidth=2.0, alpha=1.0)
    # plot variance
    plt.fill_between(t,-3*delta_z[:,0], 3*delta_z[:,0],facecolor="teal",alpha=0.3)
    ax.set_xlabel(r'time [s]',fontsize=FONTSIZE)
    ax.set_ylabel(r'error z [m]',fontsize=FONTSIZE)
    plt.xlim(0, max(t))
    plt.ylim(-0.45,0.45)

def plot_traj(pos_vicon, Xpo, anchor_pos):
    fig_traj = plt.figure(facecolor = "white",figsize=(10, 8))
    ax_t = fig_traj.add_subplot(projection='3d')
    # make the panes transparent
    ax_t.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax_t.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax_t.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines 
    ax_t.xaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax_t.yaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax_t.zaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)

    ax_t.plot(pos_vicon[:,0],pos_vicon[:,1],pos_vicon[:,2],color='b',linewidth=2.0, alpha=0.7, label='ground truth')
    ax_t.plot(Xpo[:,0], Xpo[:,1], Xpo[:,2],color='g', linewidth=3.0, alpha=0.5, label = 'estimation')
    ax_t.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2],color='Teal', s = 100, alpha = 0.5, label = 'anchors')
    ax_t.set_xlim([-3.5,3.5])
    ax_t.set_ylim([-3.9,3.9])
    ax_t.set_zlim([-0.0,3.0])
    # use LaTeX fonts in the plot
    ax_t.set_xlabel(r'X [m]',fontsize=FONTSIZE)
    ax_t.set_ylabel(r'Y [m]',fontsize=FONTSIZE)
    ax_t.set_zlabel(r'Z [m]',fontsize=FONTSIZE)
    ax_t.legend(loc='best', fontsize=FONTSIZE)
    ax_t.view_init(24, -58)
    ax_t.set_box_aspect((1, 1, 0.5))  # xy aspect ratio is 1:1, but change z axis
    
