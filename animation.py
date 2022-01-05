import argparse
import numpy as np
import os, sys
import rosbag
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import pyplot as plt
import matplotlib
import json
import matplotlib.animation as animation
from frame import *

f = open('obs_traj_t6/tdoa2_traj3.json')
obs_pose = json.load(f)
f.close()
g = open('obs_traj_t6/drone_toda2_traj3.json')
drone_pose = json.load(g)
g.close()
obs_pose = obs_pose[2:]
# downsample
new_drone_pose = []
new_obs_pose = []
for i in range(0,len(obs_pose)):
    if(i%20 == 0):
        new_drone_pose += [drone_pose[i]]
        new_obs_pose += [obs_pose[i]]
plt.rcParams['figure.facecolor'] = 'w'
fig = plt.figure()
ax = plt.axes(projection = '3d')
ax.view_init(30,-70)

frames = Frame(new_obs_pose,new_drone_pose)

def plot(ax,frames):
    ax.clear()
    ax.set_title("const4-trial6-tdoa2-traj3")
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    # make the panes transparent
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines
    ax.xaxis._axinfo["grid"]['color'] = (0.5, 0.5, 0.5, 0.5)
    ax.yaxis._axinfo["grid"]['color'] = (0.5, 0.5, 0.5, 0.5)
    ax.zaxis._axinfo["grid"]['color'] = (0.5, 0.5, 0.5, 0.5)
    ax.axes.set_xlim3d(left = -4,right = 4)
    ax.axes.set_ylim3d(bottom = -4,top = 4)
    ax.axes.set_zlim3d(bottom=0, top= 3.2)
    ax.set_box_aspect((1, 1, 0.35))
    ax.text(-6,2.5,5, 'Animation Speed: 2x',fontsize = 12)
    # Plot anchors first
    ax.scatter(-3.0101983925222484,-4.058910586060332,0.19628971181502824,color = 'r',marker='.',label='Anchors') #0
    ax.text(-3.0101983925222484,-4.058910586060332,0.29628971181502824,'An0')

    ax.scatter(-3.2586741011048006,3.6966310342207858,2.9101573298708, color='r', marker='.') #1
    ax.text(-3.2586741011048006,3.6966310342207858,3.051573298708, 'An1')

    ax.scatter(3.650786659616108,3.732984329061774,0.18955347520686663, color='r', marker='.') #2
    ax.text(3.650786659616108,3.732984329061774,0.28955347520686663,'An2')

    ax.scatter(3.5038524799758757,-3.955396056926031,3.0376408838394457, color='r',marker='.') #3
    ax.text(3.5038524799758757,-3.955396056926031,3.1876408838394457,'An3')

    ax.scatter(-2.7909463463466433,-4.13542201753855,2.847636358540227, color='r', marker='.')#4
    ax.text(-2.7909463463466433,-4.13542201753855,2.997636358540227,'An4')

    ax.scatter(3.953882519480277,-3.7624630950139464,0.20347693202729109, color='r', marker='.')#5
    ax.text(3.953882519480277,-3.7624630950139464,0.30347693202729109,'An5')

    ax.scatter(3.4654089074024688,3.55896956542648,2.9710608167433157, color='r', marker='.')#6
    ax.text(3.4654089074024688,3.55896956542648,3.1210608167433157,'An6')

    ax.scatter(-3.2112336178645373,3.304121110232922,0.17617659290117677, color='r',marker='.')#7
    ax.text(-3.2112336178645373,3.304121110232922,0.27617659290117677,'An7')

    x,y,z,x_drone,y_drone,z_drone = frames.getNext()
    x_wood1,y_wood1,z_wood1 = get_static_woodbox1()
    x_wood2,y_wood2,z_wood2 = get_static_woodbox2()
    ax.scatter3D(x_drone,y_drone,z_drone,color='g',marker = "x",label='Quadrotor')
    ax.scatter3D(x,y,z,label='Metal Obstacle')
    ax.scatter3D(x_wood1,y_wood1,z_wood1,label='Wood Obstacle',color = 'teal')
    ax.scatter3D(x_wood2,y_wood2,z_wood2,color = 'teal')
    form_boxes(ax,x_wood1,y_wood1,z_wood1,'lightseagreen')
    form_boxes(ax,x_wood2,y_wood2,z_wood2,'lightseagreen')
    ax.legend(loc='best')
    form_boxes(ax,x,y,z)
    plt.tight_layout()

def update(i):
    plot(ax,frames)

def connectlines(ax,x,y,z,index1,index2, co = 'b'):
    ax.plot([x[index1][0],x[index2][0]],[y[index1][0],y[index2][0]],[z[index1][0],z[index2][0]],color = co)
    return True

def form_boxes(ax,x,y,z,color='b'):
    ## Top Surface
    connectlines(ax, x, y, z, 0, 1,color)
    connectlines(ax, x, y, z, 1, 3,color)
    connectlines(ax, x, y, z, 0, 2,color)
    connectlines(ax, x, y, z, 2, 3,color)
    connectlines(ax, x, y, z, 1, 2,color)
    connectlines(ax, x, y, z, 0, 3,color)
    ## Bottom Surface
    connectlines(ax, x, y, z, 5, 6,color)
    connectlines(ax, x, y, z, 4, 7,color)
    connectlines(ax, x, y, z, 5, 4,color)
    connectlines(ax, x, y, z, 4, 6,color)
    connectlines(ax, x, y, z, 6, 7,color)
    connectlines(ax, x, y, z, 5, 7,color)
    ## Vertical Lines
    connectlines(ax, x, y, z, 1, 5,color)
    connectlines(ax, x, y, z, 0, 4,color)
    connectlines(ax, x, y, z, 2, 6,color)
    connectlines(ax, x, y, z, 3, 7,color)
    return True

def get_static_woodbox1():
    x = np.empty((0, 1))
    y = np.empty((0, 1))
    z = np.empty((0, 1))
    x = np.append(x, np.array([[-2.36652]]), axis=0)
    x = np.append(x, np.array([[-2.95728]]), axis=0)
    x = np.append(x, np.array([[-1.77268]]), axis=0)
    x = np.append(x, np.array([[-2.35649]]), axis=0)
    x = np.append(x, np.array([[-2.36652]]), axis=0)
    x = np.append(x, np.array([[-2.95728]]), axis=0)
    x = np.append(x, np.array([[-1.77268]]), axis=0)
    x = np.append(x, np.array([[-2.35649]]), axis=0)

    y = np.append(y, np.array([[-3.95934]]), axis=0)
    y = np.append(y, np.array([[-3.44388]]), axis=0)
    y = np.append(y, np.array([[-3.28863]]), axis=0)
    y = np.append(y, np.array([[-2.77373]]), axis=0)
    y = np.append(y, np.array([[-3.95934]]), axis=0)
    y = np.append(y, np.array([[-3.44388]]), axis=0)
    y = np.append(y, np.array([[-3.28863]]), axis=0)
    y = np.append(y, np.array([[-2.77373]]), axis=0)

    z = np.append(z, np.array([[1.18704]]), axis=0)
    z = np.append(z, np.array([[1.19034]]), axis=0)
    z = np.append(z, np.array([[1.19137]]), axis=0)
    z = np.append(z, np.array([[1.18490]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    return x,y,z

def get_static_woodbox2():
    x = np.empty((0, 1))
    y = np.empty((0, 1))
    z = np.empty((0, 1))
    x = np.append(x, np.array([[2.93887]]), axis=0)
    x = np.append(x, np.array([[3.686]]), axis=0)
    x = np.append(x, np.array([[3.29833]]), axis=0)
    x = np.append(x, np.array([[3.33691]]), axis=0)
    x = np.append(x, np.array([[2.93887]]), axis=0)
    x = np.append(x, np.array([[3.686]]), axis=0)
    x = np.append(x, np.array([[3.29833]]), axis=0)
    x = np.append(x, np.array([[3.33691]]), axis=0)

    y = np.append(y, np.array([[-3.57911]]), axis=0)
    y = np.append(y, np.array([[-3.08217]]), axis=0)
    y = np.append(y, np.array([[-2.88034]]), axis=0)
    y = np.append(y, np.array([[-3.77878]]), axis=0)
    y = np.append(y, np.array([[-3.57911]]), axis=0)
    y = np.append(y, np.array([[-3.08217]]), axis=0)
    y = np.append(y, np.array([[-2.88034]]), axis=0)
    y = np.append(y, np.array([[-3.77878]]), axis=0)

    z = np.append(z, np.array([[1.19921]]), axis=0)
    z = np.append(z, np.array([[1.19845]]), axis=0)
    z = np.append(z, np.array([[1.19866]]), axis=0)
    z = np.append(z, np.array([[1.19696]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    z = np.append(z, np.array([[0]]), axis=0)
    return x, y, z
    return True


def moving_pictures():
    ani = animation.FuncAnimation(fig,update,save_count=len(new_obs_pose),interval=50)
    ani.save('Gifs6/const4-trial6-tdoa2-traj3.gif',writer = 'pillow')
    #plt.show()

moving_pictures()