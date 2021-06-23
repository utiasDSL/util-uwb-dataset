'''
Visualize the obstacles
'''
import argparse
import numpy as np
import os, sys
import rosbag
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import pyplot as plt
import matplotlib.style as style
# select the matplotlib plotting style
style.use('ggplot')
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

# help functions for obstascle visualization
def edgecoord(pointx,pointy,pointz):
    edgex=[pointx[0],pointx[1],pointx[1],pointx[0]]
    edgey=[pointy[0],pointy[1],pointy[1],pointy[0]]
    edgez=[pointz[0],pointz[0],pointz[1],pointz[1]]
    return list(zip(edgex,edgey,edgez))

def coordConvert(x,y,lheight,uheight):
    if len(x) != len(y) and len(x)>2:
        return
    vertices=[]
    #Top layer
    vertices.append(list(zip(x,y,list(np.full(len(x),uheight)))))
    # Side layers
    for it in np.arange(len(x)):
        it1=it+1
        if it1>=len(x):
            it1=0
        vertices.append(edgecoord([x[it],x[it1]],[y[it],y[it1]],[lheight,uheight]))
    #Bottom layer
    vertices.append(list(zip(x,y,list(np.full(len(x),lheight)))))
    # print(np.array(vertices))
    return vertices

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-i', action='store', nargs=2)
    args = parser.parse_args()
    
    # access the survey results
    anchor_npz = args.i[0]
    anchor_survey = np.load(anchor_npz)
    anchor_pos = anchor_survey['an_pos']
    anchor_qaut = anchor_survey['an_quat']
    
    # print out
    anchor_file = os.path.split(sys.argv[-2])[1]
    print("loading anchor survey results: " + str(anchor_file) + "\n")

    # access rosbag
    ros_bag = args.i[1]
    bag = rosbag.Bag(ros_bag)
    bag_file = os.path.split(sys.argv[-1])[1]

    # print out
    bag_name = os.path.splitext(bag_file)[0]
    print("visualizing rosbag: " + str(bag_file) + "\n")

    # -------------- visualize obstacles -------------------- #
    wood_x = np.array([1850.02, 1724.89, 2411.24, 2533.86]) / 1000.0
    wood_y = np.array([1021.90, 793.20, 413.79, 646.41]) / 1000.0
    wood_z = np.array([0.0,  2032.84]) / 1000.0

    plastic_x = np.array([ 2484.05, 2151.78, 2365.28, 2688.82]) / 1000.0
    plastic_y = np.array([ -2977.32,  -2837.41, -2356.26, -2495.71]) / 1000.0
    plastic_z = np.array([0.0, 853.62]) / 1000.0

    metal_x = np.array([-1286.08, -791.69, -804.78, -1295.78]) / 1000.0
    metal_y = np.array([-1445.97, -1445.40, -530.59, -535.66]) / 1000.0
    metal_z = np.array([0.0, 1127.81]) / 1000.0

    vec_wood = coordConvert(wood_x, wood_y, wood_z[0], wood_z[1])
    vec_plastic = coordConvert(plastic_x, plastic_y, plastic_z[0], plastic_z[1])
    vec_metal = coordConvert(metal_x, metal_y, metal_z[0], metal_z[1])
    # markers
    woodshelf = np.array([[1850.02,  1021.90,  2032.84],
                         [2411.24,    413.79,  2032.20],
                         [1724.89,    793.20,  2035.44],
                         [2533.86,    646.41,  2031.22]
                         ]) / 1000.0

    plasticbox = np.array([[2688.82, -2495.71,  853.62],
                          [2484.05,  -2977.32,  861.63],
                          [2365.28,  -2356.26,  854.47],
                          [2151.78,  -2837.41,  861.19]
                          ]) / 1000.0 

    metal = np.array([[-1286.08,  -1445.97,  1122.17],
                      [-791.69,   -1445.40,  1127.81],
                      [-804.78,    -530.59,  1124.39],
                      [-1295.78,   -535.66,  1117.10]
                      ]) / 1000.0

    # -------------------- extract the rosbag ----------------------------- #
    gt_pose = [] 
    for topic, msg, t in bag.read_messages(['/pose_data']):
        if topic == "/pose_data":
            gt_pose.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                            msg.pose.pose.position.x,    msg.pose.pose.position.y,    msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])

    gt_pose = np.array(gt_pose)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.plot(gt_pose[:,1],gt_pose[:,2],gt_pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9, label = 'Quadcopter Trajectory')
    ax.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2], marker='o',color='red', label = 'Anchor position')
    
    ax.scatter(metal[:,0], metal[:,1], metal[:,2], marker='o',color='navy', label = 'metal')
    plt.gca().add_collection3d(Poly3DCollection(vec_metal, alpha=.75,edgecolor='k', facecolor='navy'))
    
    ax.scatter(woodshelf[:,0], woodshelf[:,1], woodshelf[:,2], marker='o',color='yellow', label = 'woodshelf')
    plt.gca().add_collection3d(Poly3DCollection(vec_wood, alpha=.75,edgecolor='k', facecolor='yellow'))

    ax.scatter(plasticbox[:,0], plasticbox[:,1], plasticbox[:,2], marker='o',color='teal', label = 'plastic box')
    plt.gca().add_collection3d(Poly3DCollection(vec_plastic, alpha=.75,edgecolor='k', facecolor='teal'))

    ax.set_xlim3d(np.amin(anchor_pos[:,0])-0.5, np.amax(anchor_pos[:,0])+0.5)  
    ax.set_ylim3d(np.amin(anchor_pos[:,1])-0.5, np.amax(anchor_pos[:,1])+0.5)  
    ax.set_zlim3d(np.amin(anchor_pos[:,2])-0.1, np.amax(anchor_pos[:,2])+0.3)  
    ax.set_xlabel(r'X [m]')
    ax.set_ylabel(r'Y [m]')
    ax.set_zlabel(r'Z [m]')
    plt.legend(loc="upper right")
    plt.title(r"Trajectory of the experiment", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )

    plt.show()




