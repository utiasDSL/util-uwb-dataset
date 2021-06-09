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
    wood_x = np.array([1892.22, 1710.66, 2264.10, 2449.43]) / 1000.0
    wood_y = np.array([2587.42, 2403.10, 1852.96, 2034.15]) / 1000.0
    wood_z = np.array([0.0,  2025.84]) / 1000.0

    plastic_x = np.array([ -2290.30,  -1956.36, -1678.54, -2014.29]) / 1000.0
    plastic_y = np.array([ 1919.69,  2326.76, 2094.41, 1688.40]) / 1000.0
    plastic_z = np.array([0.0, 846.06]) / 1000.0

    metal_x = np.array([-1231.57, -1925.50, -2243.56, -1554.55]) / 1000.0
    metal_y = np.array([-2963.78, -2364.54, -2735.56, -3334.40]) / 1000.0
    metal_z = np.array([0.0, 1142.45]) / 1000.0

    cardboard_x = np.array([2297.77, 2593.35, 3058.63, 2751.64]) / 1000.0
    cardboard_y = np.array([-2405.36, -1960.48, -2266.65, -2700.92]) / 1000.0
    cardboard_z = np.array([0.0, 1160.56]) / 1000.0

    vec_wood = coordConvert(wood_x, wood_y, wood_z[0], wood_z[1])
    vec_plastic = coordConvert(plastic_x, plastic_y, plastic_z[0], plastic_z[1])
    vec_metal = coordConvert(metal_x, metal_y, metal_z[0], metal_z[1])
    vec_cardboard = coordConvert(cardboard_x, cardboard_y, cardboard_z[0], cardboard_z[1])

    # markers
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
    ax.plot(gt_pose[:,1],gt_pose[:,2],gt_pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9, label = 'Trajectory')
    ax.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2], marker='o',color='red', label = 'Anchor position')

    ax.scatter(metal[:,0], metal[:,1], metal[:,2], marker='o',color='navy')
    plt.gca().add_collection3d(Poly3DCollection(vec_metal, alpha=.75,edgecolor='k', facecolor='navy'))

    ax.scatter(woodshelf[:,0], woodshelf[:,1], woodshelf[:,2], marker='o',color='yellow')
    plt.gca().add_collection3d(Poly3DCollection(vec_wood, alpha=.75,edgecolor='k', facecolor='yellow'))

    ax.scatter(plasticbox[:,0], plasticbox[:,1], plasticbox[:,2], marker='o',color='teal')
    plt.gca().add_collection3d(Poly3DCollection(vec_plastic, alpha=.75,edgecolor='k', facecolor='teal'))

    ax.scatter(cardboard[:,0], cardboard[:,1], cardboard[:,2], marker='o',color='brown')
    plt.gca().add_collection3d(Poly3DCollection(vec_cardboard, alpha=.75,edgecolor='k', facecolor='brown'))

    ax.set_xlim3d(np.amin(anchor_pos[:,0])-0.5, np.amax(anchor_pos[:,0])+0.5)  
    ax.set_ylim3d(np.amin(anchor_pos[:,1])-0.5, np.amax(anchor_pos[:,1])+0.5)  
    ax.set_zlim3d(np.amin(anchor_pos[:,2])-0.1, np.amax(anchor_pos[:,2])+0.3)  
    ax.set_xlabel(r'X [m]')
    ax.set_ylabel(r'Y [m]')
    ax.set_zlabel(r'Z [m]')
    plt.legend(loc="upper right")
    plt.legend(['Trajectory','Anchor position'])
    plt.title(r"Trajectory of the experiment", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )

    plt.show()




