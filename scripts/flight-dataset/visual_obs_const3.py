'''
    Visualize the manual movement trajectories and obstacles in const3
    Note: no mattress is used in manual movement experiments (const3)
'''
import argparse
import numpy as np
import os, sys
import rosbag
from matplotlib import pyplot as plt
import matplotlib

FONTSIZE = 18;   TICK_SIZE = 16
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

# help functions for obstascle visualization
def plot_obs(ob_x,obstacle):
    for i in range(4):
        for j in range(i,4):
            ob_x.plot([obstacle[i,0], obstacle[j,0]], [obstacle[i,1], obstacle[j,1]], [obstacle[i,2], obstacle[j,2]], linewidth=1, label='_nolegend_', color='k')
    
    for i in range(4,8):
        for j in range(i,8):
            ob_x.plot([obstacle[i,0], obstacle[j,0]], [obstacle[i,1], obstacle[j,1]], [obstacle[i,2], obstacle[j,2]], linewidth=1, label='_nolegend_', color='k')
    
    for i in range(4):
        ob_x.plot([obstacle[i,0], obstacle[i+4,0]], [obstacle[i,1], obstacle[i+4,1]], [obstacle[i,2], obstacle[i+4,2]], linewidth=1, label='_nolegend_', color='k')

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("ros_bag")
    args = parser.parse_args()

    # access the survey results
    cwd = os.path.dirname(__file__)
    anchor_npz = '../../dataset/flight-dataset/survey-results/anchor_const3.npz'
    anchor_survey = np.load(os.path.join(cwd, anchor_npz))
    anchor_pos = anchor_survey['an_pos']
    
    # access rosbag
    ros_bag = args.ros_bag
    bag = rosbag.Bag(ros_bag)
    bag_file = os.path.split(sys.argv[-1])[1]

    # print out
    bag_name = os.path.splitext(bag_file)[0]
    print("\nvisualizing rosbag: " + str(bag_file) + "\n")

    # extract the name of the trial in const4
    CONST = bag_name.split('-')[0]
    NLOS_TRIAL = bag_name.split('-')[1]
    if CONST != "const3" or NLOS_TRIAL != "trial7":
        print("\n Please visualize trial7 in const3. \n")
        sys.exit()

    # -------------- visualize obstacles -------------------- #
    woodenshelf = np.array([[1850.02,   1021.90,  2032.84],
                            [2411.24,    413.79,  2032.20],
                            [1724.89,    793.20,  2035.44],
                            [2533.86,    646.41,  2031.22],
                            [1850.02,   1021.90,  0.0],
                            [2411.24,    413.79,  0.0],
                            [1724.89,    793.20,  0.0],
                            [2533.86,    646.41,  0.0]
                           ]) / 1000.0

    plasticbox = np.array([[2688.82,  -2495.71,  853.62],
                           [2484.05,  -2977.32,  861.63],
                           [2365.28,  -2356.26,  854.47],
                           [2151.78,  -2837.41,  861.19],
                           [2688.82,  -2495.71,  0.0],
                           [2484.05,  -2977.32,  0.0],
                           [2365.28,  -2356.26,  0.0],
                           [2151.78,  -2837.41,  0.0]
                          ]) / 1000.0 

    metal      = np.array([[-1286.08,  -1445.97,  1122.17],
                           [-791.69,   -1445.40,  1127.81],
                           [-804.78,    -530.59,  1124.39],
                           [-1295.78,   -535.66,  1117.10],
                           [-1286.08,  -1445.97,  0.0],
                           [-791.69,   -1445.40,  0.0],
                           [-804.78,    -530.59,  0.0],
                           [-1295.78,   -535.66,  0.0]
                          ]) / 1000.0

    # -------------------- extract the rosbag ----------------------------- #
    gt_pose = [] 
    for topic, msg, t in bag.read_messages(['/pose_data']):
        if topic == "/pose_data":
            gt_pose.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                            msg.pose.pose.position.x,    msg.pose.pose.position.y,    msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])

    gt_pose = np.array(gt_pose)

    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_subplot(111, projection = '3d')
    # make the panes transparent
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines 
    ax.xaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax.yaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ax.zaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)

    ax.plot(gt_pose[:,1],gt_pose[:,2],gt_pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9, label = 'Quadcopter trajectory')
    ax.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2], s = 100, marker='o',color='red', label = 'Anchor position')
    
    # plot lines among obstacle vertices
    plot_obs(ax, metal)
    ax.scatter(metal[:,0], metal[:,1], metal[:,2],  s = 100, marker='o',color='navy', label = 'Metal box')

    plot_obs(ax, woodenshelf)
    ax.scatter(woodenshelf[:,0], woodenshelf[:,1], woodenshelf[:,2],  s = 100, marker='o',color='teal', label = 'Woodshelf')

    plot_obs(ax, plasticbox)
    ax.scatter(plasticbox[:,0], plasticbox[:,1], plasticbox[:,2],  s = 100, marker='o',color='orange', label = 'Plastic box')

    ax.set_xlim3d(-4.5, 4.5)  
    ax.set_ylim3d(-4.5, 4.5)  
    ax.set_zlim3d(0.0, 3.5)  
    ax.set_xlabel(r'X [m]', fontsize = FONTSIZE)
    ax.set_ylabel(r'Y [m]', fontsize = FONTSIZE)
    ax.set_zlabel(r'Z [m]', fontsize = FONTSIZE)
    ax.set_box_aspect((1, 1, 0.35))               # xy aspect ratio is 1:1, but change z axis
    plt.legend(fontsize=FONTSIZE)
    ax.view_init(20, -60)
    fig.tight_layout()

    plt.show()




