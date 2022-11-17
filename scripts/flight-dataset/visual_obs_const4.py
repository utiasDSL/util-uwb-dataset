'''
    Visualize static obstacles and experimental trajectories in const4
'''
import os, sys
import argparse
import rosbag
import numpy as np
import matplotlib
from matplotlib import pyplot as plt

FONTSIZE = 18;   TICK_SIZE = 16
# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
matplotlib.rc('xtick', labelsize=TICK_SIZE) 
matplotlib.rc('ytick', labelsize=TICK_SIZE) 

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
    anchor_npz = '../../dataset/flight-dataset/survey-results/anchor_const4.npz'
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
    if CONST != "const4":
        print("\n Please visualize trial{2,3,4,7} in const4. \n")
        sys.exit()

    NLOS_TRIAL = bag_name.split('-')[1]
    # -------------------- extract the rosbag ----------------------------- #
    gt_pose = [] 
    for topic, msg, t in bag.read_messages(['/pose_data']):
        if topic == "/pose_data":
            gt_pose.append([msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
                            msg.pose.pose.position.x,    msg.pose.pose.position.y,    msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ])

    gt_pose = np.array(gt_pose)


    # visualization
    # visualize the anchor, tag and obstacle
    fig_ob = plt.figure(figsize=(16, 9))
    ob_x = fig_ob.add_subplot(111, projection = '3d')
    # make the panes transparent
    ob_x.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ob_x.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ob_x.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    # change the color of the grid lines 
    ob_x.xaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ob_x.yaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)
    ob_x.zaxis._axinfo["grid"]['color'] =  (0.5,0.5,0.5,0.5)

    ob_x.plot(gt_pose[:,1],gt_pose[:,2],gt_pose[:,3],color='steelblue',linewidth=1.9, alpha=0.9, label = 'Quadcopter trajectory')
    ob_x.scatter(anchor_pos[:,0], anchor_pos[:,1], anchor_pos[:,2],  s = 100, marker='o',color='red', label = 'Anchor position')

    # select nlos trail [2,3,4,7]
    if NLOS_TRIAL == 'trial2':
        # one wood: nlos-trial2
        obstacle =  np.array([[3186.44, 2235.94, 1176.64],
                            [3393.83, 2623.23, 1184.36],
                            [2697.02, 2992.17, 1183.92],
                            [2498.03, 2602.52, 1181.13],
                            [3186.44, 2235.94, 0.0],
                            [3393.83, 2623.23, 0.0],
                            [2697.02, 2992.17, 0.0],
                            [2498.03, 2602.52, 0.0]
                    ]) / 1000.0
        ob_x.scatter(obstacle[:,0], obstacle[:,1], obstacle[:,2], s = 100, marker='o',color='teal', label = 'Wooden box')
        # plot lines among obstacle vertices
        plot_obs(ob_x,obstacle)

    elif NLOS_TRIAL == 'trial3':
        # one metal: nlos-trial3
        obstacle = np.array([[2729.07, 2952.82, 1127.28],
                            [2940.75, 3392.38, 1125.00],
                            [3552.64, 2557.96, 1125.12],
                            [3762.65, 3000.68, 1122.71],
                            [2729.07, 2952.82, 0.0],
                            [2940.75, 3392.38, 0.0],
                            [3552.64, 2557.96, 0.0],
                            [3762.65, 3000.68, 0.0]
                    ]) / 1000.0
        ob_x.scatter(obstacle[:,0], obstacle[:,1], obstacle[:,2], s = 100, marker='o',color='navy', label = 'Metal box')
        # plot lines among obstacle vertices
        plot_obs(ob_x,obstacle)

    elif NLOS_TRIAL == 'trial4':
        # one metal, 3 wood: nlos-trial4
        metal = np.array([[2687.61, 2817.24, 1127.49 ],
                        [2852.76, 3281.56, 1126.42 ],
                        [3553.28, 2519.73, 1127.18 ],
                        [3715.45, 2982.98, 1124.24 ],
                        [2687.61, 2817.24, 0.0 ],
                        [2852.76, 3281.56, 0.0 ],
                        [3553.28, 2519.73, 0.0 ],
                        [3715.45, 2982.98, 0.0 ]
                ]) / 1000.0

        wood1 = np.array([[-2104.36, 2847.21, 1178.37 ],
                        [-2799.80, 2480.11, 1169.99 ],
                        [-2310.25, 3242.73, 1177.92 ],
                        [-2999.76, 2870.60, 1169.09 ],
                        [-2104.36, 2847.21, 0.0 ],
                        [-2799.80, 2480.11, 0.0 ],
                        [-2310.25, 3242.73, 0.0 ],
                        [-2999.76, 2870.60, 0.0 ]
                ]) / 1000.0
                
        wood2 = np.array([[-2376.38, -3193.74, 1187.90],
                        [-2321.36, -4103.39, 1189.12],
                        [-1952.04, -3853.67, 1188.85],
                        [-2747.32, -3431.53, 1190.66],
                        [-2376.38, -3193.74, 0.0  ],
                        [-2321.36, -4103.39, 0.0  ],
                        [-1952.04, -3853.67, 0.0  ],
                        [-2747.32, -3431.53, 0.0 ]
                ])/1000.0

        wood3 = np.array([[2876.80,  -3583.70,  1200.27],
                        [3666.34,  -3160.65,  1199.14],
                        [3300.09,  -2919.38,  1197.27],
                        [3239.95,  -3816.29,  1198.92],
                        [2876.80,  -3583.70,  0.0],
                        [3666.34,  -3160.65,  0.0],
                        [3300.09,  -2919.38,  0.0],
                        [3239.95,  -3816.29,  0.0],
                ])/1000.0

        ob_x.scatter(metal[:,0], metal[:,1], metal[:,2], s = 100, marker='o',color='navy', label = 'Metal box')
        ob_x.scatter(wood1[:,0], wood1[:,1], wood1[:,2], s = 100, marker='o',color='teal', label = 'Wooden box')
        ob_x.scatter(wood2[:,0], wood2[:,1], wood2[:,2], s = 100, marker='o',color='teal')
        ob_x.scatter(wood3[:,0], wood3[:,1], wood3[:,2], s = 100, marker='o',color='teal')
        # plot lines among obstacle vertices
        plot_obs(ob_x,metal)
        plot_obs(ob_x,wood1)
        plot_obs(ob_x,wood2)
        plot_obs(ob_x,wood3)

    elif NLOS_TRIAL == 'trial7':
        # one metal, 3 wood: manual data collection
        metal = np.array([[1002.99, 1017.36, 1119.91],
                        [1918.01, 1029.30, 1126.73],
                        [1007.93, 527.63,  1124.59],
                        [1923.61, 537.66,  1132.03],
                        [1002.99, 1017.36, 0.0 ],
                        [1918.01, 1029.30, 0.0 ],
                        [1007.93, 527.63,  0.0 ],
                        [1923.61, 537.66,  0.0 ]
                ]) / 1000.0

        wood1 = np.array([[-1275.71, -1144.55, 1176.03 ],
                        [-835.21,  -1142.12, 1174.83 ],
                        [-1250.03, -1936.30, 1183.95 ],
                        [-815.88,  -1926.81, 1183.15],
                        [-1275.71, -1144.55, 0.0 ],
                        [-835.21,  -1142.12, 0.0 ],
                        [-1250.03, -1936.30, 0.0 ],
                        [-815.88,  -1926.81, 0.0 ]
                ]) / 1000.0
                
        wood2 = np.array([[-922.80, 991.97, 1175.39 ],
                        [-135.99, 991.32, 1175.15 ],
                        [-928.56, 553.41, 1172.59 ],
                        [-137.42, 548.27, 1177.23 ],
                        [-922.80, 991.97, 0.0 ],
                        [-135.99, 991.32, 0.0 ],
                        [-928.56, 553.41, 0.0 ],
                        [-137.42, 548.27, 0.0 ]
                ])/1000.0

        wood3 = np.array([[745.70,  -654.94,  1183.49 ],
                        [1183.48, -629.56,  1183.21 ],
                        [784.30,  -1441.95, 1186.21 ],
                        [1220.82, -1422.85, 1193.04 ],
                        [745.70,  -654.94,  0.0 ],
                        [1183.48, -629.56,  0.0 ],
                        [784.30,  -1441.95, 0.0 ],
                        [1220.82, -1422.85, 0.0 ],
                ])/1000.0

        ob_x.scatter(metal[:,0], metal[:,1], metal[:,2], s = 100, marker='o',color='navy', label = 'Metal box')
        ob_x.scatter(wood1[:,0], wood1[:,1], wood1[:,2], s = 100, marker='o',color='teal', label = 'Wooden box')
        ob_x.scatter(wood2[:,0], wood2[:,1], wood2[:,2], s = 100, marker='o',color='teal')
        ob_x.scatter(wood3[:,0], wood3[:,1], wood3[:,2], s = 100, marker='o',color='teal')
        # plot lines among obstacle vertices
        plot_obs(ob_x,metal)
        plot_obs(ob_x,wood1)
        plot_obs(ob_x,wood2)
        plot_obs(ob_x,wood3)
    else:
        print("\n Please visualize trial{2,3,4,7} in const4. \n")
        sys.exit()

    ob_x.set_xlim3d(-4.5, 4.5)  
    ob_x.set_ylim3d(-4.5, 4.5)  
    ob_x.set_zlim3d(0.0, 3.5)  
    ob_x.set_xlabel(r'X [m]', fontsize = FONTSIZE)
    ob_x.set_ylabel(r'Y [m]', fontsize = FONTSIZE)
    ob_x.set_zlabel(r'Z [m]', fontsize = FONTSIZE)
    ob_x.set_box_aspect((1, 1, 0.35)) 
    ob_x.tick_params(axis='x', which='major', pad=0)
    ob_x.tick_params(axis='y', which='major', pad=0)
    ob_x.tick_params(axis='z', which='major', pad=1.5)
    plt.legend(loc='best',fontsize=FONTSIZE)
    ob_x.view_init(20, -60)
    fig_ob.tight_layout()
    
    plt.show()




