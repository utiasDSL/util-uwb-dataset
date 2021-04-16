'''
survey util functions
'''
import numpy as np
import sys
import matplotlib.pyplot as plt

def pointcloud_alignment(src_points, dest_points):
    '''
        point cloud alignment
        Calculates SE(3) transform from src_points to dest_points
    Input:
        src_points:   point cloud from previoud frame
        dest_points:  point cloud in current frame
        [3 x N] arrays, rows are x, y, z
        [[p1_x, ..., pN_x]
            [p1_y, ..., pN_y]
            [p1_z, ..., pN_z]]
    Return:
        C: rotational matrix -  [3 x 3]
        r: translation vector - [3 x 1]
    '''
    # Note: Assume: scalar weighted and all weights are 1.
    # User can specify other weights
    src_points = src_points.T
    dest_points = dest_points.T
    # shape consistency check 
    (row, col) = src_points.shape
    if not (row == dest_points.shape[0] and col == dest_points.shape[1]):
        print('Number of points in source and destination point clouds not equal!')
        sys.exit(0)
    if src_points.shape[1] < 3:
        print('Insufficient source points. Minimum:[%d] Provided:[%d]'%(3, src_points.shape[1]))
        sys.exit(0)
    if dest_points.shape[1] < 3:
        print('Insufficient destination points. Minimum:[%d] Provided:[%d]'%(3, dest_points.shape[1]))
        sys.exit(0)
    N = src_points.shape[1]
    src_points_centroid = np.sum(src_points, axis=1) / N
    dest_points_centroid = np.sum(dest_points, axis=1) / N
    src_points_rem = src_points - src_points_centroid.reshape(-1,1)
    dest_points_rem = dest_points - dest_points_centroid.reshape(-1,1)

    # compute the Weight matrix
    W = dest_points_rem.dot(src_points_rem.T) / N
    
    # point cloud alignment: State Estimation lec 11        
    u, s, vh = np.linalg.svd(W)  # vh is already the transpose of V in SE lec11 slides
    
    D = np.array([[     1,     0,     0],
                [     0,     1,     0],
                [     0,     0,   np.linalg.det(u)*np.linalg.det(vh.T)]])
    # C is the rotation matrix rotate point cloud to Total station frame (convert src_points to dest_points)
    C = u.dot(D).dot(vh)
    
    r_a_ba = - (C.T).dot(dest_points_centroid) + src_points_centroid
    
    r_b_ba = -(C).dot(r_a_ba.reshape(-1,1))
    # the return value can be used to construct the Transformation matrix directly
    # T = [[C,   r_b_ba],
    #      [0, 0, 0, 1]]
    # Converting src points to dest points
    # dest = C.dot(src.reshape(-1,1)) +  r_b_ba.reshape(-1,1)   
    return C, r_b_ba

def getRMSE(err):
    # data format: err = [[e_x1, e_y1, e_z1],
    #                     [e_x2, e_y2, e_z2], ...]
    rmse = 0
    NUM = err.shape[0]
    for i in range(NUM):
        ei = err[i,0]**2 + err[i,1]**2 + err[i,2]**2
        rmse +=ei
    RMSE = np.sqrt(rmse / NUM)
    return RMSE


def plot_survey(anchor_pos, vicon):
    fig_traj = plt.figure(facecolor = "white")
    ax_t = fig_traj.add_subplot(111, projection = '3d')
    counter = 0
    for i in range(len(anchor_pos)):
        if counter == 0:    
            ax_t.scatter(anchor_pos[i,0], anchor_pos[i,1], anchor_pos[i,2], s=5, marker='o',color='yellow')
            counter+=1
        elif counter == 1:
            ax_t.scatter(anchor_pos[i,0], anchor_pos[i,1], anchor_pos[i,2], s=5, marker='o',color='red')
            counter+=1
        elif counter == 2:
            ax_t.scatter(anchor_pos[i,0], anchor_pos[i,1], anchor_pos[i,2], s=5, marker='o',color='green')
            counter+=1
        elif counter == 3:
            ax_t.scatter(anchor_pos[i,0], anchor_pos[i,1], anchor_pos[i,2], s=5, marker='o',color='blue')
            counter = 0
            
            
    # use LaTeX fonts in the plot
    ax_t.set_xlabel(r'X [m]')
    ax_t.set_ylabel(r'Y [m]')
    ax_t.set_zlabel(r'Z [m]')
    plt.title(r"UWB Anchor Survey Results", fontsize=13, fontweight=0, color='black', style='italic', y=1.02 )
    ax_t.set_xlim([-5.0, 5.0])
    ax_t.set_ylim([-5.0, 5.0])
    ax_t.set_zlim([0.0, 5.0])
    # plot vicon frame
    for idx in range(len(vicon)):
        ax_t.scatter(vicon[idx,0], vicon[idx,1], vicon[idx,2],s=5, marker='o',color='red')

