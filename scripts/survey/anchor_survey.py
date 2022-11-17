'''
    Survey code. Convert the Total Station survey results into an inertial frame (Vicon frame).
    Using 6 vicon markers with known positions, the survey results are converted through a point cloud alignment. 
'''
import os, sys
import argparse
import numpy as np
from pyquaternion import Quaternion      # package for quaternion
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.style as style

style.use('ggplot')

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
    
    # point cloud alignment     
    u, s, vh = np.linalg.svd(W)  
    
    D = np.array([[     1,     0,     0],
                [     0,     1,     0],
                [     0,     0,   np.linalg.det(u)*np.linalg.det(vh.T)]])
    # C is the rotation matrix rotate point cloud to Total station frame (convert src_points to dest_points)
    C = u.dot(D).dot(vh)
    
    r_a_ba = -(C.T).dot(dest_points_centroid) + src_points_centroid
    
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
    ax_t.set_zlim([0.0, 4.0])
    # plot vicon frame
    for idx in range(len(vicon)):
        ax_t.scatter(vicon[idx,0], vicon[idx,1], vicon[idx,2],s=5, marker='o',color='red')

if __name__ == "__main__":
    # get the name of survey file
    # save the anchor position and quaternion with the same name
    txt_file = os.path.split(sys.argv[-1])[1]
    txt_name = os.path.splitext(txt_file)[0]

    parser = argparse.ArgumentParser()
    parser.add_argument("survey_txt")
    args = parser.parse_args()

    # ----- parameter ----- #
    SAVE_SURVEY = False

    # read file
    f1 = open(args.survey_txt,"r")
    pos=[]
    for line in f1:
        x = line.split(",")
        # print(x[0])
        if len(x) > 3:
            arr_x = [float(x[1]), float(x[2]), float(x[3])]
            pos.append(arr_x)
        
    pos = np.array(pos)
    NUM_vicon = 6
    vicon_m = pos[0:NUM_vicon,:] / 1000.0               # positions of the NUM_vicon markers (vicon frame)
    vicon_frame = pos[NUM_vicon:2*NUM_vicon,:]          # positions of the NUM_vicon markers (total station frame)
    anchor_marker = pos[2*NUM_vicon:,:]                 # uwb anchor pose 

    dest_point = vicon_m
    src_point = vicon_frame
    # compute the C and r from src_point to dest_point
    (C_est, r_est) = pointcloud_alignment(src_point, dest_point)
    print('The result of point cloud alignment:\n')
    print("Rotation matrix (from total station survey to vicon frame):\n")
    print(C_est) 
    print("Translation vector (from total station survey to vicon frame):\n")
    print(r_est)
    print('\n')


    # vicon markers (Total Station Survey results)
    vicon_marker = np.empty((0,3))
    for idx in range(NUM_vicon):
        vf = C_est.dot(vicon_frame[idx,:].reshape(-1,1)) + r_est.reshape(-1,1)
        vf = vf.reshape(1,-1)
        vicon_marker = np.vstack((vicon_marker, vf))
        
    vicon_marker = np.asarray(vicon_marker)

    # testing the survey accuracy (vicon_marker, dest_points)
    err = vicon_m - vicon_marker
    RMSE = getRMSE(err)
    print("The RMS error of Total Station survey results is {0} [m], ({1} [mm])".format(RMSE, RMSE*1000))

    Marker_pos = np.empty([3, 0])
    for i in range(len(anchor_marker)):
        an_i = C_est.dot(anchor_marker[i,:].reshape(-1,1)) + r_est.reshape(-1,1)
        Marker_pos = np.concatenate((Marker_pos,an_i), axis=1)
        
    Marker_pos = Marker_pos.T

    # -------------------- Data Structure ------------------- #
    # An0_M0: x, y, z, 
    # An0_M1: x, y, z,
    # An0_M2: x, y, z, 　
    # An0_p:  x, y, z,
    An_num = 8
    anchor_position = np.zeros((An_num,3))
    anchor_quaterion = np.zeros((An_num,4))
    counter = 0
    idx = 0
    for i in range(An_num):
        # three markers in the inertial frame
        dest_points = Marker_pos[idx:idx+3,:]
        # 3 markers in the Anchor frame. Order: [up, left, forward]
        src_points = np.array([[110.0,   0.0,   0.0],
                               [0.0,   121.0,   0.0],
                               [0.0,     0.0,   80.0]]) / 1000.0 
        # compute the rotation from body to inertial frame
        (C_survey, t_est) = pointcloud_alignment(src_points, dest_points)   
        # save the quaternion from anchor body frame to inertial frame
        q_survey = Quaternion(matrix=C_survey)
        anchor_quaterion[counter,:] = np.array([q_survey.x, q_survey.y,q_survey.z, q_survey.w])
        # save the anchor position
        anchor_position[counter,:] = Marker_pos[idx+3,:]
        counter += 1
        idx += 4 

    print('\n')
    print("The anchor positions are")
    print(np.round(anchor_position,5))
    print('\n')
    print("The anchor orientations are")
    print(np.round(anchor_quaterion,5))

    if SAVE_SURVEY:
        # save the anchor positions and orientation into numpy array
        np.savez(txt_name, an_pos = anchor_position, an_quat = anchor_quaterion)

        # save txt for the anchor poses
        txt_file = txt_name +'_survey.txt'
        with open(txt_file,"w") as f:
            for i in range(len(anchor_position)):
                f.write('an'+str(i)+'_p,' + str(anchor_position[i,0]) + ',' + 
                        str(anchor_position[i,1]) + ',' + str(anchor_position[i,2]) + '\n')

            for j in range(len(anchor_quaterion)):
                f.write('an'+str(j)+'_quat,' + str(anchor_quaterion[j,0]) + ',' + str(anchor_quaterion[j,1]) + ',' +
                        str(anchor_quaterion[j,2]) + ',' + str(anchor_quaterion[j,3]) + '\n')

    # --------------- Visualization ---------------- #
    plot_survey(Marker_pos, vicon_marker)
    plt.show()
