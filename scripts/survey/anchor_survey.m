%    Survey code. Convert the Total Station survey results into an inertial frame (Vicon frame).
%    Using 6 vicon markers with known positions, the survey results are converted through a point cloud alignment. 
%

clear; close all
clc;

% get the full path of the current script
filepath = fileparts(mfilename('fullpath'));

% change the path to the survey results and the data
txt = fullfile(filepath, '../../dataset/flight-dataset/survey-results/raw-data/anchor_const1.txt');

survey = readtable(txt, 'ReadRowNames',true);
pos = [survey.Var1, survey.Var2, survey.Var3];

NUM_vicon = 6;

vicon_m = pos(1:NUM_vicon, :) / 1000.0;                      % positions of the NUM_vicon markers (vicon frame)
vicon_frame = pos(NUM_vicon + 1 : 2*NUM_vicon, :);           % positions of the NUM_vicon markers (total station frame)
anchor_marker = pos(2*NUM_vicon + 1 : end,:);                % uwb anchor pose

dest_point = vicon_m;
src_point = vicon_frame;

% compute the C and r from src_point to dest_point
[C_est, r_est] = pointcloud_alignment(src_point, dest_point);
disp('The result of point cloud alignment: ')
disp("Rotation matrix (from total station survey to vicon frame): ")
C_est 
disp("Translation vector (from total station survey to vicon frame): ")
r_est

% vicon markers (total station survey results)
vicon_marker = [];
for idx = 1 : NUM_vicon
    vf = C_est * vicon_frame(idx,:)' + r_est;
    vicon_marker = [vicon_marker; vf'];        % store in row
end

% test the accuracy between vicon marker and dest_points
err = vecnorm(vicon_m - vicon_marker,2,2);
RMSE = mean(err);
disp(['The RMS error of Total Station survey results is ',num2str(RMSE), '[m] (', num2str(1000*RMSE),'[mm])'])

Marker_pos = [];
for i = 1: length(anchor_marker)
    an_i = C_est * anchor_marker(i,:)' + r_est;
    Marker_pos = [Marker_pos, an_i];         % store vertically
end

Marker_pos = Marker_pos';                    % transpose

% -------------------- Data Structure ------------------- %
% An0_M0: x, y, z, 
% An0_M1: x, y, z,
% An0_M2: x, y, z, 　
% An0_p:  x, y, z,
An_num = 8; 
idx = 1;
% 3 markers in the Anchor frame. Order: [up, left, forward]
src_points = [110.0,   0.0,   0.0;
              0.0,   121.0,   0.0;
              0.0,     0.0,   80.0] / 1000.0;
          
anchor_position = [];
anchor_quaterion = [];
for i = 1 : An_num
    % three markers in the inertial frame
    dest_points = Marker_pos(idx:idx+2,:);
    % compute the rotation from body to inertial frame
    [C_survey, t_est] = pointcloud_alignment(src_points, dest_points);  
    % quaternion from anchor body frame to inertial frame
    q_survey = rot_to_quat(C_survey);
    anchor_quaterion = [anchor_quaterion; q_survey];
    % save the anchor position
    anchor_position = [anchor_position; Marker_pos(idx+3,:)];
    idx = idx +4; 
end

disp('The anchor positions are')
anchor_position
disp('The anchor orientations are')
anchor_quaterion

% Visualize
fig = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
scatter3(Marker_pos(:,1),Marker_pos(:,2),Marker_pos(:,3), 20, 'b','filled')
hold on
scatter3(vicon_marker(:,1), vicon_marker(:,2),vicon_marker(:,3), 20, 'r','filled')
hold off
title('UWB Anchor Survey Results','Interpreter','latex','Fontsize',16)
xlabel('X [m]','Interpreter','latex','Fontsize',16)
ylabel('Y [m]','Interpreter','latex','Fontsize',16)
zlabel('Z [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
grid on
xlim([-5.0, 5.0])  
ylim([-5.0, 5.0])  
zlim([0.0, 4.0]) 


function [q] = rot_to_quat(C)
% convert rotation matrix to quaternion
% reference: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/ 
    tr = trace(C);
    if(tr > 0)
        s = sqrt(tr + 1) * 2;
        qw = 0.25 * s;
        qx = (C(3,2) - C(2,3)) / s;
        qy = (C(1,3) - C(3,1)) / s;
        qz = (C(2,1) - C(1,2)) / s;
        
    elseif (C(1,1) > C(2,2) && (C(1,1) > C(3,3)))
        s = sqrt(1 + C(1,1) - C(2,2) - C(3,3)) * 2;
        qw = (C(3,2)-C(2,3)) / s;
        qx =  0.25 * s;
        qy = (C(1,2) + C(2,1)) / s;
        qz = (C(1,3) + C(3,1)) / s;
        
    elseif (C(2,2) > C(3,3))
        s = sqrt(1 + C(2,2) - C(1,1) - C(3,3)) * 2;
        qw = (C(1,3) - C(3,1)) / s;
        qx = (C(1,2) + C(2,1)) / s;
        qy = 0.25 * s;
        qz = (C(2,3) + C(3,2)) / s;
        
    else 
        s = sqrt(1 + C(3,3) - C(1,1) - C(2,2)) * 2;
        qw = (C(2,1) - C(1,2)) / s;
        qx = (C(1,3) + C(3,1)) / s;
        qy = (C(2,3) + C(3,2)) / s;
        qz = 0.25 * s;
    end
    q = [qx, qy, qz, qw];
end

function [C, r] = pointcloud_alignment(src_points, dest_points)
%  point cloud alignment
%  Calculates SE(3) transform from src_points to dest_points
%  Input:
%      src_points:   point cloud from previoud frame
%      dest_points:  point cloud in current frame
%      [3 x N] arrays, rows are x, y, z
%      [p1_x, ..., pN_x;
%       p1_y, ..., pN_y;
%       p1_z, ..., pN_z]
%  Return:
%      C: rotational matrix -  [3 x 3]
%      r: translation vector - [3 x 1]

    src_points = src_points';
    dest_points = dest_points';
    % check shape consistency  
    [row, col] = size(src_points);
    if ~(row == size(dest_points,1) & col == size(dest_points,2))
        error('Number of points in source and destination point clouds not equal!');
    end
    if size(src_points,2) < 3
        error('Insufficient source points.');
    end
    if size(dest_points,2) < 3
        error('Insufficient destination points.');
    end
    N = size(src_points,2);
    
    % compute the centroid point
    src_points_centroid = sum(src_points, 2) / N;
    dest_points_centroid = sum(dest_points, 2) / N;
    src_points_rem = src_points - reshape(src_points_centroid,[],1);
    dest_points_rem = dest_points - reshape(dest_points_centroid,[],1);
    % compute the Weight matrix
    W = (dest_points_rem * src_points_rem') / N;
    
    % point cloud alignment   
    % the SVD in matlab is different from the one in numpy.linalg.svd
    [u, s, v] = svd(W);
    
    D = [1, 0, 0;
         0, 1, 0;
         0, 0, det(u)*det(v)];
     
    % C is the rotation matrix rotate point cloud to Total station frame (convert src_points to dest_points)
    C = u * D * v';
    r_a_ab = - C' * dest_points_centroid + src_points_centroid;
    
    r_b_ba = - C * reshape(r_a_ab,[],1) ;
    
    % return C and r_b_ba
    r = r_b_ba;
    
end

