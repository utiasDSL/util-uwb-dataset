% Visualize the manual movement trajectories and obstacles
% dependent function: drawCuboid from package geom3d: 
% https://www.mathworks.com/matlabcentral/fileexchange/24484-geom3d

clear; close all
clc;

% get the full path of the current script
filepath = fileparts(mfilename('fullpath'));

% combine the path
csv = fullfile(filepath, '../../dataset/flight-dataset/csv-data/const3/const3-tdoa2-obs-log1.csv');
txt = fullfile(filepath, '../../dataset/flight-dataset/survey-results/anchor_const3_survey.txt');

% load the anchor positions
an_pose = readtable(txt);
anchor_pos = [an_pose.Var2(1:8), an_pose.Var3(1:8), an_pose.Var4(1:8)];

data = readtable(csv);

% ground truth pose: [timestamp, x, y, z, qx, qy, qz, qw]
pose = [data.t_pose(~isnan(data.t_pose)),   data.pose_x(~isnan(data.pose_x)),   data.pose_y(~isnan(data.pose_y)),   data.pose_z(~isnan(data.pose_z)),...
        data.pose_qx(~isnan(data.pose_qx)), data.pose_qy(~isnan(data.pose_qy)), data.pose_qz(~isnan(data.pose_qz)), data.pose_qw(~isnan(data.pose_qw))];
    
% marker positions of the obstacles
woodenshelf = [1850.02,   1021.90,  2032.84;
               2411.24,    413.79,  2032.20;
               1724.89,    793.20,  2035.44;
               2533.86,    646.41,  2031.22] / 1000.0;
           
plasticbox = [2688.82,   -2495.71,  853.62;
              2484.05,   -2977.32,  861.63;
              2365.28,   -2356.26,  854.47;
              2151.78,   -2837.41,  861.19
              ] / 1000.0;
    
metalcabinet = [-1286.08,  -1445.97,  1122.17;
         -791.69,   -1445.40,  1127.81;
         -804.78,    -530.59,  1124.39;
         -1295.78,   -535.66,  1117.10
        ] / 1000.0;          
          
% cuboid
wood.center = [2.1300, 0.7188, 1.0164];      wood.lwd = [0.2591 , 0.7761, 2.0324];
plastic.center = [2.4225, -2.6667, 0.427];   plastic.lwd = [0.5233, 0.3523, 0.8577];
metal.center = [-1.045, -0.9894, 0.5614];    metal.lwd = [0.4944, 0.9104, 1.1228];
%     
% visualizaton
% trajectory
fig = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
plot3(pose(:,2),pose(:,3),pose(:,4),'LineWidth', 2)
hold on
scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3), 50, 'filled')
hold on
scatter3(woodenshelf(:,1),woodenshelf(:,2),woodenshelf(:,3), 30, 'filled')
hold on
drawCuboid([wood.center   wood.lwd   62 0 0], 'FaceColor', [0.8500, 0.3250, 0.0980],'FaceAlpha',.3);
hold on
p = scatter3(plasticbox(:,1), plasticbox(:,2), plasticbox(:,3), 30, 'filled');
p.MarkerFaceColor = [0.5, 1.0, 0.83];
hold on
drawCuboid([plastic.center  plastic.lwd   66 0 0], 'FaceColor', [0.5, 1.0, 0.83], 'FaceAlpha',.3);
hold on
m = scatter3(metalcabinet(:,1), metalcabinet(:,2), metalcabinet(:,3), 30, 'filled');
m.MarkerFaceColor = [0.0, 0.0, 0.83];
hold on
drawCuboid([metal.center  metal.lwd   0 0 0], 'FaceColor', 'b', 'FaceAlpha',.3);
title('Trajectory of the quadcopter','Interpreter','latex','Fontsize',16)
xlabel('X [m]','Interpreter','latex','Fontsize',16)
ylabel('Y [m]','Interpreter','latex','Fontsize',16)
zlabel('Z [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
grid on
legend('trajectory','anchor','', 'wooden shelf', '', 'plastic box', '', 'metal cabinet')
          
          
          
          
          
          
          