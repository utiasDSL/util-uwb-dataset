%    Visualize the UWB TDOA2 and other sensor measurements.
%

clear; close all
clc;

% get the full path of the current script
filepath = fileparts(mfilename('fullpath'));

% combine the path
csv = fullfile(filepath, '../../dataset/flight-dataset/csv-data/const1/const1-trial1-tdoa2.csv');
txt = fullfile(filepath, '../../dataset/flight-dataset/survey-results/anchor_const1_survey.txt');

% load the anchor positions
an_pose = readtable(txt,'ReadRowNames',true);
anchor_pos = [an_pose.Var1(1:8), an_pose.Var2(1:8), an_pose.Var3(1:8)];

data = readtable(csv);

%% import data and remove the NAN in each sensor topic
% toda: [timestamp, idA, idB, tdoa measurement]
tdoa = [data.t_tdoa(~isnan(data.t_tdoa)),  data.idA(~isnan(data.idA)), ...
        data.idB(~isnan(data.idB)),        data.tdoa_meas(~isnan(data.tdoa_meas))];
% acceleration: [timestamp, acc_x, acc_y, acc_z]
acc  = [data.t_acc(~isnan(data.t_acc)),    data.acc_x(~isnan(data.acc_x)),...
        data.acc_y(~isnan(data.acc_y)),    data.acc_z(~isnan(data.acc_z))];
% gyroscope: [timestamp, gyro_x, gyro_y, gyro_z]
gyro = [data.t_gyro(~isnan(data.t_gyro)),  data.gyro_x(~isnan(data.gyro_x)),...
        data.gyro_y(~isnan(data.gyro_y)),  data.gyro_z(~isnan(data.gyro_z))];
% laser-ranging: [timestamp, tof] 
tof  = [data.t_tof(~isnan(data.t_tof)),    data.tof(~isnan(data.tof))];
% optical flow: [timestamp, motion_deltaX, motion_deltaY]
flow = [data.t_flow(~isnan(data.t_flow)),  data.deltaX(~isnan(data.deltaX)),...
        data.deltaY(~isnan(data.deltaY))];
% barometer: [timestamp, barometer]
baro = [data.t_baro(~isnan(data.t_baro)),  data.baro(~isnan(data.baro))];
% ground truth pose: [timestamp, x, y, z, qx, qy, qz, qw]
pose = [data.t_pose(~isnan(data.t_pose)),   data.pose_x(~isnan(data.pose_x)),   data.pose_y(~isnan(data.pose_y)),   data.pose_z(~isnan(data.pose_z)),...
        data.pose_qx(~isnan(data.pose_qx)), data.pose_qy(~isnan(data.pose_qy)), data.pose_qz(~isnan(data.pose_qz)), data.pose_qw(~isnan(data.pose_qw))];
   
%%
% translation vector from the quadcopter to UWB tag
t_uv = [-0.01245; 0.00127; 0.0908];
% translation vector from the quadcopter to laser-ranging sensor 
t_lv = [0.0; 0.0; -0.0012];

% convert the gt position to UWB antenna center
for idx = 1:size(pose,1)
    q_cf = pose(idx,5:8);
    R_iv = quat_to_rot(q_cf);
    
    gt_p = reshape(pose(idx,2:4),[],1);  % gt position of the vehicle
    uwb_p(idx,:) = R_iv * t_uv + gt_p;
end

% extract each tdoa measurement
tdoa_70 = find(tdoa(:,2)==7 & tdoa(:,3)==0);
tdoa_meas_70 = tdoa(tdoa_70, :);

tdoa_01 = find(tdoa(:,2)==0 & tdoa(:,3)==1);
tdoa_meas_01 = tdoa(tdoa_01, :);

tdoa_12 = find(tdoa(:,2)==1 & tdoa(:,3)==2);
tdoa_meas_12 = tdoa(tdoa_12, :);

tdoa_23 = find(tdoa(:,2)==2 & tdoa(:,3)==3);
tdoa_meas_23 = tdoa(tdoa_23, :);

tdoa_34 = find(tdoa(:,2)==3 & tdoa(:,3)==4);
tdoa_meas_34 = tdoa(tdoa_34, :);

tdoa_45 = find(tdoa(:,2)==4 & tdoa(:,3)==5);
tdoa_meas_45 = tdoa(tdoa_45, :);

tdoa_56 = find(tdoa(:,2)==5 & tdoa(:,3)==6);
tdoa_meas_56 = tdoa(tdoa_56, :);

tdoa_67 = find(tdoa(:,2)==6 & tdoa(:,3)==7);
tdoa_meas_67 = tdoa(tdoa_67, :);

% compute the ground truth for tdoa_ij
for i = 1 : 8
    an_i = reshape(anchor_pos(i,:),1,[]);
    d(:,i) = vecnorm(an_i - uwb_p, 2, 2); 
end

% measurement model
d_70 = d(:,1) - d(:,8);
d_01 = d(:,2) - d(:,1);
d_12 = d(:,3) - d(:,2);
d_23 = d(:,4) - d(:,3);
d_34 = d(:,5) - d(:,4);
d_45 = d(:,6) - d(:,5);
d_56 = d(:,7) - d(:,6);
d_67 = d(:,8) - d(:,7);

% visualization
% UWB1
fig1 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
subplot(2,2,1)
scatter(tdoa_meas_70(:,1), tdoa_meas_70(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_70, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An7, An0)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

subplot(2,2,2)
scatter(tdoa_meas_01(:,1), tdoa_meas_01(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_01, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An0, An1)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

subplot(2,2,3)
scatter(tdoa_meas_12(:,1), tdoa_meas_12(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_12, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An1, An2)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

subplot(2,2,4)
scatter(tdoa_meas_23(:,1), tdoa_meas_23(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_23, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An2, An3)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

% UWB 2
fig2 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
subplot(2,2,1)
scatter(tdoa_meas_34(:,1), tdoa_meas_34(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_34, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An3, An4)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

subplot(2,2,2)
scatter(tdoa_meas_45(:,1), tdoa_meas_45(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_45, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An4, An5)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

subplot(2,2,3)
scatter(tdoa_meas_56(:,1), tdoa_meas_56(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_56, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An5, An6)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

subplot(2,2,4)
scatter(tdoa_meas_67(:,1), tdoa_meas_67(:,4), 3, 'filled')
hold on
plot(pose(:,1),d_67, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements, (An6, An7)','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
legend('TDOA measurements', 'ground truth')

% laser-ranging Tof
fig3 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
scatter(tof(:,1),tof(:,2), 3, 'filled')
title('Laser-ranging measurements','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('ToF measurement [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');

% optical flow
fig4 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
subplot(2,1,1)
scatter(flow(:,1),flow(:,2), 3, 'filled')
title('Optical flow measurements','Interpreter','latex','Fontsize',16)
ylabel('motion delta x','Interpreter','latex','Fontsize',16)
subplot(2,1,2)
scatter(flow(:,1),flow(:,3), 3, 'filled')
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('motion delta x','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');

% barometer
fig5 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
scatter(baro(:,1),baro(:,2), 3, 'filled')
title('Barometer measurements','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('asl','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');

% trajectory
fig6 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
plot3(pose(:,2),pose(:,3),pose(:,4),'LineWidth', 2)
hold on
scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3), 50, 'filled')
title('Trajectory of the quadcopter','Interpreter','latex','Fontsize',16)
xlabel('X [m]','Interpreter','latex','Fontsize',16)
ylabel('Y [m]','Interpreter','latex','Fontsize',16)
zlabel('Z [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
grid on
legend('trajectory', 'anchor')
axis equal

% plot separate x,y,z
fig7 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
subplot(3,1,1)
plot(pose(:,1),pose(:,2),'LineWidth', 2)
ylabel('X [m]','Interpreter','latex','Fontsize',16)
title('Ground truth of the quadcopter trajectory','Interpreter','latex','Fontsize',16)
subplot(3,1,2)
plot(pose(:,1),pose(:,3),'LineWidth', 2)
ylabel('Y [m]','Interpreter','latex','Fontsize',16)
subplot(3,1,3)
plot(pose(:,1),pose(:,4),'LineWidth', 2)
ylabel('Z [m]','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');



function R = quat_to_rot(q)
    qw = q(4); q1 = q(1); q2 = q(2); q3=q(3);
    % first row of the rotation matrix
    r00 = 2 * (qw * qw + q1 * q1) - 1;
    r01 = 2 * (q1 * q2 - qw * q3);
    r02 = 2 * (q1 * q3 + qw * q2);
    % second row of the rotation matrix
    r10 = 2 * (q1 * q2 + qw * q3);
    r11 = 2 * (qw * qw + q2 * q2) - 1;
    r12 = 2 * (q2 * q3 - qw * q1);
    % third row of the rotation matrix
    r20 = 2 * (q1 * q3 - qw * q2);
    r21 = 2 * (q2 * q3 + qw * q1);
    r22 = 2 * (qw * qw + q3 * q3) - 1;
    
    R = [r00, r01, r02;
         r10, r11, r12;
         r20, r21, r22];
end








  
 
