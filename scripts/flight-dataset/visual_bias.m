%    Visualize the TDOA measurement biases 
%

clear; close all
clc;

% get the full path of the current script
filepath = fileparts(mfilename('fullpath'));

% combine the path
csv = fullfile(filepath, '../../dataset/flight-dataset/csv-data/const1/const1-trial1-tdoa2.csv');
txt = fullfile(filepath, '../../dataset/flight-dataset/survey-results/anchor_const1_survey.txt');

% load the anchor positions
an_pose = readtable(txt);
anchor_pos = [an_pose.Var2(1:8), an_pose.Var3(1:8), an_pose.Var4(1:8)];

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

% select the anchor pair for visualization
% possible anchor pair IDs 
% TDOA2: 7-0, 0-1, 1-2, 2-3, 3-4, 4-5, 5-6, 6-7
% TDOA3: i,j \in {0,1,2,3,4,5,6,7} 
an_i = 0;     an_j = 1;

tdoa_ij = find(tdoa(:,2)==an_i & tdoa(:,3)==an_j);
tdoa_meas_ij = tdoa(tdoa_ij, :);

% compute the ground truth for tdoa_ij
% matlab starts from 1
an_pos_i = reshape(anchor_pos(an_i+1,:),1,[]);
an_pos_j = reshape(anchor_pos(an_j+1,:),1,[]);


% To compute the bias, we need to interpolate the Vicon measurements
x_interp = interp1(pose(:,1), uwb_p(:,1), tdoa_meas_ij(:,1));
y_interp = interp1(pose(:,1), uwb_p(:,2), tdoa_meas_ij(:,1));
z_interp = interp1(pose(:,1), uwb_p(:,3), tdoa_meas_ij(:,1));


pos_interp = [reshape(x_interp,[],1), reshape(y_interp,[],1), reshape(z_interp,[],1)];

d_i = vecnorm(an_pos_i - pos_interp, 2, 2);
d_j = vecnorm(an_pos_j - pos_interp, 2, 2);

% measurement model
d_ij = d_j - d_i;

% bais = tdoa -gt
bias_ij = tdoa_meas_ij(:,4) - d_ij;

% visualization
% UWB
fig1 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
scatter(tdoa_meas_ij(:,1), tdoa_meas_ij(:,4), 12, 'filled')
hold on
plot(tdoa_meas_ij(:,1), d_ij, 'Color', [1,0,0], 'LineWidth', 1.5)
title('UWB TDOA measurements','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('TDOA measurements [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex','Fontsize',16);
grid on
legend('TDOA measurements', 'ground truth')

% UWB bias
fig2 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
scatter(tdoa_meas_ij(:,1), bias_ij, 12, 'filled')
title('TDOA measurement biases','Interpreter','latex','Fontsize',16)
xlabel('Time [s]','Interpreter','latex','Fontsize',16)
ylabel('Measurements bias [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex','Fontsize',16);
grid on
disp(['Visualize TDOA measurement biases, An: (',num2str(an_i),',',num2str(an_j),')']);

legend('TDOA measurement biases')

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
