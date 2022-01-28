%    Visualize static los signal testing data. Read the data from csv
%    
%    Created On : Jan 1, 2022
%       Author  : Wenda Zhao, Abhishek Goudar, Xinyuan Qiao
%       Email   : wenda.zhao@robotics.utias.utoronto.ca, 
%                 abhishek.goudar@robotics.utias.utoronto.ca,
%                 samxinyuan.qiao@mail.utoronto.ca
%    Affliation : Dynamic Systems Lab, Vector Institute, UofT Robotics Institute

clear; close all
clc;

% get the full path of the current script
filepath = fileparts(mfilename('fullpath'));

% combine the path
csv = fullfile(filepath, '../../dataset/static-dataset/los/angleTest/angleT2/angleT2_data.csv');
txt = fullfile(filepath, '../../dataset/static-dataset/los/angleTest/angleT2/angleT2_pose.txt');

% load the anchor positions
position = readtable(txt,'ReadRowNames',true);
an1_p = [position.Var1(1),  position.Var2(1),  position.Var3(1)];
an2_p = [position.Var1(2),  position.Var2(2),  position.Var3(2)];
tag_p = [position.Var1(3),  position.Var2(3),  position.Var3(3)];

data = readtable(csv);

%% import data and remove the NAN in each sensor topic
tdoa12 = data.tdoa12(~isnan(data.tdoa12));     tdoa21 = data.tdoa21(~isnan(data.tdoa21));

snr_an1 = data.snr_an1(~isnan(data.snr_an1));  power_dif_an1 = data.power_dif_an1(~isnan(data.power_dif_an1));

snr_an2 = data.snr_an2(~isnan(data.snr_an2));  power_dif_an2 = data.power_dif_an2(~isnan(data.power_dif_an2));

an1_rx_snr = data.an1_rx_snr(~isnan(data.an1_rx_snr)); an1_rx_powerdif = data.an1_rx_powerdif(~isnan(data.an1_rx_powerdif)); an1_tof = data.an1_tof(~isnan(data.an1_tof));

an2_rx_snr = data.an2_rx_snr(~isnan(data.an2_rx_snr)); an2_rx_powerdif = data.an2_rx_powerdif(~isnan(data.an2_rx_powerdif)); an2_tof = data.an2_tof(~isnan(data.an2_tof));

% compute the ground truth for tdoa12
gt_d12 = norm(an2_p - tag_p) - norm(an1_p - tag_p);

err_12 = tdoa12 - gt_d12;

%% visualization
HIST_COLOR = [0.3010 0.7450 0.9330];
% tdoa error
figure(1);
histogram(err_12, 30, 'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
legend('TODA error histogram','Interpreter','latex')
xlabel('TDOA error [m]','Interpreter','latex')
ylabel('Probability density function','Interpreter','latex')
xlim([-1.5 1.5])
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['TDOA measurement error, mean: ', num2str(mean(err_12)), ' [m]']);

% power value at tag side
figure(2);
subplot(2,2,1)
histogram(snr_an1, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('SNR of An1','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['SNR of anchor 1, mean: ', num2str(mean(snr_an1))]);

subplot(2,2,2)
histogram(power_dif_an1, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('Power difference of An1','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['Power difference of anchor 1, mean: ', num2str(mean(power_dif_an1)), ' [dB]']);

subplot(2,2,3)
histogram(snr_an2, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('SNR of An2','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['SNR of anchor 2, mean: ', num2str(mean(snr_an2))]);

subplot(2,2,4)
histogram(power_dif_an2, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('Power difference of An2','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['Power difference of anchor 2, mean: ', num2str(mean(power_dif_an2)), ' [dB]']);

% power value at anchor side
% an1_rx_snr, an1_rx_powerdif, an1_tof
figure(3);
subplot(2,3,1)
histogram(an1_rx_snr, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('SNR received by an1','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['SNR received by anchor 1, mean: ', num2str(mean(an1_rx_snr))]);

subplot(2,3,2)
histogram(an1_rx_powerdif, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('Power difference received by an1','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['Power difference received by anchor 1, mean: ', num2str(mean(an1_rx_powerdif)), ' [dB]']);

subplot(2,3,3)
histogram(an1_tof, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('Tof received by an1','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['Tof received by anchor 1, mean: ', num2str(mean(an1_tof)), ' [m]']);

subplot(2,3,4)
histogram(an2_rx_snr, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('SNR received by an2','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['SNR received by anchor 2, mean: ', num2str(mean(an2_rx_snr))]);

subplot(2,3,5)
histogram(an2_rx_powerdif, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('Power difference received by an2','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['Power difference received by anchor 2, mean: ', num2str(mean(an2_rx_powerdif)), ' [dB]']);

subplot(2,3,6)
histogram(an2_tof, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
xlabel('Tof received by an2','Interpreter','latex')
ylabel('PDF','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16);
disp(['Tof received by anchor 2, mean: ', num2str(mean(an2_tof)), ' [m]']);

% positions
fig4 = figure('Renderer', 'painters', 'Position', [10 10 800 600]);
scatter3(an1_p(1), an1_p(2), an1_p(3), 50, 'filled')
hold on
scatter3(an2_p(1), an2_p(2), an2_p(3), 50, 'filled')
hold on
scatter3(tag_p(1),tag_p(2), tag_p(3), 50, 'filled')
legend('An1 position','An2 position','Tag position')
xlabel('X [m]','Interpreter','latex','Fontsize',16)
ylabel('Y [m]','Interpreter','latex','Fontsize',16)
zlabel('Z [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
grid on
xlim([-4.5, 4.5])  
ylim([-4.5, 4.5])  
zlim([0.0, 3.0]) 


