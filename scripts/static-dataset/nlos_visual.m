%    Visualize static los signal testing data. Read the data from csv
%

clear; close all
clc;

% get the full path of the current script
filepath = fileparts(mfilename('fullpath'));

% combine the path
csv = fullfile(filepath, '../../dataset/static-dataset/nlos/anTag/metal/data1/metal-anTag-data1_data.csv');
txt = fullfile(filepath, '../../dataset/static-dataset/nlos/anTag/metal/data1/metal-anTag-data1_pose.txt');

% load the anchor positions
fid = fopen(txt);

position = [];
while ~feof(fid)
    lineChar = fgetl(fid);
    line = strsplit(lineChar,',');
    if length(line) == 4             % decode the anchor-tag positions and the marker positions
        position = [position;
                   str2double(line{2}), str2double(line{3}), str2double(line{4})];
    end
end

an1_p = position(1,:);
an2_p = position(2,:);
tag_p = position(3,:);
if size(position,1)>3
    obs_p = position(4:7,:);
else
    obs_p=[];
end
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
histogram(err_12, 'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
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
scatter3(tag_p(1), tag_p(2), tag_p(3), 50, 'filled')
hold on
if ~isempty(obs_p)
    % if obs_p is not empty
    scatter3(obs_p(:,1), obs_p(:,2), obs_p(:,3), 50, 'filled')
end
warning('off')
legend('An1 position', 'An1 position', 'Tag position', 'Obstacle')
xlabel('X [m]','Interpreter','latex','Fontsize',16)
ylabel('Y [m]','Interpreter','latex','Fontsize',16)
zlabel('Z [m]','Interpreter','latex','Fontsize',16)
set(gca,'TickLabelInterpreter','latex');
grid on
xlim([-4.5, 4.5])  
ylim([-4.5, 4.5])  
zlim([0.0, 3.0]) 


