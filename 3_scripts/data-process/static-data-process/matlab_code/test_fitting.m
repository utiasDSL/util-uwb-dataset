% test for numerical convolution
clear; clc;
load('uwb_error.mat');

% better for log-norm fitting
nlos_err = - anTag_nlos_err;
% remove super large outliers
for i =1:length(anAn_nlos_err)
    if abs(anAn_nlos_err(i))<10
      nlos_err_an(i) = anAn_nlos_err(i);
    end
end
%
los_norm = Normal(0.0,0.05);
norm = Normal(-0.0091277,0.063622);
lognorm = Lognormal(-3.002,1.3778);

% sample between -1.5, 1.5
x_t = linspace(-1.5,1.5,5000);

%% PDF anTag nlos 1
dist_anTag_nlos1 = Convolution(norm, lognorm);

%% PDF anTag nlos 2
% dist_anTag_nlos2(x) = dist_anTag_nlos(-x)

%% PDF nlos-nlos-los
% numerical integration over -1.5 to 1.5
f = @negative_lognorm_pdf;    % negative log-norm
g = @(y2) dist_anTag_nlos1.PDF(y2); % pdf of nlos an1-tag

% f = @(y1) lognorm.PDF(y1);
% g = @(y2) norm.PDF(y2); 

fun = @(x,z) f(z-x).*g(x);
for i = 1:length(x_t)
    dist_nlos_nlos_los(i)=integral(@(x) fun(x,x_t(i)), -Inf, Inf);
end


%% PDF los-los-nlos
dist_los_los_nlos = Normal(0.0,0.068);

%% PDF nlos-los-nlos
dist_nlos_los_nlos = Convolution(Normal(0.0,0.068), Lognormal(-3.002,1.3778));

%% PDF los-nlos-nlos
% dist_los_nlos_nlos(x) = dist_nlos_los_nlos(-x)

%% PDF nlos-nlos-nlos
dist_nlos_nlos_nlos = Convolution(Normal(0.0,0.15), Normal(0.0,0.068));

%% visualization 
NLOS_COLOR = [1.0 0.0 0.0];
LOS_COLOR  = [0, 0.4470, 0.7410];
HIST_COLOR = [0.3010 0.7450 0.9330];
close all

gcf = figure(1);
histogram(los_err, 30,'Normalization','pdf', 'FaceColor', HIST_COLOR, 'FaceAlpha',0.6)
hold on
plot(x_t, los_norm.PDF(x_t), '--','Color', LOS_COLOR, 'LineWidth',5);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('los-los-los histogram','los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'los-los-los.png')


gcf = figure(2);
histogram(nlos_err, 'Normalization','pdf', 'FaceColor', HIST_COLOR)
hold on
plot(x_t, dist_anTag_nlos1.PDF(x_t), '--', 'Color', NLOS_COLOR, 'LineWidth',5);
hold on
plot(x_t, los_norm.PDF(x_t), '--', 'Color', LOS_COLOR, 'LineWidth',2);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('los-nlos-los histogram', 'los-nlos-los pdf', 'los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'los-nlos-los.png')

gcf = figure(3);
histogram(-nlos_err, 'Normalization','pdf', 'FaceColor', HIST_COLOR)
hold on
plot(-x_t, dist_anTag_nlos1.PDF(x_t), '--','Color', NLOS_COLOR, 'LineWidth',5);
hold on
plot(x_t, los_norm.PDF(x_t),'--', 'Color', LOS_COLOR, 'LineWidth',2);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('nlos-los-los histogram', 'nlos-los-los pdf','los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'nlos-los-los.png')

gcf = figure(4);
plot(x_t, dist_nlos_nlos_los,'--', 'Color', NLOS_COLOR, 'LineWidth',5);
hold on
plot(x_t, los_norm.PDF(x_t),'--','Color', LOS_COLOR, 'LineWidth',2);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('nlos-nlos-los pdf','los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'nlos-nlos-los.png')

gcf = figure(5);
histogram(nlos_err_an, 100,'Normalization','pdf', 'FaceColor', HIST_COLOR)
hold on
plot(x_t, dist_los_los_nlos.PDF(x_t), '--', 'Color', NLOS_COLOR, 'LineWidth',5);
hold on
plot(x_t, los_norm.PDF(x_t), '--', 'Color', LOS_COLOR, 'LineWidth',2);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('los-los-nlos histogram', 'los-los-nlos pdf', 'los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'los-los-nlos.png')

gcf = figure(6);
plot(x_t, dist_nlos_los_nlos.PDF(x_t), '--','Color', NLOS_COLOR, 'LineWidth',5);
hold on
plot(x_t, los_norm.PDF(x_t), '--', 'Color', LOS_COLOR, 'LineWidth',2);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('nlos-los-nlos pdf', 'los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'nlos-los-nlos.png')

gcf = figure(7);
plot(-x_t, dist_nlos_los_nlos.PDF(x_t), '--', 'Color', NLOS_COLOR, 'LineWidth',5);
hold on
plot(x_t, los_norm.PDF(x_t), '--', 'Color', LOS_COLOR, 'LineWidth',2);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('los-nlos-nlos pdf', 'los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'los-nlos-nlos.png')

gcf = figure(8);
plot(x_t, dist_nlos_nlos_nlos.PDF(x_t), '--', 'Color', NLOS_COLOR, 'LineWidth',5);
hold on
plot(x_t, los_norm.PDF(x_t), '--', 'Color', LOS_COLOR, 'LineWidth',2);
hold off
xlim([-1.0 1.0])
ylim([ 0.0 9.0])
legend('nlos-nlos-nlos pdf', 'los-los-los pdf')
xlabel('tdoa error')
ylabel('probability density function')
set(gca,'FontSize',32)
set(gcf, 'Position', [10 10 900 600]);
saveas(gcf,'nlos-nlos-nlos.png')

