% test for numerical convolution
clear; clc;
load('uwb_error.mat');

% better for log-norm fitting
nlos_err = - anTag_nlos_err;

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

% visualization 
figure(1)
plot(x_t, dist_anTag_nlos1.PDF(x_t), 'Color', [0.4660 0.6740 0.1880]);
hold on
plot(x_t, los_norm.PDF(x_t), 'Color', [0, 0.4470, 0.7410]);
hold on
histogram(nlos_err, 'Normalization','pdf', 'FaceColor', [0.3010 0.7450 0.9330])
hold on
histogram(los_err, 'Normalization','pdf', 'FaceColor', [0, 0.4470, 0.7410])
hold off
xlim([-1.0 1.0])

figure(2)
plot(-x_t, dist_anTag_nlos1.PDF(x_t), 'Color', [0.4660 0.6740 0.1880]);
hold on
plot(x_t, los_norm.PDF(x_t), 'Color', [0, 0.4470, 0.7410]);
hold on
histogram(-nlos_err, 'Normalization','pdf', 'FaceColor', [0.3010 0.7450 0.9330])
hold on
histogram(los_err, 'Normalization','pdf', 'FaceColor', [0, 0.4470, 0.7410])
hold off
xlim([-1.0 1.0])

figure(3)
plot(x_t, dist_nlos_nlos_los, 'Color', [0.4660 0.6740 0.1880]);
hold on
plot(x_t, los_norm.PDF(x_t), 'Color', [0, 0.4470, 0.7410]);
hold on
histogram(los_err, 'Normalization','pdf', 'FaceColor', [0, 0.4470, 0.7410])
hold off
xlim([-1.0 1.0])

figure(4)
plot(x_t, dist_los_los_nlos.PDF(x_t), 'Color', [0.4660 0.6740 0.1880]);
hold on
plot(x_t, los_norm.PDF(x_t), 'Color', [0, 0.4470, 0.7410]);
hold on
histogram(los_err, 'Normalization','pdf', 'FaceColor', [0, 0.4470, 0.7410])
hold off
xlim([-1.0 1.0])

figure(5)
plot(x_t, dist_nlos_los_nlos.PDF(x_t), 'Color', [0.4660 0.6740 0.1880]);
hold on
plot(x_t, los_norm.PDF(x_t), 'Color', [0, 0.4470, 0.7410]);
hold on
histogram(los_err, 'Normalization','pdf', 'FaceColor', [0, 0.4470, 0.7410])
hold off
xlim([-1.0 1.0])

figure(6)
plot(-x_t, dist_nlos_los_nlos.PDF(x_t), 'Color', [0.4660 0.6740 0.1880]);
hold on
plot(x_t, los_norm.PDF(x_t), 'Color', [0, 0.4470, 0.7410]);
hold on
histogram(los_err, 'Normalization','pdf', 'FaceColor', [0, 0.4470, 0.7410])
hold off
xlim([-1.0 1.0])

figure(7)
plot(x_t, dist_nlos_nlos_nlos.PDF(x_t), 'Color', [0.4660 0.6740 0.1880]);
hold on
plot(x_t, los_norm.PDF(x_t), 'Color', [0, 0.4470, 0.7410]);
hold on
histogram(los_err, 'Normalization','pdf', 'FaceColor', [0, 0.4470, 0.7410])
hold off
xlim([-1.0 1.0])

