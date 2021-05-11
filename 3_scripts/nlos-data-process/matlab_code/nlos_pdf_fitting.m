% using matlab for probability computation
clear;clc;
load('anTag_nlos_error.mat');

% better for log-norm fitting
nlos_err = - nlos_err;

% sample 1000 values from raw nlos error data
samp_err = datasample(nlos_err, 1000);


% try with CUPID for probability functions
dist_normal = Normal(0.0, 0.05);
dist_lognorm = Lognormal(0.0, 1.75);
dist_gamma = RNGamma(1.5, 15);
conv_dist1 = Convolution(dist_normal, dist_lognorm);
conv_dist2 = Convolution(dist_normal, dist_gamma);

% estimate the parameter through maximum likelihood estimation
% conv_dist1.EstML(samp_err) % slow...
% conv_dist2.EstML(samp_err) % slow...


% estimation results
% 'Convolution(Normal(-0.037032,0.044138),RNGamma(0.71347,5.5271))'
est_conv_dist = Convolution(Normal(-0.037032,0.044138), RNGamma(0.71347,5.5271));

%est_conv_lognorm_dist = Convolution(Normal(-0.029165,0.073596),Lognormal(-0.23423,1.652));
est_conv_lognorm_dist = Convolution(Normal(-0.0091277,0.063622),Lognormal(-3.002,1.3778));

%[SEs, Covars] = est_conv_dist.MLSE(samp_err);

% Plot comparison of the histogram of the data, and the fit
% built-in plot: est_conv_dist.PlotDens

x_t = linspace(-1.5,1.5,5000);
% conv 1
y_estConv = est_conv_dist.PDF(x_t);
y_norm = Normal(-0.037032,0.044138).PDF(x_t);
y_gamma = RNGamma(0.71347,5.5271).PDF(x_t);

% conv 2
y_estConv_log = est_conv_lognorm_dist.PDF(x_t);
y_norm2 = Normal(-0.0091277,0.063622).PDF(x_t);
y_lognorm = Lognormal(-3.002,1.3778).PDF(x_t);

% Kolmogorov-Smirnov test: observation vector need to be a column vector
% Currently, norm * gamma is better than norm * lognorm...

samp_err = reshape(samp_err,[],1); % convert to column vector
[p1, Dmax_1] = est_conv_dist.kstest(samp_err);
[p2, Dmax_2] = est_conv_lognorm_dist.kstest(samp_err);

figure()
% plot(x_t, y_estConv, 'Color', [0 0.4470 0.7410], 'LineWidth',2)
plot(x_t, y_estConv, 'r', 'LineWidth',2)
hold on
plot(x_t, y_estConv_log, 'Color', [0 0.4470 0.7410], 'LineWidth',2)

hold on
% plot(x_t, y_norm, 'Color', [0.8500 0.3250 0.0980], 'LineWidth',2)

plot(x_t, y_norm2, 'Color', [0.8500 0.3250 0.0980], 'LineWidth',2)
hold on

% plot(x_t, y_gamma, 'Color', [0.9290 0.6940 0.1250], 'LineWidth',2)
plot(x_t, y_lognorm, 'Color', [0.9290 0.6940 0.1250], 'LineWidth',2)

hold on
histogram(nlos_err, 'Normalization','pdf', 'FaceColor', [0.3010 0.7450 0.9330])
hold on
xlim([-1.0 1.0])

