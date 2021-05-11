% using matlab for probability computation
clear;clc;
% fix random number, using the default random seed
rng('default')

load('anTag_nlos_error.mat');
.
% better for log-norm fitting
nlos_err = - nlos_err;

% sample 1000 values from raw nlos error data without replacement
samp_err = datasample(nlos_err, 1000,'Replace',false);
samp_err = reshape(samp_err,[],1);

% use CUPID for probability functions
% estimate the parameter through maximum likelihood estimation
% conv_dist1.EstML(samp_err) % slow...
% conv_dist2.EstML(samp_err) % slow...

% estimation results
dist_normal1 = Normal(-0.037032,0.044138);   dist_gamma = RNGamma(0.71347,5.5271);
dist_normal2 = Normal(-0.0091277,0.063622);  dist_lognorm = Lognormal(-3.002,1.3778);

conv_dist1 = Convolution(Normal(-0.037032,0.044138), RNGamma(0.71347,5.5271));
conv_dist2 = Convolution(Normal(-0.0091277,0.063622),Lognormal(-3.002,1.3778));

% test the fitting results
%[SEs, Covars] = conv_dist.MLSE(samp_err);

% Plot comparison of the histogram of the data, and the fit
% built-in plot: est_conv_dist.PlotDens

x_t = linspace(-1.5,1.5,5000);
% conv 1
y_estConv1 = conv_dist1.PDF(x_t);
y_norm1 = dist_normal1.PDF(x_t);
y_gamma = dist_gamma.PDF(x_t);

% conv 2
y_estConv2 = conv_dist2.PDF(x_t);
y_norm2 = dist_normal2.PDF(x_t);
y_lognorm = dist_lognorm .PDF(x_t);
%% visualize PDF
figure(1)
plot(x_t, y_estConv1, 'r', 'LineWidth',5)
hold on
plot(x_t, y_norm1, 'Color', [0, 0.4470, 0.7410], 'LineWidth',5)
hold on
plot(x_t, y_gamma, 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth',5)
hold on
histogram(nlos_err, 'Normalization','pdf', 'FaceColor', [0.3010 0.7450 0.9330])
hold off
legend('Emprical PDF', 'Norm PDF', 'Gamma PDF', 'Norm*Gamma PDF')
xlim([-1.0 1.0])

figure(2)
plot(x_t, y_estConv2, 'r', 'LineWidth',5)
hold on
plot(x_t, y_norm2, 'Color',   [0, 0.4470, 0.7410], 'LineWidth',5)
hold on
plot(x_t, y_lognorm, 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth',5)
hold on
histogram(nlos_err, 'Normalization','pdf', 'FaceColor', [0.3010 0.7450 0.9330])
hold off
legend('Emprical PDF', 'Norm PDF', 'Log-norm PDF', 'Norm*Log-norm PDF')
xlim([-1.0 1.0])

%% Kolmogorov-Smirnov test: observation vector need to be a column vector
% Currently, norm * gamma is better than norm * lognorm...

samp_err = reshape(samp_err,[],1); % convert to column vector
[p1, Dmax_1] = conv_dist1.kstest(samp_err);          % get the same result with [h1,p1] = kstest(samp_err, 'CDF', test_cdf1)
[p2, Dmax_2] = conv_dist2.kstest(samp_err);

% test with the default Kolmogorov-Smirnov test in matlab
% visualize the cdf

figure(3)
cdf1 = cdfplot(samp_err);
set(cdf1, 'LineWidth',2, 'Color', [0, 0.4470, 0.7410])
hold on
scatter(samp_err, conv_dist1.CDF(samp_err))
hold on 
scatter(samp_err, conv_dist2.CDF(samp_err))
xlim([min(samp_err) 1.0])
hold off
legend('Emprical CDF', 'Norm*Gamma CDF', 'Norm*Lognorm CDF')

test_cdf1 = [samp_err, conv_dist1.CDF(samp_err)];
test_cdf2 = [samp_err, conv_dist2.CDF(samp_err)];
% kstest: h: 0, the null hypothesis that the data comes from the assumed distributon
%         h: 1, reject the null hypothesis, the data doesn't come from the assumed distribution

[h1,p1_default] = kstest(samp_err, 'CDF', test_cdf1);   
[h2,p2_default] = kstest(samp_err, 'CDF', test_cdf2);   % get an error




