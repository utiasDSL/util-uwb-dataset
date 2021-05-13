% using multi-start initial guess for maximum likelihood pdf parameter
% estimation

clear;clc;
load('uwb_error.mat');

% better for log-norm fitting
nlos_err = - nlos_err;

% sample 1000 values from raw nlos error data
samp_err = datasample(nlos_err, 1000);

samp_err = reshape(samp_err,[],1); % convert to column vector



estDist = Convolution(Normal(0.0,0.05),Lognormal(-3.525,2.0));

EstFn = @estDist.EstML;
fnParms = {samp_err};

starting_points = [0.0, 0.05, -3.525, 2.0;
                   0.0, 0.05, -2.525, 2.0;
                   0.0, 0.05, -4.525, 0.5;
                   0.0, 0.05, -3.525, 1.5;
                   0.0, 0.05, -5.525, 1.0];

[s,EndingVals,fval,exitflag,output,allstarts] = EstManyStarts(estDist,EstFn,fnParms,starting_points)


% optimization results
% 
% s =
% 
%     'Convolution(Normal(-0.0091277,0.063622),Lognormal(-3.002,1.3778))'
% 
% 
% EndingVals =
% 
%    -0.0091    0.0636   -3.0020    1.3778
% 
% 
% fval =
% 
%  -588.0148
% 
% 
% exitflag =
% 
%      1
% 
% 
% output = 
% 
%   struct with fields:
% 
%     iterations: 788
%      funcCount: 1482
%      algorithm: 'Nelder-Mead simplex direct search'
%        message: 'Optimization terminated:↵ the current x satisfies the termination criteria using OPTIONS.TolX of 1.000000e-14 ↵ and F(X) satisfies the convergence criteria using OPTIONS.TolFun of 1.000000e-14 ↵'
% 
% 
% allstarts = 
% 
%   struct with fields:
% 
%              s: {5×1 cell}
%     EndingVals: {5×1 cell}
%           fval: [5×1 double]
%       exitflag: [5×1 double]
%         output: {5×1 cell}