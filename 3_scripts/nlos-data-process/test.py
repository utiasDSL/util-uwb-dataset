''' test code '''
import os, sys
import numpy as np
from numpy import linalg
from matplotlib import pyplot as plt
import rosbag
from scipy import stats
from scipy.stats import norm, gamma
from glob import glob     # module used for finding pathnames matching a specific pattern
from scipy.io import savemat
from scipy import signal

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# for 4K screen distplay
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 300


# current path of the script
curr = os.path.dirname(sys.argv[0])

def fast_scandir(dirname):
    # get all the subfolders in "dirname"
    subfolders= [f.path for f in os.scandir(dirname) if f.is_dir()]
    for dirname in list(subfolders):
        subfolders.extend(fast_scandir(dirname))
    return subfolders

# ------------------------- load numpy data ----------------------------- #
# load the data for nlos between anchor and tag
PATH_anTag = curr+'/nlos_error/an_tag/'
EXT = "*.npy"
subdir = fast_scandir(PATH_anTag)
all_files = []
for path in subdir:
    # print(os.path.join(path,EXT))
    files = [file for file in glob(os.path.join(path,EXT))]
    all_files.extend(files)
    
anTag_nlos_err = []
for filename in all_files:
    err = np.load(filename)
    anTag_nlos_err.append(err)

# flatten the list to numpy array
anTag_nlos_err = np.concatenate(anTag_nlos_err).ravel()

# save for matlab 
# m_data = {'nlos_err': anTag_nlos_err}
# savemat('anTag_nlos_error.mat', m_data)
# -------------------- testing with different models --------------------- #
anTag_nlos_err = - anTag_nlos_err    # easy for log-norm fitting

# reject large outliers. How to select the threshold?
outlier = []; THRESHOLD = 5.0
for idx in range(len(anTag_nlos_err)):
    if np.abs(anTag_nlos_err[idx]) > THRESHOLD:
        outlier.append(idx)

anTag_nlos_err = np.delete(anTag_nlos_err, outlier)

# log-norm
samp = anTag_nlos_err
lognorm_param = stats.lognorm.fit(samp)
std = lognorm_param[0]; loc=lognorm_param[1]; scale=lognorm_param[2]
###
# Y(samp) ~ lognormal(mu, sigma), X ~ norm(mu, sigma), exp(X) = Y
# X = np.log(samp) => np.std(X) = std, np.mean(X) = scale
###
print("The param for log-normal \nstd: {0}, loc: {1}, scale: {2}".format(std, loc, scale)) 
x=np.linspace(-1.0, 1.0, 1000)
# x=np.linspace(min(samp), max(samp), 1000000)
pdf_fitted = stats.lognorm.pdf(x, std, loc=loc, scale=scale) # fitted distribution

# the fitted function
###
# lognorm.pdf(x, std, loc, scale) is identically equivalent to 
# lognorm.pdf(y, std) / scale with y = (x - loc) / scale
###
sigma = std;  loc = loc; scale = scale;  PI=np.pi
lognorm_scratch = np.zeros_like(x)
for i in range(len(x)):
    y = (x[i] - loc)/scale
    a = (1.0/(sigma * y * np.sqrt(2*PI)))
    b = -(np.log(y))**2
    c = 2*sigma**2
    lognorm_scratch[i] = (a*np.exp(b/c))/scale

# Perform the Kolmogorov-Smirnov test for goodness of fit
# first value is the test statistics, and second value is the p-value. 
# if the p-value is less than 0.95 (for a level of significance of 5%), 
# this means that you cannot reject the Null-Hypothese that the two sample distributions are identical.
l_norm_s, l_norm_p = stats.kstest(samp, 'lognorm', args=lognorm_param)
print("The probability of being a lognorm distribution is {0}%".format(l_norm_p*100))

# ------ Gaussian + Gamma distribution ------ #
# This model is reasonable. But it's difficult to fit the param.
x_t = np.linspace(-1.0, 1.0, 1000)
a_t = np.zeros_like(x_t)
b_t = np.zeros_like(x_t)
y_t = np.zeros_like(x_t)

sigma_t = 0.05;     mu_t = 0.0
lambda_t = 3.5;     k_t = 2 
for i in range(len(x)):
    a_t[i] = 1.0/(sigma_t*np.sqrt(2*np.pi)) * np.exp(-(x[i]-mu_t)**2 / (2*sigma_t**2))
    
    gamma_dist = gamma(2.75, -0.05, 0.1)
    b_t[i] = gamma_dist.pdf(x[i])
    y_t[i] = a_t[i]/2.0 + b_t[i]/2.0

a_dist = norm(0.0, 0.05)
b_dist = gamma(2.75, -0.05, 0.1)

delta = 1e-4
big_grid = np.arange(-1.0,1.0,delta)
conv_pdf = signal.fftconvolve(a_dist.pdf(big_grid), b_dist.pdf(big_grid), 'same') * 3e-4

fig1 = plt.figure(facecolor="white")
bx = plt.subplot(111)
plt.plot(big_grid,a_dist.pdf(big_grid), label='Gaussian')
plt.plot(big_grid,b_dist.pdf(big_grid), label='Gamma')
plt.plot(big_grid,conv_pdf, label='Sum')
bx.legend()
plt.show() 
# ------------------------------- visualization ------------------------------------------ #
fig = plt.figure(facecolor="white")
mu=0;  sigma=0
ax = plt.subplot(111)
(mu, sigma) = stats.norm.fit(anTag_nlos_err)
print("mean0: ", mu, "std0: ", sigma)
print("\n")
# plt.plot(x, pdf_fitted,'--', color='orange', linewidth=1.5, label='fitted lognorm')
plt.plot(x_t, a_t/2.0,'--', color = 'orange', linewidth=1.5, label='Gaussian')
plt.plot(x_t, b_t/2.0,'--', color = 'navy', linewidth=1.5, label='Gamma')
plt.plot(x_t, y_t,'--', color = 'red', linewidth=1.5, label='Gaussian+Gamma')

# note: add param stacked doesn't make a difference
# with outlier rejection, bins = 150. without outlier rejection, bins = 15000
yhist, xhist, patches = plt.hist(anTag_nlos_err, bins=150,color='steelblue',alpha=0.45, density=True, stacked=True)   
plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'green')
plt.axvline(x=0.0, alpha=1.0, linestyle ='--', color = 'black')
plt.xlabel('nlos error [m]')
plt.ylabel('Percent of Total Frequency')
ax.set_xlim([-1.0, 1.0]) 
ax.legend()
plt.show()


