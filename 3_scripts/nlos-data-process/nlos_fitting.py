'''
Modeling TDOA meas. error under different NLOS conditions
'''
import os, sys
import numpy as np
from numpy import linalg
from matplotlib import pyplot as plt
import rosbag
from scipy import stats
from glob import glob     # module used for finding pathnames matching a specific pattern


# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

# -------------- param --------------- #
VISUAL = False;  FIT_lognorm = True


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

# load the data for nlos between anchor and anchor
PATH_anAn = curr+'/nlos_error/an_an/'
subdir = fast_scandir(PATH_anAn)
all_files = []
for path in subdir:
    # print(os.path.join(path,EXT))
    files = [file for file in glob(os.path.join(path,EXT))]
    all_files.extend(files)
    
anAn_nlos_err = []
for filename in all_files:
    err = np.load(filename)
    anAn_nlos_err.append(err)
    
# load the los data (no subfolder)
PATH_los = curr+'/nlos_error/los/'
los_err = []
for filename in os.listdir(PATH_los):
    if filename.endswith('.npy'):
        err = np.load(os.path.join(PATH_los, filename))
        los_err.append(err)
        
    
# flatten the list to numpy array
anTag_nlos_err = np.concatenate(anTag_nlos_err).ravel()
anAn_nlos_err  = np.concatenate(anAn_nlos_err).ravel()
los_err        = np.concatenate(los_err).ravel()

# compensate for the small bias in los condition
los_bias = np.mean(los_err)
anTag_nlos_err = anTag_nlos_err - los_bias
anAn_nlos_err  = anAn_nlos_err  - los_bias
los_err        = los_err        - los_bias  
# ---------------------------------------------- #

# TODO: how to effectively fit a long-tail distribution
idx = np.where(np.abs(anTag_nlos_err)<1.0)
nlos_err = -anTag_nlos_err[idx]
anTag_nlos_err = -anTag_nlos_err
# dummy
# nlos_err=stats.lognorm(0.5,loc=0,scale=1).rvs(size=5000) 

lognorm_param = stats.lognorm.fit(nlos_err)
std = lognorm_param[0]; loc=lognorm_param[1]; scale=lognorm_param[2]
###
# Y(samp) ~ lognormal(mu, sigma), X ~ norm(mu, sigma), exp(X) = Y
# X = np.log(samp) => np.std(X) = std, np.mean(X) = scale
###
print("The param for log-normal \nstd: {0}, loc: {1}, scale: {2}".format(std, loc, scale)) 
x=np.linspace(-1.5, 1.5, 1000)
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
    
l_norm_s, l_norm_p = stats.kstest(anTag_nlos_err, 'lognorm', args=lognorm_param)
print("The probability of being a lognorm distribution is {0}%".format(l_norm_p*100))    
    
fig = plt.figure(facecolor="white")
ax = plt.subplot(111)
plt.plot(x,pdf_fitted,'r--', linewidth=1.5, label='fitted lognorm')
# plt.plot(x,lognorm_scratch,'g-', linewidth=1)

plt.xlabel('Error [m]')
plt.ylabel('Percent of Total Frequency')
plt.title('nlos error') 
ax.legend()
plt.hist(anTag_nlos_err,bins=15000,density=True,alpha=.3)
ax.set_xlim([-1.5, 1.5]) 
plt.show()

if VISUAL:
    fig = plt.figure(facecolor="white")
    mu=0;  sigma=0
    ax = plt.subplot(111)
    (mu, sigma) = stats.norm.fit(anTag_nlos_err)
    print("mean0: ", mu, "std0: ", sigma)
    print("\n")
    yhist, xhist, patches = plt.hist(anTag_nlos_err, bins=15000,color='steelblue',alpha=0.75, density=True)   # 15000
    plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'red')
    plt.axvline(x=0.0, alpha=1.0, linestyle ='--', color = 'black')
    plt.xlabel('nlos error [m]')
    plt.ylabel('Percent of Total Frequency')
    # plt.title('TDOA3 err_12 los') 
    ax.set_xlim([-1.0, 1.0]) 
        
    fig1 = plt.figure(facecolor="white")
    mu=0;  sigma=0
    bx = plt.subplot(111)
    (mu, sigma) = stats.norm.fit(anAn_nlos_err)
    print("mean0: ", mu, "std0: ", sigma)
    print("\n")
    yhist, xhist, patches = plt.hist(anAn_nlos_err, bins=15000,color='steelblue',alpha=0.75, density=True)   # 15000
    plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'red')
    plt.axvline(x=0.0, alpha=1.0, linestyle ='--', color = 'black')
    plt.xlabel('nlos error [m]')
    plt.ylabel('Percent of Total Frequency')
    # plt.title('TDOA3 err_12 los') 
    bx.set_xlim([-0.5, 0.5]) 

    fig2 = plt.figure(facecolor="white")
    mu=0;  sigma=0
    bx = plt.subplot(111)
    (mu, sigma) = stats.norm.fit(los_err)
    print("mean0: ", mu, "std0: ", sigma)
    print("\n")
    yhist, xhist, patches = plt.hist(los_err, bins=100,color='steelblue',alpha=0.75, density=True)   # 15000
    plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'red')
    plt.axvline(x=0.0, alpha=1.0, linestyle ='--', color = 'black')
    plt.xlabel('los error [m]')
    plt.ylabel('Percent of Total Frequency')
    # plt.title('TDOA3 err_12 los') 
    bx.set_xlim([-0.5, 0.5]) 

    plt.show()