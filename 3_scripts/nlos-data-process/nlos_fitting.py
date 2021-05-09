'''
Modeling TDOA meas. error under different NLOS conditions
'''
import os, sys
import numpy as np
from numpy import linalg
from matplotlib import pyplot as plt
import rosbag
from scipy import stats
from scipy.stats import skewnorm
from glob import glob     # module used for finding pathnames matching a specific pattern


# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# for 4K screen distplay
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 300

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
        
# load the nlos data with a selected obstacle   
# PATH_nlos = curr + '/nlos_error/an_tag/metal';      BIN = 1000 
# PATH_nlos = curr + '/nlos_error/an_tag/woodshelf';  BIN = 100  
PATH_nlos = curr + '/nlos_error/an_tag/wood-side-cabinet'; BIN = 8000
select_nlos_err = []
for filename in os.listdir(PATH_nlos):
    if filename.endswith('.npy'):
        err = np.load(os.path.join(PATH_nlos, filename))
        select_nlos_err.append(err)
    
# flatten the list to numpy array
anTag_nlos_err = np.concatenate(anTag_nlos_err).ravel()
anAn_nlos_err  = np.concatenate(anAn_nlos_err).ravel()
los_err        = np.concatenate(los_err).ravel()
select_nlos_err= np.concatenate(select_nlos_err).ravel()

# compensate for the small bias in los condition
los_bias = np.mean(los_err)
anTag_nlos_err = anTag_nlos_err - los_bias
anAn_nlos_err  = anAn_nlos_err  - los_bias
los_err        = los_err        - los_bias  
# ---------------------------------------------- #
if FIT_lognorm:

    X = np.linspace(-1.5, 1.5)

    fig = plt.figure(facecolor="white")
    ax = plt.subplot(111)
    ## skew norm doesn't fit the data
    # plt.plot(X, skewnorm.pdf(X, *skewnorm.fit(anTag_nlos_err)), 'r--', linewidth=1.5, label='fitted skewnorm')

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

    fig3 = plt.figure(facecolor="white")
    mu=0;  sigma=0
    bx = plt.subplot(111)
    (mu, sigma) = stats.norm.fit(select_nlos_err)
    print("mean0: ", mu, "std0: ", sigma)
    print("\n")
    yhist, xhist, patches = plt.hist(select_nlos_err, bins=BIN, color='steelblue',alpha=0.75, density=True)   # 15000
    plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'red')
    plt.axvline(x=0.0, alpha=1.0, linestyle ='--', color = 'black')
    plt.xlabel('los error [m]')
    plt.ylabel('Percent of Total Frequency')
    # plt.title('TDOA3 err_12 los') 
    bx.set_xlim([-1.5, 1.5]) 
    
    plt.show()