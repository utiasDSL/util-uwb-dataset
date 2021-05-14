'''
visualize los error
'''
import os, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.style as style

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
# current path of the script
curr = os.path.dirname(sys.argv[0])
err1 = np.load(curr+'/circle_test1/0513_circle1_test1_err.npy')
err2 = np.load(curr+'/circle_test1/0513_circle1_test2_err.npy')

err3 = np.load(curr+'/circle_test1/0513_circle1_test3_err.npy')
err4 = np.load(curr+'/circle_test1/0513_circle1_test4_err.npy')

err5 = np.load(curr+'/circle_test1/0513_circle1_test5_err.npy')
err6 = np.load(curr+'/circle_test1/0513_circle1_test6_err.npy')

err7 = np.load(curr+'/circle_test1/0513_circle1_test7_err.npy')
err8 = np.load(curr+'/circle_test1/0513_circle1_test8_err.npy')

err9 = np.load(curr+'/circle_test1/0513_circle1_test9_err.npy')
err10 = np.load(curr+'/circle_test1/0513_circle1_test10_err.npy')

err11 = np.load(curr+'/circle_test1/0513_circle1_test11_err.npy')
err12 = np.load(curr+'/circle_test1/0513_circle1_test12_err.npy')

data = [err1, err2, err3, err4, err5, err6,
        err7, err8, err9, err10, err11, err12]
  

circle2_err1 = np.load(curr+'/circle_test2/0513_circle2_test1_err.npy')
circle2_err2 = np.load(curr+'/circle_test2/0513_circle2_test2_err.npy')

circle2_err3 = np.load(curr+'/circle_test2/0513_circle2_test3_err.npy')
circle2_err4 = np.load(curr+'/circle_test2/0513_circle2_test4_err.npy')

circle2_err5 = np.load(curr+'/circle_test2/0513_circle2_test5_err.npy')
circle2_err6 = np.load(curr+'/circle_test2/0513_circle2_test6_err.npy')

data2 = [circle2_err1, circle2_err2, circle2_err3,
         circle2_err4, circle2_err5, circle2_err6]

fig = plt.figure()
fig1 = plt.figure()
# Creating axes instance
ax = fig.add_subplot(111)
# Creating plot
ax.boxplot(data)
  
bx = fig1.add_subplot(111)
bx.boxplot(data2)
# show plot
plt.show()





