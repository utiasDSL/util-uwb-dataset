'''
visualize los line test error
'''
import argparse
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.style as style
from scipy import stats

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
XY_FONTSIZE = 7;   LABEL_SIZE = 12

def simpleOutlierRej(err):
	outlier = []
	bound = 1      # set error bound to be 1 m
	for i in range(len(err)):
		if np.abs(err[i]) > bound:
			outlier.append(i)
	error = np.delete(err, outlier)
	return error

	
if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('-i', action='store', nargs=1)
	args = parser.parse_args()

	los_folder = args.i[0]

	# access rosbag: los distance tests 
	err_d1 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test1_err.npy'))
	err_d2 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test2_err.npy'))

	err_d3 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test3_err.npy'))
	err_d4 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test4_err.npy'))

	err_d5 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test5_err.npy'))
	err_d6 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test6_err.npy'))

	err_d7 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test7_err.npy'))
	err_d8 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test8_err.npy'))

	err_d9 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test9_err.npy'))
	err_d10 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test10_err.npy'))

	err_d11 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test11_err.npy'))
	err_d12 = simpleOutlierRej(np.load(los_folder+'/line_test/0513_line_test12_err.npy'))

	# access rosbag: los angle tests
	err_a1 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test1_err.npy'))
	err_a2 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test2_err.npy'))

	err_a3 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test3_err.npy'))
	err_a4 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test4_err.npy'))

	err_a5 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test5_err.npy'))
	err_a6 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test6_err.npy'))

	err_a7 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test7_err.npy'))
	err_a8 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test8_err.npy'))

	err_a9 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test9_err.npy'))
	err_a10= simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test10_err.npy'))

	err_a11 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test11_err.npy'))
	err_a12 = simpleOutlierRej(np.load(los_folder+'/circle_test1/0513_circle1_test12_err.npy'))

	los_std = np.array([np.std(err_d1), np.std(err_d2), np.std(err_d3), np.std(err_d4), np.std(err_d5),
					 	np.std(err_d6), np.std(err_d7), np.std(err_d8), np.std(err_d9), np.std(err_d10), np.std(err_d11), np.std(err_d12),
						np.std(err_a1), np.std(err_a2), np.std(err_a3), np.std(err_a4), np.std(err_a5),
						np.std(err_a6), np.std(err_a7), np.std(err_a8), np.std(err_a9), np.std(err_a10), np.std(err_a11), np.std(err_a12), ])

	fig = plt.figure(facecolor="white")
	mu=0;  sigma=0
	ax = plt.subplot(111)
	(mu, sigma) = stats.norm.fit(los_std)
	print("mean0: ", mu, "std0: ", sigma)
	print("\n")
	yhist, xhist, patches = plt.hist(los_std, bins=6,color='steelblue',alpha=0.75, density=True)
	plt.axvline(x=mu, alpha=1.0, linestyle ='--', color = 'red')
	plt.legend(['mean'], fontsize=20)
	plt.xlabel('Standard deviation of LOS TDOA errors', fontsize=20)
	plt.ylabel('Percent of Total Frequency', fontsize=20)
	plt.xticks(fontsize = XY_FONTSIZE)
	plt.yticks(fontsize = XY_FONTSIZE)
	ax.set_xlim([0.0, 0.1])
	plt.show()
	

