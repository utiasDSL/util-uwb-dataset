'''
visualize los line test error
'''
import argparse
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.style as style

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('-i', action='store', nargs=1)
	args = parser.parse_args()

    # access rosbag
	los_folder = args.i[0]

	err1 = np.load(los_folder+'/line_test/0513_line_test1_err.npy')
	err2 = np.load(los_folder+'/line_test/0513_line_test2_err.npy')

	err3 = np.load(los_folder+'/line_test/0513_line_test3_err.npy')
	err4 = np.load(los_folder+'/line_test/0513_line_test4_err.npy')

	err5 = np.load(los_folder+'/line_test/0513_line_test5_err.npy')
	err6 = np.load(los_folder+'/line_test/0513_line_test6_err.npy')

	err7 = np.load(los_folder+'/line_test/0513_line_test7_err.npy')
	err8 = np.load(los_folder+'/line_test/0513_line_test8_err.npy')

	err9 = np.load(los_folder+'/line_test/0513_line_test9_err.npy')
	err10 = np.load(los_folder+'/line_test/0513_line_test10_err.npy')

	err11 = np.load(los_folder+'/line_test/0513_line_test11_err.npy')
	err12 = np.load(los_folder+'/line_test/0513_line_test12_err.npy')

	data = [err1, err2, err3, err4, err5, err6,
			err7, err8, err9, err10, err11, err12]
	
	data = np.array(data, dtype="object")

	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.boxplot(data)
	plt.ylim(-0.5, 0.5)
	plt.show()





