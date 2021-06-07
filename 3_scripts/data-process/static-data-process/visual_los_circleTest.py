'''
visualize los circle test error
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

	err1 = np.load(los_folder+'/circle_test1/0513_circle1_test1_err.npy')
	err2 = np.load(los_folder+'/circle_test1/0513_circle1_test2_err.npy')

	err3 = np.load(los_folder+'/circle_test1/0513_circle1_test3_err.npy')
	err4 = np.load(los_folder+'/circle_test1/0513_circle1_test4_err.npy')

	err5 = np.load(los_folder+'/circle_test1/0513_circle1_test5_err.npy')
	err6 = np.load(los_folder+'/circle_test1/0513_circle1_test6_err.npy')

	err7 = np.load(los_folder+'/circle_test1/0513_circle1_test7_err.npy')
	err8 = np.load(los_folder+'/circle_test1/0513_circle1_test8_err.npy')

	err9 = np.load(los_folder+'/circle_test1/0513_circle1_test9_err.npy')
	err10 = np.load(los_folder+'/circle_test1/0513_circle1_test10_err.npy')

	err11 = np.load(los_folder+'/circle_test1/0513_circle1_test11_err.npy')
	err12 = np.load(los_folder+'/circle_test1/0513_circle1_test12_err.npy')

	data1 = [err1, err2, err3, err4, err5, err6,
			err7, err8, err9, err10, err11, err12]


	circle2_err1 = np.load(los_folder+'/circle_test2/0513_circle2_test1_err.npy')
	circle2_err2 = np.load(los_folder+'/circle_test2/0513_circle2_test2_err.npy')

	circle2_err3 = np.load(los_folder+'/circle_test2/0513_circle2_test3_err.npy')
	circle2_err4 = np.load(los_folder+'/circle_test2/0513_circle2_test4_err.npy')

	circle2_err5 = np.load(los_folder+'/circle_test2/0513_circle2_test5_err.npy')
	circle2_err6 = np.load(los_folder+'/circle_test2/0513_circle2_test6_err.npy')

	data2 = [circle2_err1, circle2_err2, circle2_err3,
			circle2_err4, circle2_err5, circle2_err6]

	# convert to numpy array without warning
	data1 = np.array(data1, dtype="object")
	data2 = np.array(data2, dtype="object")

	fig1 = plt.figure()
	ax = fig1.add_subplot(111)
	ax.boxplot(data1)
	plt.ylim(-0.5, 0.5)

	fig2 = plt.figure()
	bx = fig2.add_subplot(111)
	bx.boxplot(data2)
	plt.ylim(-0.5, 0.5)
	plt.show()





