#!/usr/bin/env python

""" File
@description:
    The functions included in this file are main functions for calibration optimization.
@version: V0.36
@author:
	Fangyi Zhang	email:gzzhangfangyi@gmail.com
	Liao Wu			email:wuliaothu@gmail.com
@acknowledgement:
    ARC Centre of Excellence for Robotic Vision (ACRV)
    Queensland Univsersity of Technology (QUT)
@history:
    V0.00   21/04/2016  developed the matlab version
    V0.10	25/05/2016	converted to python scripts
    V0.20	26/05/2016	added function of saving data to a txt file and displaying an error curve
    V0.30	26/05/2016	adapted to the calibration of an on-hand realsense camera
    V0.35	27/05/2016	made the code compatible for both kinect and realsense
    V0.36	27/05/2016	added a parameter for the file to set whether to calibrate an on-hand realsense camera
"""

import source_functions as src
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
import rospy

def calibration(display=False,Th=1e-06, parent_folder=os.path.dirname(os.getcwd()), realsense=False):
	# Load data files
	# parent_folder = os.path.dirname(os.getcwd())
	if realsense:
		sensor = 'realsense'
	else:
		sensor = 'kinect'
	K2M_data_file = sensor + '-sensor_to_marker_pose.csv'
	B2E_data_file = sensor + '-base_to_end_pose.csv'

	Kinect = src.load_csv_data(parent_folder+'/'+K2M_data_file)
	Base = src.load_csv_data(parent_folder+'/'+B2E_data_file)
	M = Kinect.shape[0]

	# Initialization
	Kinect2Marker = np.zeros((4,4,M))
	Base2End = Kinect2Marker.copy()
	error = np.zeros((M,1))
	R_Kinect2Marker = np.zeros((3,3,M))
	t_Kinect2Marker = np.zeros((3,M))
	R_Base2End = np.zeros((3,3,M))
	t_Base2End = np.zeros((3,M))

	# Calculation
	# transform the collected data into transformation matrices
	# data -> homogenous matrics
	for i in range(0,M):
		Kinect2Marker[:,:,i] = src.data2Homo(Kinect[i,:])
		R_Kinect2Marker[:,:,i] = Kinect2Marker[0:3,0:3,i]
		t_Kinect2Marker[:,i] = Kinect2Marker[0:3,3,i]

		Base2End[:,:,i] = src.data2Homo(Base[i,:])
		if realsense:
			Base2End[:,:,i] = np.linalg.inv(Base2End[:,:,i])

		R_Base2End[:,:,i] = Base2End[0:3,0:3,i]; 
		t_Base2End[:,i] = Base2End[0:3,3,i];

	# Optimization
	R_BK_init,t_BK_init,t_EM_init = src.initial(R_Base2End,t_Base2End,t_Kinect2Marker)
	R_BK,t_BK,t_EM = src.iterative(R_BK_init,t_Kinect2Marker,t_BK_init,R_Base2End,t_EM_init,t_Base2End,Th)

	# Calculate the errors of each sample with respect to the optimal solution
	for i in range(0,M):
		temp = R_Base2End[:,:,i].dot(t_EM).reshape((3,1),order='F') + t_Base2End[:,i].reshape((3,1),order='F') - R_BK.dot(t_Kinect2Marker[:,i]).reshape((3,1),order='F') - t_BK.reshape((3,1),order='F')
		error[i] = np.linalg.norm(temp)
	error_mean = error.mean()
	error_var = error.var()

# - Translation: [0.000, 0.000, 0.000]
# - Rotation: in Quaternion [0.500, -0.500, 0.500, 0.500]
#             in RPY (radian) [1.571, -1.571, 0.000]
#             in RPY (degree) [90.000, -90.000, 0.000]

	if realsense:
		# get the tf from left_gripper_base to camera_link
		rgb2camera = np.matrix([0, 0, 0, 0.500, -0.500, 0.500, 0.500])
		rgb2camera = src.data2Homo(rgb2camera)
		R_BK = R_BK.dot(rgb2camera[0:3,0:3])
	# 

	# Output the calibrated transformation
	RPY_BK = src.rpy(R_BK)
	T_BK = t_BK.squeeze().tolist()
	output = [T_BK[0], T_BK[1], T_BK[2], RPY_BK[2], RPY_BK[1], RPY_BK[0]]
	results_file = parent_folder + '/' + sensor + '-Calibration_Results.txt'
	src.data2txt(M, output, error_mean, error_var, results_file) # save data to a txt file
	print "No. of samples: ", M
	print "Calibrated tf: [x, y, z, y, p, r] \n", output
	print "Error (Mean, Variance): ", error_mean, error_var
	print "Calibration results have been saved in 'Calibration_Results.txt'"
	if display:
		plt.plot(range(0,M),error)
		plt.xlabel('Point Number')
		plt.ylabel('Calibration Error /m')
		plt.show()

	return output, error_var



##########################
if __name__ == '__main__':
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    					description=calibration.__doc__)
	parser.add_argument('-r','--realsense', dest='realsense', default='False',
						help='Whether to calibrate an on-hand realsense camera (default: False)' )
	args = parser.parse_args(rospy.myargv()[1:])
	if args.realsense == 'True':
		realsense = True
	else:
		realsense = False
	calibration(realsense=realsense)