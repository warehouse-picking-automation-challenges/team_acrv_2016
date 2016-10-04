#!/usr/bin/env python

""" File
@description:
    The functions included in this file are sub-functions used by calibration optimization.
@version: V0.20
@author:
	Fangyi Zhang	email:gzzhangfangyi@gmail.com
	Liao Wu			email:wuliaothu@gmail.com
@acknowledgement:
    ARC Centre of Excellence for Robotic Vision (ACRV)
    Queensland Univsersity of Technology (QUT)
@history:
    V0.00   21/04/2016  developed the matlab version
    V0.10	25/05/2016	converted to python scripts
    V0.20   26/05/2016  added the function of saving data to a txt file
"""

import csv
import numpy as np

fieldnames = ['x', 'y', 'z', 'x_o', 'y_o', 'z_o', 'w_o']
# Load pre-defined poseset for the end-effector
def load_csv_data(file_name):
    file = open(file_name, 'rb')
    reader = csv.DictReader(file)
    data = []
    for row in reader:
        single = []
        for i in fieldnames:
            single.append(float(row[i]))
        data.append(single)
    file.close()
    data = np.matrix(data)
    return data

# Convert a quatonion to 
def Q2R(Q):
	norm = np.linalg.norm(Q)
	q0 = Q[0] / norm
	q1 = Q[1] / norm
	q2 = Q[2] / norm
	q3 = Q[3] / norm
	R = np.zeros((3,3))
	R[0,:] = [q0*q0+q1*q1-q2*q2-q3*q3, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2]
	R[1,:] = [2*q1*q2+2*q0*q3, q0*q0-q1*q1+q2*q2-q3*q3, 2*q2*q3-2*q0*q1]
	R[2,:] = [2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, q0*q0-q1*q1-q2*q2+q3*q3]
	return R

# Convert a sample to a homogeneous matrix
def data2Homo(data):
	Q = np.array([data[0,6], data[0,3], data[0,4], data[0,5]])
	R = Q2R(Q)
	Homo = np.zeros((4,4))
	Homo[0:3, 0:3] = R
	Homo[0:3, 3] = np.matrix([data[0,0], data[0,1], data[0,2]])
	Homo[3, :] = np.matrix([0,0,0,1])
	return Homo

# Initialize an optimal beggining position
def initial(R_BH,t_BH,P_W):
    M = R_BH.shape[2]
    A = np.zeros((3 * M,15))
    b = np.zeros((3 * M,1))
    for i in range(0,M):
        A[i*3:i*3+3,:] = np.concatenate((np.kron(np.transpose(P_W[:,i]),np.eye(3)), np.eye(3), -R_BH[:,:,i]), axis=1)
        b[i*3:i*3+3,0] = t_BH[:,i]
    x, resid, rank, s = np.linalg.lstsq(A,b)
    tempR_BW = np.reshape(x[0:9,0],(3,3),order='F')
    t_BW = x[9:12]
    P_H = x[12:15]
    U, S ,Vh = np.linalg.svd(tempR_BW)
    V = np.transpose(Vh)
    R_BW = U.dot(np.transpose(V))
    return R_BW, t_BW, P_H

# skew Create skew-symmetric matrix
#   
#  S = skew(V) is a skew-symmetric matrix formed from V (3x1).
#  
#             | 0   -vz  vy|
#             | vz   0  -vx|
#             |-vy   vx  0 |
def skew(V):
	S = np.zeros((3,3))
	S[0,:] = np.matrix([0, -V[2], V[1]])
	S[1,:] = np.matrix([V[2], 0, -V[0]])
	S[2,:] = np.matrix([-V[1], V[0], 0])

	return S

# Generate A
def A_maker(R_RM,P_M,R_RE):

    n = P_M.shape[1]
    crossRP_M = np.zeros((3,3,n))
    for i in range(0,n):
        crossRP_M[:,:,i] = -R_RM.dot(skew(P_M[:,i]))
    a1 = np.reshape(crossRP_M[:,0,:],(3*n,1),order='F')
    a2 = np.reshape(crossRP_M[:,1,:],(3*n,1),order='F')
    a3 = np.reshape(crossRP_M[:,2,:],(3*n,1),order='F')
    A1 = np.concatenate((a1,a2,a3),axis=1)
    A2 = np.tile(np.eye(3),(n,1))
    b1 = np.reshape(R_RE[:,0,:],(3*n,1),order='F')
    b2 = np.reshape(R_RE[:,1,:],(3*n,1),order='F')
    b3 = np.reshape(R_RE[:,2,:],(3*n,1),order='F')
    A3 = -np.concatenate((b1,b2,b3),axis=1)
    A = np.concatenate((A1,A2,A3),axis=1)
    return A

# Generate b
def b_maker(R_RM,P_M,P_RM,R_RE,P_E,P_RE):
   
    n = P_M.shape[1]
    b = np.zeros((3,n))
    for i in range(0,n):
    	# print( -R_RM.dot(P_M[:,i]).reshape((3,1),order='F'))
        temp = -R_RM.dot(P_M[:,i]).reshape((3,1),order='F') - P_RM.reshape((3,1),order='F') + R_RE[:,:,i].dot(P_E).reshape((3,1),order='F') + P_RE[:,i].reshape((3,1),order='F')
    	b[:,i] =  temp[:,0]
    b = np.reshape(b,(3*n,1),order='F')
    return b

# Generate a rotation matrix
def rotationMatrix(w,theta):

    R= np.eye(3) + skew(w).dot(np.sin(theta)) + skew(w).dot(skew(w)).dot((1-np.cos(theta)))
    return R


# Iterative optimization for calibration
def iterative(R_BW,P_W,t_BW,R_BH,P_H,t_BH,Th):

    while True:

        A = A_maker(R_BW,P_W,R_BH)
        b = b_maker(R_BW,P_W,t_BW,R_BH,P_H,t_BH)
        x, resid, rank, s = np.linalg.lstsq(A,b)
        delta_theta = x[0:3]
        delta_R = rotationMatrix(delta_theta / np.linalg.norm(delta_theta), np.linalg.norm(delta_theta))
        R_BW = R_BW.dot(delta_R)
        delta_t_BW = x[3:6]
        t_BW = t_BW + delta_t_BW
        delta_P_H = x[6:9]
        P_H = P_H + delta_P_H
        if np.linalg.norm(x) <=  Th:
        # if np.linalg.norm(x) <=  1e-06:
        	break

    return R_BW,t_BW,P_H

# Get roll, pitch and yaw
def rpy(R):

    beta = np.arctan2(-R[2,0], np.sqrt(R[0,0]**2 + R[1,0]**2))
    if np.isclose(beta, np.pi/2):
        alpha = 0
        gamma = np.archtan2(R[0,1],R[1,1])
    else:
        if np.isclose(beta, -np.pi/2):
            alpha = 0
            gamma = -np.arctan2(R[0,1],R[1,1])
        else:
            alpha = np.arctan2(R[1,0] / np.cos(beta),R[0,0] / np.cos(beta))
            gamma = np.arctan2(R[2,1] / np.cos(beta),R[2,2] / np.cos(beta))
    
    roll = gamma
    pitch = beta
    yaw = alpha
    xyz = [roll,pitch,yaw]
    return xyz

# Write data to a txt file
def data2txt(M, output, error_mean, error_var, filename):
    file = open(filename, "w")
    file.write("No. of samples:\t"+str(M)+"\n")
    file.write("calibrated tf: [x, y, z, y, p, r]\n"+str(output)+"\n")
    file.write("error mean:\t"+str(error_mean)+"\n")
    file.write("error variance:\t"+str(error_var)+"\n")
    file.close()