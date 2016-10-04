#!/usr/bin/env python

""" File
@description:
    The functions included in this file are for kinect tf calibration.
@version: V0.60
@author: Fangyi Zhang   email:gzzhangfangyi@gmail.com
@acknowledgement:
    ARC Centre of Excellence for Robotic Vision (ACRV)
    Queensland Univsersity of Technology (QUT)
@history:
    V0.00   21/04/2016  developed the first version
    V0.20   27/04/2016  refined in real experiments on a Baxter robot
    V0.30   27/04/2016  added the compatibility for both euler and quaternion modes
    V0.40   27/04/2016  added the function of trajectory speed scaling
    V0.45   27/04/2016  added the function of saving history pose data
    V0.50   26/05/2016  added the function of calculating an optimally calibrated transformation
    V0.55   26/05/2016  adapted to the calibration for an on-hand realsense camera
    V0.56   27/05/2016  added the ros parameter of realsense
    V0.60   27/05/2016  added a new control mode by replaying joint positions rather than using moveit
"""
# ROS libraries
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import tf
import cv_bridge
import rospkg

# Python libraries
import argparse
import os
import os.path
import shutil
import csv
# import cv2
import numpy as np
# import math
import time

# Import moveit relevant files
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg

# calibration optimization
import calibration as clb


# TODO:
# control the left arm to a certain pose

class KinectCalibrationWithAR(object):
    """ Setup
        An AR marker is deployed on an end-link.
        A kinect is used to recognize the pose of the marker with respect to one optical frame of the kinect sensor.
        The pose of the end-link is provided with respect to the robot base (world).
        The robot moves the arm to different positions. A set of data will be collected for each position.
        Eventually, both the transformation from the kinect optical frame to the base
        and the transformation from the marker to the end-link can be calibrated,
        but, usually only the prior will be used.
    """

    def __init__(self, limb, mode='manually', moveToInitial=False, realsense=False):

        self.is_stopping = False
        self.rate = rospy.Rate(30) # 30 Hz
        self.limb = limb

        # Set AR tracker parameters
        self.confidence_th = 0.8 # The confidence threshold to use a recognized marker
        self.pos_variance_th = 0.00003 # The variance threshold for a reliable pose recognition
        self.ar_pose_history_len = 10
        self.ar_pose_x_history = []
        self.ar_pose_y_history = []
        self.ar_pose_z_history = []
        self.ar_pose_xr_history = []
        self.ar_pose_yr_history = []
        self.ar_pose_zr_history = []
        # self.ar_pose_wr_history = []
        self.ar_pose_waiting_time = 100
        self.tf_waiting_time = 10.0

        # Set data file parameters
        rospack = rospkg.RosPack()
        if realsense:
            self.sensor = 'realsense'
        else:
            self.sensor = 'kinect'
        self.package_dir = rospack.get_path('apc_rgbd_tf_calibration')
        self.file_name_b2e = self.package_dir + '/' + self.sensor + '-base_to_end_pose.csv'
        self.file_name_k2m = self.package_dir + '/' + self.sensor + '-sensor_to_marker_pose.csv'
        self.file_name_predefined_left_poses = self.package_dir  + '/' + self.sensor + '-predefined_left_poses.csv'
        self.file_name_predefined_right_poses = self.package_dir  + '/' + self.sensor + '-predefined_right_poses.csv'
        self.file_name_left_arm_poses = self.package_dir  + '/' + self.sensor + '-left_arm_poses.csv'
        self.file_name_right_arm_poses = self.package_dir  + '/' + self.sensor + '-right_arm_poses.csv'
        self.hist_dir = self.package_dir + '/hist_of_' + self.sensor + '_to_marker_pose'
        self.file_name_hist_k2m = '-' + self.sensor + '_to_marker_pose.csv'
        self.fieldnames_q = ['x', 'y', 'z', 'x_o', 'y_o', 'z_o', 'w_o']
        self.quaternion_mode = True
        if self.quaternion_mode:
            self.fieldnames = self.fieldnames_q
        else:
            self.fieldnames = ['x', 'y', 'z', 'x_o', 'y_o', 'z_o']
        self.data_amount = 0
        self.eleminated_data_amount = 0
        self.calibrated_tf = []

        # Set optimization parameters
        self.optimization_threshold = 5 # the least number of samples for optimization
        self.realsense = realsense # whether calibrate an on-hand realsense camera

        # Set and Activate Baxter
        self.joint_speed = 0.35 #1.0
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm.set_joint_position_speed(self.joint_speed)
        self.right_arm.set_joint_position_speed(self.joint_speed)
        self.left_joints = self.left_arm.joint_names()
        self.right_joints = self.right_arm.joint_names()

        if self.limb == 'left_arm':
            self.moving_arm = self.left_arm
            self.fixed_arm = self.right_arm
            self.marker_end = '/left_gripper_base'
        elif self.limb == 'right_arm':
            self.moving_arm = self.right_arm
            self.fixed_arm = self.left_arm
            self.marker_end = '/right_gripper_base'

        # Initialize data files and folders
        self.initial_data_file()

        # Start subscribing the ar_pose_marker topic
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_pose_grabbing)
        self.tf_listener = tf.TransformListener()
        rospy.on_shutdown(self.clean_shutdown)
        rospy.loginfo('Initialization finished!')
        rospy.loginfo('Starting the rgbd tf calibration using the AR marker on right limb')

        # Start the controller
        if mode == 'manually':
            self.control_manually()
        elif mode == 'poseset':
            # Load poseset which provides potential end-effector poses
            self.left_pos = self.load_joints_posset(file=self.file_name_predefined_left_poses,fieldnames=self.left_joints)
            self.right_pos = self.load_joints_posset(file=self.file_name_predefined_right_poses,fieldnames=self.right_joints)
            # Move the right arm to a designated pos
            if self.limb == 'left_arm':
                self.fixed_desired_pos = self.right_pos[-1]
                self.moving_poses = self.left_pos
            elif self.limb == 'right_arm':
                self.fixed_desired_pos = self.left_pos[-1]
                self.moving_poses = self.right_pos
            # right_desired_pos = self.right_pos[-1]
            rospy.loginfo('Moving the end-effector of the right limb to %s ...', self.fixed_desired_pos)
            self.fixed_arm.move_to_joint_positions(self.fixed_desired_pos)
            self.control_from_poseset()
        else:
            self.control()

        # rospy.spin()    # only reactive control, wait for action cmd received

    def ar_pose_history_append(self,data):
        result_q = [data.position.x,
                    data.position.y,
                    data.position.z,
                    data.orientation.x,
                    data.orientation.y,
                    data.orientation.z,
                    data.orientation.w]
        result_euler = self.quaternion_to_euler(result_q)

        self.ar_pose_x_history.append(result_euler[0])
        self.ar_pose_y_history.append(result_euler[1])
        self.ar_pose_z_history.append(result_euler[2])
        self.ar_pose_xr_history.append(result_euler[3])
        self.ar_pose_yr_history.append(result_euler[4])
        self.ar_pose_zr_history.append(result_euler[5])
        # self.ar_pose_wr_history.append(data.orientation.w)

    def ar_pose_history_pop(self):
        self.ar_pose_x_history.pop(0)
        self.ar_pose_y_history.pop(0)
        self.ar_pose_z_history.pop(0)
        self.ar_pose_xr_history.pop(0)
        self.ar_pose_yr_history.pop(0)
        self.ar_pose_zr_history.pop(0)
        # self.ar_pose_wr_history.pop(0)

    def ar_pose_history_clear(self):
        del self.ar_pose_x_history[:]
        del self.ar_pose_y_history[:]
        del self.ar_pose_z_history[:]
        del self.ar_pose_xr_history[:]
        del self.ar_pose_yr_history[:]
        del self.ar_pose_zr_history[:]
        # del self.ar_pose_wr_history[:]

    # Transform the raw ar_pose to the pose with respect to the kinect_link_frame
    def ar_pose_grabbing(self,data):
        num_markers = 0
        marker = AlvarMarker()
        marker.confidence = 0
        for i in data.markers:
            marker = i
            num_markers = num_markers + 1

            # rospy.loginfo('confidence is: %f', i.confidence)
            # if i.confidence > self.confidence_th:
                # num_markers = num_markers + 1
                # if i.confidence > marker.conficence:
                    # marker = i
        # if num_markers > 0:
        if num_markers == 1:
            if len(self.ar_pose_x_history) < self.ar_pose_history_len:
                # Consider to compare the marker id to determine whether to append a marker
                self.ar_pose_history_append(marker.pose.pose)
            else:
                self.ar_pose_history_pop()
                self.ar_pose_history_append(marker.pose.pose)
            self.update_data_hist(marker.pose.pose)
        else:
            rospy.loginfo('No marker is reliablly detected!!!')
        # rospy.loginfo('AR pose history length: %d', len(self.ar_pose_x_history))
        # rospy.sleep(1)

    def process_ar_pose(self):
        # Initialize returning variables
        pose_reliable = False
        ar_x_mean = 0
        ar_y_mean = 0
        ar_z_mean = 0
        ar_xr_mean = 0
        ar_yr_mean = 0
        ar_zr_mean = 0
        # ar_wr_mean = 0
        if len(self.ar_pose_x_history) >= self.ar_pose_history_len:
            ar_x_var = np.nanvar(self.ar_pose_x_history)
            ar_y_var = np.nanvar(self.ar_pose_y_history)
            ar_z_var = np.nanvar(self.ar_pose_z_history)
            # ar_xr_var = np.nanvar(self.ar_pose_xr_history)
            # ar_yr_var = np.nanvar(self.ar_pose_yr_history)
            # ar_zr_var = np.nanvar(self.ar_pose_zr_history)

            if ar_x_var < self.pos_variance_th:
                if ar_y_var < self.pos_variance_th:
                    if ar_z_var < self.pos_variance_th:
                        pose_reliable = True
            if pose_reliable:
                ar_x_mean = np.nanmean(self.ar_pose_x_history)
                ar_y_mean = np.nanmean(self.ar_pose_y_history)
                ar_z_mean = np.nanmean(self.ar_pose_z_history)
                ar_xr_mean = np.nanmean(self.ar_pose_xr_history)
                ar_yr_mean = np.nanmean(self.ar_pose_yr_history)
                ar_zr_mean = np.nanmean(self.ar_pose_zr_history)
                # ar_wr_mean = np.nanmean(self.ar_pose_wr_history)

                rospy.loginfo('x (mean, variance): (%f, %f)', ar_x_mean, ar_x_var)
                rospy.loginfo('y (mean, variance): (%f, %f)', ar_y_mean, ar_y_var)
                rospy.loginfo('z (mean, variance): (%f, %f)', ar_z_mean, ar_z_var)

        return pose_reliable, ar_x_mean, ar_y_mean, ar_z_mean, ar_xr_mean, ar_yr_mean, ar_zr_mean
        # return pose_reliable, ar_x_mean, ar_y_mean, ar_z_mean, ar_xr_mean, ar_yr_mean, ar_zr_mean, ar_wr_mean

    # Initialize the files to save calibration data
    def initial_data_file(self):
        # file = open("base_to_end_data.csv", "wb")
        # rospy.loginfo(os.getcwd())
        file = open(self.file_name_b2e, 'wb')
        writer = csv.DictWriter(file,fieldnames=self.fieldnames)
        # header_txt = "R_Mean\tR_Variance\tSuccess_Rate\n"
        writer.writeheader()
        file.close()
        file = open(self.file_name_k2m, 'wb')
        writer = csv.DictWriter(file,fieldnames=self.fieldnames)
        # header_txt = "R_Mean\tR_Variance\tSuccess_Rate\n"
        writer.writeheader()
        file.close()
        file = open(self.file_name_left_arm_poses, 'wb')
        writer = csv.DictWriter(file,fieldnames=self.left_joints)
        # header_txt = "R_Mean\tR_Variance\tSuccess_Rate\n"
        writer.writeheader()
        file.close()
        file = open(self.file_name_right_arm_poses, 'wb')
        writer = csv.DictWriter(file,fieldnames=self.right_joints)
        # header_txt = "R_Mean\tR_Variance\tSuccess_Rate\n"
        writer.writeheader()
        file.close()

        # History data folder
        if not os.path.exists(self.hist_dir):
            os.makedirs(self.hist_dir)
        else:
            shutil.rmtree(self.hist_dir)
            os.makedirs(self.hist_dir)


    # Load pre-defined poseset for the end-effector
    def load_joints_posset(self, file, fieldnames):
        file = open(file, 'rb')
        reader = csv.DictReader(file)
        poseset = []
        for row in reader:
            single = {key: float(row[key]) for key in fieldnames}
            poseset.append(single)
        file.close()
        return poseset

    # Save each single frame of data into the files
    def update_data_file(self, file_name, result_list):
        file = open(file_name, "ab")
        writer = csv.DictWriter(file,fieldnames=self.fieldnames)
        if self.quaternion_mode:
            result_list = self.euler_to_quaternion(result_list)
        result_dict = {}
        dict_i = 0
        for i in result_list:
            result_dict[self.fieldnames[dict_i]] = str(i)
            dict_i = dict_i + 1
        writer.writerow(result_dict)
        # rospy.loginfo(data.position.x)
        file.close()

    def update_joints_pos_file(self, file_name, fieldnames, result_dict):
        file = open(file_name, "ab")
        writer = csv.DictWriter(file,fieldnames=fieldnames)
        writer.writerow(result_dict)
        # rospy.loginfo(data.position.x)
        file.close()

    def update_data_hist(self, data):
        file_name = self.hist_dir + '/' + str(self.data_amount+1) + self.file_name_hist_k2m
        if not os.path.exists(file_name):
            file = open(file_name, 'wb')
            writer = csv.DictWriter(file,fieldnames=self.fieldnames_q)
            writer.writeheader()
            file.close()

        result_list = [data.position.x,
                    data.position.y,
                    data.position.z,
                    data.orientation.x,
                    data.orientation.y,
                    data.orientation.z,
                    data.orientation.w]
        file = open(file_name, "ab")
        writer = csv.DictWriter(file,fieldnames=self.fieldnames_q)
        result_dict = {}
        dict_i = 0
        for i in result_list:
            result_dict[self.fieldnames_q[dict_i]] = str(i)
            dict_i = dict_i + 1
        writer.writerow(result_dict)
        # rospy.loginfo(data.position.x)
        file.close()

    def rename_eliminated_pose(self):
        file_name = self.hist_dir + '/' + str(self.data_amount+1) + self.file_name_hist_k2m
        dst_file_name = self.hist_dir + '/' + str(self.data_amount+1) + '_eliminated_' + str(self.eleminated_data_amount) + self.file_name_hist_k2m
        if os.path.exists(file_name):
            os.rename(file_name, dst_file_name)

    # Convert euler to quaternion
    def euler_to_quaternion(self, data):
        (x_o, y_o, z_o, w_o) = tf.transformations.quaternion_from_euler(data[3], data[4], data[5])
        return [data[0], data[1], data[2], x_o, y_o, z_o, w_o]

    # Convert quaternion to euler
    def quaternion_to_euler(self, data):
        quaternion = (
        data[3],
        data[4],
        data[5],
        data[6])
        (x_o,y_o,z_o) = tf.transformations.euler_from_quaternion(quaternion)
        return [data[0], data[1], data[2], x_o, y_o, z_o]

    # Control using pre-defined poseset
    # Control the end-effector to different poses and record the corresponding pose data
    # Once being stable, record the end-effector pose and the AR marker pose
    def control_manually(self):

        #print "waiting for action file: ", os.path.abspath(file_name)
        while not self.is_stopping:
            self.rate.sleep()
            rospy.loginfo('Please manually move the %s limb to desired poses ...', self.limb)
            # Wait for the press of Enter
            try:
                input('Move the arm to a desired pose, then press Enter:')
            except SyntaxError:
                pass

            # Clear previous history poses
            self.ar_pose_history_clear()

            # Wait the AR pose recognition for 20 s
            t_end = time.time() + self.ar_pose_waiting_time
            while (not self.is_stopping) and time.time() < t_end:
                self.rate.sleep()
                (pose_reliable, x, y, z, x_o, y_o, z_o) = self.process_ar_pose()

                # pose_reliable = True
                if pose_reliable:
                    # Read tf transformation within 10s
                    try:
                        self.tf_listener.waitForTransform('/base', self.marker_end, rospy.Time(0), rospy.Duration(self.tf_waiting_time))
                        (trans, rot) = self.tf_listener.lookupTransform('/base', self.marker_end, rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                    # Update the pose from /kinect to a AR marker
                    self.update_data_file(self.file_name_k2m, [x, y, z, x_o, y_o, z_o])

                    # Update the pose from /base to /left_gripper_base
                    b2e_pose = self.quaternion_to_euler([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])
                    self.update_data_file(self.file_name_b2e, b2e_pose)

                    # Update the joints pos history of the left arm
                    angles = self.left_arm.joint_angles()
                    self.update_joints_pos_file(self.file_name_left_arm_poses, self.left_joints, angles)

                    # Update the joints pos history of the right arm
                    angles = self.right_arm.joint_angles()
                    self.update_joints_pos_file(self.file_name_right_arm_poses, self.right_joints, angles)

                    self.data_amount = self.data_amount + 1
                    rospy.loginfo('Simples collected: %f', self.data_amount)

                    # Real-time caibration, for sample collection reference
                    if self.data_amount > self.optimization_threshold:
                        self.calibrated_tf = clb.calibration(parent_folder=self.package_dir,realsense=self.realsense)

                    break

            if not pose_reliable:
                self.eleminated_data_amount = self.eleminated_data_amount + 1
                rospy.loginfo('Simples eliminated: %f', self.eleminated_data_amount)
                self.rename_eliminated_pose()


    # Control the pose of the end-effector manually
    # Manually move the end-effector to different poses and record the corresponding pose data
    # Once being stable, record the end-effector pose and the AR marker pose
    def control_from_poseset(self):

        #print "waiting for action file: ", os.path.abspath(file_name)
        # while not self.is_stopping:
        for i in self.moving_poses:
            if self.is_stopping:
                break
            self.rate.sleep()
            rospy.loginfo('Moving the end-effector of the left limb to %s ...', i)
            # Move to a new joint pos
            self.moving_arm.move_to_joint_positions(i)
            rospy.sleep(2.0)

            # Clear previous history poses
            self.ar_pose_history_clear()

            # Wait the AR pose recognition for 20 s
            t_end = time.time() + self.ar_pose_waiting_time
            while (not self.is_stopping) and time.time() < t_end:
                self.rate.sleep()
                (pose_reliable, x, y, z, x_o, y_o, z_o) = self.process_ar_pose()
                # pose_reliable = True
                if pose_reliable:
                    # Read tf transformation within 10s
                    try:
                        self.tf_listener.waitForTransform('/base', self.marker_end, rospy.Time(0), rospy.Duration(self.tf_waiting_time))
                        (trans, rot) = self.tf_listener.lookupTransform('/base', self.marker_end, rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                    # Update the pose from /kinect to a AR marker
                    self.update_data_file(self.file_name_k2m, [x, y, z, x_o, y_o, z_o])

                    # Update the pose from /base to /left_gripper_base
                    b2e_pose = self.quaternion_to_euler([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])
                    self.update_data_file(self.file_name_b2e, b2e_pose)

                    self.data_amount = self.data_amount + 1
                    rospy.loginfo('Simples collected: %f', self.data_amount)
                    break

            if not pose_reliable:
                self.eleminated_data_amount = self.eleminated_data_amount + 1
                rospy.loginfo('Simples eliminated: %f', self.eleminated_data_amount)
                self.rename_eliminated_pose()

        # Calculate an optimal transformation for the samples
        if self.data_amount > self.optimization_threshold:
            self.calibrated_tf = clb.calibration(parent_folder=self.package_dir,realsense=self.realsense)
        else:
            rospy.loginfo('Not enough samples collected: %f', self.data_amount)

    def clean_shutdown(self):
        # stop the controller and the ROS node
        self.is_stopping = True
        moveit_commander.roscpp_shutdown()
        # Calculate an optimal transformation for the samples
        if self.data_amount > self.optimization_threshold:
            self.calibrated_tf = clb.calibration(parent_folder=self.package_dir,realsense=self.realsense)
        else:
            rospy.loginfo('Not enough samples collected: %f', self.data_amount)
        print("\nExiting the rgbd tf calibration!")


################################################################################
def main():
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__)
    # parser.add_argument('-l','--limb', dest='limb', default='left_arm',
    #                     choices=['right_arm','left_arm'],
    #                     help='Limb selection (default: left_arm)' )
    # parser.add_argument('-r','--realsense', dest='realsense', default=False,
    #                     help='Whether to calibrate an on-hand realsense camera (default: False)' )
    # args = parser.parse_args(rospy.myargv()[1:])

    # Get ros parameter settings
    realsense = rospy.get_param('realsense', False)
    mode = rospy.get_param('mode', 'manually')
    limb = rospy.get_param('arm', 'right_arm')
    # print "realsense: ", realsense
    print "mode: ", mode

    print("Initializing a node for rgbd tf calibration... ")
    node_name = limb + "rgbd_tf_calibration"
    rospy.init_node(node_name, anonymous=True)

    # start the controller
    # arm = KinectCalibrationWithAR(args.limb, mode='poseset', moveToInitial=False)
    arm = KinectCalibrationWithAR(limb, mode=mode, moveToInitial=False, realsense=realsense)

    # when it returns it is done
    print("Calibration Done.")

    rospy.spin()    # only reactive control, wait for action cmd received


##########################
if __name__ == '__main__':
    main()
