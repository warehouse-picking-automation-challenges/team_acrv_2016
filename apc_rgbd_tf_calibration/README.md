# README #

This repository is for a ros package used to calibrate the transformation between the link of an RGBD sensor and the Baxter base.

### Summary ###

* The calibration is achieved through minimizing the error between predictions from two different kinematic chains.
* One chain is based on the AR tag recognition result provided by ar track alvar using both RGB and depth information.
* The other chain is based on the Baxter tf trees.
* In the optimization, the algorithm actually estimate two unknown transformations: the transformation from the Baxter base to the RGBD sensor link; and the transformation from the Baxter gripper base to the AR tag. In our applications, only the former one is necessary.

### Dependencies ###

* Baxter SDK
* ar track alvar ([http://wiki.ros.org/ar_track_alvar](Link URL))

### Autonomous Calibration ###

* Attach (and fix) an AR tag on to a Baxter arm (for kinect: tag on left arm; for realsense: tag on right arm). Remember to use a white-blue AR tag instead of a white-black one, avoiding holes in depth images.
* Run the driver for an RGBD sensor.
* Before running the main scripts, remember to leave enough space for Baxter's arm to move.
* For the calibration of head kinect: roslaunch apc_rgbd_tf_calibration kinect_ar_depth_indiv.launch
* For the calibration of realsense: roslaunch apc_rgbd_tf_calibration realsense_ar_depth_indiv.launch
* After running, calibration results are saved in "kinect-Calibration_Results.txt" / "realsense-Calibration_Results.txt".
* Update the calibrated data into the launch file for publishing a static tf (baxter_kinect_calibrated / baxter_realsense_calibrated).

### Manual Calibration and Poseset Update ###
* Comment Line 26 and uncomment Line 25 in corresponding launch files (kinect_ar_depth_indiv.launch / realsense_ar_depth_indiv.launch).
* Do the same procedures as those in "Autonomous Calibration".
* When running the launch files, the arms (left and right) will not move autonomously, users can move the arms to desired poses, then press the "Enter" button on keyboard for each sample collection.
* After enough samples collected, press Ctrl + C to stop the calibration. Calibrations results are svaed in "kinect-Calibration_Results.txt" / "realsense-Calibration_Results.txt".
* Update the calibrated data into the launch file for publishing a static tf (baxter_kinect_calibrated / baxter_realsense_calibrated).
* To update the posesets for autonomous calibration in the future, after the manual calibration, replace all text in kinect-predefined_left_poses.csv by those in kinect-left_arm_poses.csv. Same for right and realsense.

### Maintainers ###

* Fangyi Zhang (fangyi.zhang@hdr.qut.edu.au)
* Liao Wu (liao.wu@qut.edu.au)
* Juxi Leitner (j.leitner@qut.edu.au)
