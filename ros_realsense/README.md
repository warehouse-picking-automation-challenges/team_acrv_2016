# ros_realsense
A ROS wrapper for Intel Realsense (TM) cameras using the librealsense library.

This package is being developed for the [Australian Centre for Robotic Vision](http://roboticvision.org)'s team in the [Amazon Picking Challenge](http://amazonpickingchallenge.org/). At this time, only a select set of image and pointcloud streams are available. A number of issues have been raised in order to implement additional functionality.

## Install
### Dependencies
Follow the installation methods at the links below:

* [librealsense](https://github.com/IntelRealSense/librealsense)
* [Point Cloud Library (PCL)](http://pointclouds.org/downloads/)
* [Robot Operating System (ROS)](http://wiki.ros.org/indigo/Installation)
- Note: currently this repository has only been tested on ROS Indigo on Ubuntu 14.04

### ros_realsense
* `git clone http://github.com/jamessergeant/ros_realsense.git`
* `catkin_make`
