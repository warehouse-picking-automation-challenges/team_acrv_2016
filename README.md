# Installation Instructions

## Various Required Packages

```
sudo apt-get install libeigen3-dev
sudo apt-get install libnlopt-dev
sudo apt-get install libxmlrpc-c++8-dev
sudo apt-get install libudev-dev
sudo apt-get install ros-indigo-pcl-conversions
sudo apt-get install ros-indigo-ar-track-alvar
sudo apt-get install python-sklearn python-termcolor
```

## Various Not Necessarily Required packages

```
sudo apt-get install cmake-curses-gui
```

## ROS Indigo

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
apt-cache search ros-indigo
sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall
```

## MoveIt!

```
sudo apt-get install ros-indigo-moveit-*
```

## PCL

Place this wherever you checkout non-ROS repos. I use a checkout folder in my home directory, `co` for short.

Note: To get version `1.8.0`, checkout `master` at the `pcl-1.8.0` tag and create the branch `local-1.8.0` so that we do not remain in a detached head state. The commands below include this step.

```
mkdir ~/co
cd ~/co
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout tags/pcl-1.8.0 -b local-1.8.0
cd ~/ros_ws/src/apc_docs/pcl_patch
./pcl_fix_pre.sh
cd -
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_GPU=ON -DBUILD_CUDA=ON
make -j7
sudo make -j7 install
cd ~/ros_ws/src/apc_docs/pcl_patch
./pcl_fix_post.sh
```

## Segmentation Library

Build and install segmentation library before building segmentation_ros.

```
cd ~/ros_ws/src/acrv_apc/segmentation
mkdir build
cd build
cmake ..
make
sudo make install
```

## librealsense

Place this wherever you checkout non-ROS repos. I use a checkout folder in my home directory, `co` for short.

```
cd ~/co
git clone https://github.com/IntelRealSense/librealsense.git
```

Then follow instructions at: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#ubuntu-installation

## Baxter SDK

Create the following directory structure:

```
|-ros_ws
|--|-baxter.sh (wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh && chmod u+x baxter.sh)
|--|-src
|--|--|-acrv_apc (This repository.)
|--|--|-baxter_sdk (This is where to put baxter sdk packages like baxter_common, baxter_interface, etc.)
|--|--|-CMakeLists.txt (This is a default CMakeLists file.)
```

Run the following commands:

```
sudo apt-get update
sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers
cd ~/ros_ws/src
mkdir baxter_sdk
cd baxter_sdk
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
source /opt/ros/indigo/setup.bash
touch baxter_common/baxter_description/CATKIN_IGNORE
touch baxter_common/rethink_ee_description/CATKIN_IGNORE
cd ~/ros_ws
catkin_make
catkin_make install
wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
chmod u+x baxter.sh
```

Make necessary changes to `baxter.sh`. See: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup#Step_6:_Configure_Baxter_Communication.2FROS_Workspace

# 3rd-Party Software

This repository makes use of 3rd-party software that falls under a different license from the software developed by Team ACRV. Our license does not apply to 3rd-party software included in this repository and the intended license of the original owners left in place. Modifications made to 3rd-party software have been made in alignment with the governing license. Modifications may not be directly documented.

## 3rd-Party Software List

 * acrv_trac_ik -> https://bitbucket.org/traclabs/trac_ik
 * apc_gripper_description -> https://github.com/RethinkRobotics/baxter_common
 * baxter_description -> https://github.com/RethinkRobotics/baxter_common
 * baxter_moveit -> https://github.com/ros-planning/moveit_robots
 * rethink_ee_description -> https://github.com/RethinkRobotics/baxter_common
 * apc_random_orders -> https://github.com/Jorge-C/apc_random_orders
 * apc_kinfu -> https://github.com/PointCloudLibrary/pcl/tree/master/gpu/kinfu
 * ros_realsense -> https://github.com/jamessergeant/ros_realsense
