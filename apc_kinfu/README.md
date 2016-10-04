# ACRV_kinfu

## Dependencies
* [PCL 1.8](https://github.com/PointCloudLibrary/pcl.git)
  * Before building, add add the files in apc_kinfu/pcl_files to the appropriate locations
    * kinfu.cpp -> gpu/kinfu/src/kinfu.cpp
    * kinfu.h -> gpu/kinfu/include/pcl/gpu/kinfu/kinfu.h
    * internal.h -> gpu/kinfu/src/internal.h
    * colors.cu -> gpu/kinfu/src/cuda/colors.cu
    * maps.cu -> gpu/kinfu/src/cuda/maps.cu

  * Follow install instructions provided with the following changes:
    * Use `ccmake ..` in place of `cmake ..` and toggle ON the CUDA and GPU options
    * Press C to configure and G to generate
  * It may be required that a number of PCL include files are in an incorrect location after making and installing
    * internal.h
    * camera_pose.h
    * safe_call.hpp
    * tsdf_volume.h
    * tsdf_volume.hpp
    * kinfu.h (use the modified file made available in the apc_kinfu/pcl_files folder)
* [apc_msgs](https://bitbucket.org/acrv/apc_msgs.git)

## Install Instructions
* `git clone https://bitbucket.org/acrv/apc_kinfu.git` into catkin workspace
* `catkin_make`
* `roslaunch ros_kinfu ros_kinfu.launch`
