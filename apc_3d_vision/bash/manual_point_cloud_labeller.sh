#!/bin/bash

cd ../models/
rosrun apc_3d_vision manual_point_cloud_labeller empty_shelf.pcd pod_lowres.pcd empty_shelf_labelled.pcd aligned_pod.pcd

pcl_viewer empty_shelf_labelled.pcd aligned_pod.pcd
