#!/bin/bash

cd ../models/
rosrun apc_3d_vision align_pca_icp_test rotated_pod_lowres.pcd grid.pcd pod_lowres_aligned_to_empty_shelf.pcd \
0.01 \
0.03 \
10 1

pcl_viewer grid.pcd pod_lowres_aligned_to_empty_shelf.pcd
