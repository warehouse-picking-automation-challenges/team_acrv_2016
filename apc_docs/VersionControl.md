At every point that the entire pipeline improves, you must create a new tag on
each of the following repositories:

 * acrv_tracik
 * apc_3d_vision
 * apc_baxter
 * apc_deep_vision
 * apc_docs
 * apc_grasping
 * apc_kinfu
 * apc_msgs
 * apc_objects
 * apc_random_orders
 * apc_rgbd_tf_calibration
 * apc_state_machine
 * moveit_lib
 * object_proposal
 * ros_realsense
 * segmentation
 * segmentation_ros

Note: Make sure to push the tag after you create it! This can be done by right
clicking on the tag and clicking push. Unfortuneately this is not included in
the main push button of GitKraken at this time.




Ideal git config specification:

These settings allow GitKraken to handle authentication and allows any user to
commit code on the command line.

```
[core]
	repositoryformatversion = 0
	filemode = true
	bare = false
	logallrefupdates = true
[remote "origin"]
	url = https://bitbucket.org/acrv/object_proposal.git
	fetch = +refs/heads/*:refs/remotes/origin/*
[branch "master"]
	remote = origin
	merge = refs/heads/master
```
