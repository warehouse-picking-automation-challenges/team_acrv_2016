Follow these instructions to get a simulated version of APC up and running.

You will need the apc_sim repository: https://bitbucket.org/acrv/apc_sim


```
roslaunch apc_sim apc_baxter_world.launch
```

wait until it to say "Gravity compensation is turned on" (this can take a while)

Note, not all bag files will work, one needs to be
recorded with the following topics:

```
/realsense/rgb/image_raw
/realsense/rgb/camera_info
/realsense/points_aligned
/realsense/depth_aligned/camera_info
/realsense/depth_aligned/image_raw
/realsense/points
```

Note: when passing in the bag file, you must use the full path.

```
roslaunch apc_sim apc_sim.launch bagfile:=/path/to/bagfile.bag
roslaunch state_machine baxter_state_machine.launch
```
