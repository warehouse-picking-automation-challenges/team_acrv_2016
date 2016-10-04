# README #


### What is this repository for? ###

* Contains the Baxter **configuration** for:

    * Sucker, SRDF file (named poses)
    * collision models and moveti configs

* Contains **parameters** and global constants --> global_params.yaml

* Contains **scripts** for:

    * Recording a pose from Baxter into SRDF format: record_pose.py
    * Turn the Kinect2_Bridge off or on: kinect_toggle.py

### kinect_toggle.py

Usage: the programme starts a rosnode with a topic called /kinect_toggle
It waits for the user to send an "on" or "off" command to kill or launch
the relevant ROS nodes for the kinect2_bridge (which in turn launches kinect2)
to test:

*  ``rostopic pub /kinect_toggle std_ms/String \"on\"``
*  ``rostopic pub /kinect_toggle std_ms/String \"off\"``

### Who do I talk to? ###

* Repo owner or admin