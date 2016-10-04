#!/bin/bash

# To use: source baxter_commands.sh

# Baxter Notes

## Baxter SDK

    # Baxter Enable
    alias be="rostopic pub -1 /robot/set_super_enable std_msgs/Bool True"
    # Baxter Disable
    alias bd="rostopic pub -1 /robot/set_super_enable std_msgs/Bool False"
    # Baxter Reset
    alias br="rostopic pub -1 /robot/set_super_reset std_msgs/Empty"
    # Baxter State
    alias bs="rostopic echo -c /robot/state"
    # Baxter disable collision avoidance
    alias bdcc="rostopic pub -r 6 /robot/limb/right/CollisionAvoidance/suppress_body_avoidance std_msgs/Empty"

## Baxter CPP Shortcuts

    # Start the hardware controllers
    alias bhardware="roslaunch baxter_control baxter_hardware.launch"
    # Simulate Baxter
    alias bgazebo="roslaunch baxter_gazebo baxter_gazebo.launch"
    # Visualize Baxter
    alias bvisualize="roslaunch baxter_control baxter_visualization.launch"

    # Baxter move_group
    alias bm="roslaunch baxter_moveit_config baxter_moveit.launch"
    # Baxter pick place
    alias bpp="roslaunch baxter_pick_place block_pick_place.launch"
    # Send Baxter to ready position, avoiding obstacles
    alias bready="rosrun baxter_moveit_scripts send_ready"

## Baxter Demos

    alias bw="rosrun joint_velocity wobbler.py"
    alias bsu="rosrun baxter_scripts sonar_enable.py --enable=0"  # thie only works when baxter_scripts are installed
    alias sonaroff="rostopic pub /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0"
    alias bsd="rosrun baxter_scripts sonar_enable.py --enable=1"
    alias bkeyboard="rosrun joint_position keyboard.py"
    alias brtare="rosrun baxter_tools tare.py -l right"
    alias bltare="rosrun baxter_tools tare.py -l left"
    alias brcalibrate="rosrun baxter_tools calibrate_arm.py -l right"
    alias blcalibrate="rosrun baxter_tools calibrate_arm.py -l left"

## Turn of MoveIt's access to depth data

    alias bdisablesensors="rosparam delete /move_group/sensors"

## Right Gripper

    alias brgripperstate="rostopic echo -c /robot/end_effector/right_gripper/state"
    alias brgrippercal="rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: calibrate, id: 65664, sender: user}'"
    alias brgripperres="rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: reset, id: 65664, sender: user}'"
    alias brgripperopen="rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: release, id: 65664, sender: user}'"
    alias brgripperclose="rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: grip, id: 65664, sender: user}'"

## Right Suction

    alias brsuckerstart="rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: go, id: 65537, sender: user}'"
    alias brsuckerstop="rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: stop, id: 65537, sender: user}'"
    alias brsuckerstart2s="rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: go, id: 65537, sender: user, args: "{\"grip_attempt_seconds\": 2}"}'"

## Left Gripper

    alias blgripperstate="rostopic echo -c /robot/end_effector/left_gripper/state"
    alias blgrippercal="rostopic pub -1 /robot/end_effector/left_gripper/command baxter_core_msgs/EndEffectorCommand '{command: calibrate, id: 65664, sender: user}'"
    alias blgripperres="rostopic pub -1 /robot/end_effector/left_gripper/command baxter_core_msgs/EndEffectorCommand '{command: reset, id: 65664, sender: user}'"
    alias blgripperopen="rostopic pub -1 /robot/end_effector/left_gripper/command baxter_core_msgs/EndEffectorCommand '{command: release, id: 65664, sender: user}'"
    alias blgripperclose="rostopic pub -1 /robot/end_effector/left_gripper/command baxter_core_msgs/EndEffectorCommand '{command: grip, id: 65664, sender: user}'"

### SSH Access

    #alias bssh="ssh osrf@011305P0009.local"
    #alias blog="ftp 011305P0009.local"
    #alias bdownloadlogs="cd ~/ros/baxter_logs/ && rm -rf cu_boulder_ftp_logs.tar.gz 011305p0009.local/ && wget -r ftp://011305P0009.local/ && tar cvzf cu_boulder_ftp_logs.tar.gz 011305p0009.local/ && scp cu_boulder_ftp_logs.tar.gz fabgatec@davetcoleman.com:~/www/"

## View Camera Streams

    alias brcamera="rosrun image_view image_view image:=/cameras/right_hand_camera/image"
    alias blcamera="rosrun image_view image_view image:=/cameras/left_hand_camera/image"
    alias bacamera="rosrun image_view image_view image:=/camera/image_raw"
    alias bdepthcamera="rosrun image_view image_view image:=/camera/rgb/image_color"

## SSH Access to Baxter   -  http://sdk.rethinkrobotics.com/wiki/SSH

    alias bstop="sudo rc-service rethink stop"  # stop baxter software
    alias bstart="sudo rc-service rethink start"  # start baxter software
    alias bshutdown="sudo shutdown -h now"
    alias breboot="sudo reboot" #restart

## Baxter Time

    # See http://sdk.rethinkrobotics.com/wiki/Time_and_NTP

    # Check the offset of time of Baxter
    alias btimeoffset="ntpdate -q 128.138.224.231"

    # Get the latest baxter IP address
    # baxter_ip_address
