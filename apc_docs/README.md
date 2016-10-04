# Installation Instructions

Create the following directory structure:

```
|-apc_ws
|--|-src
|--|--|-baxterpicks (This repository.)
|--|--|-baxtersdk (This is where to put baxter sdk packages like baxter_common, baxter_interface, etc.)
|--|--|-iai_kinect2 (git clone https://github.com/code-iai/iai_kinect2.git. See Note 1 below.)
|--|--|-libfreenect2 (git clone https://github.com/OpenKinect/libfreenect2.git.)
|--|--|-CMakeLists.txt (This is a default CMakeLists file.)
|--|-baxter.sh
```

In the root of the workstation directory (apc_ws), run ```catkin_make -j1```.
Note the ```-j1``` is important for the ROS Indigo version of catkin_make as
there seems to be a bug with dependencies not being properly waited for when
performing parallel compilation. Compiling sequentially fixes this.

You should now have a directory structure with a build and devel folder in the
root of the workspace.

## ROS Installation Instructions

We are using ROS Indigo

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

## Baxter SDK Installation Instructions

Steps from: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup

Install SDK dependencies:

```
sudo apt-get update
sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers
```

Using the wstool workspace tool, we will checkout all required Baxter Github Repositories into your ROS workspace source directory.

```
cd ~/co/apc_ws/src/BaxterSDK
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
```

You should now have the folders

## MoveIt! Installation Instructions

```
sudo apt-get install ros-indigo-moveit-*
```

## Kinect2 Installation Instructions

To build iai_kinect2 and libfreenect2, follow the instructions below.

### libfreenect2 Instructions

```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
sudo apt-get install ros-indigo-pcl-conversions
sudo apt-get install libeigen3-dev

cd ~/co/
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2/depends/
./download_debs_trusty.sh
sudo apt-get install build-essential cmake pkg-config
sudo dpkg -i debs/libusb*deb
sudo apt-get install libturbojpeg libjpeg-turbo8-dev
sudo dpkg -i debs/libglfw3*deb; sudo apt-get install -f;

(If the next command conflicts with other packages, don't do it.)
sudo apt-get install libgl1-mesa-dri-lts-vivid

cd ..
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/freenect2 -DENABLE_CXX11=ON
make
sudo make install
```

### iai_kinect2 Instructions

To install iai_kinect2, you must already have your ROS environment setup,
initialised and have installed libfreenect2.

```
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
Unplug and Replug Kinect2. To test, run ./bin/Protonect

cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws

Open iai_kinect2/kinect2_bridge/CMakeLists.txt and change "$ENV{HOME}" to "/usr/local/"

catkin_make -j1
```

Inside the file iai_kinect2/kinect2_bridge/launch/kinect2_bridge.launch, the
publish_tf argument will need to be manually changed to true. As below:

```
  <arg name="publish_tf"        default="true" />
```

NOTE!!! You may need to restart your workspace session to launch launch files.

## MoveIt Installation

```
sudo apt-get install ros-indigo-moveit-full
```

## PCL Installation Instructions


While you will already have PCL-1.7.2 installed from libfreenect2's
installation instructions, we must install the latest version of PCL from
source for usage in our code. The steps taken to achieve this are as
follows. Note: If you choose not to install PCL with sudo make install,
you must update CustomPCLConfig.cmake with the appropriate location.

```
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl/
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
```

## OpenCV 3 Installation Instructions

Note: DO NOT INSTALL Open CV 3. These steps are just here as means of the future
possibility of using Open CV 3. This breaks our packages.

The following steps were modified from: http://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html#gsc.tab=0

```
cd ~/co/
wget https://github.com/Itseez/opencv/archive/3.1.0.zip
git clone https://github.com/Itseez/opencv_contrib.git
cd opencv-3.1.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=~/co/opencv_contrib/modules ..
make -j7
sudo make install
```

## BaxterPicks Installation Instructions

The 2D Object matching algorithm depends on:

```
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update
sudo apt-get install libopencv-nonfree-dev
```

# Useful Code Snippets

## Useful ROS Things

Some handy topic/node/transform visualisation tools:

```
rosrun rqt_gui rqt_gui
rosrun rqt_tf_tree rqt_tf_tree
rosrun rqt_graph rqt_graph
rostopic list | grep kinect

rosnode info /
```

Example of recording a ROS bag file:

```
rosbag record tf /kinect2/qhd/camera_info /kinect2/qhd/image_color /kinect2/qhd/image_color/compressed /kinect2/qhd/image_color_rect /kinect2/qhd/image_color_rect/compressed /kinect2/qhd/image_depth_rect /kinect2/qhd/image_depth_rect/compressed /kinect2/qhd/image_mono /kinect2/qhd/image_mono/compressed /kinect2/qhd/image_mono_rect /kinect2/qhd/image_mono_rect/compressed /kinect2/qhd/points

rosbag record /realsense/depth/image_raw /realsense/ir/image_raw /realsense/points /realsense/rgb/image_raw /realsense/rgb_depth_aligned/image_raw /realsense/rgb_depth_aligned/camera_info /realsense/rgb/camera_info /realsense/ir/camera_info /realsense/depth/camera_info

rosbag record tf /kinect2/qhd/camera_info /kinect2/qhd/image_color /kinect2/qhd/points /empty_shelf_point_cloud /labelled_pointcloud /segmented_objects
```

Save a kinect point cloud to a pcd:

```
rosbag record -O kinect_raw.bag --duration=2 /kinect2/qhd/points
mkdir kinect_pcds
rosrun pcl_ros bag_to_pcd kinect_raw.bag /kinect2/qhd/points kinect_pcds
```

Create an image dataset:

```
Create a folder for the extracted images and cd in there.
rosrun image_view extract_images _sec_per_frame:=0.01 image:=<IMAGETOPICINBAGFILE>

In another terminal window, run rosbag play <BAGFILE>
A sequence of images will be created.
```

### Solving Dependency Issues

Sometimes when you are trying to get a new ROS package to build with catkin_make, you may get an error like the following:

```
Could not find a package configuration file provided by "qt_build" with any
  of the following names:

    qt_buildConfig.cmake
    qt_build-config.cmake
```

This means there is a ROS package called `qt-build` that the package you are building depends on. Notice the jump between underscore: `_` and dash: `-`. To remedy, just run the following and be amazed that there this package exists and that installing it fixes your compilation error.

```
sudo apt-get install ros-indigo-qt-build
```

## Useful Baxter Things

Put Face on Baxter:

```
rosrun baxter_examples xdisplay_image.py -f ../open.png
```

Turn Off Sonar Sensors (The annoying clicking sound!):

```
rostopic pub /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0
```

Find the time sync offset between this computer and Baxters' computer:

```
ntpdate -q qutbaxter
```

Set this computers time to the same time as Baxters' computer:

```
sudo date --set="$(ssh ruser@qutbaxter date)"
```

Baxters password is `rethink`.

To find Baxters IP address:

```
ping qutbaxter
```

To SSH into Baxter:

```
ssh ruser@192.168.1.162
```

Enable/Disable Grippers/Suction:

Details at: http://api.rethinkrobotics.com/baxter_core_msgs/html/msg/EndEffectorCommand.html

```
Suck for 2 seconds:
rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: go, id: 65537, sender: user, args: "{\"grip_attempt_seconds\": 2}"}'

Stop sucking:
rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{command: stop, id: 65537, sender: user}'

Change gripper suction :
rostopic pub -1 /robot/end_effector/left_gripper/command baxter_core_msgs/EndEffectorCommand '{command: go, id: 65537, sender: user, args: "{\"grip_attempt_seconds\": 5}"}'

Change gripper suction :
rostopic pub -1 /robot/end_effector/left_gripper/command baxter_core_msgs/EndEffectorCommand '{command: stop, id: 65537, sender: user}'
```

Convert a .urdf.xacro to a .urdf:

```
rosrun xacro xacro.py --inorder -o /home/baxter/co/apc_ws/src/apc_baxter/baxter_description/urdf/baxter.urdf /home/baxter/co/apc_ws/src/apc_baxter/baxter_description/urdf/baxter.urdf.xacro
```

## Useful PCL Things

Convert ply to pcd:

```
pcl_ply2pcd shelf_clean.ply test_kinect2_pcd_clean.pcd
```

```
pcl_pcd2ply shelf.pcd shelf.ply
pcl_viewer test_kinect2_pcd.pcd

mkdir kinect_pcds;rosrun pcl_ros bag_to_pcd 2016-03-17-14-53-53.bag /kinect2/qhd/points ./kinect_pcds
mkdir empty_shelf_pcds;rosrun pcl_ros bag_to_pcd 2016-03-17-14-53-53.bag /empty_shelf_point_cloud ./empty_shelf_pcds
mkdir labelled_objects_pcds;rosrun pcl_ros bag_to_pcd 2016-03-17-14-53-53.bag /labelled_pointcloud ./labelled_objects_pcds
mkdir segmented_objects_pcds;rosrun pcl_ros bag_to_pcd 2016-03-17-14-53-53.bag /segmented_objects ./segmented_objects_pcds
```

Convert an stl model to a point cloud pcd:

```
meshlab
open stl
Filter>Sampling>Poisson Disk Sampling
Choose Number of Samples (i.e. number of points you want in the point cloud representation)
export as ply
pcl_ply2pcd mymodel.ply mymodel.pcd
```

## Useful Computer Things

Split a file into max 10GB chunks:

```
split --bytes=10000000000 2016-03-17-14-53-53.bag splitbag
```

Merge split files:

```
cat splitbag* > myfile.bag
```

View available hard drive space: `df -h`

# AR Tracking

## AR Track Alvar

Here's how to launch the Alvar AR tag tracker:

```
roslaunch ar_track_alvar baxter_bundle_no_kinect.launch
```

# Calibration

## Baxter Arm Calibration

All the steps to perform a calibration of Baxter's arms can be found [here](http://sdk.rethinkrobotics.com/wiki/Arm_Calibration).

## Stereo Calibration

```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.058 right:=/kinect2/qhd/image_color left:=/cameras/left_hand_camera/image right_camera:=/kinect2 left_camera:=/cameras/left_hand_camera
```

### Head Rotation Node

Baxters head camera transform is rotated incorrectly for some readon. We fixed by creating a static transform publisher that republishes the camera stream under the corrected transform.

```
rosrun amazon_picking camera_head_rot
roslaunch src/BaxterPicks/BaxterSDKCustom/BaxterKinect/Launch/baxter_head.launch
```

## Kinect to Robot Transform Calibration

I found a package called [baxter_kinect_calibration](https://github.com/h2r/baxter_h2r_packages/blob/indigo-devel/baxter_kinect_calibration/README.md) that claims to be purpose built to calculate the transform between a Kinect and the Baxter robot.

To run the calibration:

```
roslaunch baxter_kinect_calibration baxter_bundle_calibrate.launch
rosrun tf tf_echo /world /kinect2_link
```

It seems that the transform assumes a left arm fixed orientation. Need to
investigate.

Another alternative calibration tool that we briefly investigated is the [multi camera calibration](https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/A5hwdDKzRns) tool.

## Kinect 2 Camera Calibration

https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration

If we ended up wiping the Baxter companion computer, you will need to checkout
the repo from the link above and compile everything as per instructions. Will
also need to checkout and build everything at https://github.com/OpenKinect/libfreenect2#debianubuntu-1404-perhaps-earlier.

Chessboard used is the laminated one attached to flat piece of steel floating around the S11 labs. Square size is 58 mm or 0.058 m. Size of the chessboard is 6x8 which is equal to the number of feature points the calibration tool uses. NOT the number of grid squares along each side.

Need Kinect bridge running. Follow same steps as would be followed to setup for
a Baxter demo. Need to start Baxter as the main ROS instance is run on his
computer.

Run through the Detailed steps as at the URL above. Change \_fps_limit to 5 instead as its less annoying that way.

Note: We tried to use the ROS camera calibration package but it saves the images in a different format to the iai_kinect2 tool and doesn't work with ir or depth images.

Note: Alternative calibration step involves using the Matlab tool `cameraCalibrator`. After running the calibration in Matlab, you will need to use Inkyu's script to convert the calibration file to a ROS compatible calibration file.

To see the Kinect camera view (or other camera views), you can run `rosrun rqt_gui rqt_gui`. Once this is open, in the top menu, click `Plugins->Visualisation->Image View`.
Now on the right side of the screen, click on the menu bar (possibly is blank at this point) and select the ROS topic with your feed. I.e. `kinect2/qhd/image_color`.

Make sure to scroll down to the Example results section on the web page and look at the superimposed depth and colour images after you have finished calibration. When looking at the point cloud image with superimposed colour, you can zoom around with the mouse and use the plus and minus keys to increase and decrease the size of the pixels.

# Development Environment Setup

Its always important to spend a little time to setup your development environment in a nice way to maximise productivity and allow you to focus on work.

## Terminator

Save a profile and save a layout.
Can add custom initial command for each terminal window under the layout
settings.
Launch terminator by running:
terminator -l layout_name -p profile_name

## Qt Creator

Qt Creator is an IDE great for developing C++ code. To open a project with it,
open the CMakeLists.cmake file for your project.

### Installation Instructions

To install Qt Creator, you must download the latest version from the Qt website.
Don't use the apt-get version as its old. Make sure to uninstall with `sudo
apt-get remove qtcreator` if you have already installed it this way.

### Themes!!!

Personally, I dislike the included text editor colour schemes included with
Qt Creator. To install a new one like the pleasing Tomorrow Night theme,
download the xml from https://github.com/chriskempson/tomorrow-theme/blob/master/QtCreator/styles/Tomorrow-Night.xml
and copy it to  `~/.config/QtProject/qtcreator/styles`. If the styles folder
does not exist, create it. Once the xml is in the folder, you should see it
under Tools->Options->Text Editor->Fonts & Colors->Color Scheme.

## Atom

While Qt Creator is excellent for writing, compiling, running and debugging code, there are some features in Atom that make writing code faster. I usually jump back and forth between Atom and Qt Creator.

Packages to install:

 * `minimap`

Packages to remove/disable:

 * `markdown-preview`

# Other Notes

Don't get inspiration for commit messages from: http://whatthecommit.com/

# Running Everything

Enable the robot and joint trajectory action server:

```
rosrun baxter_tools tuck_arms.py -u
rosrun baxter_interface joint_trajectory_action_server.py
```

Enable the kinect and its static transform:

```
roslaunch src/baxterpicks/BaxterSDKCustom/BaxterKinect/Launch/baxter_kinect.launch
roslaunch kinect2_bridge kinect2_bridge.launch
```

Register a pre-saved kinect-captured version of the shelf, align it to the live
kinect version and publish on /registered_empty_shelf:

```
roslaunch apc_vision shelf_registration_ros_node.launch
```

MoveIt's octomap will receive /registered_empty_shelf. To change this, change
the topic name under:

```
BaxterSDKCustom/baxter_moveit/baxter_moveit_config/config/kinect2_sensor.yaml
```

Remove the corresponding points between /registered_empty_shelf and
/kinect2/qhd/points and publish the points that remain on /floating_objects:

```
roslaunch apc_vision float_objects_ros_node.launch
```

Start up these service nodes for performing 3D cad model matching:

 * segmentation_ros_node: segment a point cloud into a labelled point cloud
 * split_labelled_point_cloud_ros_node: split a labelled point cloud into an array of point clouds
 * fit_cad_models_ros_node: attempt to fit a list of cad models to an array of point clouds

```
rosrun segmentation_ros segmentation_ros_node
roslaunch apc_vision split_labelled_point_cloud_ros_node.launch
roslaunch apc_vision fit_cad_models_ros_node.launch
```

Use a static transform for aligning the shelf with respect to the robot:

```
roslaunch src/baxterpicks/BaxterSDKCustom/BaxterKinect/Launch/shelf_tf.launch
```

Launch RVIZ and get the correct model:

```
roslaunch baxter_moveit_config apc_baxter_moveit.launch
```

Load the cad model into the MoveIt planning scene:

```
Scene Objects
Import From Text
baxterpicks/BaxterSDKCustom/baxter_moveit/baxter_moveit_config/baxter_scenes/apc.scene
Under Context click Publish Current Scene
```

Launch the realsense camera on James' computer:

```
roslaunch ros_realsense realsense_camera.launch
```

Run the state machine:
Note:
mission_plan/mission_descriptions/pick.json contains the order in which objects are being picked from the shelf, see bottom of the file under 'work_order'

```
rosrun moveit_lib moveit_robot
roslaunch mission_plan mission_plan.launch
rosparam load src/apc_state_machine/state_machine/parameters/shelf_layout.yaml
rosrun state_machine shelfPicking_smach.py
```

Run 2D detection:

```
roslaunch find_object_2d apc.launch
rosrun find_object_2d find_object_2d_wrapper.py
```

# Hardware - Software Interfacing Issues

I got no permission for /dev/video0, solved by adding the user (dockpc) to the
group (video).

```
sudo usermod -a -G video dockpc
```
