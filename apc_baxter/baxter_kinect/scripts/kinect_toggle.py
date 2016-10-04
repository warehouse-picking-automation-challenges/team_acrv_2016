#!/usr/bin/env python
import subprocess
import rospy
from std_msgs.msg import String

class KinectToggle(object):
    """docstring for """
    def __init__(self):
        self.subscriber = rospy.Subscriber("/kinect_toggle", String,
                                           self.callback)

    # read once from the topic then unregister and wait
    def callback(self, data):
        if not (data.data == "on" or data.data == "off"):
            rospy.logerr(rospy.get_caller_id() +
                        'input received is not valid: ' + data.data)
            return
        rospy.loginfo(rospy.get_caller_id() + " I heard: %s", data.data)
        state = self.checkKinectState()
        if data.data == state:
            print 'kinect2_bridge state = ', state
            return   # we are already in the current state
        if data.data == "on":           # we need to turn the kinect on
            rospy.loginfo("Trying to turn ON the kinect2")
            self.turnOnKinect()
            print "after on"
        if data.data == "off":
            rospy.loginfo("Trying to turn OFF the kinect2")
            self.turnOffKinect()
            print "after off"

    def checkKinectState(self):
        # out = os.system("rostopic list")
        try:
            output = subprocess.check_output("rosnode list | grep kinect2_bridge",
                                             shell=True)
            print "\nNumber of outputs received: %d\n" % len(output)
            if len(output) > 0: return "on"
            return "off"
        except subprocess.CalledProcessError, e:
            if e.returncode == 1: return "off"
            else: print e

    def turnOnKinect(self):
        try:
            subprocess.Popen("roslaunch kinect2_bridge kinect2_bridge.launch",
                             shell=True)
        except subprocess.CalledProcessError, e:
            print e

    def turnOffKinect(self):
        try:
            output = subprocess.check_output(
                "rosnode kill /kinect2 /kinect2_bridge /kinect2_points_xyzrgb_hd /kinect2_points_xyzrgb_qhd /kinect2_points_xyzrgb_sd",
                shell=True)
            print "\nNumber of outputs received: %d\n" % len(output)
        except subprocess.CalledProcessError, e:
            print e


# Usage: the programme starts a rosnode with a topic called /kinect_toggle
# It waits for the user to send an "on" or "off" command to kill or launch
# the relevant ROS nodes for the kinect2_bridge (which in turn launches kinect2)
#   to test:
#       rostopic pub /kinect_toggle std_msgs/String \"on\"
#       rostopic pub /kinect_toggle std_msgs/String \"off\"

if __name__ == '__main__':
    rospy.init_node('kinect_toggle')# anonymous=True)
    rospy.loginfo("Kinect Toggle ROS node is started!")
    finished = False
    rec = KinectToggle()
    rospy.spin()
    rospy.loginfo("Kinect Toggle ROS node is shutting down!")
