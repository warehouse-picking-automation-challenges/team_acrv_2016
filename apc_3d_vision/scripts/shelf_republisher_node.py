#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool


def republishShelf():
    rospy.init_node('shelf_republisher_node', anonymous=True)
    rospy.loginfo("Shelf Republisher ROS node is started!")
    pub = rospy.Publisher('/republish_prealigned_shelf', Bool, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    msg = Bool()
    msg.data = True
    counter = 0
    max_tries = rospy.get_param('max_republish_tries','20')
    rospy.sleep(10)
    while counter < max_tries:
        pub.publish(msg)
        rate.sleep()
        counter = counter + 1

    rospy.loginfo("Shelf Republished!")
    rospy.loginfo("Shelf Republisher ROS node is shutting down!")


if __name__ == '__main__':
    try:
        republishShelf()
    except rospy.ROSInterruptException:
        pass
