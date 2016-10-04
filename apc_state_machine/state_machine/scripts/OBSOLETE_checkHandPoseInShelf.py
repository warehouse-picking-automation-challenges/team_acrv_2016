import rospy
import smach
import moveit_lib.srv
import numpy as np
import baxter_interface
from std_msgs.msg import String
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from copy import deepcopy

import tf

class CheckHandPoseInShelf(smach.State):
    # send either a prefix to be added to (usually) the BIN name from the
    def __init__(self, relative_pose=None, pose_frame_id='',
                 movegroup=None, velocity_scale=1.0):
        smach.State.__init__(self, outcomes=['in_shelf', 'not_in_shelf', 'failed'])

        self.tf_listener = tf.TransformListener()

    # ==========================================================
    def execute(self, userdata):
        try:
            # painful tf lookup!
            # rospy.loginfo('Looking up transform from %s into %s frames....' % (frame_id, hand_frame))
            self.tf_listener.waitForTransform('/shelf', '/left_endpoint', rospy.Time(0), rospy.Duration(3.0))

            common_time = self.tf_listener.getLatestCommonTime('/shelf', '/left_endpoint')

            # this creates the pose of the hand, expressed in the goal frame
            hand_in_shelf_frame = self.tf_listener.lookupTransform('/shelf', '/left_endpoint', common_time)

            print 'hand_in_shelf_frame = '
            print hand_in_shelf_frame

            if hand_in_shelf_frame[0][2] < 0:
                rospy.loginfo("We are still in the shelf!")
                return 'in_shelf'
            else:
                rospy.loginfo("All good, we are out of the shelf!")
                return 'not_in_shelf'

        except Exception, e:
            rospy.logerr('Cannot lookup a transform from %s to %s now, but required to perform relative motion of the %s move group.'  % (hand_frame, frame_id, self.movegroup_))
            print e
            return 'failed'

        return 'failed'
