import rospy
import smach
import smach_ros
import threading
from operator import itemgetter
import numpy as np

from apc_msgs.srv import DetectObject2D
from apc_msgs.srv import DetectObject2DRequest
from geometry_msgs.msg import Pose, Point, Quaternion

# ==========================================================
class ChooseWhereToGoState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tote','bin'],
            input_keys=['next_item_to_pick','pick_locations'],
            output_keys=['next_item_to_pick']
            )

    # ==========================================================
    def execute(self, userdata):

        if userdata['pick_locations'][0] == 'tote':
            return 'tote'
        else:
            # Make sure to fill in next_item_to_pick bin location for rest of pipeline
            userdata['next_item_to_pick']['bin'] = userdata['pick_locations'][0]
            return 'bin'
