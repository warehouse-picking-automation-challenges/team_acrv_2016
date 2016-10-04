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
class RemoveObjectFromPickPlan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
            input_keys=['pick_plan','pick_locations','next_item_to_pick','pick_plan_base_bin'],
            output_keys=['pick_plan','pick_locations','next_item_to_pick']
            )

    # ==========================================================
    def execute(self, userdata):

        # Pop the first list index
        del userdata['pick_plan'][0]
        del userdata['pick_locations'][0]
        print userdata['pick_plan_base_bin']
        userdata['next_item_to_pick']['bin'] = userdata['pick_plan_base_bin']

        return 'succeeded'
