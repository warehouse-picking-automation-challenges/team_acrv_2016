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
class ChooseTheNextItemToPick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
            input_keys=['next_item_to_pick','pick_plan','pick_locations','object_2d_medians'],
            output_keys=['next_item_to_pick','object_2d_median']
            )

    # ==========================================================
    def execute(self, userdata):

        # Grab next item in list and update next_item_to_pick
        item_id = int(userdata['pick_plan'][0])
        userdata['next_item_to_pick']['id'] = item_id

        # Make sure that the next item to pick bin matches the plan
        # userdata['next_item_to_pick']['bin'] = userdata['pick_locations'][0]

        # Recall userdata['pick_plan'] is a list of item_ids as strings
        userdata['object_2d_median'] = userdata['object_2d_medians'][userdata['pick_plan'][0]]

        return 'succeeded'
