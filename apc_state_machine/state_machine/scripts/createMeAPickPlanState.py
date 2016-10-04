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
class CreateMeAPickPlanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
            input_keys=['next_item_to_pick','object_2d_medians','object_2d_depths'],
            output_keys=['pick_plan','pick_locations','pick_plan_base_bin']
            )

    # ==========================================================
    def execute(self, userdata):
        # A list of item ids ordered by depth values
        pick_plan = sorted(userdata['object_2d_depths'], key=userdata['object_2d_depths'].get)
        userdata['pick_plan'] = pick_plan

        pick_locations = []
        userdata['pick_locations'] = []

        for item_id_str in pick_plan:
            if item_id_str == str(userdata['next_item_to_pick']['id']):
                pick_locations.append('tote')
            else:
                # TODO replace this with a more intelligent selection
                pick_locations.append('bin_A')

        userdata['pick_locations'] = pick_locations
        pick_plan_base_bin = userdata['next_item_to_pick']['bin']
        userdata['pick_plan_base_bin'] = pick_plan_base_bin
        print 'Pick Plan:'
        print pick_plan
        print pick_locations
        print pick_plan_base_bin

        return 'succeeded'
