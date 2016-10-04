#from spaceBot import *
import rospy
import smach
import smach_ros
import threading

from suctionState import SuctionState


class AbortState(smach.State):
    def __init__(self, duration=1.0):
        smach.State.__init__(self, outcomes=['succeeded'], #, 'failed'],
                             input_keys=['move_group']) # suctionState needs that one
        self.suctionStateOff = SuctionState(state='off')


    # ==========================================================
    def execute(self, userdata):
        # wait for the specified duration or until we are asked to preempt
        rospy.loginfo("In the AbortState execution...")
        self.suctionStateOff.execute(userdata)
        # = SuctionState(state='off')
        # if self.preempt_requested():
        # else:
        #     return 'succeeded'
        return 'succeeded'
