import rospy
import smach
import mission_plan.srv


class PopItemState(smach.State):
    """ This state should only be used when in PICKING mode, to remove items from the mission plan after
        the picking was UNSUCCESSFUL and should not be attempted again!

        If picking was SUCCESSFUL, use the new PickSuccessfulState instead.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
        output_keys=['next_item_to_pick']
        )

        # wait for the service to appear
        rospy.loginfo('Waiting for services of mission_plan/pop_item to come online ...')
        try:
            rospy.wait_for_service('/mission_plan/pop_item', timeout=2)
        except:
            rospy.logerr('Services of /mission_plan/pop_item not available. Restart and try again.')
            # rospy.signal_shutdown('')


        self.service = rospy.ServiceProxy('/mission_plan/pop_item', mission_plan.srv.PopItem, 1)

    # ==========================================================
    def execute(self, userdata):

        # try to call the service
        # !! warning, this could potentially block forever
        try:
            response = self.service.call()
            userdata['next_item_to_pick'] = {'bin':'', 'item':''}
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to mission_plan failed. Reason: \n %s' % e)
            return 'aborted'
