import rospy
import smach
import mission_plan.srv


class GetToteContents(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
        output_keys=['next_item_to_pick'] # objects_in_bin
        )

        # wait for the service to appear
        rospy.loginfo('Waiting for services of mission_plan/get_next_item to come online ...')
        try:
            rospy.wait_for_service('/mission_plan/get_tote_items', timeout=1)
        except:
            rospy.logerr('Services of /mission_plan/get_tote_items not available. Restart and try again.')
            # rospy.signal_shutdown('')


        self.get_tote_items = rospy.ServiceProxy('/mission_plan/get_tote_items', mission_plan.srv.GetToteItems)

    # ==========================================================
    def execute(self, userdata):

        # try to call the service
        # !! warning, this could potentially block forever
        try:
            bin_contents = rospy.get_param('/mission_plan/bin_contents')
            response = self.get_tote_items.call()
            # NOTE not using bin_contents from the service response because it is wrong # Dem Hacks
            userdata['next_item_to_pick'] = {'tote_contents':response.tote_contents,'bin_contents':bin_contents,'bin':'tote'}
            return 'succeeded'
        except Exception,e:
            rospy.logerr('Service call to mission_plan failed. Reason: \n %s' % e)
            return 'aborted'
