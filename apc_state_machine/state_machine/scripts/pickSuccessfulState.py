import rospy
import smach
import smach_ros
import threading
from std_msgs.msg import String
import mission_plan.srv


class PickSuccessfulState(smach.State):
    """ Call this state whenever a pick operation was successful. It will update the mission plan
        and keep track of where all the objects are. This is important to maintain a valid JSON representation
        of the current state of the shelf and tote.

        Use the StowSuccessfulState if you are in stowing mode!
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['next_item_to_pick'],
                             output_keys=['next_item_to_pick']
                             )

        # wait for the service to appear
        rospy.loginfo('Waiting for services of mission_plan/move_item to come online ...')
        try:
            rospy.wait_for_service('/mission_plan/move_item', timeout=2)
        except:
            rospy.logerr('Services of /mission_plan/move_item not available. Restart and try again.')
            # rospy.signal_shutdown('')
        self.srv_move = rospy.ServiceProxy('/mission_plan/move_item', mission_plan.srv.MoveItem)

        # wait for the service to appear
        rospy.loginfo('Waiting for services of mission_plan/pop_item to come online ...')
        try:
            rospy.wait_for_service('/mission_plan/pop_item', timeout=2)
        except:
            rospy.logerr('Services of /mission_plan/pop_item not available. Restart and try again.')
            # rospy.signal_shutdown('')
        self.srv_pop = rospy.ServiceProxy('/mission_plan/pop_item', mission_plan.srv.PopItem)

    # ==========================================================
    def execute(self, userdata):

        # try to call the service
        # !! warning, this could potentially block forever
        try:
            request = mission_plan.srv.MoveItem()
            request.item = userdata['next_item_to_pick']['item']
            request.from_bin = userdata['next_item_to_pick']['bin']
            request.to_bin = 'tote'

            response = self.srv_move.call(request.item, request.from_bin, request.to_bin)
            if response.success:
                self.srv_pop.call()
                userdata['next_item_to_pick'] = {'bin': '', 'item': ''}
                return 'succeeded'
            else:
                rospy.logwarn('Something went wrong when calling MoveItem from the PickSuccessfulState.')
                rospy.logwarn('The representation of the state of objects in the shelf and tote is most likely inconsistent now.')
                return 'aborted'

        except Exception, e:
            rospy.logerr('Service call to mission_plan failed. Reason: \n %s' % e)
            return 'aborted'
